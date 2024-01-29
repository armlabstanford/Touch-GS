from concurrent.futures import thread
import cv2
import numpy as np
import threading
import time
import os

from timeit import default_timer as timer

### model setting 
import torch
from Img2Depth.img2depthforce import getDepth, getForce
from Img2Depth.networks.DenseNet import DenseDepth
from Img2Depth.networks.STForce import DenseNet_Force


from skimage.metrics import structural_similarity as compare_ssim

import IPython

class RunCamera:
    def __init__(self, port1, sensornum, netuse, camopen = True):
        super(RunCamera, self).__init__()

        # array for determining whether the sensor can do position estimation and force estimation or not
        sen_pf = np.array([[1,1,1],
                        [2,1,1],
                        [3,0,1],
                        [4,0,0],
                        [5,0,1],
                        [6,1,0],
                        [101,1,0],
                        [102,1,0]])
        # whether it use pos or force
        self.ispos = sen_pf[sen_pf[:,0]==sensornum][0,1]
        self.isforce = sen_pf[sen_pf[:,0]==sensornum][0,2]
        
        # IPython.embed()
        self.id = port1

        self.cen_x, self.cen_y, self.exposure = self.get_sensorinfo(sensornum)
        self.flag = 0
        self.camopened = True
        self.netuse = netuse
        # Params
        self.image = None
        self.img_noncrop = np.zeros((768,1024))
        self.maxrad = 16.88
        self.minrad = 12.23
        self.input_width = 640
        self.imgsize = int(self.input_width/2)

        self.device_num = self.id
        self.imgidx = np.load('Img2Depth/calib_idx/mask_idx_{}.npy'.format(sensornum))
        self.radidx = np.load('Img2Depth/calib_idx/pts_2ndmask_{}_80deg.npy'.format(sensornum))

        self.baseimg = cv2.imread('../data/sen_{}_basic.jpg'.format(sensornum))
        if self.baseimg is None:
            print("No base Img")
            self.baseimg = cv2.imread('../data/sen_{}_basic_uncropped.jpg'.format(sensornum))

            self.baseimg = self.rectifyimg(self.baseimg)
            cv2.imwrite('../data/sen_{}_basic.jpg'.format(sensornum), self.baseimg)

        ################## CAM setting ################

        print("done?")
        self.cap = cv2.VideoCapture(self.device_num)

        
        if not (self.cap.isOpened()):
            print("Cannot open the camera")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 768)
        self.cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))


        #########   ######### CAM setting done ################
        if netuse: 
            ######## model setting ######
            if self.ispos == 1:
                self.model_pos = DenseDepth(max_depth = 256, pretrained = False)
                modelname = 'Img2Depth/position_sensor_{}.pth'.format(sensornum)
                print(modelname)
                checkpoint_pos = torch.load(modelname)
                self.ispf = 'pos'
                self.model_pos = torch.nn.DataParallel(self.model_pos)
                self.model_pos.load_state_dict(checkpoint_pos['model'])
                self.model_pos.eval()
                self.model_pos.cuda()
                # self.imgDepth = self.img2Depth(np.ones((640,640,3)))

            if self.isforce == 1:
                self.model_force = DenseNet_Force(pretrained= False)
                modelname = 'Img2Depth/force_sensor_{}.pth'.format(sensornum)
                print(modelname)
                checkpoint_force = torch.load(modelname)
                self.ispf = 'force'
                self.model_force = torch.nn.DataParallel(self.model_force)
                self.model_force.load_state_dict(checkpoint_force['model'])
                self.model_force.eval()
                self.model_force.cuda()
                # self.imgForce = self.img2Force(np.ones((640,640,3)))


    def get_sensorinfo(self, calibnum):
        """
        get center of each sensor.
        """
        brightness = 160
        senarr = np.array([[6, 520, 389, 150, 320],
                            [1, 522, 343, 100, 314],
                            [2, 520, 389, 150, 322],
                            [3, 522, 343, 100, 316],
                            [4, 522, 343, 100, 307],
                            [5, 547, 384, 159, 303],
                            [101, 512, 358, 100, 298],
                            [102, 545, 379, 100, 300],
                            [103, 522, 343, 100, 300]])

        cen_x = senarr[senarr[:,0]==calibnum][0,1]
        cen_y = senarr[senarr[:,0]==calibnum][0,2]
        brightness = senarr[senarr[:,0]==calibnum][0,3]
        self.radius = senarr[senarr[:,0]==calibnum][0,4]
        return cen_x, cen_y, brightness

    def set_manual_exposure(self, video_id, exposure_time):
        '''
        Another option to set the manual exposure
        '''
        commands = [
            ("v4l2-ctl --device /dev/video"+str(video_id)+" -c exposure_auto=3"),
            ("v4l2-ctl --device /dev/video"+str(video_id)+" -c exposure_auto=1"),
            ("v4l2-ctl --device /dev/video"+str(video_id)+" -c exposure_absolute="+str(exposure_time)),
       ]
        for c in commands: 
            os.system(c)

    def rectresult(self, frame):

        mask = np.zeros((640,640), dtype="uint8")
        cv2.circle(mask, (320, 320), self.radius, 255, -1)
        masked = cv2.bitwise_and(frame, frame, mask=mask)
        return cv2.cvtColor(masked, cv2.COLOR_GRAY2RGB)

    def rectifyimg(self, frame2):
        '''
            function for rectifying the image based on the given camera node 
            Now the function manually get the center of each circular shape and match the function. 

            Key is to match the center of pixel correctly so that we can get the right match process with original sensor.

        '''
        beforeRectImg2 = frame2.copy()
        (h, w) = beforeRectImg2.shape[:2]

        img_reshape = beforeRectImg2.reshape(w*h, 3)
        mask = np.ones(img_reshape.shape[0], dtype=bool)
        mask[self.imgidx[self.radidx]] = False
        img_reshape[mask, :] = np.array([0, 0, 0])
        img2 = img_reshape.reshape(h, w, 3)

        beforeRectImg2 = img2[self.cen_y-self.imgsize:self.cen_y+self.imgsize,self.cen_x-self.imgsize:self.cen_x+self.imgsize]
        
        rectImg2 = beforeRectImg2
        return rectImg2

    def image_callback(self, image):
        # make depth img without using ROS

        self.imgDepth2 = self.img2Depth(self.image_original2)
        imgDepth_rgb2 = cv2.cvtColor(self.imgDepth2, cv2.COLOR_GRAY2RGB)

        self.img_pub2.publish(self.bridge.cv2_to_imgmsg(imgDepth_rgb2, "rgb8"))

        # isidx = 2
        self.depth2Ptcloud(self.imgDepth2, 2)

        self.updateDepth2 = 0


    def getPSNR(self, img1, img2):
            gray1 = cv2.cvtColor(img1, cv2.COLOR_BGR2GRAY)
            gray2 = cv2.cvtColor(img2, cv2.COLOR_BGR2GRAY)
            # compute the Structural Similarity Index (SSIM) between the two
            # images, ensuring that the difference image is returned
            (score, diff) = compare_ssim(gray1, gray2, full=True)
            diff = (diff * 255).astype("uint8")
            # print("SSIM: {}".format(score))
            return score


    def CAM_camerashow(self):
        time.sleep(1)

        print('reading..')

        depthImg = self.img_noncrop
        forceEst = 0
        while(True):
            # Capture frame-by-frame
            ret, frame = self.cap.read()
            keypress = 1
            # rectify img based on the param on the launch file 
            rectImg = self.rectifyimg(frame)
            # cv2.imshow('frame', frame)
            cv2.imshow('frame', rectImg)
            # cv2.imshow('depth', depthImg)
            

            key = cv2.waitKey(33)
            # 'q': quit
            if key == 27: # Esc
                cv2.destroyAllWindows()
                break
            elif key == ord('d'):
                if netuse: 
                    if self.isforce == 1:
                        forceEst = getForce(self.model_force, rectImg)
                        print("Force: ", forceEst)
                    if self.ispos == 1:
                        # depthImg = self.imgDepth(rectImg)
                        depthImg = getDepth(self.model_pos, rectImg)
                        # print(np.max(depthImg)/255.0*(self.maxrad-self.minrad)+self.minrad)
                        # cv2.imshow('depth', cv2.cvtColor(depthImg, cv2.COLOR_GRAY2RGB))
                        cv2.imshow('depth', self.rectresult(depthImg))
                else:
                    print('please make netuse True')

            elif key == ord('c'):
                # check psnr with current image 
                psnr = self.getPSNR(rectImg, self.baseimg)
                print(psnr)
                # cv2.imwrite('now.jpg',rectImg)
            elif key == ord('v'):
                self.expo = self.expo - 10
                self.set_manual_exposure(self.device_num, self.expo )
            elif key == ord('b'):
                self.expo = self.expo + 10
                self.set_manual_exposure(self.device_num, self.expo )
            elif key == ord('s'):
                cv2.imwrite('rgb_{}.jpg'.format(self.flag), rectImg)
                cv2.imwrite('depth_{}.jpg'.format(self.flag), depthImg)
                self.flag +=1


        self.cap.release()

        cv2.destroyAllWindows()

def cleanup():
    print
    "Shutting down vision node."
    cv2.destroyAllWindows()


if __name__ == '__main__':
    start = timer()

    os.chdir('Tactile_sensor_gen2/DTv2_cam/scripts/')
    camopen = True
    netuse = True


    sennum = 102

    # first value: video # for dtv2 camera (ex: n = 4 if dtv2 cam is dev/video4)
    # 2nd value: sensor number 
    # 3rd value: netuse = True if you want to use network option(shape / force estimation)
    # 4th value: PSNR checking mode

    cSerial = RunCamera(4, sennum, netuse, camopen)


    if camopen:
        
        th1 = threading.Thread(target = cSerial.CAM_camerashow)
        th1.start()        
 
    try:
        a=1


    except KeyboardInterrupt:
        print
        "Shutting down vision node."
        cv2.destroyAllWindows()
