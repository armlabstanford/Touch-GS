

#### code for publishing the webcam feed as well as the DTv2 depth / color images
import sys

# print(sys.exec_prefix)
sys.path.append('/home/wkdo/miniconda3/envs/dtros2/lib/python3.10/site-packages')

import torch

import rclpy
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import time
from rclpy.node import Node 
import os, re
import threading

from .utils.utils import get_video_device_number
from .Img2Depth.img2depthforce import getDepth, getForce
from .Img2Depth.networks.DenseNet import DenseDepth
from .Img2Depth.networks.STForce import DenseNet_Force


class camPublisher(Node):
    def __init__(self):
        super().__init__('cam_publisher_dt')

        # print("curr directory", os.getcwd())
        path_base = os.getcwd()
        os.chdir(os.path.join(path_base, "src/vtnf_camera/vtnf_camera/"))
        # print("curr directory", os.getcwd())


        self.declare_parameter('camera_id', '/dev/video2')  # Default value if not provided

        self.camera_id = self.get_parameter('camera_id').get_parameter_value().string_value
        print("dt's camera id: {}".format(self.camera_id))

        # self.device_num = 0
        device_number = get_video_device_number(self.camera_id)
        # if it's odd number, reduce by 1
        if device_number % 2 == 1:
            device_number -= 1

        print ("dtv2_device_number: ", device_number)
        self.camera_id = device_number

        self.camera_id = 0
        queue_size = 1
        self.pub_dtv2 = self.create_publisher(Image, 'vtnf/camera_{}'.format(self.camera_id), queue_size)
        self.pub_dtv2_depth = self.create_publisher(Image, 'vtnf/depth', queue_size)
        # self.pub_dtv2_depthview = self.create_publisher(Image, 'vtnf/depthview', queue_size)


        self.br = CvBridge()

        ##################### depth estimation 

        sensornum = 2
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
        self.isforce = 0 # disable force for now

        self.imgidx = np.load('Img2Depth/calib_idx/mask_idx_{}.npy'.format(sensornum))
        self.radidx = np.load('Img2Depth/calib_idx/pts_2ndmask_{}_80deg.npy'.format(sensornum))

        self.cen_x, self.cen_y, self.exposure = self.get_sensorinfo(sensornum)
        self.flag = 0
        self.camopened = True
        self.netuse = True
        # Params
        self.image = None
        self.img_noncrop = np.zeros((768,1024))
        self.maxrad = 16.88
        self.minrad = 12.23
        self.input_width = 640
        self.imgsize = int(self.input_width/2)

        if self.netuse: 
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


        ##################### depth estimation done

        use_timer = True
        # should we done with timer for ensuring stable frame rate?
        # 25 frame rate
        if use_timer:
            timerspeed = 0.04
            self.timer = self.create_timer(timerspeed, self.timer_callback)
            print("OpenCV Version: {}".format(cv2.__version__))
        
        
            self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)
            if not (self.cap.isOpened()):
                print("Cannot open the camera")


            self.cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 768)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            self.cap.set(cv2.CAP_PROP_AUTO_WB, 1)
            self.cap.set(cv2.CAP_PROP_APERTURE, 150)
            commands = [
                ("v4l2-ctl --device /dev/video"+str(self.camera_id)+" -c auto_exposure=3"),
                ("v4l2-ctl --device /dev/video"+str(self.camera_id)+" -c auto_exposure=1"),
                ("v4l2-ctl --device /dev/video"+str(self.camera_id)+" -c exposure_time_absolute="+str(150)),
        ]
            for c in commands: 
                os.system(c)


            print(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            print(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            # show fourcc code in interpretable way
            fourcc = self.cap.get(cv2.CAP_PROP_FOURCC)
            fourcc = int(fourcc)
            fourcc = fourcc.to_bytes(4, 'little').decode()
            print(fourcc)
            print(self.cap.get(cv2.CAP_PROP_FOURCC))
            print(self.cap.get(cv2.CAP_PROP_AUTOFOCUS))
            print(self.cap.get(cv2.CAP_PROP_SETTINGS))
            print(self.cap.get(cv2.CAP_PROP_FPS))

        else:
            self.capture_thread = threading.Thread(target=self.capture_and_publish)
            self.capture_thread.start()

    def timer_callback(self):
        """
            callback function for the publishing and reading the sensor img
        """
        ret, frame = self.cap.read()
        if ret:

            msg = self.br.cv2_to_imgmsg(frame)
            self.pub_dtv2.publish(msg)

            rectImg = self.rectifyimg(frame)

            depthImg = getDepth(self.model_pos, rectImg)
            # print("depthImg: ", depthImg.shape)
            msg_depth = self.br.cv2_to_imgmsg(depthImg, "8UC1")
            self.pub_dtv2_depth.publish(msg_depth)

        else:
            self.get_logger().error('Failed to capture frame')

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

    def capture_and_publish(self):
        print("OpenCV Version: {}".format(cv2.__version__))
        cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)
        # cap = cv2.VideoCapture(self.camera_id, cv2.CAP_GSTREAMER)
        if not (cap.isOpened()):
            print("Cannot open the camera")


        cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 768)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_AUTO_WB, 1)
        cap.set(cv2.CAP_PROP_APERTURE, 150)
        commands = [
            ("v4l2-ctl --device /dev/video"+str(self.camera_id)+" -c auto_exposure=3"),
            ("v4l2-ctl --device /dev/video"+str(self.camera_id)+" -c auto_exposure=1"),
            ("v4l2-ctl --device /dev/video"+str(self.camera_id)+" -c exposure_time_absolute="+str(150)),
       ]
        for c in commands: 
            os.system(c)


        print(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        print(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        # show fourcc code in interpretable way
        fourcc = cap.get(cv2.CAP_PROP_FOURCC)
        fourcc = int(fourcc)
        fourcc = fourcc.to_bytes(4, 'little').decode()
        print(fourcc)
        print(cap.get(cv2.CAP_PROP_FOURCC))
        print(cap.get(cv2.CAP_PROP_AUTOFOCUS))
        print(cap.get(cv2.CAP_PROP_SETTINGS))
        print(cap.get(cv2.CAP_PROP_FPS))


        while rclpy.ok():
            # print out how much time it takes to capture and publish
            start_time = time.time()

            
            ret, frame = cap.read()
            if not ret:
                self.get_logger().error('Failed to capture frame')
                break
            msg = self.br.cv2_to_imgmsg(frame)
            self.pub_dtv2.publish(msg)

            rectImg = self.rectifyimg(frame)

            depthImg = getDepth(self.model_pos, rectImg)
            # print("depthImg: ", depthImg.shape)
            msg_depth = self.br.cv2_to_imgmsg(depthImg, "8UC1")
            self.pub_dtv2_depth.publish(msg_depth)

            # imgDepth_rgb = cv2.cvtColor(depthImg, cv2.COLOR_GRAY2RGB)
            # msg_depthshow = self.br.cv2_to_imgmsg(imgDepth_rgb, "rgb8")
            # self.pub_dtv2_depthview.publish(msg_depthshow)

            # if netuse: 
            #     if self.isforce == 1:
            #         forceEst = getForce(self.model_force, rectImg)
            #         # print("Force: ", forceEst)
            #         wrench_stamped_msg = WrenchStamped()
            #             # Set the force and torque values in the message
            #         wrench_stamped_msg.header.stamp = rospy.Time.now()
            #         wrench_stamped_msg.wrench.force = Vector3(*forceEst[:3])
            #         wrench_stamped_msg.wrench.torque = Vector3(*forceEst[3:])
            #         self.force_pub.publish(wrench_stamped_msg)
            #     if self.ispos == 1:
            #         # depthImg = self.imgDepth(rectImg)
            #         depthImg = getDepth(self.model_pos, rectImg)
            #         msg_depth = self.br.cv2_to_imgmsg(depthImg, "8UC1")
            #         msg_depth.header.stamp = rospy.get_rostime()
            #         self.img_pub_depth.publish(msg_depth)

            #         imgDepth_rgb = cv2.cvtColor(depthImg, cv2.COLOR_GRAY2RGB)
            #         msg_depthshow = self.br.cv2_to_imgmsg(imgDepth_rgb, "rgb8")
            #         msg_depthshow.header.stamp = rospy.get_rostime()
            #         self.img_pub_depth_show.publish(msg_depthshow)

            # msg = self.br.cv2_to_imgmsg(frame, 'bgr8')


            end_time = time.time()




def main(args=None):
    rclpy.init(args=args)
    # get camera_id among ros2 parameters
    

    
    cam_pub = camPublisher()

    rclpy.spin(cam_pub)
    cam_pub.capture_thread.join()
    cam_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
