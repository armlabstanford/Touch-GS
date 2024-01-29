#! /usr/bin/python3
####!/usr/bin/env python


# print ('abc')
####################
##  [python 3 /2 problems](https://answers.ros.org/question/345942/modulenotfounderror-no-module-named-netifaces/)
####################

import sys
import os
 
print(sys.exec_prefix)
sys.path.append('/home/wkdo/miniconda3/envs/densetact/lib/python3.8/site-packages')



import roslib;

# roslib.load_manifest('rbx1_vision')
import rospy
import struct
import sys
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel as C
import matplotlib.pyplot as plt
import math
from matplotlib import gridspec
# from random import randint
import cv2
import threading
import serial # need to install serial module: pip install pyserial
import time
import argparse
import mpl_toolkits.mplot3d.axes3d as ax3d
import matplotlib.pyplot as plt
import os
import re

import math
import random
from timeit import default_timer as timer

# pointcloud related header

import torch
from Img2Depth.convert_img2depth import getDepth, getDepth_square
import Img2Depth.model as mod
from PIL import Image as ImagePIL

# for camerainfo publisher
import yaml
from sensor_msgs.msg import CameraInfo


#Todo
# : This code runs camera for DenseTact and camera from new webcam.
# This code is called from flightroom_Tactile_cam.launch file in tactile_camera/launch folder.
# Also recommended to use with the code in capturedata folder.
# Since the opencv might lose control on some parameters on new webcam,
# it is recommended to set up the camera parameters before executing this code through launch file.:



class RunCamera:
    def __init__(self, port1, port2):
        super(RunCamera, self).__init__()

        self.id = port1
        self.id2 = port2

        self.camopened = True

        # Params
        self.image = None
        self.br = CvBridge()
        # Node cycle rate (in Hz).
        self.loop_rate = rospy.Rate(1)


        self.device_num = 0
        if os.path.exists(self.id):
            device_path = os.path.realpath(self.id)
            device_re = re.compile("\/dev\/video(\d+)")
            info = device_re.match(device_path)
            if info:
                self.device_num = int(info.group(1))
                # Need to add -1 because it matches the metadata
                if self.device_num %2 != 0 :
                    print("Need to adjust device num with -1")
                    self.device_num -= 1

                print(self.id+ " corresponds to the /dev/video" + str(self.device_num))

 
        #### added code for new webcam (05042022) #######
        self.device_num_webcam = 0

        if os.path.exists(self.id2):
            device_path_webcam = os.path.realpath(self.id2)
            device_re_webcam = re.compile("\/dev\/video(\d+)")

            info_webcam = device_re_webcam.match(device_path_webcam)
            print(device_path_webcam, info_webcam.group(0), info_webcam.group(1))

            if info_webcam:
                self.device_num_webcam = int(info_webcam.group(1))
                # Need to add -1 because it matches the metadata
                if self.device_num_webcam % 2 != 0:
                    print("Need to adjust webcam device num with -1")
                    self.device_num_webcam -= 1

                print("For webcam, " + self.id2 + " corresponds to the /dev/video" + str(self.device_num_webcam))

        print(self.device_num, self.device_num_webcam, self.device_num_webcam %2)

        # find current path
        # directory = os.getcwd()
        # print(directory)
        # '/home/won/.ros'

	# in new laptop, the path has changed into ../catkin_ws/src/tactile_camera... need to change into relative path
        path = '../catkin_ws/src/tactile_camera/'

        self.imgidx = np.load(os.path.join(path, 'include/mask_idx.npy'))
        self.radidx = np.load(os.path.join(path, 'include/2nd_mask_idx_ang_75_rad_25.8.npy'))

        ################## CAM setting ################

        print("done?")
        # self.cap = cv2.VideoCapture(self.device_num, cv2.CAP_DSHOW)
        # self.cap_webcam = cv2.VideoCapture(self.device_num_webcam, cv2.CAP_DSHOW)

        self.cap = cv2.VideoCapture(self.device_num, cv2.CAP_V4L2)
        self.cap_webcam = cv2.VideoCapture(self.device_num_webcam, cv2.CAP_V4L2)
        if not (self.cap.isOpened()):
            print("Cannot open the camera")

        if not (self.cap_webcam.isOpened()):
            print("Cannot open the webcam")

        print("done?")

        self.cap_webcam.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap_webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)




        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
        self.cap_webcam.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap_webcam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        ########################### most important one!!!!!!!!!!!!!! ##################
        # https://forums.raspberrypi.com/viewtopic.php?t=35689
        # https://docs.opencv.org/4.x/d4/d15/group__videoio__flags__base.html#gga023786be1ee68a9105bf2e48c700294dab6ac3effa04f41ed5470375c85a23504
        # https://stackoverflow.com/questions/59726776/how-to-make-cv2-videocapture-read-faster
        ### can solve below error:
        # [ WARN:0] global ../modules/videoio/src/cap_gstreamer.cpp (935) open OpenCV | GStreamer warning: Cannot query video position: status=0, value=-1, duration=-1


        self.cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
        self.cap_webcam.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
        self.cap.set(cv2.CAP_PROP_FPS, 30)


        #########################################

        # # gamma 72!!!!
        # we capture the first frame for the camera to adjust itself to the exposure
        ret_val , cap_for_exposure = self.cap.read()
        ret_val , cap_for_exposure = self.cap_webcam.read()


        # # in windows, it's -10 & linux: 67
        # self.cap.set(cv2.CAP_PROP_EXPOSURE, 67)
        self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 4500)
        self.cap.set(cv2.CAP_PROP_GAMMA, 72)
        self.cap.set(cv2.CAP_PROP_BRIGHTNESS, 0)
        self.cap.set(cv2.CAP_PROP_CONTRAST, 41)
        self.cap.set(cv2.CAP_PROP_HUE, 0)
        self.cap.set(cv2.CAP_PROP_BACKLIGHT, 0)
        self.cap.set(cv2.CAP_PROP_SHARPNESS, 0)
        # self.cap.set(cv2.CAP_PROP_APERTURE, 60)
        self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        # self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)



        # self.set_manual_exposure(self.device_num, 67, 4500)

        print("\nConrast: ", self.cap.get(cv2.CAP_PROP_CONTRAST))
        print("Brightness: ", self.cap.get(cv2.CAP_PROP_BRIGHTNESS))
        print("Saturation: ", self.cap.get(cv2.CAP_PROP_SATURATION))
        print("Hue: ", self.cap.get(cv2.CAP_PROP_HUE))
        print("Gain: ", self.cap.get(cv2.CAP_PROP_GAIN))
        print("Exposure: ", self.cap.get(cv2.CAP_PROP_EXPOSURE))
        print("Auto Exposure: ", self.cap.get(cv2.CAP_PROP_AUTO_EXPOSURE))
        print("Auto wb: ",self.cap.get(cv2.CAP_PROP_AUTO_WB))
        print("Backlight: ",self.cap.get(cv2.CAP_PROP_BACKLIGHT))
        print("Sharpness: ",self.cap.get(cv2.CAP_PROP_SHARPNESS))
        print("Gamma: ",self.cap.get(cv2.CAP_PROP_GAMMA))
        print("WB temperature: ",self.cap.get(cv2.CAP_PROP_WB_TEMPERATURE))

        ################## CAM setting done ################


        ### call for webcam params : just for

        ##################### Model parameter update ####################
        # pathsrc = '../Documents/ws_moveit/src/tactile_camera/src/'
        pathsrc = '../catkin_ws/src/tactile_camera/src'

        checkpoint = os.path.join(pathsrc, 'Img2Depth/epoch15_densenet_final.pth')
        self.img_format = cv2.cvtColor(cv2.imread(os.path.join(pathsrc, 'Img2Depth/Img_example.jpg')), cv2.COLOR_BGR2GRAY)
        self.imgDepth = self.img_format.copy()

        self.image570 = cv2.imread(os.path.join(pathsrc, 'Img2Depth/Img_example_570.png'))
        self.image_original = ImagePIL.fromarray(cv2.cvtColor(self.image570, cv2.COLOR_BGR2RGB))
        self.image_original2 = ImagePIL.fromarray(cv2.cvtColor(self.image570, cv2.COLOR_BGR2RGB))
        self.convertedImg = cv2.imread(os.path.join(pathsrc, 'Img2Depth/Img_example_570.png'))

        self.updateDepth = 0
        self.updatePtcloud = 0

        self.updateDepth2 = 0
        self.updatePtcloud2 = 0

        self.threshold =0
        self.icpCount = 0

        if len(checkpoint) and not os.path.isfile(checkpoint):
            raise FileNotFoundError("{} no such file".format(checkpoint))

        self.device = "cuda"
        self.device = torch.device("cuda" if self.device == "cuda" else "cpu")
        # self.device = torch.device("cpu")
        # self.device = torch.device("cuda")
        print("Using device: {}".format(self.device))

        # map_location = torch.device('cpu')
        # map_location = torch.device("cuda")

        # Initializing the model and loading the pretrained model
        # self.ckpt = torch.load(checkpoint, map_location)
        self.ckpt = torch.load(checkpoint)

        self.model = mod.DenseDepth(encoder_pretrained=False)

        self.model.load_state_dict(self.ckpt["model_state_dict"])
        self.model = self.model.to(self.device)
        print("model load from checkpoint complete ...")
        # set model as evaluate mode
        self.model.eval()

        ################# model setting ended



        # define publisher for img camera - differentiate the name with DenseTact

        self.img_pub = rospy.Publisher("/RunCamera/image_raw_1", Image, queue_size=3)

        self.img_pub_webcam = rospy.Publisher("/RunCamera/webcam", Image, queue_size=3)

        # publish depth
        self.img_pub_depth = rospy.Publisher("/RunCamera/imgDepth", Image, queue_size=2)
        self.img_pub_depth_show = rospy.Publisher("/RunCamera/imgDepth_show", Image, queue_size=2)

        self.pub_caminfo = rospy.Publisher("/RunCamera/camera_info", CameraInfo, queue_size=3)
        
        self.camera_info = self.yaml_to_CameraInfo(os.path.join(path, 'include/caminfo.yaml'))

        print("camera & network setup done")
        # Subscribe to the camera image and depth topics and set
        # the appropriate callbacks
        # self.image_sub = rospy.Subscriber("/RunCamera/image_raw", Image, self.image_callback)

        # rospy.loginfo("Waiting for image topics...")

        # print("abc")

    def yaml_to_CameraInfo(self, yaml_fname):
        """
        Parse a yaml file containing camera calibration data (as produced by 
        rosrun camera_calibration cameracalibrator.py) into a 
        sensor_msgs/CameraInfo msg.
        
        Parameters
        ----------
        yaml_fname : str
            Path to yaml file containing camera calibration data
        Returns
        -------
        camera_info_msg : sensor_msgs.msg.CameraInfo
            A sensor_msgs.msg.CameraInfo message containing the camera calibration
            data
        """
        print('*************file name:     ' ,yaml_fname)
        # Load data from file
        with open(yaml_fname, "r") as file_handle:
            calib_data = yaml.load(file_handle, Loader=yaml.Loader)
                


        # Parse'

        camera_info_msg = CameraInfo()
        camera_info_msg.width = calib_data["image_width"]
        camera_info_msg.height = calib_data["image_height"]
        camera_info_msg.K = calib_data["camera_matrix"]["data"]
        camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
        camera_info_msg.R = calib_data["rectification_matrix"]["data"]
        camera_info_msg.P = calib_data["projection_matrix"]["data"]
        camera_info_msg.distortion_model = calib_data["distortion_model"]
        return camera_info_msg

    def set_manual_exposure(self, video_id, exposure_time, wb_temp):
        '''
        Another option to set the manual exposure
        '''
        commands = [
            ("v4l2-ctl --device /dev/video"+str(video_id)+" -c exposure_auto=3"),
            ("v4l2-ctl --device /dev/video"+str(video_id)+" -c exposure_auto=1"),
            ("v4l2-ctl --device /dev/video"+str(video_id)+" -c exposure_absolute="+str(exposure_time)),

            ("v4l2-ctl --device /dev/video"+str(video_id)+" -c white_balance_temperature_auto=True"),
            ("v4l2-ctl --device /dev/video"+str(video_id)+" -c white_balance_temperature_auto=False"),
            ("v4l2-ctl --device /dev/video"+str(video_id)+" -c white_balance_temperature="+str(wb_temp))
        ]
        for c in commands: 
            os.system(c)
    # usage 
    # set_manual_exposure(1, 18)

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

        beforeRectImg2 = img2[0:570,130:700]
        
        rectImg2 = beforeRectImg2
        # rectImg2 = frame



        return rectImg2

    def CAM_camerashow(self):
        time.sleep(1)

        print('reading..')

        while not rospy.is_shutdown():  

            # Capture frame-by-frame
            ret, frame = self.cap.read()

            ret_webcam, frame_webcam = self.cap_webcam.read()


            rectImg = self.rectifyimg(frame)

            msg_tact = self.br.cv2_to_imgmsg(rectImg, "bgr8")

            msg_tact.header.stamp = rospy.get_rostime()


            # Since the format has been changed from yuyv to mjpg, the cv2 format also have been changed from rgb8 to bgr8
            self.img_pub.publish(msg_tact)

            msg_web = self.br.cv2_to_imgmsg(frame_webcam, "bgr8")

            msg_web.header.stamp = rospy.get_rostime()
            self.camera_info.header.stamp = msg_web.header.stamp
            self.camera_info.header.frame_id = 'camera_frame'
            msg_web.header.frame_id = 'camera_frame'

            self.img_pub_webcam.publish(msg_web)


            self.pub_caminfo.publish(self.camera_info)


            ################## depth image ###########

            framePil = cv2.cvtColor(rectImg, cv2.COLOR_BGR2RGB)
            self.image_original = ImagePIL.fromarray(framePil)

            self.imgDepth, self.imgDepth_show = getDepth_square(self.model, self.image_original, self.device, self.img_format)

            msg_depth = self.br.cv2_to_imgmsg(self.imgDepth, "32FC1")

            msg_depth.header.stamp = rospy.get_rostime()

            self.img_pub_depth.publish(msg_depth)


            imgDepth_rgb = cv2.cvtColor(self.imgDepth_show, cv2.COLOR_GRAY2RGB)

            msg_depthshow = self.br.cv2_to_imgmsg(imgDepth_rgb, "rgb8")


            msg_depthshow.header.stamp = rospy.get_rostime()

            self.img_pub_depth_show.publish(msg_depthshow)

            ####################################

        # When everything done, release the capture

        self.cap.release()
        self.cap_webcam.release()

        cv2.destroyAllWindows()



def cleanup():
    print
    "Shutting down vision node."
    cv2.destroyAllWindows()

if __name__ == '__main__':
    start = timer()
    camopen = True

    ################ Bring ROS param and match the camera idx #############
    rospy.init_node("Pythonnode")

    # What we do during shutdown
    rospy.on_shutdown(cleanup)

    try:
        rospy.get_param_names()
    except ROSException:
        print("could not get param name")

    param_name = rospy.search_param('camname1')
    param_name2 = rospy.search_param('webcamname')
    PortNum1 = rospy.get_param(param_name)
    PortNum2 = rospy.get_param(param_name2)
    print("OpenCV Version: {}".format(cv2.__version__))

    print(PortNum1, PortNum2)
    cSerial = RunCamera(PortNum1, PortNum2)


    if camopen:
        
        # cSerial.CAM_camerashow
        th1 = threading.Thread(target = cSerial.CAM_camerashow)
        th1.start()        

    try:

        # cvBridgeDemo()

        rospy.spin()

    

    except KeyboardInterrupt:
        print
        "Shutting down vision node."
        cv2.destroyAllWindows()
