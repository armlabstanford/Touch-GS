

#### code for publishing the webcam feed as well as the DTv2 depth / color images

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

class camPublisher(Node):
    def __init__(self):
        super().__init__('cam_publisher_dt')

        self.declare_parameter('camera_id', '/dev/video2')  # Default value if not provided

        self.camera_id = self.get_parameter('camera_id').get_parameter_value().string_value
        print("dt's camera id: {}".format(self.camera_id))

        # self.device_num = 0
        dtv2_device_number = get_video_device_number(self.camera_id)
        print ("dtv2_device_number: ", dtv2_device_number)
        self.camera_id = dtv2_device_number

        self.camera_id = 0
        queue_size = 1
        self.pub_dtv2 = self.create_publisher(Image, 'vtnf/camera_{}'.format(self.camera_id), queue_size)

        # self.pub_dtdepth = self.create_publisher(Image, 'depth', queue_size)
        # self.pub_dtcolor = self.create_publisher(Image, 'color', queue_size)

        # time_period = 0.0001  # seconds
        
        # timer for callback 
        # os.system("v4l2-ctl -d0 --set-fmt-video=width=1920,height=1080,pixelformat=0") 
        # os.system("v4l2-ctl -d0 --set-parm=60")

        # self.cap = cv2.VideoCapture(self.camera_id)
        # self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_FFMPEG)
        self.br = CvBridge()


        self.capture_thread = threading.Thread(target=self.capture_and_publish)
        self.capture_thread.start()

    def capture_and_publish(self):
        print("OpenCV Version: {}".format(cv2.__version__))
        cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)
        # cap = cv2.VideoCapture(self.camera_id, cv2.CAP_GSTREAMER)
        if not (cap.isOpened()):
            print("Cannot open the camera")

        # cap = cv2.VideoCapture(self.camera_id)
        # cap = cv2.VideoCapture(self.camera_id, cv2.CAP_DSHOW)

        # cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(1920))
        # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(1080))
        # # cap.set(cv2.CAP_PROP_FORMAT, -1)
        # cap.set(cv2.CAP_PROP_FPS, 60)
        # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1024)
        # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 768)
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

        # os.system("v4l2-ctl -d0 --set-fmt-video=width=1920,height=1080,pixelformat=0") 
        # os.system("v4l2-ctl -d0 --set-parm=60")

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

            # msg = self.br.cv2_to_imgmsg(frame, 'bgr8')
            msg = self.br.cv2_to_imgmsg(frame)
            self.pub_dtv2.publish(msg)

            # cv2.imshow('frame', frame)
            # if cv2.waitKey(1) & 0xFF == ord('q'): # makes about 40fps.. 
            #     break

            # print out how much time it takes to capture and publish
            end_time = time.time()
            # print("time: {}".format(end_time - start_time))

    # def capture_and_publish(self):
    #     ret, frame = self.cap.read()
    #     if ret:
    #         # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    #         # msg = self.br.cv2_to_imgmsg(frame, "bgr8")
    #         msg = self.br.cv2_to_imgmsg(frame)
    #         self.pub_webcam.publish(msg)
    #     else:
    #         self.get_logger().info('No frame')

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
