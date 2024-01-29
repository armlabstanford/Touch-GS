

#### code for publishing the webcam feed as well as the DTv2 depth / color images

import rclpy
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import time
from rclpy.node import Node
import os

class camPublisher(Node):
    def __init__(self, camera_id=0):
        super().__init__('cam_publisher_{}'.format(camera_id))
        self.camera_id = camera_id
        queue_size = 1
        self.pub_webcam = self.create_publisher(Image, 'vtnf/camera_{}'.format(camera_id), queue_size)

        # self.pub_dtdepth = self.create_publisher(Image, 'depth', queue_size)
        # self.pub_dtcolor = self.create_publisher(Image, 'color', queue_size)

        time_period = 0.005  # seconds
        
        # timer for callback 
        os.system("v4l2-ctl -d0 --set-fmt-video=width=1920,height=1080,pixelformat=0") 
        os.system("v4l2-ctl -d0 --set-parm=60")
        # self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)
        self.cap = cv2.VideoCapture(self.camera_id)
        # self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_DSHOW)
        self.br = CvBridge()

        if not (self.cap.isOpened()):
            print("Cannot open the webcam")

        W, H = 1920, 1080
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
        self.cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter.fourcc('M','J','P','G'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
        # https://github.com/antmicro/ros2-camera-node/tree/main
        

        # self.cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
        # self.cap.set(cv2.CAP_PROP_FPS, 60)


        # # # disable autofocus
        # # self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        # self.cap.set(cv2.CAP_PROP_SETTINGS, -1)
        # # # camera frame rate 60 fps

        # print out camera properties
        print('camera properties')
        print(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        print(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(self.cap.get(cv2.CAP_PROP_FOURCC))
        print(self.cap.get(cv2.CAP_PROP_AUTOFOCUS))
        print(self.cap.get(cv2.CAP_PROP_SETTINGS))
        print(self.cap.get(cv2.CAP_PROP_FPS))


        self.timer = self.create_timer(time_period, self.timer_callback)


    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            msg = self.br.cv2_to_imgmsg(frame, "bgr8")
            self.pub_webcam.publish(msg)
        else:
            self.get_logger().info('No frame')

def main(args=None):
    rclpy.init(args=args)
    cam_pub = camPublisher(camera_id = 0)

    rclpy.spin(cam_pub)
    cam_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
