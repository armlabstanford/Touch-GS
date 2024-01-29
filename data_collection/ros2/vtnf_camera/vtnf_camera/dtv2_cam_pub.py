

#### code for publishing the webcam feed as well as the DTv2 depth / color images

import rclpy
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import time
from rclpy.node import Node


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
        self.timer = self.create_timer(time_period, self.timer_callback)

        # self.cap = cv2.VideoCapture(self.camera_id)
        self.cap = cv2.VideoCapture(self.camera_id)
        self.br = CvBridge()
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        # self.cap.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
        # disable autofocus
        # self.cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        # self.cap.set(cv2.CAP_PROP_SETTINGS, 1)
        # camera frame rate 60 fps
        # self.cap.set(cv2.CAP_PROP_FPS, 60)

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
