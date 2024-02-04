

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
        super().__init__('cam_publisher_webcam')

        self.declare_parameter('camera_id', '/dev/video0')  # Default value if not provided

        self.camera_id = self.get_parameter('camera_id').get_parameter_value().string_value
        print("webcam's camera id: {}".format(self.camera_id))
        # self.device_num = 0

        device_number = get_video_device_number(self.camera_id)
        # if it's odd number, reduce by 1
        if device_number % 2 == 1:
            device_number -= 1

        print ("webcam_device_number: ", device_number)
        self.camera_id = device_number

        queue_size = 1
        self.pub_webcam = self.create_publisher(Image, 'vtnf/camera_{}'.format(self.camera_id), queue_size)


        self.br = CvBridge()
        # # https://github.com/antmicro/ros2-camera-node/tree/main
        # # https://stackoverflow.com/questions/69029560/when-i-build-opencv-it-does-not-recognise-my-installed-ffmpeg


        use_timer = True
            # should we done with timer for ensuring stable frame rate?
            # 50 frame rate
        if use_timer:
            timerspeed = 0.02
            self.timer = self.create_timer(timerspeed, self.timer_callback)

            print("OpenCV Version: {}".format(cv2.__version__))
            self.cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)

            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(1920))
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(1080))
            self.cap.set(cv2.CAP_PROP_FPS, 60)
            # os.system("v4l2-ctl -d0 --set-fmt-video=width=1920,height=1080,pixelformat=0") 
            # os.system("v4l2-ctl -d0 --set-parm=60")
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(1920))
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(1080))
            self.cap.set(cv2.CAP_PROP_FPS, 60)

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
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to capture frame')
            return
        else:
            # msg = self.br.cv2_to_imgmsg(frame, 'bgr8')
            msg = self.br.cv2_to_imgmsg(frame)
            self.pub_webcam.publish(msg)


    def capture_and_publish(self):
        print("OpenCV Version: {}".format(cv2.__version__))
        cap = cv2.VideoCapture(self.camera_id, cv2.CAP_V4L2)

        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(1920))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(1080))
        cap.set(cv2.CAP_PROP_FPS, 60)
        # os.system("v4l2-ctl -d0 --set-fmt-video=width=1920,height=1080,pixelformat=0") 
        # os.system("v4l2-ctl -d0 --set-parm=60")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(1920))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(1080))
        cap.set(cv2.CAP_PROP_FPS, 60)

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

# docker run \
#   -e DISPLAY=$DISPLAY \
#   -v $HOME/.Xauthority:/wkdo/.Xauthority:ro \
#   -v ~/catkin_ws/:/catkin_ws \
#    noetic_wkdo
# docker run -it \
#    -e DISPLAY=$DISPLAY \
#    -v /tmp/.X11-unix:/tmp/.X11-unix \
#    -v ~/catkin_ws/:/catkin_ws \
#    --device=/dev/dri:/dev/dri \
#   noetic_wkdo
# docker run -it \
#    --net=host \
#    -e DISPLAY=$DISPLAY \
#    -v $HOME/.Xauthority:/root/.Xauthority:rw \
#    -v /tmp/.X11-unix:/tmp/.X11-unix \
#    -v ~/catkin_ws/:/catkin_ws \
#   noetic_wkdo
# docker run -it --rm \
#     --privileged \
#     --network host \
#     -e NVIDIA_VISIBLE_DEVICES=all \
#     -e NVIDIA_DRIVER_CAPABILITIES=all \
#     --env="DISPLAY" \
#     --env="QT_X11_NO_MITSHM=1" \
#     --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
#     --name "noetic_wkdo" \
#     --runtime nvidia \
#     xrf-robot-repo \
#     /bin/bash

        while rclpy.ok():
            # print out how much time it takes to capture and publish
            start_time = time.time()

            
            ret, frame = cap.read()
            if not ret:
                self.get_logger().error('Failed to capture frame')
                break

            # msg = self.br.cv2_to_imgmsg(frame, 'bgr8')
            msg = self.br.cv2_to_imgmsg(frame)
            self.pub_webcam.publish(msg)
            # cv2.imshow('frame', frame)
            
            # if cv2.waitKey(1) & 0xFF == ord('q'): # makes about 40fps.. 
            #     break

            end_time = time.time()
            # print("time: {}".format(end_time - start_time))


def main(args=None):
    rclpy.init(args=args)
    cam_pub = camPublisher()

    rclpy.spin(cam_pub)
    cam_pub.capture_thread.join()
    cam_pub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
