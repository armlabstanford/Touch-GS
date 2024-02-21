import rospy
import cv2
from cv_bridge import CvBridge
import os
import rosbag
from sensor_msgs.msg import Image
import tf
import numpy as np
from geometry_msgs.msg import TransformStamped

# Specify the path to your ROS1 bag file
bag_file_path = '/media/wkdo/Extreme SSD/vtnf_ros1bag/bunny_1.bag'

def extract_images_and_tf(bag_file_path):
    # Ensure the bag file exists
    if not os.path.exists(bag_file_path):
        raise IOError("Bag file not found")

    try:
        bag = rosbag.Bag(bag_file_path, 'r')
        bridge = CvBridge()

        # Create directories to save extracted images and TF data
        images_dir = "extracted_images"
        if not os.path.exists(images_dir):
            os.makedirs(images_dir)
        
        tf_dir = "extracted_tf"
        if not os.path.exists(tf_dir):
            os.makedirs(tf_dir)

        # Loop through the bag topics
        for topic, msg, t in bag.read_messages(topics=['/camera/color/image_raw', '/vtnf/camera_2', '/vtnf/depth', '/tf']):
            if topic in ['/vtnf/camera_0', '/vtnf/camera_2', '/vtnf/depth']:
                # Convert ROS Image message to OpenCV image
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                
                # Generate a unique filename
                img_filename = os.path.join(images_dir, "{}_{}.jpg".format(topic.replace('/', '_'), t.to_nsec()))
                cv2.imwrite(img_filename, cv_image)
                print("Saved image to", img_filename)

            elif topic == '/tf' and msg.transforms:
                # Process TF messages
                for transform in msg.transforms:
                    # Save TF data to a file or process as needed
                    tf_filename = os.path.join(tf_dir, "tf_{}_{}.txt".format(transform.child_frame_id, t.to_nsec()))
                    with open(tf_filename, 'w') as f:
                        f.write(str(transform))
                    print("Saved TF data to", tf_filename)

    except Exception as e:
        print("Failed to process bag file:", e)
    finally:
        bag.close()

if __name__ == "__main__":
    extract_images_and_tf(bag_file_path)
