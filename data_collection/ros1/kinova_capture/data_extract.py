import rospy
import cv2
from cv_bridge import CvBridge
import os
import rosbag
import json
from collections import defaultdict
import rospy
import cv2
from cv_bridge import CvBridge
import os
import rosbag
from sensor_msgs.msg import Image
import tf
import numpy as np
from geometry_msgs.msg import TransformStamped
import json
from tf.transformations import quaternion_matrix


# Specify the path to your ROS1 bag file
idx_bunny = 3
bag_file_path = f'/media/wkdo/Extreme SSD/vtnf_ros1bag/bunny_{idx_bunny}.bag'
save_dir = f'/media/wkdo/Extreme SSD/vtnf_ros1bag/bunny_{idx_bunny}'
def find_closest(target, collection):
    """
    Find the element in 'collection' closest to 'target'.
    """
    return min(collection, key=lambda x: abs(x[0] - target))

def save_image(bridge, msg, path):
    """
    Convert a ROS image message to an OpenCV image and save it to 'path'.
    """
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    cv2.imwrite(path, cv_image)

def extract_images_and_tf(bag_file_path, start_offset=0):
    if not os.path.exists(bag_file_path):
        raise IOError("Bag file not found")
    
    bag = rosbag.Bag(bag_file_path, 'r')
    bridge = CvBridge()

    # Prepare directories
    images_dir = os.path.join(save_dir, "images")
    os.makedirs(images_dir, exist_ok=True)
    train_dir = os.path.join(images_dir, "train")
    depth_dir = os.path.join(images_dir, "depth")
    os.makedirs(train_dir, exist_ok=True)
    os.makedirs(depth_dir, exist_ok=True)

    data = {
        "camera_angle_x": 0.9272952079772949,
        "frames": []
    }

    last_timestamp = None
    frame_entry = {}
    # Determine the start time for data extraction
    bag_start_time = bag.get_start_time() + start_offset


    # Collect messages
    rgb_messages = [(t.to_nsec(), msg) for _, msg, t in bag.read_messages(topics=['/camera/color/image_raw'])]
    depth_messages = [(t.to_nsec(), msg) for _, msg, t in bag.read_messages(topics=['/camera/depth/image_rect_raw'])]
    tf_messages = [(t.to_nsec(), msg) for _, msg, t in bag.read_messages(topics=['/tf', '/tf_static'])]

    # Sort messages by timestamp to optimize the matching process
    rgb_messages.sort(key=lambda x: x[0])
    depth_messages.sort(key=lambda x: x[0])
    tf_messages.sort(key=lambda x: x[0])

    print(f"Found {len(rgb_messages)} RGB images")
    print(f"Found {len(depth_messages)} depth images")
    print(f"Found {len(tf_messages)} TF messages")
    for transform in msg.transforms:
        if transform.child_frame_id in ["camera_link", "camera_depth_frame"]:
            # Calculate transformation matrix
            trans = transform.transform.translation
            rot = transform.transform.rotation
            matrix = quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
            matrix[0][3] = trans.x
            matrix[1][3] = trans.y
            matrix[2][3] = trans.z
            
            # Add to the last frame entry if it matches the time
            if frame_entry:
                key = "transform_matrix" if transform.child_frame_id == "camera_link" else "transform_matrix_depth"
                frame_entry[key] = matrix.tolist()


    for rgb_time, rgb_msg in rgb_messages:
        # Find the closest depth message
        closest_depth_time, closest_depth_msg = find_closest(rgb_time, depth_messages)

        # Save RGB and depth images with the same index
        rgb_path = os.path.join(train_dir, f"{image_counter:04d}.png")
        depth_path = os.path.join(depth_dir, f"{image_counter:04d}.png")

        # Save transformation data

        
        save_image(bridge, rgb_msg, rgb_path)
        print(f"Saved RGB image to {rgb_path}")
        save_image(bridge, closest_depth_msg, depth_path)
        print(f"Saved depth image to {depth_path}")
        
        # Save transformation data


        image_counter += 1

    # Write to JSON file
    json_filename = os.path.join(save_dir, "transformations.json")
    with open(json_filename, 'w') as f:
        json.dump(data, f, indent=4)

        print("Saved transformations to", json_filename)

    bag.close()



if __name__ == "__main__":
    extract_images_and_tf(bag_file_path)
