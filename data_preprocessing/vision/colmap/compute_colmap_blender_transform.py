import json

import numpy as np
import torch
from scipy.spatial.transform import Rotation as R


def read_camera_poses_colmap(path):
    # read the camera poses from the colmap images txt file
    # return a dictionary with the camera id as the key and the camera pose as the value
    
    camera_id_to_pose = {}
    with open(path, 'r') as file:
        for line in file:
            if "jpg" in line[-4:-1] or "png" in line[-4:-1]:
                entries = line.split(" ")
                qw = float(entries[1])
                qx = float(entries[2])
                qy = float(entries[3])
                qz = float(entries[4])
                tx = float(entries[5])
                ty = float(entries[6])
                tz = float(entries[7])
                camera_id = int(entries[0])
                image_name = entries[9].strip()
                
                # transform the points from colmap to world to world to colmap frame
                r = R.from_quat([qx, qy, qz, qw])
                r_mat = r.as_matrix()
                
                T = np.array([tx, ty, tz])
                
                T_prime = -r_mat.T @ T
                
                tx_prime, ty_prime, tz_prime = T_prime[0], T_prime[1], T_prime[2]
                
                camera_id_to_pose[camera_id] = {"orientation": r_mat.T, 
                                                "translation": [tx_prime, ty_prime, tz_prime], "image_name": image_name}
                
    
    return camera_id_to_pose


def read_camera_poses_blender(json_file):
    
    with open(json_file, 'r') as file:
        data = json.load(file)
        
        camera_poses = {}
        for frame in data["frames"]:
            # Extract file path and transform matrix
            file_path = frame["file_path"]
            transform_matrix = np.array(frame["transform_matrix"])
            r = transform_matrix[:3, :3]
            t = transform_matrix[:3, 3]
            camera_id = int(file_path.split('/')[-1].split('.')[0])
            camera_poses[camera_id] = {"orientation": r, 
                                       "translation": [t[0], t[1], t[2]], "image_name": file_path}
            
            
    return camera_poses


def compute_scaling_factor(camera_id_to_pose_colmap, camera_id_to_pose_blender):
    length = len(camera_id_to_pose_colmap)
    ratios = []
    for i in range(1, length):
        b_1 = camera_id_to_pose_blender[i]["translation"]
        c_1 = camera_id_to_pose_colmap[i]["translation"]
        for j in range(i+1, length+1):
            b_2 = camera_id_to_pose_blender[j]["translation"]
            c_2 = camera_id_to_pose_colmap[j]["translation"]
            
            diff_blender = np.array(b_1) - np.array(b_2)
            diff_colmap = np.array(c_1) - np.array(c_2)
            
            dist_blender = np.linalg.norm(diff_blender)
            dist_colmap = np.linalg.norm(diff_colmap)
            
            ratio = dist_colmap/dist_blender
            ratios.append(ratio)
            
    ratios = np.array(ratios)
    return np.mean(ratios), np.std(ratios)

if __name__ == '__main__':
    camera_id_to_pose_colmap = read_camera_poses_colmap("data_preprocessing/vision/point_cloud/sample_colmap_data/images.txt")
    
    # read camera id to pose from blender data
    camera_id_to_pose_blender = read_camera_poses_blender("data_preprocessing/vision/point_cloud/sample_blender_data/transforms_train.json")
    
    print(camera_id_to_pose_blender)
    
    mu, std = compute_scaling_factor(camera_id_to_pose_colmap, camera_id_to_pose_blender)
    
    print('scaling factor mean: ', mu)
    
    print(0.05 * mu)