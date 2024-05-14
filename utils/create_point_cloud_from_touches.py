# Stanford University, ARMLab 2023
# Touch-GS

import math

import argparse
import os
import random
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import numpy as np
import cv2
import open3d as o3d

import transforms_utils


def get_point_cloud_from_depth_and_color(depth_image, color_image, camera_intrinsics, camera_extrinsics, scaling_factor=1):
    """
    gets the point cloud from the depth and color images
    
    Args:

    depth_image (np.array): The depth image.
    color_image (np.array): The color image.
    camera_intrinsics (list): The camera intrinsics.
    camera_extrinsics (np.array): The camera extrinsics.
    scaling_factor (float): The scaling factor for the depth image.
    
    Returns:
    
    points (np.array): The 3D points.
    """
    fx = camera_intrinsics[0]
    fy = camera_intrinsics[1]
    
    cx = camera_intrinsics[2]
    cy = camera_intrinsics[3]
    
    R = camera_extrinsics[:3, :3]
    t = camera_extrinsics[:3, 3]
    
    # Prepare the 3D point cloud by backprojecting the depth image
    points = []
    colors = []
    
    for v in range(depth_image.shape[0]):
        for u in range(depth_image.shape[1]):
            if depth_image[v, u] == 0: 
                continue  # Skip no depth
            color = color_image[v, u]
            Z = depth_image[v, u] / scaling_factor  # Assume a scaling factor if the depth is not in meters
            if Z == 0: continue  # Skip no depth
            X = (u - cx) * Z / fx
            Y = (v - cy) * Z / fy
            points.append([X, Y, Z])
            colors.append(color)
            
    # Transform points to world coordinates
    
    t = np.array([[t[0]], [t[1]], [t[2]]])  # Example translation vector
    
    R = R @ np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])  # Example rotation matrix
    
    points_world = np.dot(R, np.transpose(points)) + t
    points_world = np.transpose(points_world)

    points = np.array(points_world)
    colors = np.array(colors)
    colors_normalized = colors / 255.0

    return points, colors_normalized    


def matplot_3d_point_cloud(points, colors):
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Scatter plot
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], s=1, c=colors, marker='.')

    # Set axes labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Optionally set equal aspect ratio
    max_range = np.array([points[:, 0].max()-points[:, 0].min(), points[:, 1].max()-points[:, 1].min(), points[:, 2].max()-points[:, 2].min()]).max() / 2.0
    mid_x = (points[:, 0].max()+points[:, 0].min()) * 0.5
    mid_y = (points[:, 1].max()+points[:, 1].min()) * 0.5
    mid_z = (points[:, 2].max()+points[:, 2].min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    # Show the plot
    plt.show()
    
def open3d_point_cloud(points, colors):
    pcd = o3d.geometry.PointCloud()

    # Assign the points to the point cloud
    pcd.points = o3d.utility.Vector3dVector(points)

    # Assign the colors to the point cloud
    pcd.colors = o3d.utility.Vector3dVector(colors)

    # Visualize the point cloud
    o3d.visualization.draw_geometries([pcd])


def get_all_point_clouds(image_dir, touch_depth_dir, touch_var_dir, camera_transformations, camera_intrinsics, i_take, percent_take, viz=False):
    image_filenames  = sorted(os.listdir(image_dir))
    depth_filenames = sorted(os.listdir(touch_depth_dir))
    
    limited_points_xyz = None
    limited_points_rgb = None
    
    for i in range(len(image_filenames)):
        image_filename = image_filenames[i]
        depth_filename = depth_filenames[i]
        
        if i in i_take:
            image = cv2.imread(f'{image_dir}/{image_filename}')
            depth = cv2.imread(f'{touch_depth_dir}/{depth_filename}', cv2.IMREAD_ANYDEPTH) / 1000
            var = cv2.imread(f'{touch_var_dir}/{depth_filename}', cv2.IMREAD_ANYDEPTH) / 1000
            
            # take depths with var leq than 0.7
            # depth[var > 0.7] = 0
            
            # resize everything to dim of image
            
            if image.shape[0] != depth.shape[0]:
                depth = cv2.resize(depth, (image.shape[0], image.shape[1]))
                var = cv2.resize(var, (image.shape[1], image.shape[0]))
            
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            camera_extrinsics = camera_transformations[image_filename.split('.')[0]]
            
            image = cv2.imread(f'{image_dir}/{image_filename}')
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            print(camera_intrinsics)
            points, colors = get_point_cloud_from_depth_and_color(depth, image, camera_intrinsics, camera_extrinsics)
            if limited_points_xyz is None:
                limited_points_xyz = points
                limited_points_rgb = colors
            else:
                limited_points_xyz = np.concatenate((limited_points_xyz, points))
                limited_points_rgb = np.concatenate((limited_points_rgb, colors))
            print('Got point cloud for', image_filename)
            
    # take 100 percent of the points
    percentage = percent_take
    
    total_indices = len(limited_points_xyz)
    num_indices_to_select = int(total_indices * (percentage / 100))

    # Generate a list of all indices
    all_indices = list(range(total_indices))

    # Use random.sample to randomly select indices
    selected_indices = random.sample(all_indices, num_indices_to_select)
    
    limited_points_rgb = limited_points_rgb[selected_indices]
    limited_points_xyz = limited_points_xyz[selected_indices]
    
    if viz:
        open3d_point_cloud(limited_points_xyz, limited_points_rgb)
    return limited_points_xyz, limited_points_rgb * 255.0
    
    
def get_train_eval_split_fraction(image_filenames, train_split_fraction):
    """
    Get the train/eval split fraction based on the number of images and the train split fraction.

    Args:
        image_filenames: list of image filenames
        train_split_fraction: fraction of images to use for training
    """

    # filter image_filenames and poses based on train/eval split percentage
    num_images = len(image_filenames)
    num_train_images = math.ceil(num_images * train_split_fraction)
    num_eval_images = num_images - num_train_images
    i_all = np.arange(num_images)
    i_train = np.linspace(
        0, num_images - 1, num_train_images+1, dtype=int
    )  # equally spaced training images starting and ending at 0 and num_images-1
    # remove last value from i_train
    i_train = i_train[:-1]
    i_eval = np.setdiff1d(i_all, i_train)  # eval images are the remaining images
    assert len(i_eval) == num_eval_images
    
    print("Train images indices: ", i_train)

    return i_train, i_eval
    
    
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Get indices ')
    
    parser.add_argument('--root_dir', type=str, required=True, help='Root dir.')
    parser.add_argument('--image_dir', type=str, required=True, help='Directory for images.')
    parser.add_argument('--touch_depth_dir', type=str, required=True, help='Directory for touch depths.')
    parser.add_argument('--touch_var_dir', type=str, required=True, help='Directory for touch var.')
    
    parser.add_argument('--transform_json_path', type=str, required=True, help='Path to transforms.json.')
    
    parser.add_argument('--train_split', type=float, required=True, help='Train split.')
    
    parser.add_argument('--percent_take', type=float, default=100, help='Percent of touch points to take.')
    
    parser.add_argument('--viz', action='store_true', help='Whether or not to viz.')
    
    
    args = parser.parse_args()
    
    root_dir = args.root_dir
    image_dir = os.path.join(root_dir, args.image_dir)
    touch_depth_dir = os.path.join(root_dir, args.touch_depth_dir)
    touch_var_dir = os.path.join(root_dir, args.touch_var_dir)
    transform_json_path = os.path.join(root_dir, args.transform_json_path)
    
    percent_take = args.percent_take
    
    train_split = args.train_split
    
    viz = args.viz
    
    i_train, i_eval = get_train_eval_split_fraction(os.listdir(image_dir), train_split)
    
    transforms, camera_instrinsics = transforms_utils.read_nerfstudio_transform_positions(transform_json_path, return_full_transforms=True)
    
    # transforms = colmap_utils_transforms.read_blender_transform_positions('transforms_train_full.json', return_full_transforms=True)
    
    points, colors = get_all_point_clouds(image_dir=image_dir, touch_depth_dir=touch_depth_dir, 
                                          touch_var_dir=touch_var_dir, camera_transformations=transforms, 
                                          camera_intrinsics=camera_instrinsics, i_take=i_train, percent_take=percent_take, viz=viz)
    
    # save points and colors to .npy files
    np.save(f'{root_dir}/points_touch.npy', points)
    np.save(f'{root_dir}/points_colors.npy', colors)