# Stanford University, ARMLab 2023
# Touch-GS

import numpy as np
import os
from PIL import Image

import argparse

import cv2


def convert_intrinsics(img, old_intrinsics = (360, 360, 243, 137.8), new_intrinsics = (1297, 1304, 620.91, 238.28), new_size=(1280, 720)):
    """
    Convert a set of images to a different set of camera intrinsics.
    Parameters:
    - images: List of input images.
    - old_intrinsics: Tuple (fx, fy, cx, cy) of the old camera intrinsics.
    - new_intrinsics: Tuple (fx, fy, cx, cy) of the new camera intrinsics.
    - new_size: Tuple (width, height) defining the size of the output images.
    Returns:
    - List of images converted to the new camera intrinsics.
    """
    old_fx, old_fy, old_cx, old_cy = old_intrinsics
    new_fx, new_fy, new_cx, new_cy = new_intrinsics
    width, height = new_size
    
    # Constructing the old and new intrinsics matrices
    K_old = np.array([[old_fx, 0, old_cx], [0, old_fy, old_cy], [0, 0, 1]])
    K_new = np.array([[new_fx, 0, new_cx], [0, new_fy, new_cy], [0, 0, 1]])
    # Compute the inverse of the new intrinsics matrix for remapping
    K_new_inv = np.linalg.inv(K_new)
    
    # Construct a grid of points representing the new image coordinates
    x, y = np.meshgrid(np.arange(width), np.arange(height))
    homogenous_coords = np.stack([x.ravel(), y.ravel(), np.ones_like(x).ravel()], axis=-1).T
    
    # Convert to the old image coordinates
    old_coords = K_old @ K_new_inv @ homogenous_coords
    old_coords /= old_coords[2, :]  # Normalize to make homogeneous
    
    # Reshape for remapping
    map_x = old_coords[0, :].reshape(height, width).astype(np.float32)
    map_y = old_coords[1, :].reshape(height, width).astype(np.float32)
    
    # Remap the image to the new intrinsics
    converted_img = cv2.remap(img, map_x, map_y, interpolation=cv2.INTER_LINEAR)
    return converted_img

def convert(image, fx_original, fy_original, cx_original, cy_original, fx_new, fy_new, cx_new, cy_new):
    """
    Convert an image (RGB or depth) from one camera's perspective to another camera's perspective.

    Args:
        image (np.array): The image to be converted.
        fx_original (float): The focal length in the x-direction of the original camera.
        fy_original (float): The focal length in the y-direction of the original camera.
        cx_original (float): The x-coordinate of the principal point of the original camera.
        cy_original (float): The y-coordinate of the principal point of the original camera.
        fx_new (float): The focal length in the x-direction of the new camera.
        fy_new (float): The focal length in the y-direction of the new camera.
        cx_new (float): The x-coordinate of the principal point of the new camera.

    Returns:
        warped_image (np.array): The image warped to the new camera's perspective.
    """

    # Original camera intrinsic parameters
    K_original = np.array([[fx_original, 0, cx_original],
                        [0, fy_original, cy_original],
                        [0, 0, 1]])

    # New camera intrinsic parameters
    K_new = np.array([[fx_new, 0, cx_new],
                    [0, fy_new, cy_new],
                    [0, 0, 1]])

    # Assuming no rotation or translation between the two cameras for simplicity,
    # the homography matrix can be approximated as the product of the new intrinsic matrix
    # and the inverse of the original intrinsic matrix
    H = np.dot(K_new, np.linalg.inv(K_original))

    # Warp the original image to the new camera's perspective
    height, width = image.shape[:2]
    warped_image = cv2.warpPerspective(image, H, (width, height))
    return warped_image

def read_depth_map(image_path):
    # Load the image
    depth_image = Image.open(image_path)
    
    # Convert the image to a numpy array to work with the pixel values
    depth_map = np.array(depth_image)
    
    depth_map = (depth_map / 255) * 3000 
    depth_meters = depth_map / 1000
    
    
    return depth_meters

def read_depth_map_v2(image_path):
    # Load the image
    # load npy file
    depth_image = np.load(image_path) / 1000
    
    return depth_image

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process realsense depth images.')
    parser.add_argument('--base_repo_path', type=str, required=True,
                        help='Path to the base repository')

    args = parser.parse_args()

    base_repo_path = args.base_repo_path
    
    print(f"Base repo path: {base_repo_path}")
    
    # create dir if it hasn't been create already
    save_dir = os.path.join(base_repo_path, 'realsense_depths')
    os.makedirs(save_dir, exist_ok=True)
    
    # create real sense depth dir
    realsense_depths_dir = os.path.join(base_repo_path, 'realsense_depth')
    realsense_depths = sorted(os.listdir(realsense_depths_dir))
    
    ##################################################################################
    
    for depth_file in realsense_depths:
        if ".npy" in depth_file:
            realsense_depth = read_depth_map_v2(os.path.join(realsense_depths_dir, depth_file))
            realsense_depth = convert_intrinsics(realsense_depth)
        
            depth_img_name = depth_file.split('.')[0]
            
            # Convert depth to suitable format and save as PNG
            realsense_depth = (realsense_depth * 1000).astype(np.uint16)
            cv2.imwrite(os.path.join(save_dir, f'{depth_img_name}.png'), realsense_depth)
            print('Wrote', os.path.join(save_dir, f'{depth_img_name}.png'))

