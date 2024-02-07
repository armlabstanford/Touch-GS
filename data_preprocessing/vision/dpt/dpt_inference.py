
from transformers import AutoImageProcessor, DPTFeatureExtractor, DPTForDepthEstimation

from PIL import Image
import requests
import matplotlib.pyplot as plt
import torch
import numpy as np

import cv2

import open3d as o3d


import os
from refine_depth import refine_dpt_depth
from interpolate_depth import interpolate_depth_image


def open_depths_in_folder(folder_path, depth_type='geometric'):
    # open depths in the depth_maps folder
    depths = {}
    for filename in os.listdir(folder_path):
        if depth_type in filename:
            file_path = os.path.join(folder_path, filename)

            with open(file_path, "rb") as fid:
                width, height, channels = np.genfromtxt(
                    fid, delimiter="&", max_rows=1, usecols=(0, 1, 2), dtype=int
                )
                fid.seek(0)
                num_delimiter = 0
                byte = fid.read(1)
                while True:
                    if byte == b"&":
                        num_delimiter += 1
                        if num_delimiter >= 3:
                            break
                    byte = fid.read(1)
                array = np.fromfile(fid, np.float32)
            array = array.reshape((width, height, channels), order="F")
            depth =  np.transpose(array, (1, 0, 2)).squeeze()
            original_image_name = filename.split('.')[0]
            depths[original_image_name] = depth
    return depths

def open_images_in_folder(folder_path):
    # List to store opened images
    images = {}

    # Iterate over the files in the folder
    for filename in os.listdir(folder_path):
        if filename.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.gif')):
            # Construct the full file path
            file_path = os.path.join(folder_path, filename)
            
            try:
                # Open the image and append to the list
                img = Image.open(file_path)
                original_image_name = filename.split('.')[0]

                images[original_image_name] = img
                print(f'Opened image: {filename}')
            except IOError:
                print(f'Failed to open image: {filename}')

    return images


def refine_depth(dpt_depth, colmap_depth, method='scale'):
    height, width = dpt_depth.shape
    ratios = []
    for y in range(height):
        for x in range(width):
            # Access the pixel value
            d_d = dpt_depth[y, x]
            d_c = colmap_depth[y, x]
            ratio = d_d / d_c
            if ratio != np.inf and ratio != np.nan:
                ratios.append(ratio)
    scale_factor = np.median(ratios)
    if method == 'scale':
        result =  dpt_depth / scale_factor
        return result, scale_factor
    elif method == 'interpolate':
        result = interpolate_depth_image(colmap_depth, method='linear')
        return result, scale_factor
    else:
        # more proper refinement of depth
        dpt_depth /= scale_factor
        result = np.copy(dpt_depth)
        for y in range(height):

            for x in range(width):
                # Access the pixel value
                d_d = dpt_depth[y, x]
                d_c = colmap_depth[y, x]
                # print(d_c)
                if d_c == 0 or d_c == np.inf or d_c == np.nan:
                    result[y, x] = d_d
                else:
                    result[y, x] = d_c
                    
                # print(f"Depth at pixel ({x}, {y}): {depth_value}")
        return result, scale_factor

def create_point_cloud_from_depth(depth_image, fx, fy, cx, cy, depth_scale=1):
    # Load the depth image
    depth = depth_image / depth_scale

    # Create a grid of coordinates corresponding to the depth image pixels
    height, width = depth.shape
    x, y = np.meshgrid(np.arange(width), np.arange(height))

    # Back-project the 2D pixel coordinates to 3D points
    z = depth
    x = (x - cx) * z / fx
    y = (y - cy) * z / fy

    # Stack the coordinates into a point cloud
    points = np.stack((x, y, z), axis=-1).reshape(-1, 3)

    # Remove invalid points (where depth is 0)
    points = points[depth.flatten() > 0]

    # Create Open3D point cloud
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    return pcd

# Visualize the point cloud
def visualize_point_cloud(pcd):
    o3d.visualization.draw_geometries([pcd])
    
    
def output_depth_images(depth_image, depth_path, depth_scale_to_integer_factor=1000):
    depth_img = (depth_scale_to_integer_factor * depth_image).astype(np.uint16)

    cv2.imwrite(depth_path, depth_img)  # type: ignore
    print(depth_path, "saved")


def get_feature_extractor_and_model():
    feature_extractor = DPTFeatureExtractor.from_pretrained("Intel/dpt-large")
    model = DPTForDepthEstimation.from_pretrained("Intel/dpt-large")
    return feature_extractor, model


if __name__ == '__main__':

    feature_extractor, model = get_feature_extractor_and_model()

    images = open_images_in_folder('images')
    colmap_depths = open_depths_in_folder('depth_maps')

    for img_name in images:
        # iterate through every image and get 
        image = images[img_name]
        colmap_depth = colmap_depths[img_name]
        
        colmap_raw_depth = colmap_depth
        
        
        print(img_name)
        pixel_values = feature_extractor(image, return_tensors="pt").pixel_values

        with torch.no_grad():
            outputs = model(pixel_values)
            predicted_depth = outputs.predicted_depth


        # interpolate to original size
        prediction = torch.nn.functional.interpolate(
                            predicted_depth.unsqueeze(1),
                            size=colmap_depth.shape,
                            mode="bicubic",
                            align_corners=False,
                    ).squeeze()
        output = prediction.cpu().numpy()
        
        output, scale_factor = refine_depth(output, colmap_depth, method='scale')
        
        # output = refine_dpt_depth(output, colmap_depth, scale_factor=1)
        output_depth_images(output, f"dpt_depth/{img_name}.png")


        # formatted = (output * 255 / np.max(output)).astype('uint8')
        # depth = Image.fromarray(formatted)
        

        # colmap_depth = (colmap_depth - np.min(colmap_depth)) / (np.max(colmap_depth) - np.min(colmap_depth))
        # colmap_depth_formatted = (colmap_depth * 255).astype('uint8')

        # colmap_image_depth = Image.fromarray(colmap_depth_formatted)
        # # plt.imshow(depth)
        # # plt.imshow(colmap_image_depth)
        # fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(10, 5))

        # # Display the first image
        # ax1.imshow(depth)
        # ax1.set_title('DPT Dense Depth')
        # ax1.axis('off')  # Turn off axis

        # # Display the second image
        # ax2.imshow(colmap_image_depth)
        # ax2.set_title('Colmap Dense Depth')
        # ax2.axis('off')  # Turn off axis

        # ax3.imshow(image)
        # ax3.set_title('Image')
        # ax3.axis('off')  # Turn off axis

        # # Show the plot
        # plt.show()
