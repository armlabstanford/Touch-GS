import os


import cv2
import matplotlib.pyplot as plt
import numpy as np

from scipy.spatial import KDTree
from scipy.ndimage import distance_transform_edt, sobel, binary_dilation, gaussian_filter

def get_colmap_depth(root_dir, idx, scale=1000):
    depth_paths = os.listdir(root_dir)
    depth_paths_sorted = sorted(depth_paths)
    
    depth_valid_selected = depth_paths_sorted[idx]

    full_depth_image_path = os.path.join(root_dir, depth_valid_selected)
    print(full_depth_image_path)
    depth_image = cv2.imread(full_depth_image_path, cv2.IMREAD_UNCHANGED)/ scale
    depth_image = depth_image[:1099, :1799]
    return depth_image


def create_uncertainty_from_depth(depth, colmap_points, alpha=0.5, max_depth=100.0, distance_weight=1):
    # uncertainty is higher for points that are further away
    # uncertainty is lower for points that are closer
    # implement this as a function of depth
    uncertainty = (depth ** 2) * distance_weight
    
    return uncertainty


def compute_uncertainty_map_with_edges(dense_depth, sparse_depth,edge_weight=1.0, distance_uncertainty_weight=0.01, proximity_weight=0.5, dilation_size=1, depth_difference_weight=1.0):
    """
    Compute an uncertainty map from dense and sparse depth images,
    adding extra uncertainty to edges.

    :param dense_depth: Dense depth image as a 2D numpy array.
    :param sparse_depth: Sparse depth image as a 2D numpy array of the same size.
    :param max_depth: Maximum depth value to normalize distance-based uncertainty.
    :param alpha: Weight factor to balance distance and proximity contributions.
    :param edge_weight: Weight factor for uncertainty at edges.
    :return: Uncertainty map as a 2D numpy array.
    """
    # Normalize dense depth and invert for distance-based uncertainty
    distance_uncertainty = (dense_depth ** 2) * distance_uncertainty_weight

    # Create a mask from sparse depth (1 where depth is available, 0 otherwise)
    sparse_mask = sparse_depth > 0

    # Distance transform for proximity-based uncertainty (normalized)
    proximity_uncertainty = distance_transform_edt(~sparse_mask) / np.max(distance_transform_edt(~sparse_mask))
    proximity_uncertainty *= proximity_weight
    
    # sparse_influence = gaussian_filter(sparse_mask.astype(float), sigma=proximity_weight)
    # proximity_uncertainty= (1 - sparse_influence)

    # Edge detection using Sobel operator
    edge_x = sobel(dense_depth, axis=0)
    edge_y = sobel(dense_depth, axis=1)
    edge_magnitude = 20 * np.sqrt(edge_x**2 + edge_y**2)
    edge_uncertainty = (edge_magnitude / np.max(edge_magnitude)) * edge_weight
    
    edge_mask = edge_magnitude > np.percentile(edge_magnitude, 97)  # Threshold to identify significant edges

    # Dilate edge regions
    dilated_edge_mask = binary_dilation(edge_mask, iterations=dilation_size)
    edge_uncertainty = dilated_edge_mask.astype(float) * edge_weight
    
    
    depth_difference_uncertainty = np.abs(dense_depth - sparse_depth)
    depth_difference_uncertainty[~sparse_mask] = 0  # Ignore where sparse depth is not available
    # depth_difference_uncertainty /= np.max(depth_difference_uncertainty) # Normalize * 100
    depth_difference_uncertainty *= depth_difference_weight
    
    
    # Combine the uncertainties
    uncertainty_map = distance_uncertainty +  proximity_uncertainty + edge_uncertainty + depth_difference_uncertainty

    # Normalize the uncertainty map
    uncertainty_map /= np.max(uncertainty_map)

    return uncertainty_map


def create_uncertainty_map(depth_map, sparse_depth, distance_scale=0.01, sparse_scale=5):
    """
    Create an uncertainty map for a depth map.

    :param depth_map: Dense depth map as a 2D numpy array.
    :param sparse_depth: Sparse depth map as a 2D numpy array of the same size.
    :param distance_scale: Scaling factor for uncertainty based on depth.
    :param sparse_scale: Influence scale of sparse depth points on uncertainty.
    :return: Uncertainty map as a 2D numpy array.
    """
    # Step 1: Increase uncertainty with depth
    uncertainty_map = (depth_map ** 2) * distance_scale

    # Step 2: Decrease uncertainty near sparse depth points
    sparse_mask = sparse_depth > 0
    sparse_influence = gaussian_filter(sparse_mask.astype(float), sigma=sparse_scale)
    uncertainty_map *= (1 - sparse_influence)

    # Normalize the uncertainty map
    max_uncertainty = np.max(uncertainty_map)
    if max_uncertainty > 0:
        uncertainty_map /= max_uncertainty

    return uncertainty_map


def read_depth_image(path, scale=1000.0):
    depth = cv2.imread(path, cv2.IMREAD_ANYDEPTH)
    depth = depth / scale
    return depth



if __name__ == '__main__':
    depth_dir = 'dense_depth_imgs'
    depth_image_paths = sorted(os.listdir(depth_dir))
    
    colmap_depth_dir = 'colmap_depth'
    
    for idx, depth_image_paths in enumerate(depth_image_paths):
        
        colmap_depth = get_colmap_depth(colmap_depth_dir, idx, scale=1)
        
        depth_path = os.path.join(depth_dir, depth_image_paths)
        depth = read_depth_image(depth_path)
        
        # uncertainty = create_uncertainty_from_depth(depth, None)
        
        uncertainty = compute_uncertainty_map_with_edges(depth, colmap_depth, edge_weight=1.0, distance_uncertainty_weight=0.02, proximity_weight=3.0)
        
        # plot depth and uncertainty
        fig, axs = plt.subplots(1, 3)
        axs[0].imshow(depth, cmap='viridis', label='depth')
        axs[1].imshow(colmap_depth, cmap='viridis')
        axs[2].imshow(uncertainty, cmap='viridis')
        plt.show()
    
    # depth = read_depth_image(depth_path)
    
    # colmap_depth = get_colmap_depth('colmap_depth', 0)
    
    # uncertainty = create_uncertainty_from_depth(depth, None)
    
    # # uncertainty = create_uncertainty_map
    
    # plot depth and uncertainty
    
    # fig, axs = plt.subplots(1, 3)
    # axs[0].imshow(depth, cmap='viridis')
    # axs[1].imshow(colmap_depth, cmap='viridis')
    # axs[2].imshow(uncertainty, cmap='viridis')
    # plt.show()