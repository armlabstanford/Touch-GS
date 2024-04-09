# Stanford University, ARMLab 2023
# Touch-GS

import cv2
import numpy as np

from scipy.ndimage import distance_transform_edt, sobel, binary_dilation, gaussian_filter

def compute_uncertainty_map_with_edges(dense_depth, sparse_depth,edge_weight=1.0, distance_uncertainty_weight=0.1, proximity_weight=0.5, dilation_size=1, depth_difference_weight=1.0):
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
    distance_uncertainty = (dense_depth ** 1) * distance_uncertainty_weight

    # Create a mask from sparse depth (1 where depth is available, 0 otherwise)
    sparse_mask = sparse_depth > 0

    # Distance transform for proximity-based uncertainty (normalized)
    proximity_uncertainty = distance_transform_edt(~sparse_mask) / np.max(distance_transform_edt(~sparse_mask))
    proximity_uncertainty *= proximity_weight
    
    proximity_uncertainty= cv2.blur(proximity_uncertainty, (250, 250))
    
    proximity_uncertainty = cv2.medianBlur(proximity_uncertainty.astype('float32'), 5)
    
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
    # uncertainty_map /= np.max(uncertainty_map)

    return uncertainty_map