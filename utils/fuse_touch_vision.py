# Stanford University, ARMLab 2023
# Touch-GS

# functions to fuse vision and touch

import os

import numpy as np
import cv2
import matplotlib.pyplot as plt 

import create_uncertainty_from_depth as cu

import argparse

from scipy.optimize import minimize


def create_sparse_depth_map(dense_depth_map, keep_percentage=0.01):
    # Create a mask with the same shape as the dense depth map, initially all False
    mask = np.zeros(dense_depth_map.shape, dtype=bool)
    
    # Calculate the total number of pixels to keep
    total_pixels = dense_depth_map.size
    pixels_to_keep = int(total_pixels * keep_percentage)
    
    # Randomly choose indices to keep
    # np.arange creates an array of indices [0, total_pixels - 1]
    # np.random.choice selects 'pixels_to_keep' indices without replacement
    selected_indices = np.random.choice(total_pixels, pixels_to_keep, replace=False)
    
    # Convert the 1D indices to 2D indices and set those positions in the mask to True
    mask.flat[selected_indices] = True
    
    # Create the sparse depth map: keep depth values at selected indices, set others to 0
    sparse_depth_map = np.zeros_like(dense_depth_map)
    sparse_depth_map[mask] = dense_depth_map[mask]
    
    return sparse_depth_map

def compute_scale_and_offset_best(sparse_depth, dense_depth, weights=None, scale_bounds=(0, None), offset_bounds=(0, None)):
    """
    Compute the non-negative scale factor and offset between sparse and dense depth maps using constrained optimization.

    :param sparse_depth: Sparse depth map (2D numpy array).
    :param dense_depth: Dense depth map (2D numpy array, same size).
    :param weights: Optional weights for each pixel (2D numpy array, same size).
    :return: scale_factor, offset (both non-negative)
    """
    # Mask to consider only the non-zero elements of the sparse depth map
    mask = sparse_depth > 0

    # Flattening the arrays and applying the mask
    sparse_depth_flat = sparse_depth[mask].flatten()
    dense_depth_flat = dense_depth[mask].flatten()
    
    # Define the optimization objective function
    def objective(params, dense_depth, sparse_depth):
        scale, offset = params
        prediction = scale * dense_depth + offset
        return np.sum((prediction - sparse_depth) ** 2)
    
    # Initial guess for scale and offset
    initial_guess = [1, 0]

    # Bounds to ensure scale and offset are non-negative
    bounds = [scale_bounds, offset_bounds]  # (min, max) pairs for scale and offset, None means unbounded

    # Perform the optimization
    result = minimize(objective, initial_guess, args=(dense_depth_flat, sparse_depth_flat), bounds=bounds)

    scale_factor, offset = result.x  # Extract optimized scale and offset

    return scale_factor, offset

def fuse_depth_maps_with_uncertainty(touch_depth_map, vision_depth_map, touch_uncertainty_map, vision_uncertainty_map, viz=False):
    # --- fuse depth uncertainties ---
    assert touch_depth_map.shape == vision_depth_map.shape, "Depth maps must have the same shape."
    
    # Plot each map with imshow
    if viz:
        _, axs = plt.subplots(2, 2, figsize=(10, 8))
        axs[0, 0].imshow(touch_depth_map, cmap='viridis')
        axs[0, 0].set_title('Touch Depth Map')

        axs[0, 1].imshow(vision_depth_map, cmap='viridis')
        axs[0, 1].set_title('Vision Depth Map')

        axs[1, 0].imshow(touch_uncertainty_map, cmap='hot')
        axs[1, 0].set_title('Touch Uncertainty Map')

        axs[1, 1].imshow(vision_uncertainty_map, cmap='hot')
        axs[1, 1].set_title('Vision Uncertainty Map')

        # Adjust layout to prevent overlap
        plt.tight_layout()

        # Show the plots
        plt.show()
        
        # save each of these plotted images
        plt.imsave('touch_depth.png', touch_depth_map, cmap='viridis')
        plt.imsave('vision_depth.png', vision_depth_map, cmap='viridis')
        plt.imsave('touch_uncertainty.png', touch_uncertainty_map, cmap='hot')
        plt.imsave('vision_uncertainty.png', vision_uncertainty_map, cmap='hot')
    
    fused_depth_uncertainty_map = np.copy(vision_uncertainty_map)
    
    mask = touch_uncertainty_map > 0
    
    uncertainty_touch_valid = touch_uncertainty_map
    uncertainty_vision_valid = vision_uncertainty_map
    
    eps = np.finfo(float).eps
    
    uncertainty_vision_valid_reciprocal = 1 / uncertainty_vision_valid
    uncertainty_touch_valid_reciprocal = 1 / uncertainty_touch_valid
    
    # set inf values to 0
    uncertainty_touch_valid_reciprocal[np.isinf(uncertainty_touch_valid_reciprocal)] = 0
    uncertainty_vision_valid_reciprocal[np.isinf(uncertainty_vision_valid_reciprocal)] = 0
    
    
    fused_depth_uncertainty_map = 1 / (uncertainty_touch_valid_reciprocal + uncertainty_vision_valid_reciprocal)
    
    fused_depth_uncertainty_map[np.isinf(fused_depth_uncertainty_map)] = 0
    
    # mask out invalid values
    # fused_depth_uncertainty_map = fused_depth_uncertainty_map * mask
    
    
    sigma = fused_depth_uncertainty_map
    
    # ---------------- fuse depth maps ----------------
    # Initialize the fused depth map as a copy of the dense depth map
    fused_depth_map = np.copy(vision_depth_map)
    
    mu_touch_valid = touch_depth_map * mask
    mu_vision_valid = vision_depth_map
    
    
    mu_touch_var_weighted = mu_touch_valid / uncertainty_touch_valid
    mu_touch_var_weighted[np.isnan(mu_touch_var_weighted)] = 0
    
    mu_vision_var_weighted = mu_vision_valid / uncertainty_vision_valid
    mu_vision_var_weighted[np.isnan(mu_vision_var_weighted)] = 0
    
    fused_depth_map = sigma * (mu_touch_var_weighted + mu_vision_var_weighted)
    
    
    if viz:
        # show the fused depth map and uncertainty map
        _, axs = plt.subplots(1, 2, figsize=(10, 8))
        axs[0].imshow(fused_depth_map, cmap='viridis')
        axs[0].set_title('Fused Depth Map')
        
        axs[1].imshow(fused_depth_uncertainty_map , cmap='magma')
        axs[1].set_title('Fused Uncertainty Map')
        
        # Adjust layout to prevent overlap
        plt.tight_layout()

        # Show the plots
        plt.show()
        
        # plt.imsave('fused_depth.png', fused_depth_map, cmap='viridis')
        # plt.imsave('fused_uncertainty.png', fused_depth_uncertainty_map, cmap='magma')
        
        # normalized_depth = (fused_depth_map - np.min(fused_depth_map)) / (np.max(fused_depth_map) - np.min(fused_depth_map))

        # # Apply a colormap (e.g., 'jet', 'viridis', 'plasma', etc.)
        # colored_image = plt.get_cmap('viridis')(normalized_depth)[:, :, :3]  # Discard the alpha channel

        # # Save the color-mapped depth image
        # plt.imsave('fused_depth.png', colored_image)
        
        # normalized_uncertainty = (fused_depth_uncertainty_map - np.min(fused_depth_uncertainty_map)) / (np.max(fused_depth_uncertainty_map) - np.min(fused_depth_uncertainty_map))

        # # Apply a colormap (e.g., 'jet', 'viridis', 'plasma', etc.)
        # colored_uncertainty = plt.get_cmap('inferno')(normalized_uncertainty)[:, :, :3]  # Discard the alpha channel

        # # Save the color-mapped depth image
        # plt.imsave('fused_uncertainty.png', colored_uncertainty)
        
        # # save vision
        # normalized_depth = (vision_depth_map - np.min(vision_depth_map)) / (np.max(vision_depth_map) - np.min(vision_depth_map))

        # # Apply a colormap (e.g., 'jet', 'viridis', 'plasma', etc.)
        # colored_image = plt.get_cmap('viridis')(normalized_depth)[:, :, :3]  # Discard the alpha channel

        # # Save the color-mapped depth image
        # plt.imsave('zoe_depth.png', colored_image)
        
        # normalized_uncertainty = (vision_uncertainty_map - np.min(vision_uncertainty_map)) / (np.max(vision_uncertainty_map) - np.min(vision_uncertainty_map))

        # # Apply a colormap (e.g., 'jet', 'viridis', 'plasma', etc.)
        # colored_uncertainty = plt.get_cmap('hot')(normalized_uncertainty)[:, :, :3]  # Discard the alpha channel

        # # Save the color-mapped depth image
        # plt.imsave('uncertainty.png', colored_uncertainty)
    
    return fused_depth_map, fused_depth_uncertainty_map

def fuse_depth_maps(sparse_depth_map, dense_depth_map):
    """
    Fuse two depth maps: a sparse one and a dense one.
    
    Parameters:
    - sparse_depth_map: NumPy array representing the sparse depth map.
    - dense_depth_map: NumPy array representing the dense depth map.
    
    Returns:
    - fused_depth_map: NumPy array representing the fused depth map.
    """
    # Ensure the input maps have the same shape
    assert sparse_depth_map.shape == dense_depth_map.shape, "Depth maps must have the same shape."
    
    # Initialize the fused depth map as a copy of the dense depth map
    fused_depth_map = np.copy(dense_depth_map)
    
    # Identify valid (non-zero/non-null) depth values in the sparse depth map
    valid_sparse_indices = np.where(sparse_depth_map > 0)  # Assuming 0 indicates invalid/missing depth
    
    # Overwrite values in the fused map with valid values from the sparse map
    fused_depth_map[valid_sparse_indices] = sparse_depth_map[valid_sparse_indices]
    
    return fused_depth_map

def construct_dirs(output_dir, fused_output_dir):
    os.makedirs(output_dir, exist_ok=True)
    os.makedirs(output_dir + "_baseline", exist_ok=True)
    
    os.makedirs(fused_output_dir, exist_ok=True)
    os.makedirs(f'{fused_output_dir}_uncertainty', exist_ok=True)
    
def visualize_all(touch_depth_image, vision_depth_image, grounded_depth_image):
    fig, axs = plt.subplots(1, 3, figsize=(10, 8))
    axs[0].imshow(touch_depth_image, cmap='viridis')
    axs[0].set_title('Touch Depth Map')

    axs[1].imshow(vision_depth_image, cmap='viridis')
    axs[1].set_title('Vision Depth Map')

    axs[2].imshow(grounded_depth_image, cmap='viridis')
    axs[2].set_title('Grounded Depth Map')

    # Adjust layout to prevent overlap
    plt.tight_layout()

    # Show the plots
    plt.show()
 
def visualize_two(ds_gs_visual_depth, full_fused_depth_map):
    fig, axs = plt.subplots(1, 2, figsize=(10, 8))
    axs[0].imshow(ds_gs_visual_depth, cmap='viridis')
    axs[0].set_title('DS-GS Depth Map')

    axs[1].imshow(full_fused_depth_map, cmap='viridis')
    axs[1].set_title('Fused Depth Map')
    
    plt.tight_layout()
    plt.show()
    
def get_depths_by_idx(img_number, grounded_depth_dir, touch_depth_dir, vision_depth_dir, touch_var_dir, idx, grounded_depth, touch_depths, vision_depths):
    grounded_depth_image = cv2.imread(f'{grounded_depth_dir}/{grounded_depth}', cv2.IMREAD_ANYDEPTH)
    touch_depth_image = cv2.imread(f'{touch_depth_dir}/{touch_depths[idx]}', cv2.IMREAD_ANYDEPTH)
    vision_depth_image = cv2.imread(f'{vision_depth_dir}/{vision_depths[idx]}', cv2.IMREAD_ANYDEPTH)
    touch_uncertainty_image = cv2.imread(f'{touch_var_dir}/{img_number}.png', cv2.IMREAD_ANYDEPTH)
    
    vision_depth_image = vision_depth_image / 1000
    touch_depth_image = touch_depth_image / 1000
    grounded_depth_image = grounded_depth_image / 1000
    
    
    touch_uncertainty_image = touch_uncertainty_image / 1000
    
    grounded_depth_image = cv2.resize(grounded_depth_image, (1280, 720), interpolation=cv2.INTER_LINEAR)
    
    return (grounded_depth_image, touch_depth_image, vision_depth_image, touch_uncertainty_image)

def align_vision_depth(grounded_depth_image, touch_depth_image, vision_depth_image):
    # first align (Depth Supervised Gaussian Splatting)
    scale, offset = compute_scale_and_offset_best(grounded_depth_image, vision_depth_image, None, (0, None), (None, None))
    
    # compute a new vision depth map
    vision_depth_image = (scale * vision_depth_image) + offset
    
    # this is the DS-GS visual depth map for baseline
    ds_gs_visual_depth = np.copy(vision_depth_image)
    
    # filter out diffs greater than 3 meters between touch and vision before the second stage of alignment
    diff = vision_depth_image - touch_depth_image
    diff[diff > 3] = 0
    touch_depth_image_to_align = touch_depth_image * (diff > 0)
    mask = touch_depth_image_to_align > 0
    
    # second align (our method Touch-GS)
    scale, offset = compute_scale_and_offset_best(touch_depth_image_to_align, vision_depth_image, None, (1, 1), (None, None))
    
    # only update the vision depth image with the aligned touch depth image mask
    vision_depth_image[mask] = vision_depth_image[mask] + offset
    # vision_depth_image = vision_depth_image + offset
    vision_depth_image = np.clip(vision_depth_image, a_min=0, a_max=None)
    
    # create uncertainty maps
    # we support including distance uncertainty from sparse points and proximity weight uncertainty, but set it to 0 in our work
    vision_uncertainty = cu.compute_uncertainty_map_with_edges(vision_depth_image, grounded_depth_image, edge_weight=0, distance_uncertainty_weight=0.05, proximity_weight=0.0, depth_difference_weight=0.0, dilation_size=5)
    # clip uncertainty to reasonble values [0, 10]
    vision_uncertainty = np.clip(vision_uncertainty, a_min=0, a_max=10)
    vision_uncertainty = vision_uncertainty + 5
    
    return ds_gs_visual_depth, vision_depth_image, vision_uncertainty
 
def fuse_vision_and_touch(grounded_depth_dir, touch_depth_dir, vision_depth_dir, viz=False, output_dir='vision_aligned', fused_output_dir='fused', touch_var_dir='touch_var', use_uncertainty=True):
    """Fuse grounded depth with touch depth data.

    Args:
        `grounded_depth_dir (_type_)`: _description_
        
        `touch_depth_dir (_type_)`: _description_
        
        `vision_depth_dir (_type_)`: _description_
        `viz (bool, optional)`: _description_. Defaults to False.
        
        `output_dir (str, optional): _description_. Defaults to 'vision_aligned'.
        
        `fused_output_dir (str, optional)`: _description_. Defaults to 'fused'.
        
        `touch_var_dir (str, optional)`: _description_. Defaults to 'touch_var'.
        
    """
    construct_dirs(output_dir, fused_output_dir)
    
    grounded_depths = sorted(os.listdir(grounded_depth_dir))
    touch_depths = sorted(os.listdir(touch_depth_dir))
    vision_depths = sorted(os.listdir(vision_depth_dir))

    for idx, grounded_depth in enumerate(grounded_depths):
        img_number = touch_depths[idx].split('/')[-1][:-4]
        
        grounded_depth_image, touch_depth_image, vision_depth_image, touch_uncertainty_image = get_depths_by_idx(img_number, grounded_depth_dir, touch_depth_dir, vision_depth_dir, touch_var_dir, idx, grounded_depth, touch_depths, vision_depths)
        
        # show all three depth maps in one plot
        if viz:
            visualize_all(touch_depth_image, vision_depth_image, grounded_depth_image)
            
        # TODO: sparsify the grounded realsense depth map, but not for sparse points in the blender scene
        grounded_depth_image = create_sparse_depth_map(grounded_depth_image, keep_percentage=0.01)
        
        ds_gs_visual_depth, vision_depth_image, vision_uncertainty = align_vision_depth(grounded_depth_image, touch_depth_image, vision_depth_image)
        
        # fuse depth maps
        if use_uncertainty:
            full_fused_depth_map, fused_uncertainty = fuse_depth_maps_with_uncertainty(touch_depth_image, vision_depth_image, touch_uncertainty_image, vision_uncertainty, viz=viz)
            full_fused_depth_map = np.clip(full_fused_depth_map, a_min=0, a_max=None)
            fused_uncertainty = np.clip(fused_uncertainty, a_min=0, a_max=10)
        else:
            full_fused_depth_map = fuse_depth_maps(touch_depth_image, vision_depth_image)
            full_fused_depth_map = np.clip(full_fused_depth_map, a_min=0, a_max=None)
        
        if viz:
            # show vision depth map and full fused in one plot
            visualize_two(ds_gs_visual_depth, full_fused_depth_map)
        
        save(output_dir, fused_output_dir, img_number, vision_depth_image, ds_gs_visual_depth, full_fused_depth_map, fused_uncertainty)
 
def save(output_dir, fused_output_dir, img_number, vision_depth_image, ds_gs_visual_depth, full_fused_depth_map, fused_uncertainty):
    vision_depth_np = (vision_depth_image * 1000).astype(np.uint16)
    ds_gs_visual_depth = (ds_gs_visual_depth * 1000).astype(np.uint16)
    fused_depth_np = (full_fused_depth_map * 1000).astype(np.uint16)
    fused_uncertainty_np = (fused_uncertainty * 1000).astype(np.uint16)
    
    cv2.imwrite(f'{output_dir}/{img_number}.png', vision_depth_np)
    print(f'Saved vision depth image: {output_dir}/{img_number}.png')
    
    cv2.imwrite(f'{output_dir}_baseline/{img_number}.png', ds_gs_visual_depth)
    print(f'Saved vision depth image: {output_dir}_baseline/{img_number}.png')
    
    cv2.imwrite(f'{fused_output_dir}/{img_number}.png', fused_depth_np)
    print(f'Saved fused depth image: {fused_output_dir}/{img_number}.png')
    
    cv2.imwrite(f'{fused_output_dir}_uncertainty/{img_number}.png', fused_uncertainty_np)
    print(f'Saved fused uncertainty image: {fused_output_dir}_uncertainty/{img_number}.png')
            

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Fuse grounded depth with touch depth data.')
    
    parser.add_argument('--root_dir', type=str, required=True, help='Root dir.')
    
    parser.add_argument('--aligning_depths', type=str, required=False, help='Path to the depths to align the ZOE depth result.')
    parser.add_argument('--zoe_depth_path', type=str, required=True, help='Path to the Zoe depth data files.')
    
    parser.add_argument('--touch_depth', type=str, required=True, help='Path to the touch depth data file.')
    parser.add_argument('--touch_var', type=str, required=True, help='Path to the touch var data file.')
    
    parser.add_argument('--viz', action='store_true', help='Whether or not to viz.')
    parser.add_argument('--use_uncertainty', action='store_true', help='Whether or not to viz.')
    
        
    parser.add_argument('--vision_output_dir', type=str, required=True, help='Output directory for vision outputs.')
    parser.add_argument('--fused_output_dir', type=str, required=True, help='Output directory for fused data.')
    
    
    args = parser.parse_args()
    
    root_dir = args.root_dir
    
    use_uncertainty = args.use_uncertainty
    
    full_aligning_depths = os.path.join(root_dir, args.aligning_depths)
    full_touch_depth = os.path.join(root_dir, args.touch_depth)
    full_zoe_depth = os.path.join(root_dir, args.zoe_depth_path)
    full_vision_output_dir = os.path.join(root_dir, args.vision_output_dir)
    full_fused_output_dir = os.path.join(root_dir, args.fused_output_dir)
    full_touch_var = os.path.join(root_dir, args.touch_var)
    
    fuse_vision_and_touch(full_aligning_depths, full_touch_depth, full_zoe_depth, args.viz, full_vision_output_dir, full_fused_output_dir, full_touch_var, use_uncertainty)