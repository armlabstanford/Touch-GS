# Stanford University, ARMLab 2023
# Touch-GS

import os
import glob
from matplotlib import pyplot as plt
import numpy as np
import argparse
import cv2

SCALE_FACTOR = 1000

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process realsense depth images.')
    parser.add_argument('--base_repo_path', type=str, required=True, help='Path to the base repository')
    args = parser.parse_args()
    
    base_repo_path = args.base_repo_path
    print(f"Base repo path: {base_repo_path}")
    
    # Directories are now relative to base_repo_path
    output_depth_path = os.path.join(base_repo_path, 'touch_depth')
    output_uncertainty_path = os.path.join(base_repo_path, 'touch_var')
    
    input_depth_path = os.path.join(base_repo_path, 'gpis_depth')
    input_variance_path = os.path.join(base_repo_path, 'gpis_var')
    
    old_imgs_path = os.path.join(base_repo_path, 'imgs')
    
        
    # Create output directories if they don't exist
    os.makedirs(output_depth_path, exist_ok=True)
    os.makedirs(output_uncertainty_path, exist_ok=True)
    
    old_imgs_list = sorted(glob.glob(os.path.join(old_imgs_path, '*.png')))
    depth_np_paths = sorted(glob.glob(os.path.join(input_depth_path, '*.npy')))
    
    for idx, old_img_path in enumerate(old_imgs_list):
        img_number = os.path.basename(old_img_path)[:-4]  # Improved compatibility across OS
        
        depth_path = os.path.join(input_depth_path, f'Image{img_number}.npy')
        depth_np = np.load(depth_path)
        
        uncertainty_path = os.path.join(input_variance_path, f'Image{img_number}.npy')
        uncertainty = np.load(uncertainty_path)
        
        # Handling NaN values
        uncertainty[np.isnan(uncertainty)] = 0
        depth_np[np.isnan(depth_np)] = 0
        
        # Convert to np.uint16 and save
        depth_np = (depth_np * SCALE_FACTOR).astype(np.uint16)
        uncertainty = (uncertainty * SCALE_FACTOR).astype(np.uint16)
        
        cv2.imwrite(os.path.join(output_depth_path, f'{img_number}.png'), depth_np)
        cv2.imwrite(os.path.join(output_uncertainty_path, f'{img_number}.png'), uncertainty)
        
        print("Wrote", os.path.join(output_depth_path, f'{img_number}.png'))
        print("Wrote", os.path.join(output_uncertainty_path, f'{img_number}.png'))
        
        # Optionally, show the image
        # uncomment to see the touch depth
        # touch_depth_image = cv2.imread(os.path.join(output_depth_path, f'{img_number}.png'), cv2.IMREAD_ANYDEPTH) / 1000
        # plt.imshow(touch_depth_image, cmap='viridis')
        # plt.show()
