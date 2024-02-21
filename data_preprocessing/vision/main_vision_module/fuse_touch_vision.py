from matplotlib import pyplot as plt
import numpy as np
import cv2

import os

def visualize_depth_images(depth_image1, depth_image2):
    # Load the images

    # Create a figure with two subplots
    fig, axes = plt.subplots(1, 2, figsize=(12, 6))

    # Display first depth image
    axes[0].imshow(depth_image1, cmap='viridis')
    axes[0].set_title('Depth Image 1')
    axes[0].axis('off')  # Hide axes ticks

    # Display second depth image
    axes[1].imshow(depth_image2, cmap='viridis')
    axes[1].set_title('Depth Image 2')
    axes[1].axis('off')  # Hide axes ticks

    # Show the plot
    plt.show()


class DepthFusion():
    def __init__(self, touch_depth_path, vision_depth_path, touch_depth_scale_factor=1, vision_depth_scale_factor=1000):
        self.touch_depths = sorted(os.listdir(touch_depth_path))
        self.vision_depths = sorted(os.listdir(vision_depth_path))
        
        self.touch_depth_path = touch_depth_path
        self.vision_depth_path = vision_depth_path
        
        self.touch_depth_scale_factor = touch_depth_scale_factor
        self.vision_depth_scale_factor = vision_depth_scale_factor
        
        self.touch_depth_images = []
        self.vision_depth_images = []
        
        self.read_files(visualize=True)
    
    def read_files(self, visualize=False):
        for idx, touch_depth_filename in enumerate(self.touch_depths):
            touch_depth_image = cv2.imread(os.path.join(self.touch_depth_path, touch_depth_filename), cv2.IMREAD_ANYDEPTH)
            
            
            touch_depth_image = touch_depth_image.astype(np.float64) / self.touch_depth_scale_factor
            vision_depth_filename = self.vision_depths[idx]
            vision_depth_image = cv2.imread(os.path.join(self.vision_depth_path, vision_depth_filename), cv2.IMREAD_ANYDEPTH)
            vision_depth_image = vision_depth_image.astype(np.float64) / self.vision_depth_scale_factor
            
            self.touch_depth_images.append(touch_depth_image)
            self.vision_depth_images.append(vision_depth_image)
            if visualize:
                visualize_depth_images(touch_depth_image, vision_depth_image)
            print('Read files', touch_depth_filename, vision_depth_filename)
        print('Finished reading files.')
        
    def fuse_all_depths(self):
        for idx in range(self.touch_depth_images):
            touch_depth_image = self.touch_depth_images[idx]
            vision_depth_image = self.vision_depth_images[idx]
            
            fused_depth_image = self.fuse_depth_no_uncertainty(touch_depth_image, vision_depth_image)
            
    
    def fuse_depth_no_uncertainty(self, touch_depth, vision_depth):
        # fuse touch depth and vision depth
        # touch depth is the ground truth, if there is empty data in touch depth, use vision depth
        # if there is empty data in vision depth, use touch depth
        # if there is data in both, use touch depth
        # if there is no data in both, use vision depth
        # touch depth is 0 where there is no data
        # vision depth is 0 where there is no data
        pass
    
    
if __name__ == '__main__':
    depth_fusion = DepthFusion('depth_for_nerf', 'dense_blender_depth')