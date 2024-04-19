import os

from PIL import Image
import cv2
import numpy as np

from dpt_module import DPT
from zoe_depth import get_zoe_model

import matplotlib.pyplot as plt

import argparse

def open_image(image_path):
    image = Image.open(image_path)
    return image

def compute_scale_and_offset(sparse_depth, dense_depth, weights=None):
    """
    Compute the scale factor and offset between sparse and dense depth maps.

    :param sparse_depth: Sparse depth map (2D numpy array).
    :param dense_depth: Dense depth map (2D numpy array, same size).
    :return: scale_factor, offset
    """
    # Mask to consider only the non-zero elements of the sparse depth map
    mask = sparse_depth > 0
    if weights is None:
        # Flattening the arrays and applying the mask
        sparse_depth_flat = sparse_depth[mask].flatten()
        dense_depth_flat = dense_depth[mask].flatten()

        # Performing linear regression
        A = np.vstack([dense_depth_flat, np.ones_like(dense_depth_flat)]).T
        scale_factor, offset = np.linalg.lstsq(A, sparse_depth_flat, rcond=None)[0]

        return scale_factor, offset
    else:
        sparse_depth_flat = sparse_depth[mask].flatten()
        dense_depth_flat = dense_depth[mask].flatten()
        weights_flat = weights[mask].flatten()

        # Incorporating weights into the linear regression
        W = np.diag(weights_flat)
        A = np.vstack([dense_depth_flat, np.ones_like(dense_depth_flat)]).T
        AW = np.dot(W, A)
        BW = np.dot(W, sparse_depth_flat)

        # Performing weighted least squares regression
        scale_factor, offset = np.linalg.lstsq(AW, BW, rcond=None)[0]
        
        return scale_factor, offset


class VisualPipeline:
    def __init__(self, root_img_dir, output_depth_path='output', scale_factor=1000):
        """Initializes the visual pipeline

        Args:
            root_img_dir (_type_): _description_
        """
        self.dpt_model = DPT()
        self.zoe_model = get_zoe_model()
        # self.depth_anything_model = pipeline(task="depth-estimation", model="LiheYoung/depth-anything-base-hf")
        # self.depth_anything_model = None
        
        self.root_img_dir = root_img_dir
        
        self.img_paths = sorted(os.listdir(self.root_img_dir))
        
        self.images = []
        
        self.get_all_images()
        
        self.output_depth_path = output_depth_path
        self.scale_factor = scale_factor
        
        if not os.path.exists(self.output_depth_path):
            os.mkdir(self.output_depth_path)
            
            
    def get_all_images(self):
        for _, img_path in enumerate(self.img_paths):
            full_path = os.path.join(self.root_img_dir, img_path)
            image = open_image(full_path)
            self.images.append(image)
            print('Loaded:', full_path)
            
    def get_images(self):
        return self.image
    
    def predict(self, visualize=False):
        for i in range(len(self.images)):
            img_np = np.array(self.images[i])
            
            if len(img_np.shape) > 2 and img_np.shape[2] == 4:
                # convert the image from RGBA2RGB
                img_np = cv2.cvtColor(img_np, cv2.COLOR_BGRA2BGR)
            self.images[i] = Image.fromarray(img_np)
            
            predicted_depth = self.predict_depth_from_image(self.images[i], model_type='zoe')
            
            final_depth_int = (self.scale_factor * predicted_depth).astype(np.uint16)
            depth_paths = os.listdir(self.root_img_dir)
            
            depth_paths_sorted = sorted(depth_paths)
            
            depth_valid_selected = depth_paths_sorted[i]
            
            if visualize:
                plt.imshow(final_depth_int, cmap='viridis')
                plt.show()
                    
            cv2.imwrite(f'{self.output_depth_path}/{depth_valid_selected}', final_depth_int)
            print(f'Saved depth image {self.output_depth_path}/{depth_valid_selected}')
        
                
    def visualize(self, colmap_depth, predicted_depth, refined_depth, labels=['Colmap Depth', 'Predicted Depth', 'Refined Depth']):
        # Apply a colormap for visualization
        # You can change 'plasma' to any other colormap (like 'viridis', 'magma', etc.)
        
        plt.figure(figsize=(12, 6))

        # Display the first depth image
        plt.subplot(1, 3, 1)  # (1 row, 2 columns, first subplot)
        plt.imshow(colmap_depth, cmap='viridis')
        plt.title(labels[0])
        plt.axis('off')  # Turn off axis numbers

        # Display the second depth image
        plt.subplot(1, 3, 2)  # (1 row, 2 columns, second subplot)
        plt.imshow(predicted_depth, cmap='viridis')
        plt.title(labels[1])
        plt.axis('off')  # Turn off axis numbers
        
        plt.subplot(1, 3, 3)  # (1 row, 2 columns, second subplot)
        plt.imshow(refined_depth, cmap='viridis')
        plt.title(labels[2])
        plt.axis('off')  # Turn off axis numbers

        # Show the plot
        plt.show()
        
        
    def predict_depth_from_image(self, image, model_type='zoe'):
        if model_type == 'zoe':
            depth = self.zoe_model.infer_pil(image)
        elif model_type == 'depth_anything':
            depth = self.depth_anything_model(image)
            depth_tensor = depth['predicted_depth'].numpy().squeeze(axis=0) / 255
            
            depth = depth_tensor / np.max(depth_tensor)
            
            depth = 1 - depth_tensor
        else:
            depth = self.dpt_model(image)
            
        return depth


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process RGB images through zoe_Depth.')

    # Add arguments for the image directory and COLMAP depth directory
    parser.add_argument('--root_dir', type=str, default='bunny_real_data', help='Root directory path')
    parser.add_argument('--img_dir', type=str, default='imgs', help='Image directory path')
    
    parser.add_argument('--output_depth_path', type=str, default='zoe_depth_result', help='Output depth directory path')
    
    parser.add_argument('--viz', action='store_true', help='Whether or not to viz.')
    
    # Parse the arguments
    args = parser.parse_args()
    
    viz = args.viz
    
    imgs_path = os.path.join(args.root_dir, args.img_dir)
    
    output_depth_path = os.path.join(args.root_dir, args.output_depth_path)
    
    visual_pipeline = VisualPipeline(root_img_dir=imgs_path, output_depth_path=output_depth_path)
    
    visual_pipeline.predict(visualize=viz)