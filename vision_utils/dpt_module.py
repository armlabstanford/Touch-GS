import os

import torch
import torch.nn.functional as F

from transformers import AutoImageProcessor, DPTForDepthEstimation

from transformers import DPTFeatureExtractor, DPTForDepthEstimation
from zoe_depth import get_zoe_model


from PIL import Image
import matplotlib.pyplot as plt
import numpy as np


def open_image(image_path):
    image = Image.open(image_path)
    return image


class DPT():
    def __init__(self, model_type='vanilla_dpt'):
        self.model_type = model_type
        if self.model_type == 'vanilla_dpt':
            self.feature_extractor = DPTFeatureExtractor.from_pretrained("Intel/dpt-large")
            self.model = DPTForDepthEstimation.from_pretrained("Intel/dpt-large")
        elif self.model_type == 'dino':
            self.feature_extractor = AutoImageProcessor.from_pretrained("facebook/dpt-dinov2-base-nyu")
            self.model = DPTForDepthEstimation.from_pretrained("facebook/dpt-dinov2-base-nyu")
        else:
            print(f'Invalid model type of {self.model_type}. Exiting')
            exit()

        print('Successfully loaded model!')

    def __call__(self, image, visualize=False):
        inputs = self.feature_extractor(images=image, return_tensors="pt")

        with torch.no_grad():
            outputs = self.model(**inputs)
            predicted_depth = outputs.predicted_depth
    
        prediction = torch.nn.functional.interpolate(
                        predicted_depth.unsqueeze(1),
                        size=image.size[::-1],
                        mode="bicubic",
                        align_corners=False,
                ).squeeze()
        
        output = prediction.cpu().numpy()
        if visualize:
            formatted = (output * 255 / np.max(output)).astype('uint8')
            depth = Image.fromarray(formatted)
            plt.imshow(depth)
            plt.show()

        return output
        
if __name__ == '__main__':
    
    zoe_visualize = True
    
    dpt_model = DPT()
    
    zoe_model = get_zoe_model()

    root_img_dir = 'bunny_imgs'

    img_paths = os.listdir(root_img_dir)
    for img_path in img_paths:
        full_path = os.path.join(root_img_dir, img_path)
        
        image = open_image(full_path)

        depth = dpt_model(image)
        
        zoe_depth = zoe_model.infer_pil(image)
        
        if zoe_visualize:
            formatted = (zoe_depth * 255 / np.max(zoe_depth)).astype('uint8')
            depth = Image.fromarray(formatted)
            plt.imshow(depth)
            plt.show()

