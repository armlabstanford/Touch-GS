import os
import torch
import cv2
from glob import glob
from torchvision import transforms
from .networks.DenseNet import DenseDepth
from .networks.STForce import DenseNet_Force
import numpy as np
from PIL import Image
import IPython

# Define the normalization transformation
normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])

def transform_image(img):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    im_pil = Image.fromarray(img)  # Corrected function name
    img3 = np.asarray(im_pil, dtype=np.float32) / 255.0
    img3 = transforms.ToTensor()(img3)
    img3 = normalize(img3)
    return img3[None, :]

def getDepth(model, img):
    img3 = transform_image(img)
    with torch.no_grad():
        depth_est = model(img3)
        pred_depth = depth_est.cpu().numpy().squeeze()
    pred_depth = np.uint8(np.clip(pred_depth, 0, 255))
    return pred_depth

def getForce(model, img):
    img3 = transform_image(img)
    with torch.no_grad():
        force_est = model(img3)
        pred_force = force_est.cpu().numpy().squeeze()
    force_residual = np.asarray([5.5, 5.5, 11, 0.25, 0.25, 0.05 ])
    force_range = np.asarray([11, 11, 14, 0.5, 0.5, 0.1])
    forceval = (pred_force ) * force_range - force_residual
    return forceval

def test():
    print("testing..")

if __name__ == '__main__':
    test()