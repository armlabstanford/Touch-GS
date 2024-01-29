import os

import torch
import cv2
from glob import glob
from torchvision import transforms

from Img2Depth.networks.DenseNet import DenseDepth
from Img2Depth.networks.STForce import DenseNet_Force

from timeit import default_timer as timer
import numpy as np
from PIL import Image

import IPython
#########################################
def _is_pil_image(img):
    return isinstance(img, Image.Image)

def _is_numpy_image(img):
    return isinstance(img, np.ndarray) and (img.ndim in {2, 3})

def to_tensor(pic):
        if not (_is_pil_image(pic) or _is_numpy_image(pic)):
            raise TypeError(
                'pic should be PIL Image or ndarray. Got {}'.format(type(pic)))
        
        if isinstance(pic, np.ndarray):
            img = torch.from_numpy(pic.transpose((2, 0, 1)))
            return img
        
        # handle PIL Image
        if pic.mode == 'I':
            img = torch.from_numpy(np.array(pic, np.int32, copy=False))
        elif pic.mode == 'I;16':
            img = torch.from_numpy(np.array(pic, np.int16, copy=False))
        else:
            img = torch.ByteTensor(torch.ByteStorage.from_buffer(pic.tobytes()))
        # PIL image mode: 1, L, P, I, F, RGB, YCbCr, RGBA, CMYK
        if pic.mode == 'YCbCr':
            nchannel = 3
        elif pic.mode == 'I;16':
            nchannel = 1
        else:
            nchannel = len(pic.mode)
        img = img.view(pic.size[1], pic.size[0], nchannel)
        
        img = img.transpose(0, 1).transpose(0, 2).contiguous()
        if isinstance(img, torch.ByteTensor):
            return img.float()
        else:
            return img

def getDepth(model, img):

    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    im_pil = Image.fromarray(img)
    img3 = np.asarray(im_pil, dtype=np.float32) / 255.0
    img3 = to_tensor(img3)
    normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    img3 = normalize(img3)
    img3 = img3[None,:]

    start = timer()
    with torch.no_grad():
        depth_est = model(img3)
        pred_depth = depth_est.cpu().numpy().squeeze()

    # print(pred_depth.shape)
    pred_depth = np.uint8(np.clip(pred_depth, 0, 255))

    return pred_depth

def getForce(model, img):

    idx = 0

    # image = np.asarray(img, dtype=np.float32) / 255.0
    # # print(image)
    # img = torch.from_numpy(image.transpose((2, 0, 1)))
    # img = img.unsqueeze(0)

    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    im_pil = Image.fromarray(img)
    img3 = np.asarray(im_pil, dtype=np.float32) / 255.0
    img3 = to_tensor(img3)
    normalize = transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    img3 = normalize(img3)
    img3 = img3[None,:]

    with torch.no_grad():

        force_est = model(img3)
        pred_force = force_est.cpu().numpy().squeeze()
    force_residual = np.asarray([5.5, 5.5, 11, 0.25, 0.25, 0.05 ])
    force_range = np.asarray([11, 11, 14, 0.5, 0.5, 0.1])
    forceval = (pred_force ) * force_range - force_residual


    # print(pred_force.shape)
    # print(pred_force)

    return forceval

def test():
    print("testing..")

def getDepth_square(model, img, device, img_format):
    """
        depth estimation for DTv1
    """
    idx = 0

    # img: PIL-readable image
    loaded_images = []
    x = np.clip(np.asarray(img.resize((640, 480)), dtype=float) / 255, 0, 1).transpose(2, 0, 1)
    loaded_images.append(x)
    np.stack(loaded_images, axis=0)


    img = torch.Tensor(np.array(loaded_images)).float().to(device)

    start = timer()
    with torch.no_grad():
        preds = DepthNorm(model(img).squeeze(0))
    a = preds.data

    a = a.reshape(1, 320 * 240).cpu()
    a = (a.numpy() - 10) / (990) * 255
    a = np.clip(a, 0, 255).astype(int)
    result = a.reshape(240, 320)

    imgre = cv2.resize(result.astype('float32'), (570, 570), interpolation=cv2.INTER_LINEAR_EXACT)
    img_real = img_format
    img_real[0:570, 130:700] = imgre

    return imgre, img_real


if __name__ == '__main__':

    test()
