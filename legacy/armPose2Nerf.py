import os
import cv2
import json
import numpy as np

Rx = lambda theta: np.array([[1, 0, 0],
                             [0, np.cos(theta), -1*np.sin(theta)],
                             [0, np.sin(theta), np.cos(theta)]])

Ry = lambda phi: np.array([[np.cos(phi), 0, np.sin(phi)],
                           [0, 1, 0],
                           [-1*np.sin(phi), 0, np.cos(phi)]])

Rz = lambda psi: np.array([[np.cos(psi), -1*np.sin(psi), 9],
                           [np.sin(psi), np.cos(psi), 0],
                           [0, 0, 1]])


def get_fl(H, W, config):
    fl_x = None
    fl_y = None

    if 'fl_x' in config and 'fl_y' in config:
        fl_x = config['fl_x']
        fl_y = config['fl_y']

    elif 'fl_x' in config:
        fl_x = config['fl_x']
        fl_y = config['fl_x']

    elif 'fl_y' in config:
        fl_x = config['fl_y']
        fl_y = config['fl_y']

    elif 'camera_angle_x' in config and 'camera_angle_y' in config:
        fl_x = W/(2*np.tan(config['camera_angle_x']/2))
        fl_y = H/(2*np.tan(config['camera_angle_y']/2))

    elif 'camera_angle_x' in config:
        fl_x = fl_y = W/(2*np.tan(config['camera_angle_x']/2))

    elif 'camera_angle_y' in config:
        fl_x = fl_y = H/(2*np.tan(config['camera_angle_y']/2))
    else:
        raise RuntimeError('Failed to load focal length, please check the transforms.json!')    

    return fl_x, fl_y

def get_center_px(H, W, config):
    if 'cx' in config:
        cx = config['cx']
    else:
        cx = W/2
    if 'cy' in config:
        cy = config['cy']
    else:
        cy = H/2
        
    return cx, cy

def get_nearfar(config):
    if 'near' in config:
        near = config['near']
    else:
        near = 0

    if 'far' in config:
        far = config['far']
    else:
        far = np.inf

        
    return near, far

def main(config):
    modes = config["modes"]
    root_path = config["root_path"]
    root_path = os.path.expanduser(root_path)
    transform_name = config["transform_name"]
    units = config["units"]

    data_transform = {}
    data_frames = []
    for mode in modes:
        mode_path = os.path.join(root_path, mode)
        transform_path = os.path.join(mode_path, transform_name)
        
        with open(transform_path, 'r') as f:
            transform = json.load(f)

        frames = transform["frames"]

        for frame in frames:
            
            file_path = os.path.join(mode_path, frame["file_path"])
            # print(file_path)
            # stop
            if not os.path.exists(file_path):
                print("passed: ", file_path)
                continue

            # file_path = os.path.basename(file_path) + ".png"
            image = cv2.imread(file_path, cv2.IMREAD_UNCHANGED)
            

            W = image.shape[1]
            H = image.shape[0]

            if 'cameras' in transform:
                camera = transform["cameras"][frame["camera"]]

                fl_x, fl_y = get_fl(H, W, camera)
                near, far = get_nearfar(camera)
                cx, cy = get_center_px(H, W, camera)
            else:
                fl_x, fl_y = get_fl(H, W, transform)
                near, far = get_nearfar(transform)
                cx, cy = get_center_px(H, W, transform)

            if mode == "depth" or mode == "touch":
                im_name = file_path.split('/')[-1]
                im_name = im_name.split('.')

                depth_image = (image.astype(np.float32)/(255.)*(far - near) + near)

                depth_image[depth_image == near] = 0
                cv2.imwrite(os.path.join(mode_path,"train",im_name[0]+"_depth.png"), image.astype(np.uint16))

            pose = np.array(frame['transform_matrix'], dtype=np.float32)
            # transform pose to be correct orientation
            T = np.eye(4)
            T[:3,:3] = Ry(np.pi/2)
            pose = T@pose
            pose = np.linalg.inv(pose)
            T = np.eye(4)
            T[:3, :3] = Rx(np.pi)
            pose = T@pose
            pose = np.linalg.inv(pose)
            pose[:3, 3] = units*pose[:3, 3]
            
            data_frame = {}

            if mode == "color" or mode == "depth":
                data_frame["camera_model"] = "OPENCV"
            elif mode == "touch":
                data_frame["camera_model"] = "OPENCV_FISHEYE"

            data_frame["fl_x"] = fl_x
            data_frame["fl_y"] = fl_y
            data_frame["cx"] = cx
            data_frame["cy"] = cy
            data_frame["w"] = W
            data_frame["h"] = H

            # camera distortion parameters (k for radial, and p for tangential)
            # assume that they are 0 for now. potental to change if needed
            data_frame["k1"] = 0.0
            data_frame["k2"] = 0.0
            data_frame["k3"] = 0.0
            data_frame["k4"] = 0.0
            data_frame["p1"] = 0.0
            data_frame["p2"] = 0.0

            # remove the './' of path
            path = frame["file_path"].split('/')[1:]
            data_frame["file_path"] = os.path.join('.',mode,*path)
            
            if mode == "depth" or mode == "touch":
                im_name = file_path.split('/')[-1]
                im_name = im_name.split('.')
                data_frame["depth_file_path"] = os.path.join("./", mode, "train",im_name[0]+"_depth.png")
            
            data_frame["transform_matrix"] = pose.tolist()
            data_frames.append(data_frame)

        # print(poses)
        data_transform["frames"] = data_frames
        # print(data_transform)
        out_file = open("./transforms.json", 'w')
        json.dump(data_transform, out_file, indent = 4)

if __name__=='__main__':
    config = {
                "modes": ["color"],
                "root_path": "~/torch-ngp/data/touchnerf_04142023/touchnerf_rgbdepth/",
                "transform_name": "transforms_train.json",
                "units": 1000 # m to mm
             }
    main(config)
