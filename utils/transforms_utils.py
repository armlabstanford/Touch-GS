from scipy.spatial.transform import Rotation as R

import numpy as np
import json


def read_blender_transform_positions(file_path, return_full_transforms=False):
    # open the file as json
    # read the camera positions
    # return the camera positions
    json_file = open(file_path)
    data = json.load(json_file)
    
    image_positions = {}
    Ts = {}
    for frame in data["frames"]:
        # Extracting the ID from the file name
        image_id = frame["file_path"].split('/')[-1].split('.')[0] # Assuming format "tmpXXXX.png"
        # The position is the last column of the first three rows of the transform matrix
        T = np.array(frame["transform_matrix"])
        t = T[:3, 3]
        image_positions[image_id] = t
        Ts[image_id] = T

    if return_full_transforms:
        return Ts

    return image_positions


def read_nerfstudio_transform_positions(file_path, return_full_transforms=False):
    # open the file as json
    # read the camera positions
    # return the camera positions
    json_file = open(file_path)
    data = json.load(json_file)
    
    image_positions = {}
    Ts = {}
    for frame in data["frames"]:
        # Extracting the ID from the file name
        image_id = frame["file_path"].split('/')[-1].split('.')[0] # Assuming format "tmpXXXX.png"
        # The position is the last column of the first three rows of the transform matrix
        T = np.array(frame["transform_matrix"])
        t = T[:3, 3]
        image_positions[image_id] = t
        Ts[image_id] = T

    camera_instrinsics = [data["fl_x"], data["fl_y"], data["cx"], data["cy"]]

    if return_full_transforms:
        return Ts, camera_instrinsics
    
    return image_positions

def read_colmap_images_txt(file_path, return_full_transforms=False):
    """
    Read camera positions from COLMAP's images.txt file.

    Args:
    - file_path: Path to the images.txt file.

    Returns:
    - A dictionary mapping image names to their camera world positions (TX, TY, TZ).
    """
    camera_positions = {}
    transformations = {}
    with open(file_path, 'r') as file:
        lines = file.readlines()
        # Skip the header lines and any commented lines
        lines = [line for line in lines if not line.startswith('#') and len(line) > 1]
        points = []
        
        for i in range(0, len(lines), 2):  # Each image entry spans 2 lines; data is on the even lines
            parts = lines[i].split()
            if len(parts) < 10:
                continue  # Skip incomplete lines
            image_id = parts[0]
            qw, qx, qy, qz = map(float, parts[1:5])
            tx, ty, tz = map(float, parts[5:8])
            image_name = parts[9]
            # The negative of the translation components gives the camera center in the world frame
            
            r = R.from_quat([qx, qy, qz, qw])
            
            old_pos = np.array([tx, ty, tz])
            
            new_pos = -r.as_matrix().T @ np.array([tx, ty, tz])
            
            new_T = np.eye(4)
            new_T[:3, :3] = r.as_matrix().T
            new_T[:3, 3] = new_pos
            
            image_str = image_name.split('.')[0]
            camera_positions[image_str] = new_pos
            transformations[image_str] = new_T
            points.append(new_pos)

        points = np.array(points)
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]

        # Plotting
        if False:
            fig = plt.figure(figsize=(10, 8))
            ax = fig.add_subplot(111, projection='3d')

            ax.scatter(x, y, z)
            ax.set_xlabel('X Label')
            ax.set_ylabel('Y Label')
            ax.set_zlabel('Z Label')
            ax.set_title('3D Points Visualization (Each Point as np.array)')

            plt.show()
    
    if return_full_transforms:
        return transformations
    return camera_positions


def compute_scale_factor(colmap_camera_positions, blender_camera_positions):
    """
    Compute the scale factor to convert the Blender camera positions to the COLMAP coordinate system.

    Args:
    - colmap_camera_positions: A dictionary mapping image names to their camera world positions (TX, TY, TZ).
    - blender_camera_positions: A dictionary mapping image names to their camera world positions (TX, TY, TZ).

    Returns:
    - The scale factor to convert the Blender camera positions to the COLMAP coordinate system.
    """
    # Compute the average distance of the COLMAP camera positions from the origin
    camera_values = list(colmap_camera_positions.keys())
    print(camera_values)
    
    ratios = []
    
    for i in range(len(camera_values)):
        for j in range(i+1, len(camera_values)):
            if i != j:
                dist_blender_cams = np.linalg.norm(blender_camera_positions[camera_values[i]] - blender_camera_positions[camera_values[j]])
                dist_colmap_cams = np.linalg.norm(colmap_camera_positions[camera_values[i]] - colmap_camera_positions[camera_values[j]])
                print(dist_colmap_cams, dist_blender_cams)
                ratio = dist_colmap_cams / dist_blender_cams
                print("Ratio", ratio)
                ratios.append(ratio)
    scale_factor = np.mean(ratios)
    print(f"Scale factor: {scale_factor}")


if __name__ == '__main__':
    # Example usage
    file_path = 'images.txt'
    colmap_camera_positions = read_colmap_images_txt(file_path)
    # for image_name, position in colmap_camera_positions.items():
    #     print(f"Image: {image_name}, Camera Position: {position}")

    blender_camera_positions = read_nerfstudio_transform_positions('transforms_train_full.json')

    print(colmap_camera_positions)
