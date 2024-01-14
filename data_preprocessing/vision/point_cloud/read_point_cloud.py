import numpy as np
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt

import open3d as o3d
import cv2


def get_camera_params(path):
    camera_params = {}
    with open(path, 'r') as file:
        for line in file:
            try:
                if int(line[0]) == 1:
                    line_stripped = line.strip()
                    entries = line_stripped.split(" ")
                    print(entries)
                    camera_params['focal_length'] = float(entries[4])
                    camera_params['c_x'] = float(entries[5])
                    camera_params['c_y'] = float(entries[6])
                    
            except:
                continue
    return camera_params

def read_point_cloud(path, visualize=False):
    pcd = o3d.io.read_point_cloud(path)
    if pcd.has_normals():
        print("Point cloud has normals")
    else:
        print("Point cloud does not have normals")
        
    if visualize:
        o3d.visualization.draw_geometries([pcd],
                                  width=800,
                                  height=600,
                                  left=50,
                                  top=50)
        
    return pcd


def read_camera_poses(path):
    # read the camera poses from the colmap txt file
    # return a dictionary with the camera id as the key and the camera pose as the value
    
    camera_id_to_pose = {}
    with open(path, 'r') as file:
        for line in file:
            if "jpg" in line[-4:-1] or "png" in line[-4:-1]:
                entries = line.split(" ")
                qw = float(entries[1])
                qx = float(entries[2])
                qy = float(entries[3])
                qz = float(entries[4])
                tx = float(entries[5])
                ty = float(entries[6])
                tz = float(entries[7])
                camera_id = int(entries[0])
                image_name = entries[9].strip()
                
                # transform the points from colmap to world to world to colmap frame
                r = R.from_quat([qx, qy, qz, qw])
                r_mat = r.as_matrix()
                
                T = np.array([tx, ty, tz])
                
                T_prime = -r_mat.T @ T
                
                tx_prime, ty_prime, tz_prime = T_prime[0], T_prime[1], T_prime[2]
                
                camera_id_to_pose[camera_id] = {"orientation": r_mat, 
                                                "translation": [tx, ty, tz], "image_name": image_name}
                

    
    return camera_id_to_pose


def transform_points(points, camera_position, camera_orientation):
    # transform points into camera coordinate system
    # Translate points based on camera position
    translated_points = points - camera_position
    # Rotate points based on camera orientation
    return np.dot(translated_points, camera_orientation)



def get_points_from_cameras(point_cloud, camera_id_to_pose, camera_params):
    # get the points from the camera based on the camera pose and normal of the points
    # we will use this 
    for camera_id in camera_id_to_pose.keys():
        camera_pose_obj = camera_id_to_pose[camera_id]
        points = get_points_from_camera(point_cloud, camera_pose_obj, camera_params)
        
        print(camera_id, 'processed')
    
    return None
    
    
def get_points_from_camera(point_cloud, camera_pose, camera_params):
    """
    Args:
        point_cloud (_type_): point cloud from colmap dense MVS
        camera_pose (_type_): _description_
    """
    # get the points from the camera based on the camera pose and normal of the points
    # we will use this 
    camera_position = camera_pose["translation"]
    camera_orientation = camera_pose["orientation"]
    
    normals = np.asarray(point_cloud.normals)
    points = np.asarray(point_cloud.points)
    
    transformed_points = transform_points(points, camera_position, camera_orientation)
    
    def is_point_visible(point, normal, camera_orientation):
        # A point is visible if the dot product of its normal and the vector from the camera to the point is positive
        vector_from_camera = -np.dot(point, camera_orientation)
        vector_from_camera_normalized = vector_from_camera / np.linalg.norm(vector_from_camera)
        return np.dot(normal, vector_from_camera_normalized) > 0
    
    # to-do vectorize this
    visible_points = [point for point, normal in zip(transformed_points, normals) if is_point_visible(point, normal, camera_orientation)]
    
    # create camera extrinsics matrix
    camera_extrinsics = np.zeros((3,4))
    camera_extrinsics[:3, :3] = camera_orientation
    camera_extrinsics[:3, 3] = camera_position
    # print(camera_extrinsics)
    
    visible_points = get_viewable_points(camera_params['focal_length'], camera_params['focal_length'], camera_params['c_x'], camera_params['c_y'], visible_points, camera_extrinsics)
    return visible_points
    
    
    
    
def project_points(points_3d, camera_matrix, camera_pose):
    """
    Project 3D points onto 2D image plane.

    :param points_3d: Nx3 numpy array of 3D points.
    :param camera_matrix: 3x3 numpy array representing the camera intrinsic matrix.
    :param camera_pose: 4x4 numpy array representing the camera pose.
    :return: Nx2 numpy array of 2D points.
    """
    # Homogeneous coordinates for the 3D points
    points_3d_hom = np.hstack((points_3d, np.ones((points_3d.shape[0], 1))))

    # Transform points to camera coordinates
    points_cam = np.dot(camera_pose, points_3d_hom.T).T

    # Project points onto image plane
    points_2d_hom = np.dot(camera_matrix, points_cam.T).T

    # Normalize by the last coordinate
    points_2d = points_2d_hom[:, :2] / points_2d_hom[:, 2, np.newaxis]

    return points_2d


def get_viewable_points(focal_length_x, focal_length_y, optical_center_x, optical_center_y, points_3d, camera_extrinsics):
    # Example data
    camera_matrix = np.array([[focal_length_x, 0, optical_center_x],
                            [0, focal_length_y, optical_center_y],
                            [0, 0, 1]])
    # camera_extrinsics = np.eye(4)  # Replace with the actual camera pose

    # Project points
    points_3d = np.array(points_3d)
    points_2d = project_points(points_3d, camera_matrix, camera_extrinsics)

    # Filter points within image boundaries
    image_width, image_height = 640, 480  # Replace with your image dimensions
    in_view = (points_2d[:, 0] >= 0) & (points_2d[:, 0] < image_width) & \
            (points_2d[:, 1] >= 0) & (points_2d[:, 1] < image_height)

    visible_points = points_3d[in_view]

    # Do something with the visible points
    print(visible_points.shape)
        
        
        
        
        
def get_camera_pose(camera_position, camera_orientation):
    # Create a transformation matrix from the camera position and orientation
    pose = np.eye(4)  # Start with an identity matrix
    pose[:3, :3] = camera_orientation  # Set the upper left 3x3 to the rotation matrix
    pose[:3, 3] = camera_position  # Set the upper right 3x1 to the translation vector
    return pose

def transform_point_cloud(point_cloud, transformation_matrix):
    return point_cloud.transform(transformation_matrix)

def create_point_cloud_from_depth(depth_map, intrinsics, scale):
    """
    Convert a depth map to a point cloud object.

    :param depth_map: A 2D numpy array representing the depth map.
    :param intrinsics: The camera intrinsics as a dictionary with 'fx', 'fy', 'cx', 'cy'.
    :return: Open3D point cloud object
    """
    # Create Open3D depth image from the depth map
    depth_o3d = o3d.geometry.Image(depth_map.astype(np.float32))

    # Create intrinsic parameters object
    intrinsics_o3d = o3d.camera.PinholeCameraIntrinsic(
        depth_map.shape[1], depth_map.shape[0],
        intrinsics['focal_length'] * scale, intrinsics['focal_length'] * scale,
        intrinsics['cx'] * scale, intrinsics['cy'] * scale
    )

    # Create a point cloud from the depth image and intrinsics
    pcd = o3d.geometry.PointCloud.create_from_depth_image(
        depth_o3d, intrinsics_o3d
    )

    return pcd

def project_points_with_colors(points_3d, colors, intrinsic_matrix, extrinsic_matrix, image_width, image_height, scale=1):
    # Transform points to camera coordinate system
    points_3d_hom = np.hstack((points_3d, np.ones((points_3d.shape[0], 1))))
    points_cam = np.dot(extrinsic_matrix, points_3d_hom.T).T

    # Project points onto image plane
    points_2d_hom = np.dot(intrinsic_matrix, points_cam[:, :3].T).T
    points_2d = points_2d_hom[:, :2] / points_2d_hom[:, 2, np.newaxis]
    
    points_3d_final = points_2d_hom/ points_2d_hom[:, 2, np.newaxis]
    
    
    # depth = points_3d_final[:, 2]
    # depth_min = np.min(depth)
    # depth_max = np.max(depth)
    # normalized_depth = (depth - depth_min) / (depth_max - depth_min)
    
    valid = points_cam[:, 2] > 0
    depths = points_cam[:, 2][valid]
    
    # create depth image
    depth_image = np.full((image_height, image_width), np.inf)
    
    for (point, depth) in zip(points_2d[valid], depths):
        u, v = point[0], point[1]
        u, v = int(u * scale), int(v * scale)
        if 0 <= u < int(image_width) and 0 <= v < int(image_height):
            if depth_image[v, u] > depth:
                depth_image[v, u] = depth

    depth_image_modified = np.where(np.isinf(depth_image), 0, depth_image)

    # Normalize the depth image, ignoring the -1 values
    # min_depth = np.min(depth_image_modified[depth_image_modified != -1])
    # max_depth = np.max(depth_image_modified)
    # normalized_depth = (depth_image_modified - min_depth) / (max_depth - min_depth)

    # Set the originally infinite values to a value that maps to black
    depth_image_modified[depth_image_modified == -1] = 0

    # Filter points that are in front of the camera
    
    return points_2d[valid], colors[valid], depth_image_modified


def create_depth_image_from_point_cloud(pcd, intrinsic_matrix, extrinsic_matrix, image_width, image_height):
    """
    Create a depth image from a point cloud using camera intrinsics and extrinsics.

    :param pcd: Open3D point cloud object.
    :param intrinsic_matrix: Camera intrinsic matrix (3x3).
    :param extrinsic_matrix: Camera extrinsic matrix (4x4).
    :param image_width: Width of the depth image.
    :param image_height: Height of the depth image.
    :return: 2D numpy array representing the depth image.
    """
    # Transform point cloud to camera coordinates
    points = np.asarray(pcd.points)
    points_hom = np.hstack((points, np.ones((len(points), 1))))
    points_cam = (extrinsic_matrix @ points_hom.T).T[:, :3]

    # Project points onto image plane
    points_img = (intrinsic_matrix @ points_cam.T).T
    points_img /= points_img[:, 2, np.newaxis]  # Normalize by depth
    print(points_img.shape)

    # Initialize depth image with infinity (or a large value to represent no measurement)
    depth_image = np.full((image_height, image_width), np.inf)

    # Fill depth image
    
    for (u, v, depth) in points_img:
        u, v = int(u), int(v)
        print(u, v)
        if 0 <= u < image_width and 0 <= v < image_height:
            if depth_image[v, u] > depth:  # Check if new point is closer
                depth_image[v, u] = depth

    return depth_image


def render_image(points_2d, image_width, image_height, colors=None):
    """
    Render the 2D points as an image.

    :param points_2d: Nx2 numpy array of 2D points.
    :param image_width: Width of the image.
    :param image_height: Height of the image.
    :param colors: Optional Nx3 array for point colors.
    """
    plt.figure(figsize=(12, 8))
    if colors is not None:
        plt.scatter(points_2d[:, 0], points_2d[:, 1], c=colors, s=1, cmap='viridis')
    else:
        plt.scatter(points_2d[:, 0], points_2d[:, 1], s=1)
    plt.xlim(0, image_width)
    plt.ylim(image_height, 0)  # Inverted y-axis for proper image orientation
    plt.axis('off')
    plt.show()
    
    
def render_depth_image(points_3d, image_width, image_height, colors=None):
    """
    Render the 3D points as an image.

    :param points_2d: Nx2 numpy array of 2D points.
    :param image_width: Width of the image.
    :param image_height: Height of the image.
    :param colors: Optional Nx3 array for point colors.
    """
    plt.figure(figsize=(12, 8))
    if colors is not None:
        plt.scatter(points_2d[:, 0], points_2d[:, 1], c=colors, s=1)
    else:
        plt.scatter(points_2d[:, 0], points_2d[:, 1], s=1)
    plt.xlim(0, image_width)
    plt.ylim(image_height, 0)  # Inverted y-axis for proper image orientation
    plt.axis('off')
    plt.show()


def create_coordinate_frame(size, trans, rot):
    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
    transformation_matrix = np.identity(4)
    transformation_matrix[:3, 3] = trans
    transformation_matrix[:3, :3] = rot
    
    frame.transform(transformation_matrix)
    return frame


def get_camera_viewing_direction(extrinsic_matrix):
    # The camera's viewing direction is the negative Z-axis in its coordinate system
    # This is the third column of the rotation matrix part of the extrinsic matrix
    viewing_direction = -extrinsic_matrix[:3, 2]
    return viewing_direction


def filter_points_by_normals(points, normals, viewing_direction, angle_threshold_deg):
    """
    Filter points based on the angle between point normals and camera viewing direction.

    :param points: Nx3 array of 3D points.
    :param normals: Nx3 array of normals corresponding to the points.
    :param viewing_direction: The 3D vector of the camera's viewing direction.
    :param angle_threshold_deg: The threshold angle in degrees. Points with normals deviating more than this angle are filtered out.
    :return: Filtered points and normals and mask
    """
    # Normalize the normals and viewing direction
    normals_normalized = normals / np.linalg.norm(normals, axis=1)[:, np.newaxis]
    viewing_direction_normalized = viewing_direction / np.linalg.norm(viewing_direction)

    # Calculate the dot product (cosine of the angle)
    cos_angle = np.dot(normals_normalized, viewing_direction_normalized)

    # Convert angle threshold to cosine value
    cos_threshold = np.cos(np.radians(angle_threshold_deg))

    # Filter points where the cosine of the angle is greater than the cosine of the threshold
    mask = cos_angle > cos_threshold
    return points[mask], normals[mask], mask 


        
if __name__ == "__main__":
    # point_cloud = read_point_cloud("data_preprocessing/vision/point_cloud/sample_pc_data/fused_bunny_red.ply", visualize=False)
    
    # get the camera poses for each camera and store as a dict
    camera_id_to_pose = read_camera_poses("data_preprocessing/vision/point_cloud/sample_colmap_data/images_28.txt")
    camera_params = get_camera_params("data_preprocessing/vision/point_cloud/sample_colmap_data/cameras_28.txt")
    
    # Load the point cloud
    pcd = read_point_cloud('data_preprocessing/vision/point_cloud/sample_pc_data/fused_bunny_red.ply', visualize=True)

    frames_n_points = []
    
    transformation_matrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
            ])

    transformed_pcd = pcd.transform(transformation_matrix)
    
    frames_n_points.append(transformed_pcd)
    
    cam_positions = []
    cam_orientations = []
    
    take_cam_pose_idx = 1
    take_cam_pose = None
    idxs = [1, 20, 40, 60, 80, 100]
    for i in range(1, 101):
        camera_position = np.array(camera_id_to_pose[i]['translation'])# Replace with actual position
        camera_orientation = camera_id_to_pose[i]['orientation']
        
        camera_pose = get_camera_pose(camera_position, camera_orientation)
        take_cam_pose = camera_pose
        custom_frame = create_coordinate_frame(1, camera_position, camera_orientation)
        # if i == take_cam_pose_idx:
        frames_n_points.append(custom_frame)
        
        # o3d.visualization.draw_geometries(frames_n_points)
        
        
        scale = 7096.24006719
        
        image_width, image_height = (1800 /scale) , (1100 / scale)  # Replace with actual image dimensions

        # Define camera intrinsics (replace with actual values)
        # focal length millimeters
        
        s = 1
        
        fx = camera_params['focal_length'] * s / scale
        fy = camera_params['focal_length'] * s / scale
        cx = camera_params['c_x'] / scale
        cy = camera_params['c_y'] / scale
        
        camera_intrinsics = {"focal_length": fx/scale, "cx": cx/scale, "cy": cy/scale}
        
        # convert pixel to distance
        # focal length in pixels is 2500.
        # focal length in mm is 50 mm, 0.05 m
        # each pixel is 0.00002 based on physical size of sensor
        
        # mapping from units in blender to colmap is:
        
        # camera_intrinsics = o3d.camera.PinholeCameraIntrinsic(image_width, image_height, fx/scale, fy/scale, cx/scale, cy/scale)

        # Filter points within the camera's field of view
        
        intrinsic_matrix = np.array([[fx, 0, cx],
                                [0, fy, cy],
                                [0, 0, 1]])

        extrinsic_matrix = camera_pose
        
        points_3d = np.asarray(pcd.points)
        normals = np.asarray(pcd.normals)
        
        viewing_direction = get_camera_viewing_direction(extrinsic_matrix)
        angle_threshold_deg = 75  # Adjust as needed
        filtered_points, filtered_normals, mask = filter_points_by_normals(points_3d, normals, viewing_direction, angle_threshold_deg)
        
        # transformed_pcd = pcd.transform((camera_pose))
        
        # colors = np.asarray(pcd.colors)[mask]
        
        # points = np.asarray(pcd.points)[mask]
        
        colors = np.asarray(pcd.colors)
        
        points = np.asarray(pcd.points)
        
        points_2d, colors_2d, depth = project_points_with_colors(points, colors, intrinsic_matrix, extrinsic_matrix, 1800, 1100, scale=scale)
        # pcd_depth = create_point_cloud_from_depth(depth, camera_intrinsics, scale)
        # o3d.visualization.draw_geometries([pcd_depth])
        
        
        if True:
            render_image(points_2d, image_width, image_height, colors=colors_2d)  # Omit 'colors' if not available
            # render_image(points_2d, image_width, image_height, colors=depth)  # Omit 'colors' if not available
            final_depth_int = (1000 * depth).astype(np.uint16)
            pad_num = i // 10
            # get amount of zeros to pad
            pad = '0' * (4-pad_num)
            cv2.imwrite(f'{pad}{i}.depth.png', final_depth_int)
            # plt.imshow(depth, cmap='viridis')
            # plt.colorbar(label='Normalized Depth')
            # plt.title("Depth Image")
            # plt.axis('off')  # Turn off axis numbers and labels
            # plt.show()
        print(i, 'processed')

    """
    
    interlude
    """
    
    o3d.visualization.draw_geometries(frames_n_points)
    
    transformed_pcd = pcd.transform(np.linalg.inv(camera_pose))
    # frames_n_points.pop()
    base_frame = create_coordinate_frame(1, np.array([0, 0, 0]), np.identity(3))
    o3d.visualization.draw_geometries([transformed_pcd, base_frame])
    
    


