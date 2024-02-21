import numpy as np


def save_depth_image_matrix_as_npy(depth_image, file_path):
    try:
        np.save(file_path, depth_image)
        return True
    except Exception as e:
        print(f'Error saving depth image to {file_path}. Error: {e}')
        return False


if __name__  == '__main__':
    pass