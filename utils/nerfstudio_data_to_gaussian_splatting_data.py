import os

import shutil


def nerfstudio_data_to_gaussian_splatting_data(nerfstudio_data_path, gaussian_splatting_data_path):
    """
    convert nerfstudio data to gaussian splatting data
    """
    transforms_file_path = f'{nerfstudio_data_path}/transforms.json'

    # Source directory (the directory to be copied)
    src_directory = "path/to/source/directory"

    # Destination directory (the directory to copy to)
    # Note: The destination directory should not exist; it will be created.
    dest_directory = "path/to/destination/directory"

    try:
        # Copy the directory
        shutil.copytree(src_directory, dest_directory)
        print(f"Directory copied successfully from {src_directory} to {dest_directory}")
    except Exception as e:
        print(f"Error occurred while copying directory: {e}")



if __name__ == '__main__':
    pass
