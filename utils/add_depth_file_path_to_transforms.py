# Stanford University, ARMLab 2023
# Touch-GS

import json
import os

import argparse


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Update the path to depth filenames in the transforms.json.')
    parser.add_argument('--base_repo_path', type=str, required=True, help='Path to the base repo.')
    parser.add_argument('--filename', type=str, required=True, help='Path to the transforms.json.')
    parser.add_argument('--depth_file_path_template', type=str, required=True, help='Depth file path')
    parser.add_argument('--uncertainty_file_path_template', type=str, required=True, help='Uncertainty file path')
    
    
    args = parser.parse_args()
    
    filename = args.filename
    base_repo_path = args.base_repo_path
    
    full_filename = os.path.join(base_repo_path, filename)
    
    # Read the JSON data from the file
    with open(full_filename, 'r') as file:
        data = json.load(file)

    
    depth_file_path_template = args.depth_file_path_template
    uncertainty_file_path_template = args.uncertainty_file_path_template

    depth_file_path_template = depth_file_path_template + '/{}'
    uncertainty_file_path_template = uncertainty_file_path_template + '/{}'

    # Iterate through each frame and add depth_file_path
    for frame in data['frames']:
        if 'file_path' in frame:
            
            # Extract the original file name and directory
            original_file_path_parts = frame['file_path'].split('/')
            
            original_file_name = original_file_path_parts[-1]  
            # Get the last part as the file name
            depth_file_name = depth_file_path_template.format(original_file_name)  
            # Format the new file name
            uncertainty_file_name = uncertainty_file_path_template.format(original_file_name)
            frame['depth_file_path'] = depth_file_name  
            # Assign the new depth file path
            frame['uncertainty_file_path'] = uncertainty_file_name

    with open(full_filename, 'w') as file:
        json.dump(data, file, indent=4)

    print(f"Modified data has been saved to {full_filename}.")