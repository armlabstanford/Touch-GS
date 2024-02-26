import cv2
import glob
import cv2
import numpy as np
import os
import json
import shutil

# List all jpg images in the color/train/ folder
base_path = '/home/wkdo/Desktop/touchnerf_bunny2'
color_path = os.path.join(base_path, 'color', 'train')
depth_path = os.path.join(base_path, 'depth', 'train')

# if the path is not made, make the directory
new_path = '/home/wkdo/Desktop/touchnerf_bunny2_filtered'
new_color_path = os.path.join(new_path, 'color', 'train')
new_depth_path = os.path.join(new_path, 'depth', 'train')

# if exists sth in the path, remove it
if os.path.exists(new_path):
    shutil.rmtree(new_path)


if not os.path.exists(new_path):
    os.makedirs(new_path)
    os.makedirs(new_color_path)
    os.makedirs(new_depth_path)

image_paths = glob.glob(os.path.join(base_path, 'color', 'train', '*.jpg'))
json_paths = os.path.join(base_path, 'color', 'transforms_train.json')
depth_json_paths = os.path.join(base_path, 'depth', 'transforms_train.json')


new_json_path = os.path.join(new_path, 'color', 'transforms_train.json')
new_depth_json_path = os.path.join(new_path, 'depth', 'transforms_train.json')


# make a ring buffer (length 10) to check the psnr of the new images, and update the buffer, and check
# if the psnr is lower than the threshold, remove the image from the list
#first define ring buffer
ring_buffer = []

def is_image_blurry_psnr(image_path, threshold=8.5):
    image = cv2.imread(image_path)
    if image is None:
        raise ValueError("Image not found or unable to read.")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if len(ring_buffer) < 10:
        ring_buffer.append(gray)
        return False
    else:
        ring_buffer.pop(0)
        ring_buffer.append(gray)
        psnr = 0
        for i in range(10):
            for j in range(i+1, 10):
                psnr += cv2.PSNR(ring_buffer[i], ring_buffer[j])
        psnr /= 45
        # print(f'PSNR: {psnr}')
        if psnr > threshold:
            print(f'similar image detected: {image_path}, PSNR: {psnr}')
            return True
        else:
            return False


def is_image_blurry(image_path, threshold=150):
    image = cv2.imread(image_path)
    if image is None:
        raise ValueError("Image not found or unable to read.")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
    if laplacian_var < threshold:
        print(f'Blurry image detected: {image_path}')
        return True
    else:
        return False

# Filter out blurry images
sharp_images = [img for img in image_paths if not is_image_blurry(img)]

# Filter out blurry images using PSNR
sharp_images = [img for img in sharp_images if not is_image_blurry_psnr(img)]

# Load the transformations from the JSON file
with open(json_paths, 'r') as file:
    transforms = json.load(file)



# Filter transformations to keep only those corresponding to sharp images
sharp_image_filenames = [path.split('/')[-1] for path in sharp_images]
sharp_transforms = [frame for frame in transforms['frames'] if frame['file_path'].split('/')[-1] in sharp_image_filenames]

transforms['frames'] = sharp_transforms

with open(depth_json_paths, 'r') as file:
    depth_transforms = json.load(file)

#change format c_0.jpg to d_0.jpg for the depth images and change the path /color/train to /depth/train
sharp_depth_image_filenames = [path.replace('color', 'depth').replace('c_', 'd_') for path in sharp_image_filenames]
sharp_depth_transforms = [frame for frame in transforms['frames'] if frame['file_path'].split('/')[-1] in sharp_depth_image_filenames]

depth_transforms['frames'] = sharp_depth_transforms

#print total filtered images
print(f"Total images: {len(image_paths)}")
print(f"filtered images: {len(sharp_images)}")
print(f"Total transformations: {len(transforms['frames'])}")

# save all
for image_path in sharp_images:
    image_filename = image_path.split('/')[-1]
    new_image_path = os.path.join(new_color_path, image_filename)
    shutil.copy(image_path, new_image_path)

# Save sharp images to the new directory for depth
sharp_depth_images = [img.replace('color', 'depth').replace('c_', 'd_') for img in sharp_images]
for image_path in sharp_depth_images:
    image_filename = image_path.split('/')[-1]
    new_image_path = os.path.join(new_depth_path, image_filename)

    shutil.copy(image_path, new_image_path)


with open(new_json_path, 'w') as file:
    json.dump(transforms, file, indent=4)

with open(new_depth_json_path, 'w') as file:
    json.dump(depth_transforms, file, indent=4)


print("Updated transformations saved to transforms_train_updated.json")
