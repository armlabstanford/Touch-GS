import os, re

def get_video_device_number(symlink_path):
    # Read the target of the symlink
    target_device_path = os.readlink(symlink_path)
    
    # Use a regular expression to extract the video device number
    match = re.search(r'video(\d+)$', target_device_path)
    if match:
        return int(match.group(1))
    else:
        return None
