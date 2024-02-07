#!/bin/bash

# Check if an argument was provided
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 /path/to/dataset"
    exit 1
fi

# Assign the first argument to DATASET_PATH
DATASET_PATH=$1

# Ensure the path contains an "images" folder
if [ ! -d "$DATASET_PATH/images" ]; then
    echo "The specified dataset path does not contain an 'images' folder."
    exit 1
fi

# Run the COLMAP commands with the specified dataset path
colmap feature_extractor --database_path $DATASET_PATH/database.db --image_path $DATASET_PATH/images --ImageReader.single_camera 1 --ImageReader.camera_model SIMPLE_PINHOLE
colmap exhaustive_matcher --database_path $DATASET_PATH/database.db
mkdir -p $DATASET_PATH/sparse
colmap mapper --database_path $DATASET_PATH/database.db --image_path $DATASET_PATH/images --output_path $DATASET_PATH/sparse
