#!/bin/bash

# Shell script for training a nerf with depth supervision and dense depth

# Check if an argument was provided
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 /path/to/dataset"
    exit 1
fi

# Assign the first argument to DATASET_PATH
DATASET_PATH=$1

# Ensure the path contains an "images" folder
if [ ! -d "$DATASET_PATH/imgs" ]; then
    echo "The specified dataset path does not contain an 'imgs' folder."
    exit 1
fi

# uncomment to run colmap or comment to not run it
bash run_colmap.sh $DATASET_PATH


ns-process-data images --data $DATASET_PATH/imgs --no-only-copy-depth --output-dir $DATASET_PATH

# run dense depth inference to get dense depth images and uncertainties.

# copy over sparse colmap depths and regular images to the dinov2 folder

IMGS_DENSE_DEPTH_FOLDER_BASE=horns_imgs
SPARSE_DEPTH_FOLDER_BASE=horn_colmap_depth
OUTPUT_DEPTH_FOLDER_BASE=horn_dense_depth
OUTPUT_DEPTH_FOLDER_BASE_UNCERTAINTY=$OUTPUT_DEPTH_FOLDER_BASE'_uncertainty'

echo $OUTPUT_DEPTH_FOLDER_BASE_UNCERTAINTY

IMGS_DENSE_DEPTH_FOLDER=data_preprocessing/vision/main_vision_module/$IMGS_DENSE_DEPTH_FOLDER_BASE

SPARSE_DEPTH_FOLDER=data_preprocessing/vision/main_vision_module/$SPARSE_DEPTH_FOLDER_BASE

mkdir $IMGS_DENSE_DEPTH_FOLDER
mkdir $SPARSE_DEPTH_FOLDER


cp $DATASET_PATH/images/* $IMGS_DENSE_DEPTH_FOLDER

cp $DATASET_PATH/depth/* $SPARSE_DEPTH_FOLDER



cd data_preprocessing/vision/main_vision_module

# uncomment the below line if you have not installed the "vision module"
# pip install .

python3 visual_pipeline.py --root_img_dir $IMGS_DENSE_DEPTH_FOLDER_BASE --colmap_depth_dir $SPARSE_DEPTH_FOLDER_BASE --output_depth_path $OUTPUT_DEPTH_FOLDER_BASE --visualize False


# copy over the depth images and uncertainties to the original dataset folder
mkdir $DATASET_PATH/uncertainty
cp $OUTPUT_DEPTH_FOLDER_BASE/* $DATASET_PATH/depth
cp $OUTPUT_DEPTH_FOLDER_BASE_UNCERTAINTY/* $DATASET_PATH/uncertainty

ns-process-data images --data $DATASET_PATH/imgs --only-copy-depth --output-dir $DATASET_PATH


# feel free to change the below line to suit your needs

# nerfacto
ns-train depth-nerfacto --data $DATASET_PATH --pipeline.model.depth-loss-mult 0.005 --pipeline.model.depth_loss_type SIMPLE_LOSS colmap --train-split-fraction 0.1

# gaussian splatting
# ns-train depth-gaussian-splatting --data $DATASET_PATH --pipeline.model.depth-loss-mult 0.005 --pipeline.model.depth_loss_type SIMPLE_LOSS colmap --train-split-fraction 0.1
