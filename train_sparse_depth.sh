#!/bin/bash

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

# feel free to change the below line to suit your needs
# ns-train depth-nerfacto --data $DATASET_PATH --pipeline.model.depth-loss-mult 0.01 --pipeline.model.depth_loss_type SIMPLE_LOSS colmap --train-split-fraction 0.1

# ns-train depth-gaussian-splatting --data $DATASET_PATH --pipeline.model.depth-loss-mult 0.01 --pipeline.model.depth_loss_type SIMPLE_LOSS colmap --train-split-fraction 0.1
