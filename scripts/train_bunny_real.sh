#!/bin/bash

echo "Starting train script for bunny real data"


echo "Reading realsense depths..."
echo 
sleep 1

python3 utils/read_realsense_depth.py --base_repo_path touch-gs-data/bunny_real_data

echo "Reading touch depths..."
echo 
sleep 1

python3 utils/read_touch_depths.py --base_repo_path touch-gs-data/bunny_real_data

# compute zoe depth maps
echo "Running monocular depth"
echo 
sleep 1

python3 vision_utils/run_zoe_depth.py --root_dir touch-gs-data/bunny_real_data/ --img_dir imgs --output_depth_path bunny_zoe_real_world_depth



echo "Fusing vision and touch..."
echo 
sleep 1

# python3 utils/fuse_touch_vision.py --root_dir 'block_data' --aligning_depths 'realsense_depths' --touch_depth 'touch_depth' --zoe_depth_path 'blocks_zoe_depth' --vision_output_dir 'vision' --fused_output_dir 'fused_output_dir' --touch_var 'touch_var'

python3 utils/fuse_touch_vision.py --root_dir 'touch-gs-data/bunny_real_data' --aligning_depths 'realsense_depths' --touch_depth 'touch_depth' --zoe_depth_path 'bunny_zoe_real_world_depth' --use_uncertainty --vision_output_dir 'vision' --fused_output_dir 'fused_output_dir' --touch_var 'touch_var'

echo "Adding depths and uncertainties to transforms.json..."
echo 
sleep 1

python3 utils/add_depth_file_path_to_transforms.py --base_repo_path 'touch-gs-data/bunny_real_data' --filename 'transforms.json' --depth_file_path_template 'fused_output_dir' --uncertainty_file_path_template 'fused_output_dir_uncertainty'


# create point cloud from the touches

echo "Creating point cloud from touch depths..."
echo 
sleep 1

python3 utils/create_point_cloud_from_touches.py --root_dir 'touch-gs-data/bunny_real_data' --touch_depth_dir 'touch_depth' --touch_var_dir 'touch_var' --image_dir 'imgs' --transform_json_path 'transforms.json' --train_split 0.08

# train GS
cd touch-gs-data
ns-train depth-gaussian-splatting --data bunny_real_data/ --viewer.quit-on-train-completion True --pipeline.model.depth-loss-mult 0.005 --pipeline.model.depth-loss-type DEPTH_UNCERTAINTY_WEIGHTED_LOSS  --pipeline.model.uncertainty_weight 0.01 nerfstudio-data --train-split-fraction 0.08

export IS_REAL_WORLD=True
python3 ../experiment_utils/run_eval.py --input_dir outputs/bunny_real_data/depth-gaussian-splatting --output_dir ../experiments --exp_name bunny_real --past_n_trials 1

cd ..