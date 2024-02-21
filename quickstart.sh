#!/bin/bash


cd train/nerfs/nerfstudio

# install nerfstudio
pip install .

cd ../../..

cd data_preprocessing/vision/main_vision_module
pip install -r requirements.txt

cd ../../..

bash train_dense_depth.sh $(pwd)/sample_horns
