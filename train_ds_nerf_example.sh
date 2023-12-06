#!/bin/bash

# Install nerfstudio
cd train/nerfs/nerfstudio
pip install -e .
cd ../../..

# Print a message
echo "Running depth-nerfacto on the bunny dataset"
sleep 1

ns-train depth-nerfacto blender-data  --data data/sim/blender/bunny_dataset/