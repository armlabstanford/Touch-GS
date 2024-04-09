# 🐇 Touch-GS: Visual-Tactile Supervised 3D Gaussian Splatting

### [Aiden Swann*](https://aidenswann.com/), [Matthew Strong*](https://peasant98.github.io/), [Won Kyung Do](https://arm.stanford.edu/people/wonkyung-do), [Gadiel Sznaier Camps](https://msl.stanford.edu/people/gadielsznaiercamps/), [Mac Schwager](https://web.stanford.edu/~schwager/), [Monroe Kennedy III](https://monroekennedy3.com/)

<!-- insert image here -->
![Local Image](https://touch-gs.github.io/static/images/method.png)



[![Project](https://img.shields.io/badge/Project_Page-Touch_GS-green)](hhttps://touch-gs.github.io/)
[![ArXiv](https://img.shields.io/badge/Arxiv-Touch_GS-red)](https://arxiv.org/abs/2403.09875) 


This repo houses code and data for our work in Touch-GS.


## Quick Start and Setup


The pipeline has been tested on Ubuntu 22.04.


### Requirements:

- CUDA 11+
- Python 3.8+

### Dependencies (from Nerfstudio)

Install PyTorch with CUDA (this repo has been tested with CUDA 11.7 and CUDA 11.8) and [tiny-cuda-nn](https://github.com/NVlabs/tiny-cuda-nn).
`cuda-toolkit` is required for building `tiny-cuda-nn`.

For CUDA 11.8:

```bash
pip install torch==2.1.2+cu118 torchvision==0.16.2+cu118 --extra-index-url https://download.pytorch.org/whl/cu118

conda install -c "nvidia/label/cuda-11.8.0" cuda-toolkit
pip install ninja git+https://github.com/NVlabs/tiny-cuda-nn/#subdirectory=bindings/torch
```

See [Dependencies](https://github.com/nerfstudio-project/nerfstudio/blob/main/docs/quickstart/installation.md#dependencies)
in the Installation documentation for more.

**Repo Cloning**
To clone the repo, since we have submodules, run the following command:

```bash
git clone --recurse-submodules https://github.com/armlabstanford/Touch-GS
```

## Install Our Version of Nerfstudio

You can find more detailed instructions in Nerfstudio's README.


```bash
# install nerfstudio
bash install_ns.sh
```

## Getting Data Setup and Training

We have made an end-to-end pipeline that will take care of setting up the data, training, and evaluating our method.

To prepare each scene:

0. Install Python packages

```sh
pip install -r requirements.txt
```


1. Real Bunny Scene

```sh
bash bunny_real_data/train.sh
```

2. Mirror Scene

```sh
bash mirror_data/train.sh
```

3. Prism Scene

```sh
bash block_data /train.sh
```

4. Bunny Blender Scene (in progress)


Structure of each Dataset:


```
- imgs: All images for train and test
- train.sh: Main training script 
- touches: Raw touches (not used in this repo). Contains raw images from the DenseTact optical tactile sensor and depths.
- transforms.json: Contains camera poses and paths to files and depth/uncertainties.
- realsense_depth/s: Realsense sensor depths
- gpis_depth/var: Raw results from the GPIS 
- touch_depth/var: Depths and variances from touch
- <scene>_zoe_depth: list of depths after running monocular depth
- vision_baseline: Baseline depths (aligned) for training a 3D-GS
- vision: Aligned vision
- zoe_depth_aligned: (not needed) as is
- fused_output_dir/uncertainty: Fused results with our method.
```

