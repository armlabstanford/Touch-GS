# 🐇 Touch-GS: Visual-Tactile Supervised 3D Gaussian Splatting

### [Aiden Swann*](https://aidenswann.com/), [Matthew Strong*](https://peasant98.github.io/), [Won Kyung Do](https://wonkyungdo.github.io/website_wkdo/), [Gadiel Sznaier Camps](https://msl.stanford.edu/people/gadielsznaiercamps/), [Mac Schwager](https://web.stanford.edu/~schwager/), [Monroe Kennedy III](https://monroekennedy3.com/)

_International Conference on Intelligent Robots and Systems (IROS) 2024_ **Oral**!

<!-- insert image here -->
![Local Image](https://touch-gs.github.io/static/images/method.png)



[![Project](https://img.shields.io/badge/Project_Page-Touch_GS-green)](https://touch-gs.github.io/)
[![ArXiv](https://img.shields.io/badge/Arxiv-Touch_GS-red)](https://arxiv.org/abs/2403.09875) 


This repo houses code and data for our work in Touch-GS.


## Quick Start and Setup


The pipeline has been tested on Ubuntu 22.04.


### Requirements:

- CUDA 11+
- Python 3.8+
- Conda or Mamba (optional)

### Dependencies (from Nerfstudio)

Install PyTorch with CUDA (this repo has been tested with CUDA 11.7 and CUDA 11.8) and [tiny-cuda-nn](https://github.com/NVlabs/tiny-cuda-nn).
`cuda-toolkit` is required for building `tiny-cuda-nn`.

For CUDA 11.8:

```bash
conda create --name touch-gs python=3.8
conda activate touch-gs

pip install torch==2.0.1 torchvision==0.15.2 torchaudio==2.0.2 --index-url https://download.pytorch.org/whl/cu118

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

## Installing the GPIS code

We have implemented our own GPIS (Gaussian Process Implicit Surface) from scratch [here](https://github.com/armlabstanford/GPIS)!

Please follow the steps to install the repo there.

## Install Our Version of Nerfstudio

You can find more detailed instructions in Nerfstudio's README.


```bash
# install nerfstudio
bash install_ns.sh
```

## Getting Touch-GS Data Setup and Training

We have made an end-to-end pipeline that will take care of setting up the data, training, and evaluating our method. Note that we will release the code for running the ablations (which includes the baselines) soon!

To prepare each scene:

0. Install Python packages

```sh
pip install -r requirements.txt
```


1. Real Bunny Scene (our method)

```sh
bash scripts/train_bunny_real.sh
```

2. Mirror Scene

```sh
bash scripts/train_mirror_data.sh
```

3. Prism Scene

```sh
bash scripts/train_block_data.sh
```

4. Bunny Blender Scene (processing to a unified format in progress)

## Get Renders

The renders on the test set are created in the main script under `exp-name-renders` within `touch-gs-data`.


## Get Rendered Video

You can get rendered videos with a custom camera path detailed [here](https://docs.nerf.studio/quickstart/first_nerf.html#render-video). This is how we were able to get our videos on our website.
