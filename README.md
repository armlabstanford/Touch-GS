# Visual Tactile Neural Field
Visual Tactile Neural Field (VTNF)

<!-- insert image here -->
![Local Image](misc_imgs/mvp_architecture.png)

Aiden Swann, Matt Strong, Won-Kyung Do

This repo houses code and data for our work in Visual Tactile Neural fields.

The flow can be broadly broken up into the following sections:

1. Data Collection

    This contains code to create the data from simulation in Blender and real life.
    We use a 7 DoF Franka Panda for our experiments.

2. Data

    This contains the data in Blender in simulation and on the real robot. The format is as follows:


    ```
    data
    |---real
    |   |---touchnerf_<date-1>
            |---touchnerf_rgb_depth_<date-1>
                |---color
                |---depth
                |---recorded_path
                |---touch
                |---touch_raw
            |---touchnerf_touch_1_<date-1>
            |---touchnerf_touch_2<date-1>
    |   |---touchnerf_<date-2>
    |   |---...
    |---sim
        |---blender
            |---tbd
    ```

    We expect a unified format for the raw data, but only require two sources of data: Images, $I_{cam}$, and DenseTact depth images, $D_{DT}$ which form dataset $\mathcal{D}$.

    Within each `touchnerf_<rgb_depth|touch>..` folder are the following folders and contents:

    `color/` folder: contains a `transforms_train.json` which contains the transform (from world frame to camera frame) of each camera and the image path. It also contains a `train/` folder with all of the images in it. 

    Sample image:

    ![Local Image](data/real/touchnerf_092723/touchnerf_rgbdepth_092723/color/train/c_0.jpg)


    `depth/` folder:

    `recorded_path/` folder: not used. But you can check it out.

    `touch/` folder:

    `touch_raw/` folder:



3. Data Preprocessing

    Contains the code for processing RGB images and touches with the DenseTact.


4. 


## Misc

1. The `legacy` folder contains prior code with a different approach that we had.

2. The `utils` folder contains 

## Environment Setup

We use conda to run our experiments.


## Running the Pipeline

Running the pipeline end to end consists of the following steps:

1. Collect Data



## Datasets 

The datasets consist of two components - one for collecting RGB and depth images, and another for gathering touch image information.

The folder named 'touchnerf_rgbdepth' houses the captured color images and depth images. The corresponding transformation for each image is found in the 'transforms_train.json' file. For additional camera parameters, please refer to the 'multicamera.py' file in the 'simulated_data' branch.

Here is the link to the dataset:

https://drive.google.com/drive/folders/1T0hKlzefZOtlZaseSx5uLss1g5FmX8bv?usp=sharing

