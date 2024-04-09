# Copyright 2022 the Regents of the University of California, Nerfstudio Team and contributors. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Data parser for blender dataset"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Type

import imageio
import numpy as np
import torch

from nerfstudio.cameras.cameras import Cameras, CameraType
from nerfstudio.data.dataparsers.base_dataparser import (
    DataParser,
    DataParserConfig,
    DataparserOutputs,
)
from nerfstudio.data.scene_box import SceneBox
from nerfstudio.utils.colors import get_color
from nerfstudio.utils.io import load_from_json

import os
import shutil
import json
os.environ["OPENCV_IO_ENABLE_OPENEXR"]="1"


@dataclass
class BlenderDataParserConfig(DataParserConfig):
    """Blender dataset parser config"""

    _target: Type = field(default_factory=lambda: Blender)
    """target class to instantiate"""
    data: Path = Path("data/blender/lego")
    """Directory specifying location of data."""
    scale_factor: float = 1.0
    """How much to scale the camera origins by."""
    alpha_color: str = "white"
    """alpha color of background"""
    depth_unit_scale_factor: float = 1e-3
    """Scales the depth values to meters. Default value is 0.001 for a millimeter to meter conversion."""


@dataclass
class Blender(DataParser):
    """Blender Dataset
    Some of this code comes from https://github.com/yenchenlin/nerf-pytorch/blob/master/load_blender.py#L37.
    """

    config: BlenderDataParserConfig

    def __init__(self, config: BlenderDataParserConfig):
        super().__init__(config=config)
        self.data: Path = config.data
        self.scale_factor: float = config.scale_factor
        self.alpha_color = config.alpha_color

    def _add_depths_to_transform_file(self, path, split):
        print("Adding depths to transform file")
        print(str(path))
        transforms_filename = str(path)
        with open(transforms_filename, "r") as json_file:
            data = json.load(json_file)

        frames = data['frames']
        filenames = os.listdir(f'{self.data}/depths')
        idx_to_depth_filenames = {}

        for filename in filenames:
            idx = int(filename.split('.')[0][-4:])
            idx_to_depth_filenames[idx] = filename

        new_frames = []
        img_idx = 1
        for frame in frames:
            new_frame = frame
            new_fname = idx_to_depth_filenames[img_idx]
            new_frame['depth_filename'] = f'depths/{new_fname}'

            img_idx += 1

            new_frames.append(new_frame) 

        data['frames'] = new_frames

        with open(transforms_filename, "w") as jsonFile:
            json.dump(data, jsonFile, indent=2)

        # copy over files to val or test
        if split != 'train':
            root_dir = str(self.data)
            new_path = os.path.join(root_dir, split)
            train_path = os.path.join(root_dir, 'train')

            if not os.path.exists(new_path):
                shutil.copytree(train_path, new_path)



    def _generate_dataparser_outputs(self, split="train"):
        if self.alpha_color is not None:
            alpha_color_tensor = get_color(self.alpha_color)
        else:
            alpha_color_tensor = None
        if split == 'val':
            split = 'test'

        # add depth filenames to transforms file
        # self._add_depths_to_transform_file(self.data / f"transforms_{split}.json", split)
        
        meta = load_from_json(self.data / f"transforms_{split}.json")

        image_filenames = []
        depth_filenames = []
        poses = []
        positions = []
        for frame in meta["frames"]:
            fname = self.data / Path(frame["file_path"].replace("./", ""))
            image_filenames.append(fname)

            depth_fname = self.data / Path(frame["depth_filename"].replace("./", ""))
            depth_filenames.append(depth_fname)

            poses.append(np.array(frame["transform_matrix"]))
            # get position to move frames
        poses = np.array(poses).astype(np.float32)

        img_0 = imageio.v2.imread(image_filenames[0])
        image_height, image_width = img_0.shape[:2]
        camera_angle_x = float(meta["camera_angle_x"])
        focal_length = 0.5 * image_width / np.tan(0.5 * camera_angle_x)

        cx = image_width / 2.0
        cy = image_height / 2.0
        camera_to_world = torch.from_numpy(poses[:, :3])  # camera to world transform

        # in x,y,z order
        camera_to_world[..., 3] *= self.scale_factor
        scene_box = SceneBox(aabb=torch.tensor([[-1.5, -1.5, -1.5], [1.5, 1.5, 1.5]], dtype=torch.float32))

        cameras = Cameras(
            camera_to_worlds=camera_to_world,
            fx=focal_length,
            fy=focal_length,
            cx=cx,
            cy=cy,
            camera_type=CameraType.PERSPECTIVE,
        )

        # get depth filenames by looking into the depth folder.
        # meta = load_from_json(self.data / f"transforms_{split}.json")

        dataparser_outputs = DataparserOutputs(
            image_filenames=image_filenames,
            cameras=cameras,
            alpha_color=alpha_color_tensor,
            scene_box=scene_box,
            dataparser_scale=self.scale_factor,
            metadata={
                "depth_filenames": depth_filenames if len(depth_filenames) > 0 else None,
                "depth_unit_scale_factor": self.config.depth_unit_scale_factor
            }
        )

        return dataparser_outputs
