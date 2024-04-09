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

"""
Nerfacto augmented with depth supervision.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Tuple, Type

import numpy as np 
import torch
import matplotlib.pyplot as plt

from nerfstudio.cameras.rays import RayBundle
from nerfstudio.cameras.cameras import Cameras


from nerfstudio.model_components import losses
from nerfstudio.model_components.losses import DepthLossType, depth_loss, depth_ranking_loss, basic_depth_loss, depth_uncertainty_weighted_loss
from nerfstudio.models.nerfacto import NerfactoModel, NerfactoModelConfig

from nerfstudio.utils import colormaps

from nerfstudio.models.gaussian_splatting import GaussianSplattingModelConfig, GaussianSplattingModel


@dataclass
class DepthGaussianSplattingModelConfig(GaussianSplattingModelConfig):
    """Additional parameters for depth supervision."""

    _target: Type = field(default_factory=lambda: DepthGaussianSplattingModel)
    depth_loss_mult: float = 0.1
    """Lambda of the depth loss."""
    is_euclidean_depth: bool = False
    """Whether input depth maps are Euclidean distances (or z-distances)."""
    depth_sigma: float = 0.01
    """Uncertainty around depth values in meters (defaults to 1cm)."""
    should_decay_sigma: bool = False
    """Whether to exponentially decay sigma."""
    starting_depth_sigma: float = 0.2
    """Starting uncertainty around depth values in meters (defaults to 0.2m)."""
    sigma_decay_rate: float = 0.99985
    """Rate of exponential decay."""
    depth_loss_type: DepthLossType = DepthLossType.SIMPLE_LOSS
    """Depth loss type."""
    
class DepthGaussianSplattingModel(GaussianSplattingModel):
    """Depth loss augmented splatfacto model.

    Args:
        config: Splatfacto configuration to instantiate model
    """
    config: DepthGaussianSplattingModelConfig
    
    def _get_sigma(self):
        if not self.config.should_decay_sigma:
            return self.depth_sigma

        self.depth_sigma = torch.maximum(
            self.config.sigma_decay_rate * self.depth_sigma, torch.tensor([self.config.depth_sigma])
        )
        return self.depth_sigma

    
    def populate_modules(self):
        """Set the fields and modules."""
        super().populate_modules()
        
        self.moving_depth_loss_average = []
        
        self.max_window_length = 100
        
        self.iter = 0

        if self.config.should_decay_sigma:
            self.depth_sigma = torch.tensor([self.config.starting_depth_sigma])
        else:
            self.depth_sigma = torch.tensor([self.config.depth_sigma])
            
    def get_outputs(self, camera: Cameras):
        outputs = super().get_outputs(camera)
        return outputs
    
    
    def get_metrics_dict(self, outputs, batch):
        metrics_dict = super().get_metrics_dict(outputs, batch)
        if self.training:
            if (
                losses.FORCE_PSEUDODEPTH_LOSS
                and self.config.depth_loss_type not in losses.PSEUDODEPTH_COMPATIBLE_LOSSES
            ):
                raise ValueError(
                    f"Forcing pseudodepth loss, but depth loss type ({self.config.depth_loss_type}) must be one of {losses.PSEUDODEPTH_COMPATIBLE_LOSSES}"
                )
            if self.config.depth_loss_type in (DepthLossType.SIMPLE_LOSS,):
                metrics_dict["depth_loss"] = torch.Tensor([0.0]).to(self.device)
                
                termination_depth = batch["depth_image"].to(self.device)
                metrics_dict["depth_loss"] = basic_depth_loss(
                    termination_depth, outputs["depth"])
                
            elif self.config.depth_loss_type in (DepthLossType.DEPTH_UNCERTAINTY_WEIGHTED_LOSS,):
                metrics_dict["depth_loss"] = torch.Tensor([0.0]).to(self.device)
                termination_depth = batch["depth_image"].to(self.device)
                termination_uncertainty = batch["depth_uncertainty"].to(self.device)
                
                metrics_dict["depth_loss"] = depth_uncertainty_weighted_loss(
                    None, termination_depth, outputs["depth"], termination_uncertainty, None, None, uncertainty_weight=1)
                
            elif self.config.depth_loss_type in (DepthLossType.SPARSENERF_RANKING,):
                metrics_dict["depth_ranking"] = depth_ranking_loss(
                    outputs["depth"], batch["depth_image"].to(self.device)
                )
            else:
                raise NotImplementedError(f"Unknown depth loss type {self.config.depth_loss_type}")
        return metrics_dict
    
    def get_loss_dict(self, outputs, batch, metrics_dict=None):
        loss_dict = super().get_loss_dict(outputs, batch, metrics_dict)
        if self.training:
            assert metrics_dict is not None and ("depth_loss" in metrics_dict or "depth_ranking" in metrics_dict)
            if "depth_ranking" in metrics_dict:
                loss_dict["depth_ranking"] = (
                    self.config.depth_loss_mult
                    * np.interp(self.step, [0, 2000], [0, 0.2])
                    * metrics_dict["depth_ranking"]
                )
            self.moving_depth_loss_average.append(metrics_dict["depth_loss"])
            self.iter += 1
                
            if self.iter % 2 == 0:
                # print(average)
                self.iter = 0
            if len(self.moving_depth_loss_average) > self.max_window_length:
                self.moving_depth_loss_average.pop(0)
            
            if "depth_loss" in metrics_dict:
                loss_dict["depth_loss"] = self.config.depth_loss_mult * metrics_dict["depth_loss"]
        if self.config.depth_loss_mult >= 0.005:
            self.config.depth_loss_mult = max(0.005, self.config.depth_loss_mult * 0.9995)
        return loss_dict
    
    
    def get_image_metrics_and_images(
        self, outputs: Dict[str, torch.Tensor], batch: Dict[str, torch.Tensor]
    ) -> Tuple[Dict[str, float], Dict[str, torch.Tensor]]:
        """Appends ground truth depth to the depth image."""
        
        is_real_world = batch["is_real_world"]
        
        if not is_real_world:
        
            scale = 0.25623789273
            metrics, images = super().get_image_metrics_and_images(outputs, batch)
            
            supervised_depth = batch["depth_image"].to(self.device) / scale
            
            outputs["depth"] = outputs["depth"] / scale
            
            if supervised_depth.shape[1] == 899:
                supervised_depth = supervised_depth[:548, :898, :]
            
            print(supervised_depth.shape, outputs["depth"].shape, 'supervised_depth')
            
            supervised_depth_mask = supervised_depth > 0
            metrics["supervised_depth_mse"] = float(
                torch.nn.functional.mse_loss(outputs["depth"][supervised_depth_mask], supervised_depth[supervised_depth_mask]).cpu()
            ) / 7.27
            
            if "gt_object_depth_image" in batch and "gt_depth_image" in batch:
            
                gt_depth = batch["gt_depth_image"].to(self.device)
                
                gt_object_depth = batch["gt_object_depth_image"].to(self.device)
                
                print(gt_depth.shape, gt_object_depth.shape)
                
                depth_mask = gt_depth > 0
                metrics["gt_depth_mse"] = float(
                    torch.nn.functional.mse_loss(outputs["depth"][depth_mask], gt_depth[depth_mask]).cpu()
                ) / 7.27
                
                object_depth_mask = gt_object_depth > 0
                metrics["gt_object_depth_mse"] = float(
                    torch.nn.functional.mse_loss(outputs["depth"][object_depth_mask], gt_object_depth[object_depth_mask]).cpu()
                ) / 7.27
        else:
            metrics, images = super().get_image_metrics_and_images(outputs, batch)
            
            supervised_depth = batch["depth_image"].to(self.device)
            
            outputs["depth"] = outputs["depth"]
            
            supervised_depth_mask = supervised_depth > 0
            metrics["supervised_depth_mse"] = float(
                torch.nn.functional.mse_loss(outputs["depth"][supervised_depth_mask], supervised_depth[supervised_depth_mask]).cpu()
            )
            
        return metrics, images