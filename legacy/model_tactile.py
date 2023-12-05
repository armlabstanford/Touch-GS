# Copyright 2022 The Nerfstudio Team. All rights reserved.
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
Nerfacto augmented with depth supervision where training can be done without RGB images.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, Tuple, Type

import torch

from nerfstudio.cameras.rays import RayBundle
from nerfstudio.model_components.losses import (
    DepthLossType, 
    depth_loss,
    MSELoss,
    distortion_loss,
    interlevel_loss,
)
from nerfstudio.models.nerfacto import NerfactoModel, NerfactoModelConfig
from nerfstudio.utils import colormaps


@dataclass
class TactileNerfConfig(NerfactoModelConfig):
    """Additional parameters for depth supervision."""

    _target: Type = field(default_factory=lambda: TactileModel)
    depth_loss_mult: float = 1e-3
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
    depth_loss_type: DepthLossType = DepthLossType.DS_NERF
    """Depth loss type."""


class TactileModel(NerfactoModel):
    """Depth loss augmented nerfacto model.

    Args:
        config: Nerfacto configuration to instantiate model
    """

    config: TactileNerfConfig

    def populate_modules(self):
        """Set the fields and modules."""
        super().populate_modules()

        if self.config.should_decay_sigma:
            self.depth_sigma = torch.tensor([self.config.starting_depth_sigma])
        else:
            self.depth_sigma = torch.tensor([self.config.depth_sigma])

    def get_outputs(self, ray_bundle: RayBundle):
        outputs = super().get_outputs(ray_bundle)
        if ray_bundle.metadata is not None and "directions_norm" in ray_bundle.metadata:
            outputs["directions_norm"] = ray_bundle.metadata["directions_norm"]
        return outputs

    # def get_metrics_dict(self, outputs, batch):
    #     metrics_dict = super().get_metrics_dict(outputs, batch)
    #     if self.training:
    #         metrics_dict["depth_loss"] = 0.0
    #         sigma = self._get_sigma().to(self.device)
    #         termination_depth = batch["depth_image"].to(self.device)
    #         for i in range(len(outputs["weights_list"])):
    #             metrics_dict["depth_loss"] += depth_loss(
    #                 weights=outputs["weights_list"][i],
    #                 ray_samples=outputs["ray_samples_list"][i],
    #                 termination_depth=termination_depth,
    #                 predicted_depth=outputs["depth"],
    #                 sigma=sigma,
    #                 directions_norm=outputs["directions_norm"],
    #                 is_euclidean=self.config.is_euclidean_depth,
    #                 depth_loss_type=self.config.depth_loss_type,
    #             ) / len(outputs["weights_list"])

    #     return metrics_dict
    
    def get_metrics_dict(self, outputs, batch):
        metrics_dict = {}

        if "image" in batch:
            image = batch["image"].to(self.device)
            metrics_dict["psnr"] = self.psnr(outputs["rgb"], image)
        
        if self.training:
            metrics_dict["distortion"] = distortion_loss(outputs["weights_list"], outputs["ray_samples_list"])

            metrics_dict["depth_loss"] = 0.0
            sigma = self._get_sigma().to(self.device)
            termination_depth = batch["depth_image"].to(self.device)
            for i in range(len(outputs["weights_list"])):
                metrics_dict["depth_loss"] += depth_loss(
                    weights=outputs["weights_list"][i],
                    ray_samples=outputs["ray_samples_list"][i],
                    termination_depth=termination_depth,
                    predicted_depth=outputs["depth"],
                    sigma=sigma,
                    directions_norm=outputs["directions_norm"],
                    is_euclidean=self.config.is_euclidean_depth,
                    depth_loss_type=self.config.depth_loss_type,
                ) / len(outputs["weights_list"])

        return metrics_dict

    # def get_loss_dict(self, outputs, batch, metrics_dict=None):
    #     loss_dict = super().get_loss_dict(outputs, batch, metrics_dict)
    #     if self.training:
    #         assert metrics_dict is not None and "depth_loss" in metrics_dict
    #         loss_dict["depth_loss"] = self.config.depth_loss_mult * metrics_dict["depth_loss"]

    #     return loss_dict
    
    def get_loss_dict(self, outputs, batch, metrics_dict=None):
        loss_dict = {}
        if "image" in batch:
            image = batch["image"].to(self.device)
            loss_dict["rgb_loss"] = self.rgb_loss(image, outputs["rgb"])
            
        if self.training:
            loss_dict["interlevel_loss"] = self.config.interlevel_loss_mult * interlevel_loss(
                outputs["weights_list"], outputs["ray_samples_list"]
            )
            assert metrics_dict is not None and "distortion" in metrics_dict
            loss_dict["distortion_loss"] = self.config.distortion_loss_mult * metrics_dict["distortion"]
            if self.config.predict_normals:
                # orientation loss for computed normals
                loss_dict["orientation_loss"] = self.config.orientation_loss_mult * torch.mean(
                    outputs["rendered_orientation_loss"]
                )

                # ground truth supervision for normals
                loss_dict["pred_normal_loss"] = self.config.pred_normal_loss_mult * torch.mean(
                    outputs["rendered_pred_normal_loss"]
                )

            assert metrics_dict is not None and "depth_loss" in metrics_dict
            loss_dict["depth_loss"] = self.config.depth_loss_mult * metrics_dict["depth_loss"]
        
        return loss_dict
    


    # def get_image_metrics_and_images(
    #     self, outputs: Dict[str, torch.Tensor], batch: Dict[str, torch.Tensor]
    # ) -> Tuple[Dict[str, float], Dict[str, torch.Tensor]]:
    #     """Appends ground truth depth to the depth image."""
    #     metrics, images = super().get_image_metrics_and_images(outputs, batch)
    #     ground_truth_depth = batch["depth_image"]
    #     if not self.config.is_euclidean_depth:
    #         ground_truth_depth = ground_truth_depth * outputs["directions_norm"]

    #     ground_truth_depth_colormap = colormaps.apply_depth_colormap(ground_truth_depth)
    #     predicted_depth_colormap = colormaps.apply_depth_colormap(
    #         outputs["depth"],
    #         accumulation=outputs["accumulation"],
    #         near_plane=torch.min(ground_truth_depth),
    #         far_plane=torch.max(ground_truth_depth),
    #     )
    #     images["depth"] = torch.cat([ground_truth_depth_colormap, predicted_depth_colormap], dim=1)
    #     depth_mask = ground_truth_depth > 0
    #     metrics["depth_mse"] = torch.nn.functional.mse_loss(
    #         outputs["depth"][depth_mask], ground_truth_depth[depth_mask]
    #     )
    #     return metrics, images
    

    def get_image_metrics_and_images(
        self, outputs: Dict[str, torch.Tensor], batch: Dict[str, torch.Tensor]
    ) -> Tuple[Dict[str, float], Dict[str, torch.Tensor]]:
        
        acc = colormaps.apply_colormap(outputs["accumulation"])
        depth = colormaps.apply_depth_colormap(
            outputs["depth"],
            accumulation=outputs["accumulation"],
        )

        
        combined_acc = torch.cat([acc], dim=1)
        combined_depth = torch.cat([depth], dim=1)

        ground_truth_depth = batch["depth_image"]
        if not self.config.is_euclidean_depth:
            ground_truth_depth = ground_truth_depth * outputs["directions_norm"]

        ground_truth_depth_colormap = colormaps.apply_depth_colormap(ground_truth_depth)
        predicted_depth_colormap = colormaps.apply_depth_colormap(
            outputs["depth"],
            accumulation=outputs["accumulation"],
            near_plane=torch.min(ground_truth_depth),
            far_plane=torch.max(ground_truth_depth),
        )

        if "image" in batch:
            image = batch["image"].to(self.device)
            rgb = outputs["rgb"]
            combined_rgb = torch.cat([image, rgb], dim=1)

            # Switch images from [H, W, C] to [1, C, H, W] for metrics computations
            image = torch.moveaxis(image, -1, 0)[None, ...]
            rgb = torch.moveaxis(rgb, -1, 0)[None, ...]

            psnr = self.psnr(image, rgb)
            ssim = self.ssim(image, rgb)
            lpips = self.lpips(image, rgb)

            # all of these metrics will be logged as scalars
            metrics_dict = {"psnr": float(psnr.item()), "ssim": float(ssim)}  # type: ignore
            metrics_dict["lpips"] = float(lpips)

            images_dict = {"img": combined_rgb, "accumulation": combined_acc, "depth": combined_depth}

            for i in range(self.config.num_proposal_iterations):
                key = f"prop_depth_{i}"
                prop_depth_i = colormaps.apply_depth_colormap(
                    outputs[key],
                    accumulation=outputs["accumulation"],
                )
                images_dict[key] = prop_depth_i

            return metrics_dict, images_dict



    def _get_sigma(self):
        if not self.config.should_decay_sigma:
            return self.depth_sigma

        self.depth_sigma = torch.maximum(  # pylint: disable=attribute-defined-outside-init
            self.config.sigma_decay_rate * self.depth_sigma, torch.tensor([self.config.depth_sigma])
        )
        return self.depth_sigma

