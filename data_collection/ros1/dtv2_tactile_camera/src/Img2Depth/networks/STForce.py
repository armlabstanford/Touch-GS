import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision.models as models  

# from .swin_transformer import SwinTransformer
from .swin_transformer_force import SwinTransformerV2 as SwinTransformer
########################################################################################################################


class STForce(nn.Module):
    """
    Force network based on swin transformer architecture.
    """
    def __init__(self, version=None, inv_depth=False, pretrained=None, 
                    frozen_stages=-1, **kwargs):
        super().__init__()

        self.inv_depth = inv_depth
        self.with_auxiliary_head = False
        self.with_neck = False

        norm_cfg = dict(type='BN', requires_grad=True)
        # norm_cfg = dict(type='GN', requires_grad=True, num_groups=8)

        window_size = int(version[-2:])

        if version[:-2] == 'base':
            embed_dim = 128
            depths = [2, 2, 18, 2]
            num_heads = [4, 8, 16, 32]
            in_channels = [128, 256, 512, 1024]
        elif version[:-2] == 'large':
            embed_dim = 192
            depths = [2, 2, 18, 2]
            num_heads = [6, 12, 24, 48]
            in_channels = [192, 384, 768, 1536]

        elif version[:-2] == 'largedense':
            embed_dim = 192
            depths = [2, 2, 18, 2]
            num_heads = [6, 12, 24, 48]
            in_channels = [192, 384, 768, 1536]
        elif version[:-2] == 'tiny':
            embed_dim = 96
            depths = [2, 2, 6, 2]
            num_heads = [3, 6, 12, 24]
            in_channels = [96, 192, 384, 768]


        # for force implementation
        embed_dim = 192
        depths = [2, 2, 18, 2]
        num_heads = [6, 12, 24, 48]
        window_size = 20

        backbone_cfg = dict(
            img_size=640,
            embed_dim=embed_dim, # 192
            depths=depths, # 2,2,18,2
            num_heads=num_heads, # 6,12,24,48
            window_size=window_size, # 07 to 20
            # num_classes=6,
            ape=False,
            drop_path_rate=0.3,
            patch_norm=True,
            use_checkpoint=False,
            frozen_stages=frozen_stages
        )

        embed_dim = 512
        decoder_cfg = dict(
            in_channels=in_channels,
            in_index=[0, 1, 2, 3],
            pool_scales=(1, 2, 3, 6),
            channels=embed_dim,
            dropout_ratio=0.0,
            num_classes=32,
            norm_cfg=norm_cfg,
            align_corners=False
        )

        self.backbone = SwinTransformer(**backbone_cfg)
        win = 7

        self.force_head = ForceHead(input_dim=1000, output_dim = 6)

        self.up_mode = 'bilinear'
        if self.up_mode == 'mask':
            self.mask_head = nn.Sequential(
                nn.ReLU(inplace=True),
                nn.Conv2d(64, 16*9, 1, padding=0))


        self.init_weights(pretrained=pretrained)

    def init_weights(self, pretrained=None):
        """Initialize the weights in backbone and heads.

        Args:
            pretrained (str, optional): Path to pre-trained weights.
                Defaults to None.
        """
        print(f'== Load encoder backbone from: {pretrained}')
        self.backbone.init_weights(pretrained=pretrained)
        if self.with_auxiliary_head:
            if isinstance(self.auxiliary_head, nn.ModuleList):
                for aux_head in self.auxiliary_head:
                    aux_head.init_weights()
            else:
                self.auxiliary_head.init_weights()


    def forward(self, imgs):

        feats = self.backbone(imgs)
        if self.with_neck:
            feats = self.neck(feats)
        # print(feats.size())
        depth =self.force_head(feats)

        return depth
class DenseNet_Force(nn.Module):
    def __init__(self, pretrained=False):
        super(DenseNet_Force, self).__init__()
        # # self.norm1 = nn.BatchNorm2d(input_dim)
        # self.conv1 = nn.Conv2d(input_dim, 1, 3, padding=1)
        # # self.relu = nn.ReLU(inplace=True)
        # self.sigmoid = nn.Sigmoid()

        # for 6 outputs
        self.encoder = Encoder_force(encoder_pretrained= pretrained)
        #use output as 
        self.force_head = ForceHead(input_dim=1000, output_dim = 6)


    def forward(self, x):
        # # x = self.relu(self.norm1(x))
        # x = self.sigmoid(self.conv1(x))
        x1 = self.encoder(x)
        x =self.force_head(x1)

        return x

class DenseNet_Force_tf(nn.Module):
    def __init__(self, pretrained=False):
        super(DenseNet_Force_tf, self).__init__()

        # for 6 outputs
        self.encoder = Encoder_force_tf(encoder_pretrained= pretrained)
        #use output as 
        self.force_head = ForceHead(input_dim=1000, output_dim = 6)


    def forward(self, x):
        # # x = self.relu(self.norm1(x))
        # x = self.sigmoid(self.conv1(x))
        x1 = self.encoder(x)
        x =self.force_head(x1)

        return x


class Encoder_force_tf(nn.Module):
    """
    input: W x H x 6 (usually W = 640), where first WxHx0:3 is image from dataloader and WxHx 3:6 is base image 

    """
    def __init__(self, encoder_pretrained=False):
        super(Encoder_force_tf, self).__init__()
        self.densenet = models.densenet161(pretrained=encoder_pretrained)

        self.conv1 = nn.Conv2d(6,4, 3, 1, 1)
        self.batch1 = nn.BatchNorm2d(4)
        self.relu = nn.ReLU(inplace = True)

        self.conv2 = nn.Conv2d(4,3, 3, 1, 1)
        self.batch2 = nn.BatchNorm2d(3)
        self.relu = nn.ReLU(inplace = True)

    def forward(self, x):

        # feature_maps = [x]

        # for key, value in self.densenet.features._modules.items():
        #     feature_maps.append(value(feature_maps[-1]))
        x1 = self.relu(self.batch1(self.conv1(x)))
        x2 = self.relu(self.batch2(self.conv2(x1)))

        # print(x2.size())

        x = self.densenet(x2)
        return x 

class Encoder_force(nn.Module):
    def __init__(self, encoder_pretrained=False):
        super(Encoder_force, self).__init__()
        self.densenet = models.densenet161(pretrained=encoder_pretrained)

    def forward(self, x):

        # feature_maps = [x]

        # for key, value in self.densenet.features._modules.items():
        #     feature_maps.append(value(feature_maps[-1]))
        x = self.densenet(x)

        return x 
class ForceHead(nn.Module):
    def __init__(self, input_dim=100, output_dim = 6):
        super(ForceHead, self).__init__()
        # # self.norm1 = nn.BatchNorm2d(input_dim)
        # self.conv1 = nn.Conv2d(input_dim, 1, 3, padding=1)
        # # self.relu = nn.ReLU(inplace=True)
        # self.sigmoid = nn.Sigmoid()

        # for 6 outputs
        self.fc1 = nn.Linear(input_dim, int(input_dim/2))
        self.fc2 = nn.Linear(int(input_dim/2), output_dim)

    def forward(self, x):
        # # x = self.relu(self.norm1(x))
        # x = self.sigmoid(self.conv1(x))

        x = self.fc2(self.fc1(x))

        return x
