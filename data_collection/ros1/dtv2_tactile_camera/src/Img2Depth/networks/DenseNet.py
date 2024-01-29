import torch  
import torch.nn as nn  
import torch.nn.functional as F 
import torchvision.models as models  
import numpy as np
#9g1d7rmx
#need to change input into 570x570, now 640x480
#              output into 570x570, now 320x240

#



class TotalNet(nn.Module):
    def __init__(self, input_channels = 570, output_channels = 570):

        super(TotalNet, self).__init__()

        self.block = AddTwo()

    def forward(self, x):

        x = self.block(x)
        x = self.block(x)

        return x




class AddTwo(nn.Module):
    def __init__(self, input_channels = 570, output_channels = 570):

        super(AddTwo, self).__init__()

        self.encoderdecoder = EncDoc_allrelu_deeper()
        self.localconv = LocalNet()

    def forward(self, x):

        x1 = self.encoderdecoder(x)
        x2 = self.localconv(x)

        return x1+x2




class LocalNet(nn.Module):
    def __init__(self, input_channels = 570):

        super(LocalNet, self).__init__()
        # super(self).__init__()

        self.layer = nn.Sequential(
            # input is (3) x 570 x 570

            # input is (3) x 570 x 570
            nn.Conv2d(3, 8, 11, stride=1, padding=5), 
            nn.LeakyReLU(0.2, inplace=True),
            # state size. 3 x 570x570
            nn.BatchNorm2d(8),
            nn.Conv2d(8, 16, 9, stride=1, padding=4), 
            nn.LeakyReLU(0.2, inplace=True),
            # state size. 3 x 570x570
            nn.BatchNorm2d(16),
            nn.Conv2d(16, 16, 7, stride=1, padding=3), 
            nn.LeakyReLU(0.2, inplace=True),
            nn.Conv2d(16, 8, 5, stride=1, padding=2), 
            nn.LeakyReLU(0.2, inplace=True),
            nn.Conv2d(8, 4, 3, stride=1, padding=1), 
            nn.LeakyReLU(0.2, inplace=True),
            nn.BatchNorm2d(4),
            nn.Conv2d(4, 1, 3, stride=1, padding=1), 
            nn.LeakyReLU(0.2, inplace=True),
        )
        
    def forward(self, x):

        return self.layer(x)



class EncDoc_allrelu_deeper(nn.Module):
    def __init__(self, input_channels = 570, output_channels = 570):
        super(EncDoc_allrelu_deeper, self).__init__()
        # super(self).__init__()
        first_omega_0 = 30
        self.layer = nn.Sequential(

            nn.Conv2d(3, 8, 7, 3, 2),

            nn.LeakyReLU(0.2, inplace=True),
            nn.Conv2d(8, 16, 7, 3, 3),
            nn.LeakyReLU(0.2, inplace=True),

            nn.Conv2d(16, 32, 5,3, 2),
            nn.LeakyReLU(0.2, inplace=True),

            nn.BatchNorm2d(32),
            nn.Conv2d(32, 64,  3,  1,  1),
            nn.LeakyReLU(0.2, inplace=True),

            nn.Conv2d(64, 128, 3,  1,  1),
            nn.LeakyReLU(0.2, inplace=True),
            nn.Conv2d(128, 256, 3,  1,  1),
            nn.LeakyReLU(0.2, inplace=True),
            nn.Conv2d(256, 256, 3,  1,  1),
            nn.LeakyReLU(0.2, inplace=True),

            nn.BatchNorm2d(256),

            nn.ConvTranspose2d(256, 128, 3,  1,  1),
            nn.LeakyReLU(0.2, inplace=True),

            nn.ConvTranspose2d(128, 64, 3, stride = 1, padding= 1),
            nn.LeakyReLU(0.2, inplace=True),
            nn.ConvTranspose2d(64, 32, 5, stride = 3, padding= 2),
            nn.LeakyReLU(0.2, inplace=True),
            nn.ConvTranspose2d(32, 16, 7, stride = 3, padding= 3),
            nn.LeakyReLU(0.2, inplace=True),
            nn.ConvTranspose2d(16, 8, 7, stride = 3, padding= 2),
            nn.LeakyReLU(0.2, inplace=True),
            nn.BatchNorm2d(8),
            nn.ConvTranspose2d(8, 4, 3, stride = 1, padding= 1),
            nn.LeakyReLU(0.2, inplace=True),
            nn.ConvTranspose2d(4, 1, 3, stride = 1, padding= 1),
            nn.LeakyReLU(0.2, inplace=True),


        )


    def forward(self, x):

        return self.layer(x)




class EncDoc_allrelu(nn.Module):
    def __init__(self, input_channels = 570, output_channels = 570):

        super(EncDoc_allrelu, self).__init__()
        # super(self).__init__()
        first_omega_0 = 30
        self.layer = nn.Sequential(

            nn.Conv2d(3, 8, 7, 3, 2),

            nn.LeakyReLU(0.2, inplace=True),
            nn.Conv2d(8, 16, 7, 3, 3),
            nn.LeakyReLU(0.2, inplace=True),

            nn.Conv2d(16, 32, 5,3, 2),
            nn.LeakyReLU(0.2, inplace=True),

            nn.BatchNorm2d(32),
            nn.Conv2d(32, 64,  3,  1,  1),
            nn.LeakyReLU(0.2, inplace=True),

            nn.Conv2d(64, 128, 3,  1,  1),
            nn.LeakyReLU(0.2, inplace=True),

            nn.ConvTranspose2d(128, 64, 3, stride = 1, padding= 1),
            nn.LeakyReLU(0.2, inplace=True),
            nn.ConvTranspose2d(64, 32, 5, stride = 3, padding= 2),
            nn.LeakyReLU(0.2, inplace=True),
            nn.ConvTranspose2d(32, 16, 7, stride = 3, padding= 3),
            nn.LeakyReLU(0.2, inplace=True),
            nn.ConvTranspose2d(16, 8, 7, stride = 3, padding= 2),
            nn.LeakyReLU(0.2, inplace=True),
            nn.BatchNorm2d(8),
            nn.ConvTranspose2d(8, 4, 3, stride = 1, padding= 1),
            nn.LeakyReLU(0.2, inplace=True),
            nn.ConvTranspose2d(4, 1, 3, stride = 1, padding= 1),
            nn.LeakyReLU(0.2, inplace=True),


        )


    def forward(self, x):

        return self.layer(x)



class EncDoc_onesinelayer(nn.Module):
    def __init__(self, input_channels = 570, output_channels = 570):

        super(EncDoc_onesinelayer, self).__init__()
        # super(self).__init__()
        first_omega_0 = 30
        self.layer = nn.Sequential(

            SineLayerConv(3, 8, kern = 7, strd = 3, pad = 2, is_first=True, omega_0=first_omega_0),

            nn.LeakyReLU(0.2, inplace=True),
            nn.Conv2d(8, 16, 7, 3, 3),
            nn.LeakyReLU(0.2, inplace=True),

            nn.Conv2d(16, 32, 5,3, 2),
            nn.LeakyReLU(0.2, inplace=True),

            nn.BatchNorm2d(32),
            nn.Conv2d(32, 64,  3,  1,  1),
            nn.LeakyReLU(0.2, inplace=True),

            nn.Conv2d(64, 128, 3,  1,  1),
            nn.LeakyReLU(0.2, inplace=True),

            nn.ConvTranspose2d(128, 64, 3, stride = 1, padding= 1),
            nn.LeakyReLU(0.2, inplace=True),
            nn.ConvTranspose2d(64, 32, 5, stride = 3, padding= 2),
            nn.LeakyReLU(0.2, inplace=True),
            nn.ConvTranspose2d(32, 16, 7, stride = 3, padding= 3),
            nn.LeakyReLU(0.2, inplace=True),
            nn.ConvTranspose2d(16, 8, 7, stride = 3, padding= 2),
            nn.LeakyReLU(0.2, inplace=True),
            nn.BatchNorm2d(8),
            nn.ConvTranspose2d(8, 4, 3, stride = 1, padding= 1),
            nn.LeakyReLU(0.2, inplace=True),
            nn.ConvTranspose2d(4, 1, 3, stride = 1, padding= 1),
            nn.LeakyReLU(0.2, inplace=True),


        )


    def forward(self, x):

        return self.layer(x)




class EncDoc(nn.Module):
    def __init__(self, input_channels = 570, output_channels = 570):

        super(EncDoc, self).__init__()
        # super(self).__init__()
        first_omega_0 = 30
        self.layer = nn.Sequential(

            SineLayerConv(3, 8, kern = 7, strd = 3, pad = 2, is_first=True, omega_0=first_omega_0),

            nn.LeakyReLU(0.2, inplace=True),
            SineLayerConv(8, 16, kern = 7, strd = 3, pad = 3, is_first=False),
            SineLayerConv(16, 32, kern = 5, strd = 3, pad = 2, is_first=False),
            nn.BatchNorm2d(32),
            SineLayerConv(32, 64, kern = 3, strd = 1, pad = 1, is_first=False),
            SineLayerConv(64, 128, kern = 3, strd = 1, pad = 1, is_first=False),

            nn.ConvTranspose2d(128, 64, 3, stride = 1, padding= 1),
            nn.LeakyReLU(0.2, inplace=True),
            nn.ConvTranspose2d(64, 32, 5, stride = 3, padding= 2),
            nn.LeakyReLU(0.2, inplace=True),
            nn.ConvTranspose2d(32, 16, 7, stride = 3, padding= 3),
            nn.LeakyReLU(0.2, inplace=True),
            nn.ConvTranspose2d(16, 8, 7, stride = 3, padding= 2),
            nn.LeakyReLU(0.2, inplace=True),
            nn.BatchNorm2d(8),
            nn.ConvTranspose2d(8, 4, 3, stride = 1, padding= 1),
            nn.LeakyReLU(0.2, inplace=True),
            nn.ConvTranspose2d(4, 1, 3, stride = 1, padding= 1),
            nn.LeakyReLU(0.2, inplace=True),


        )


    def forward(self, x):

        return self.layer(x)


class ConvNet_withsine(nn.Module):
    def __init__(self, input_channels = 570):

        super(ConvNet_withsine, self).__init__()
        # super(self).__init__()
        first_omega_0 = 30
        self.layer = nn.Sequential(
            # input is (3) x 570 x 570

            # doesn't work, the input should be 5 vector(x,y,r,g,b) for just one pixel, 
            # and the trainin should be done by each pixels 

            SineLayerConv(3, 3, kern = 5, strd = 1, pad = 2, is_first=True, omega_0=first_omega_0),

            # input is (3) x 570 x 570
            nn.Conv2d(3, 6, 7, stride=1, padding=3), 
            nn.LeakyReLU(0.2, inplace=True),
            # state size. 3 x 570x570
            nn.BatchNorm2d(3 * 2),
            SineLayerConv(6, 12, kern = 5, strd = 1, pad = 2, is_first=False), 
            nn.LeakyReLU(0.2, inplace=True),
            # state size. 3 x 570x570
            nn.BatchNorm2d(3 * 4),
            SineLayerConv(12, 12, kern = 5, strd = 1, pad = 2, is_first=False), 
            SineLayerConv(12, 12, kern = 5, strd = 1, pad = 2, is_first=False), 
            SineLayerConv(12,24, kern = 5, strd = 1, pad = 2, is_first=False), 
            SineLayerConv(24, 48, kern = 5, strd = 1, pad = 2, is_first=False), 
            SineLayerConv(48, 48, kern = 5, strd = 1, pad = 2, is_first=False), 
            SineLayerConv(48, 48, kern = 5, strd = 1, pad = 2, is_first=False), 
            SineLayerConv(48, 24, kern = 5, strd = 1, pad = 2, is_first=False), 
            # state size. 3 x 570x570
            SineLayerConv(24, 12, kern = 5, strd = 1, pad = 2, is_first=False), 

            nn.BatchNorm2d(12),
            SineLayerConv(12,3, kern = 7, strd = 1, pad = 3, is_first=False),

            nn.Conv2d(3, 1, 7, stride=1, padding=3), 

            nn.LeakyReLU(0.2, inplace=True),

        )


    def forward(self, x):

        return self.layer(x)

#4 le
class ConvNet(nn.Module):
    def __init__(self, input_channels = 570):

        super(ConvNet, self).__init__()
        # super(self).__init__()
        self.layer = nn.Sequential(
            # input is (3) x 570 x 570
            nn.Conv2d(3, 6, 7, stride=1, padding=3), 
            nn.LeakyReLU(0.2, inplace=True),
            # state size. 3 x 570x570
            nn.BatchNorm2d(3 * 2),
            nn.Conv2d(6, 3, 7, stride=1, padding=3), 
            nn.LeakyReLU(0.2, inplace=True),
            # state size. 3 x 570x570
            nn.BatchNorm2d(3),
            nn.Conv2d(3, 1, 7, stride=1, padding=3), 
            nn.LeakyReLU(0.2, inplace=True),
 48, (6, 12, 36, 24), 96
        )
        # self.net = []
        # SineLayer(in_features, hidden_fea 48, (6, 12, 36, 24), 96tures,is_first=True, omega_0=first_omega_0)
        # sine layer uses fc layer, which is too slow... 
        # self.net.append()
        

    def forward(self, x):

        return self.layer(x)


class SineLayerConv(nn.Module):
    # See paper sec. 3.2, final paragraph, and supplement Sec. 1.5 for discussion of omega_0.
    
    # If is_first=True, omega_0 is a frequency factor which simply multiplies the activations before the 
    # nonlinearity. Different signals may require different omega_0 in the first layer - this is a 
    # hyperparameter.
    
    # If is_first=False, then the weights will be divided by omega_0 so as to keep the magnitude of 
    # activations constant, but boost gradients to the weight matrix (see supplement Sec. 1.5)
    
    def __init__(self, in_features, out_features,kern, strd, pad, bias=True,
                 is_first=False, omega_0=30):
        super().__init__()
        self.omega_0 = omega_0
        self.is_first = is_first
        self.out_features = out_features
        self.in_features = in_features
        self.kern = kern
        self.strd = strd
        self.pad = pad
        # self.linear = nn.Linear(in_features, out_features, bias=bias)
        # instead of using linear layer, how about add the initial weight on the conv2d layer? 
        #for our case, input is 3x570x570 and the kernel size 5 have same output channels by using
        # padding = 2, stride = 1
        self.linear = nn.Conv2d(self.in_features, self.out_features, self.kern, stride=self.strd, padding=self.pad)
        
        self.init_weights()
    
    def init_weights(self):
        with torch.no_grad():
            if self.is_first:
                self.linear.weight.uniform_(-1 / self.in_features, 
                                             1 / self.in_features)      
            else:
                self.linear.weight.uniform_(-np.sqrt(6 / self.in_features) / self.omega_0, 
                                             np.sqrt(6 / self.in_features) / self.omega_0)
        
    def forward(self, input):
        return torch.sin(self.omega_0 * self.linear(input))
    
    def forward_with_intermediate(self, input): 
        # For visualization of activation distributions
        intermediate = self.omega_0 * self.linear(input)
        return torch.sin(intermediate), intermediate



class SineLayer(nn.Module):
    # See paper sec. 3.2, final paragraph, and supplement Sec. 1.5 for discussion of omega_0.
    
    # If is_first=True, omega_0 is a frequency factor which simply multiplies the activations before the 
    # nonlinearity. Different signals may require different omega_0 in the first layer - this is a 
    # hyperparameter.
    
    # If is_first=False, then the weights will be divided by omega_0 so as to keep the magnitude of 
    # activations constant, but boost gradients to the weight matrix (see supplement Sec. 1.5)
    
    def __init__(self, in_features, out_features, bias=True,
                 is_first=False, omega_0=30):
        super().__init__()
        self.omega_0 = omega_0
        self.is_first = is_first
        
        self.in_features = in_features
        self.linear = nn.Linear(in_features, out_features, bias=bias)
        
        self.init_weights()
    
    def init_weights(self):
        with torch.no_grad():
            if self.is_first:
                self.linear.weight.uniform_(-1 / self.in_features, 
                                             1 / self.in_features)      
            else:
                self.linear.weight.uniform_(-np.sqrt(6 / self.in_features) / self.omega_0, 
                                             np.sqrt(6 / self.in_features) / self.omega_0)
        
    def forward(self, input):
        return torch.sin(self.omega_0 * self.linear(input))
    
    def forward_with_intermediate(self, input): 
        # For visualization of activation distributions
        intermediate = self.omega_0 * self.linear(input)
        return torch.sin(intermediate), intermediate



class Encoder(nn.Module):
    def __init__(self, encoder_pretrained=False):
        super(Encoder, self).__init__()
        self.densenet = models.densenet161(pretrained=encoder_pretrained)

    def forward(self, x):

        feature_maps = [x]

        for key, value in self.densenet.features._modules.items():
            feature_maps.append(value(feature_maps[-1]))

        return feature_maps



class Upsample(nn.Module):
    def __init__(self, input_channels, output_channels):

        super(Upsample, self).__init__()

        self.input_channels = input_channels
        self.output_channels = output_channels

        self.convA = nn.Conv2d(input_channels, output_channels, 3, 1, 1)
        self.leakyrelu = nn.LeakyReLU(0.2)
        self.convB = nn.Conv2d(output_channels, output_channels, 3, 1, 1)

    def forward(self, x, concat_with):

        concat_h_dim = concat_with.shape[2]
        concat_w_dim = concat_with.shape[3]

        upsampled_x = F.interpolate(
            x, size=[concat_h_dim, concat_w_dim], mode="bilinear", align_corners=True
        )
        upsampled_x = torch.cat([upsampled_x, concat_with], dim=1)

        upsampled_x = self.convA(upsampled_x)
        upsampled_x = self.leakyrelu(upsampled_x)
        upsampled_x = self.convB(upsampled_x)
        upsampled_x = self.leakyrelu(upsampled_x)

        return upsampled_x


class Decoder(nn.Module):
    def __init__(self, num_features=2208, decoder_width=0.5, scales=[1, 2, 4, 8]):

        super(Decoder, self).__init__()

        features = int(num_features * decoder_width)

        self.conv2 = nn.Conv2d(num_features, features, 1, 1, 1)

        self.upsample1 = Upsample(
            features // scales[0] + 384, features // (scales[0] * 2)
        )
        self.upsample2 = Upsample(
            features // scales[1] + 192, features // (scales[1] * 2)
        )
        self.upsample3 = Upsample(
            features // scales[2] + 96, features // (scales[2] * 2)
        )
        self.upsample4 = Upsample(
            features // scales[3] + 96, features // (scales[3] * 2)
        )

        self.conv3 = nn.Conv2d(features // (scales[3] * 2), 1, 3, 1, 1)
        self.dispheader = DispHead(input_dim = 1)
    def forward(self, features):

        x_block0 = features[3]
        x_block1 = features[4]
        x_block2 = features[6]
        x_block3 = features[8]
        x_block4 = features[11]

        x0 = self.conv2(x_block4)
        x1 = self.upsample1(x0, x_block3)
        x2 = self.upsample2(x1, x_block2)
        x3 = self.upsample3(x2, x_block1)
        x4 = self.upsample4(x3, x_block0)
        x5 = self.conv3(x4)
        x6 = self.dispheader(x5, 2)
        # print(x0.size(), x1.size(), x2.size(), x3.size(), x4.size(), x5.size(), x6.size())
        return x6



class DispHead(nn.Module):
    def __init__(self, input_dim=320):
        super(DispHead, self).__init__()
        # self.norm1 = nn.BatchNorm2d(input_dim)
        self.conv1 = nn.Conv2d(input_dim, 1, 3, padding=1)
        # self.relu = nn.ReLU(inplace=True)
        self.sigmoid = nn.Sigmoid()

    def forward(self, x, scale):
        # x = self.relu(self.norm1(x))
        x = self.sigmoid(self.conv1(x))
        if scale > 1:
            x = upsample(x, scale_factor=scale)
        return x

def upsample(x, scale_factor=2, mode="bilinear", align_corners=False):
    """Upsample input tensor by a factor of 2
    """
    return F.interpolate(x, scale_factor=scale_factor, mode=mode, align_corners=align_corners)


class DenseDepth(nn.Module):
    def __init__(self, max_depth= 100.0, pretrained=False ):

        super(DenseDepth, self).__init__()

        self.encoder = Encoder(encoder_pretrained=pretrained)
        self.decoder = Decoder()
        self.max_depth = max_depth

    def forward(self, x):

        x = self.encoder(x)
        x = self.decoder(x)
        x = x* self.max_depth
        return x




