import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision.models as models


# Set some constants
NB_INPUT_CHANNELS = 3
IMAGE_SHAPE = (70, 210)


class Multimodal(nn.Module):
    
    def __init__(self,
                 nb_classes=10,
                 nb_input_channels=3):
        
        super(Multimodal, self).__init__()
        
        self.nb_input_channels = nb_input_channels
        
        ## Multimodal images processing ##
        # Load the ResNet18 model with pretrained weights
        self.resnet18 = models.resnet18()
        # self.resnet18 = models.resnet18(
        #     weights=models.ResNet18_Weights.DEFAULT)
        
        # Replace the first convolutional layer to accept more than 3 channels
        self.resnet18.conv1 = nn.Conv2d(
            in_channels=nb_input_channels,
            out_channels=64,
            kernel_size=7, 
            stride=2,
            padding=3,
            bias=False
        )
        # Replace the last fully-connected layer to have n classes as output
        self.resnet18.fc = nn.Linear(self.resnet18.fc.in_features+1, 256)
        
        # Add a second fully-connected layer
        self.fc = nn.Linear(256, nb_classes)
        
        
    def forward(self,
                x_img: torch.Tensor,
                x_dense: torch.Tensor) -> torch.Tensor:
        
        # Element-wise product of the activation map and the main-
        # channel input
        # x = x_img * x_dense
        x = x_img
        
        # Forward pass through the ResNet18
        x = self.resnet18.conv1(x)
        x = self.resnet18.bn1(x)
        x = self.resnet18.relu(x)
        x = self.resnet18.maxpool(x)
        
        x = self.resnet18.layer1(x)
        x = self.resnet18.layer2(x)
        x = self.resnet18.layer3(x)
        x = self.resnet18.layer4(x)
        
        x = F.avg_pool2d(x, x.size()[2:])
        x = x.view(x.size(0), -1)
        
        # Concatenate the output of the ResNet18 with the dense input
        x = torch.cat((x, x_dense), dim=1)
        
        x = self.resnet18.fc(x)
        
        # x = F.relu(x)
        
        x = self.fc(x)
        
        return x


# Load the model
model = Multimodal()

# Load the fine-tuned weights
model.load_state_dict(torch.load("models/multimodal/multimodal.params"))

# Create dummy tensors
images = torch.randn(1,
                     NB_INPUT_CHANNELS,
                     IMAGE_SHAPE[0],
                     IMAGE_SHAPE[1])  # (batch, channels, height, width)
velocities = torch.randn(1, 1)  # (batch, features)

# Set the model in evaluation mode
# (should be done before exporting the model)
model.eval()

torchscript_model = torch.jit.trace(model, (images, velocities))
torchscript_model.save("models/multimodal/multimodal.pt")
