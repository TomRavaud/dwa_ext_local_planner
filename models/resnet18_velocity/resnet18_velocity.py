import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision.models as models
from torchvision import transforms
import PIL
import cv2


class ResNet18Velocity(nn.Module):
    
    def __init__(self, num_classes=10, num_input_features=1):
        
        super(ResNet18Velocity, self).__init__()
        
        self.resnet18 = models.resnet18()
        
        self.resnet18.fc = nn.Linear(self.resnet18.fc.in_features, num_classes)
        
        self.fc = nn.Linear(num_input_features, 210*70*3)
        
    
    def forward(self, x_img, x_dense):
        
        # FC layer to convert the numeric input to the same shape as the
        # activation map
        x_dense = self.fc(x_dense)
        x_dense = x_dense.view(-1, 3, 70, 210)
        
        # Element-wise product of the activation map and the main-
        # channel input
        x = x_img * x_dense
        
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
        x = self.resnet18.fc(x)
        
        return x

# Load the model
model = ResNet18Velocity()

# Load the fine-tuned weights
model.load_state_dict(torch.load("models/resnet18_velocity/resnet18_velocity.params"))

# Create dummy tensors
images = torch.randn(1, 3, 70, 210)  # (batch, channels, height, width)
velocities = torch.randn(1, 1)  # (batch, features)

# Set the model in evaluation mode
# (should be done before exporting the model)
model.eval()

torchscript_model = torch.jit.trace(model, (images, velocities))
torchscript_model.save("models/resnet18_velocity/resnet18_velocity.pt")
