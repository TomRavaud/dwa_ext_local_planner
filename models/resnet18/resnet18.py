import torch
import torch.nn as nn
import torchvision.models as models
from torchvision import transforms
import PIL
import cv2


# Load the model
model = models.resnet18()

# Replace the last layer by a fully-connected one with 1 output
model.fc = nn.Linear(model.fc.in_features, 10)

# Load the fine-tuned weights
model.load_state_dict(torch.load("models/resnet18/resnet18.params"))

# Create a dummy input
images = torch.rand(1, 3, 70, 210)

# Set the model in evaluation mode
# (should be done before exporting the model)
model.eval()

torchscript_model = torch.jit.trace(model, images)
torchscript_model.save("models/resnet18/resnet18.pt")
