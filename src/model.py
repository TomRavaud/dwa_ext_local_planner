import torch
import torch.nn as nn
import torchvision.models as models
from torchvision import transforms
from PIL import Image
import cv2

# Load the model
model = models.resnet18()

# Replace the last layer by a fully-connected one with 1 output
model.fc = nn.Linear(model.fc.in_features, 10)

# Load the fine-tuned weights
model.load_state_dict(torch.load("src/resnet18_classification.params"))

example_input = torch.rand(1, 3, 70, 210)

torchscript_model = torch.jit.trace(model, example_input)
torchscript_model.save("resnet18_classification.pt")


# image = cv2.imread("src/00000.png")

# image_resized = cv2.resize(image, (210, 70))

# transform = transforms.Compose([
#     transforms.ToTensor(),
#     transforms.Normalize(
#         mean=[0.3426, 0.3569, 0.2914],
#         std=[0.1363, 0.1248, 0.1302]
#     ),
# ])

# image_transformed = transform(image_resized)

# batch = image_transformed.unsqueeze(0)

    
# output = model(torch.tensor(batch))
# print(output)
