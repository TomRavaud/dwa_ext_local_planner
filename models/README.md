# CNN models

This folder contains CNN models trained on a custom dataset in a self-supervised manner for terrain traversability analysis. A TorchScript program is generated from the Python model and the trained weights, and is used in the C++ application.

What's more, each model is associated with bins midpoints, stored in a **bins_midpoints.npy** file (that can be loaded with *numpy* in a Python script). These values are used to convert the output of the CNN (a probability distribution over the bins) to a single value, which is the traversability score.

Depending on the model, inputs are either RGB images, depth images, normal maps or a combination of these. The code has to be adapted accordingly.