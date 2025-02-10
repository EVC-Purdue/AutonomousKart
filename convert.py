# import torch

# model = torch.load("fast_scnn.pth", map_location=torch.device('cpu'))  # Load PyTorch model
# dummy_input = torch.randn(1, 3, 512, 512)  # Dummy input for conversion
# torch.onnx.export(model, dummy_input, "fast_scnn.onnx", opset_version=11)