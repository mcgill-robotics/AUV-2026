import torch
import torch.onnx
import onnx
from ultralytics import YOLO

def load_model(model_path):
    # Load the trained YOLO model from the Ultralytics library
    model = YOLO(model_path)  # This automatically loads the model and architecture
    return model

def create_dummy_input():
    # Create a dummy input tensor with the shape expected by the model
    return torch.randn(1, 3, 640, 640)  # Adjust to your model's expected input shape (YOLO models typically use 640x640)

def export_model_to_onnx(model, dummy_input, output_path):
    # Export the model to ONNX format
    torch.onnx.export(model.model, dummy_input, output_path, 
                      export_params=True,
                      opset_version=12, 
                      do_constant_folding=True,
                      input_names=['input'],
                      output_names=['output'],
                      dynamic_axes={'input': {0: 'batch_size'}, 'output': {0: 'batch_size'}})

if __name__ == '__main__':
    # Load the YOLO model using the Ultralytics library
    model = load_model('yolo11s-best.pt')

    # Prepare dummy input (for YOLOv5, the input is usually 640x640)
    dummy_input = create_dummy_input()

    # Export the model to ONNX
    export_model_to_onnx(model, dummy_input, 'yolo11s-best.onnx')