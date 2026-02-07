import cv2
import numpy as np
import sys

# Path to the model
model_path = "/home/sohaib/projects/AUV-2026/ros2_ws/src/vision/yolov11s_augmented_synthetic_best_model.onnx"

def test_backend(backend, target, name):
    print(f"Testing {name}...")
    try:
        net = cv2.dnn.readNetFromONNX(model_path)
        net.setPreferableBackend(backend)
        net.setPreferableTarget(target)
        
        # Create dummy input
        blob = cv2.dnn.blobFromImage(np.zeros((640, 640, 3), np.uint8), 1.0/255.0, (640, 640), (0, 0, 0), swapRB=True, crop=False)
        net.setInput(blob)
        
        # Forward pass
        outs = net.forward()
        print(f"{name}: SUCCESS")
    except Exception as e:
        print(f"{name}: FAILED - {e}")

# Test CPU
test_backend(cv2.dnn.DNN_BACKEND_OPENCV, cv2.dnn.DNN_TARGET_CPU, "CPU")

# Test CUDA
test_backend(cv2.dnn.DNN_BACKEND_CUDA, cv2.dnn.DNN_TARGET_CUDA, "CUDA")

# Test CUDA FP16
test_backend(cv2.dnn.DNN_BACKEND_CUDA, cv2.dnn.DNN_TARGET_CUDA_FP16, "CUDA FP16")
