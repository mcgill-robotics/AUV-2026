import pyzed.sl as sl
from ultralytics import YOLO
import numpy as np
import time
import cv2
from pathlib import Path


init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.SVGA
init_params.camera_fps=30
init_params.coordinate_units = sl.UNIT.METER
init_params.set_from_stream("127.0.0.1", 30000)
# BASE DIR for paths
SCRIPT_DIR = Path(__file__).parent.resolve()
BASE_DIR = SCRIPT_DIR.parent

test_time = 30 # seconds
model_path = BASE_DIR / "runs/detect/yolov11s_sim_dataset7/weights/best.pt"
model = YOLO(str(model_path))

zed = sl.Camera()

err = zed.open(init_params)

runtime_params = sl.RuntimeParameters()

if err != sl.ERROR_CODE.SUCCESS:
    print("Error with zed camera")
    exit(1)


num_images = 0
total_time = 0.0

image = sl.Mat()
loop_start_time = time.time()

print("Starting inference... Press 'q' to quit.")

# while (time.time() < loop_start_time + test_time):
while True:
    if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
        # Retrieve image
        zed.retrieve_image(image, sl.VIEW.LEFT)
        image_np = image.get_data()
        # Convert to BGR for OpenCV (ZED returns BGRA)
        image_bgr = cv2.cvtColor(image_np, cv2.COLOR_BGRA2BGR)
        
        start_time = time.time()
        # Run inference (only show detections with >50% confidence)
        results = model.predict(image_bgr, verbose=False)
        end_time = time.time()
        
        total_time += end_time - start_time
        num_images += 1
        
        # Draw results on the image
        annotated_frame = results[0].plot()
        
        # Display the frame
        cv2.imshow("AUV Vision - YOLO Inference", annotated_frame)
        
        # Check for user input to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

# Cleanup
cv2.destroyAllWindows()
zed.close()

print(f"Total images processed: {num_images}")
print(f"Total time spend inferring: {total_time}")
print(f"Average inference time: {total_time / num_images * 1000} ms")
