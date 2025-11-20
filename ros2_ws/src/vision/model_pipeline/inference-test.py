import pyzed.sl as sl
from ultralytics import YOLO
import time

init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD720
init_params.camera_fps=60
test_time = 30 # seconds

model = YOLO("runs/detect/yolo11s/weights/best.pt")

zed = sl.Camera()

err = zed.open(init_params)

runtime_params = sl.RuntimeParameters();

if err != sl.ERROR_CODE.SUCCESS:
    print("Error with zed camera")

num_images = 0
total_time = 0.0

image = sl.Mat()
loop_start_time = time.time()
while (time.time() < loop_start_time + test_time):
    if zed.grab(runtime_params) <= sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(image, sl.VIEW.LEFT)
        image_np = image.get_data()
        image_rgb = image_np[:, :, :3][:, :, ::-1]
        start_time = time.time()
        results = model.predict(image_rgb, verbose=False)
        total_time += time.time() - start_time
        num_images+=1

print(f"Total images processed: {num_images}")
print(f"Total time spend inferring: {total_time}")
print(f"Average inference time: {total_time / num_images * 1000} ms")

