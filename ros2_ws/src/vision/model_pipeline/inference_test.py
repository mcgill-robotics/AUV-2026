from inference_models import AutoModel
import cv2
import time
import os
import random
import argparse
import supervision as sv
from tqdm import tqdm

# Parse arguments
parser = argparse.ArgumentParser(description="Run inference benchmark on a folder of images")
parser.add_argument("folder", type=str, help="Path to folder containing images")
parser.add_argument("--amount", type=int, default=10000, help="Number of inference iterations")
# parser.add_argument("--delay", type=int, default=1, help="cv2.waitKey delay (ms), 0 for manual stepping")
args = parser.parse_args()

# Collect image paths from folder
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tiff", ".webp"}
image_paths = [
    os.path.join(args.folder, f)
    for f in os.listdir(args.folder)
    if os.path.splitext(f)[1].lower() in IMAGE_EXTENSIONS
]
assert len(image_paths) > 0, f"No images found in {args.folder}"
print(f"Found {len(image_paths)} images in {args.folder}")

# Load the local model
model = AutoModel.from_pretrained(
    args.folder,
    onnx_execution_providers=["TensorrtExecutionProvider", "CUDAExecutionProvider"],
    default_onnx_trt_options=False # Enables FP16 and engine caching automatically
)
print("Model loaded successfully")

# Create Annotators
bounding_box_annotator = sv.BoxAnnotator()
label_annotator = sv.LabelAnnotator()

# Run inference
total_inference_time = 0.0
for i in tqdm(range(args.amount)):
    image_path = random.choice(image_paths)
    image = cv2.imread(image_path)
    
    start_time = time.time()
    predictions = model(image, confidence=0.4)
    total_inference_time += (time.time() - start_time)

    # Convert to supervision and visualize
    detections = predictions[0].to_supervision()
    
    # labels = [
    #     f"Class: {class_id} {confidence:.2f}"
    #     for class_id, confidence in zip(detections.class_id, detections.confidence)
    # ]
    
    # annotated_image = bounding_box_annotator.annotate(scene=image.copy(), detections=detections)
    # annotated_image = label_annotator.annotate(scene=annotated_image, detections=detections, labels=labels)
    
    # cv2.imshow("Inference", annotated_image)
    # if cv2.waitKey(args.delay) & 0xFF == ord('q'):
    #     print("Visualization interrupted by user.")
    #     break
        
print(f"Last image: {image_path}")
print("Detections: ", predictions[0].to_supervision())
print("Benchmark (inference only): ", total_inference_time / (i + 1))
cv2.destroyAllWindows()