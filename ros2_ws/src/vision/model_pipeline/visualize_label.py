#!/usr/bin/env python3
"""
Random Image Inference Viewer

Load random images from a folder, run inference (YOLO or RF-DETR), and display results.

Usage:
    python3 visualize_label.py --folder data/processed/test/images
    python3 visualize_label.py --folder /path/to/images --model-type rfdetr --model best_rf_detr_small_model.pth

Controls:
    - Press any key to load the next random image
    - Press 'q' to quit
"""

import argparse
import random
from pathlib import Path

import cv2
import yaml
from ultralytics import YOLO

import supervision as sv
from rfdetr import RFDETRSmall
from PIL import Image as PILImage

# BASE DIR for paths
SCRIPT_DIR = Path(__file__).parent.resolve()
BASE_DIR = SCRIPT_DIR.parent

IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp"}


def get_image_files(folder: Path) -> list[Path]:
    """Get all image files from a folder."""
    images = [f for f in folder.iterdir() if f.suffix.lower() in IMAGE_EXTENSIONS]
    if not images:
        print(f"No images found in {folder}")
        exit(1)
    return images


def load_classes() -> list[str]:
    """Load class names from classes.yaml"""
    classes_path = SCRIPT_DIR / "classes.yaml"
    if not classes_path.exists():
        return []
    with open(classes_path, 'r') as f:
        data = yaml.safe_load(f)
        return data.get("names", [])


def main():
    parser = argparse.ArgumentParser(description="View YOLO or RF-DETR predictions on random images")
    parser.add_argument(
        "--folder", "-f",
        type=str,
        required=True,
        help="Path to folder containing images"
    )
    parser.add_argument(
        "--model-type", "-t",
        type=str,
        choices=['yolo', 'rfdetr'],
        default='yolo',
        help="Type of model to use (yolo or rfdetr)"
    )
    parser.add_argument(
        "--model", "-m",
        type=str,
        default=str(BASE_DIR / "best_yolov11s_model.pt"),
        help="Path to model weights"
    )
    parser.add_argument(
        "--conf", "-c",
        type=float,
        default=0.5,
        help="Confidence threshold (default: 0.5)"
    )
    args = parser.parse_args()

    # Load model
    if args.model_type == 'rfdetr':
        model = RFDETRSmall(pretrain_weights=args.model)
        class_names = load_classes()
    else:
        model = YOLO(args.model)
        class_names = list(model.names.values())
        
    box_annotator = sv.BoxAnnotator(thickness=2)
    label_annotator = sv.LabelAnnotator(smart_position=True, text_thickness=1, text_scale=0.3, text_padding=5)

    print(f"Loaded {args.model_type} model: {args.model}")
    
    cv2.namedWindow("Inference Browser", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Inference Browser", 1280, 720)

    # Get images
    folder = Path(args.folder)
    images = get_image_files(folder)
    print(f"Found {len(images)} images in {folder}")
    print("Press any key for next image, 'q' to quit.\n")

    while True:
        # Pick random image
        img_path = random.choice(images)
        print(f"Image: {img_path.name}")

        # Load and run inference
        img = cv2.imread(str(img_path))
        
        if args.model_type == 'rfdetr':
            detections = model.predict(img, threshold=args.conf)
        else:
            results = model.predict(img, verbose=False, conf=args.conf)
            detections = sv.Detections.from_ultralytics(results[0])

        annotated = img.copy()
        labels = [
            f"{class_names[class_id] if class_id < len(class_names) else str(class_id)} {confidence:.2f}"
            for class_id, confidence in zip(detections.class_id, detections.confidence)
        ]
        annotated = box_annotator.annotate(scene=annotated, detections=detections)
        annotated = label_annotator.annotate(scene=annotated, detections=detections, labels=labels)

        # Show
        cv2.imshow("Inference Browser", annotated)

        # Wait for keypress
        key = cv2.waitKey(0) & 0xFF
        if key == ord('q'):
            break

    cv2.destroyAllWindows()
    print("Done.")


if __name__ == "__main__":
    main()
