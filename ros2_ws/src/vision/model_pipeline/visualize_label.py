#!/usr/bin/env python3
"""
Random Image Inference Viewer

Load random images from a folder, run inference (YOLO or RF-DETR), and display results.

Usage:
    python3 visualize_label.py --folder data/processed/test/images
    python3 visualize_label.py --folder /path/to/images --model-type rfdetr --model best_rf_detr_small_model.pth
    python3 visualize_label.py --folder data/raw_import/images --from-label
    python3 visualize_label.py --folder data/raw_import/images --from-label --task segment

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
import numpy as np

import supervision as sv
from rfdetr import RFDETRSmall, RFDETRSegSmall
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
        data = yaml.safe_load(f) or {}
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
        "--task",
        type=str,
        choices=["detect", "segment"],
        default="detect",
        help="Task type: detect (bounding boxes) or segment (polygons)"
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
    parser.add_argument(
        "--from-label",
        action="store_true",
        help="Use YOLO label files to annotate images instead of running a model",
    )
    parser.add_argument(
        "--labels-dir",
        type=str,
        default=None,
        help="Directory containing YOLO label files (optional). If omitted, the script will look next to each image or in sibling 'labels' folders.",
    )
    args = parser.parse_args()

    # Load model or class names depending on mode
    if args.from_label:
        model = None
        class_names = load_classes()
    else:
        if args.model_type == 'rfdetr':
            if args.task == "segment":
                model = RFDETRSegSmall(pretrain_weights=args.model)
            else:
                model = RFDETRSmall(pretrain_weights=args.model)
            class_names = load_classes()
        else:
            model = YOLO(args.model)
            class_names = list(model.names.values())
        
    box_annotator = sv.BoxAnnotator(thickness=2)
    label_annotator = sv.LabelAnnotator(smart_position=True, text_thickness=1, text_scale=0.3, text_padding=5)
    mask_annotator = sv.MaskAnnotator()
    polygon_annotator = sv.PolygonAnnotator(thickness=2)

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
        h, w = img.shape[:2]
        
        if args.from_label:
            # locate corresponding label file
            label_path = None
            stem_txt = img_path.with_suffix('.txt')
            if args.labels_dir:
                candidate = Path(args.labels_dir) / (img_path.stem + '.txt')
                if candidate.exists():
                    label_path = candidate
            if label_path is None and stem_txt.exists():
                label_path = stem_txt
            # check common sibling 'labels' folders
            if label_path is None:
                sibling = img_path.parent / 'labels' / (img_path.stem + '.txt')
                if sibling.exists():
                    label_path = sibling
            if label_path is None:
                parent_sibling = img_path.parent.parent / 'labels' / (img_path.stem + '.txt')
                if parent_sibling.exists():
                    label_path = parent_sibling

            if label_path is None:
                print(f"No label file found for image {img_path.name}, assuming empty background.")
                detections = sv.Detections(
                    xyxy=np.empty((0, 4)),
                    confidence=np.empty((0,)),
                    class_id=np.empty((0,), dtype=int)
                )
            else:
                boxes = []
                confidences = []
                class_ids = []
                masks = []
                
                with open(label_path, 'r', encoding='utf-8') as lf:
                    for line in lf:
                        parts = line.strip().split()
                        if not parts:
                            continue
                        try:
                            cls = int(parts[0])
                            
                            if args.task == "detect":
                                if len(parts) >= 5:
                                    x_c = float(parts[1]) * w
                                    y_c = float(parts[2]) * h
                                    bw = float(parts[3]) * w
                                    bh = float(parts[4]) * h

                                    x1 = x_c - bw / 2.0
                                    y1 = y_c - bh / 2.0
                                    x2 = x_c + bw / 2.0
                                    y2 = y_c + bh / 2.0
                                    boxes.append([x1, y1, x2, y2])
                                    confidences.append(1.0)
                                    class_ids.append(cls)
                                    
                            elif args.task == "segment":
                                if len(parts) >= 7:
                                    coords = [float(x) for x in parts[1:]]
                                    pts = []
                                    for i in range(0, len(coords), 2):
                                        pts.append([coords[i]*w, coords[i+1]*h])
                                        
                                    pts_arr = np.array(pts, dtype=np.int32)
                                    
                                    # Create bbox from polygon
                                    x_min, y_min = np.min(pts_arr, axis=0)
                                    x_max, y_max = np.max(pts_arr, axis=0)
                                    boxes.append([x_min, y_min, x_max, y_max])
                                    
                                    # Create boolean mask
                                    mask = np.zeros((h, w), dtype=np.uint8)
                                    cv2.fillPoly(mask, [pts_arr], 1)
                                    masks.append(mask.astype(bool))
                                    
                                    confidences.append(1.0)
                                    class_ids.append(cls)
                                    
                        except Exception:
                            print(f"Skipping invalid label line in {label_path}: {line.strip()}")
                            continue

                if boxes:
                    if args.task == "segment" and masks:
                        detections = sv.Detections(
                            xyxy=np.array(boxes),
                            mask=np.array(masks),
                            confidence=np.array(confidences),
                            class_id=np.array(class_ids, dtype=int),
                        )
                    else:
                        detections = sv.Detections(
                            xyxy=np.array(boxes),
                            confidence=np.array(confidences),
                            class_id=np.array(class_ids, dtype=int),
                        )
                else:
                    detections = sv.Detections(
                        xyxy=np.empty((0, 4)),
                        confidence=np.empty((0,)),
                        class_id=np.empty((0,), dtype=int)
                    )
        else:
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
        
        if args.task == "segment":
            annotated = mask_annotator.annotate(scene=annotated, detections=detections)
            annotated = polygon_annotator.annotate(scene=annotated, detections=detections)
        else:
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
