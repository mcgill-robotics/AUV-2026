#!/usr/bin/env python3
"""
Model Prediction Exporter (YOLO Format)

Run inference with an RF-DETR or YOLO model on a folder of images and export 
the predictions as .txt files in YOLO format.

Usage:
    python3 export_labels.py --folder /path/to/images --model-type rfdetr --model best_rf_detr_small_model.pth
    python3 export_labels.py --folder /path/to/images --model-type yolo --model best_yolov11s_model.pt
    python3 export_labels.py --folder /path/to/images --task segment
"""

import argparse
import numpy as np
from pathlib import Path
import os
import cv2
import yaml
import shutil

import supervision as sv
from ultralytics import YOLO
from rfdetr import RFDETRSmall, RFDETRSegSmall

IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp"}
SCRIPT_DIR = Path(__file__).parent.resolve()

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
    parser = argparse.ArgumentParser(description="Export Model predictions to YOLO format labels")
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
        default='rfdetr',
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
        required=True,
        help="Path to model weights (.pth or .pt)"
    )
    parser.add_argument(
        "--output", "-o",
        type=str,
        default=None,
        help="Path to output folder for labels (default: creates a 'labels' folder adjacent to images folder)"
    )
    parser.add_argument(
        "--conf", "-c",
        type=float,
        default=0.5,
        help="Confidence threshold (default: 0.5)"
    )
    args = parser.parse_args()

    # Determine folders
    img_folder = Path(args.folder).resolve()
    if args.output:
        lbl_folder = Path(args.output).resolve()
        lbl_folder.mkdir(parents=True, exist_ok=True)
    else:
        lbl_folder = img_folder.parent / f"{img_folder.name}_prelabeled"
        print(f"Copying original folder to: {lbl_folder}")
        shutil.copytree(img_folder, lbl_folder, dirs_exist_ok=True)
    
    print(f"Reading images from: {img_folder}")
    print(f"Saving labels to: {lbl_folder}")

    class_names = load_classes()
    if class_names:
        classes_txt_path = lbl_folder / "classes.txt"
        with open(classes_txt_path, "w") as f:
            for name in class_names:
                f.write(f"{name}\n")
        print(f"Generated {classes_txt_path.name} for Roboflow import.")

    # Load Model
    print(f"Loading {args.model_type.upper()} model: {args.model}")
    if args.model_type == 'rfdetr':
        if args.task == "segment":
            model = RFDETRSegSmall(pretrain_weights=args.model)
        else:
            model = RFDETRSmall(pretrain_weights=args.model)
    else:
        model = YOLO(args.model)

    images = get_image_files(img_folder)
    total_images = len(images)
    print(f"Found {total_images} images to process.\n")

    for idx, img_path in enumerate(images, start=1):
        img = cv2.imread(str(img_path))
        if img is None:
            print(f"Failed to load image: {img_path}")
            continue

        h_img, w_img, _ = img.shape
        
        yolo_masks_xyn = None
        
        # Inference
        if args.model_type == 'rfdetr':
            detections = model.predict(img, threshold=args.conf)
        else:
            results = model.predict(img, verbose=False, conf=args.conf)
            detections = sv.Detections.from_ultralytics(results[0])
            if args.task == "segment" and results[0].masks is not None:
                yolo_masks_xyn = results[0].masks.xyn
            
        label_file = lbl_folder / f"{img_path.stem}.txt"
        
        with open(label_file, "w") as f:
            if args.task == "detect":
                for i in range(len(detections)):
                    x1, y1, x2, y2 = detections.xyxy[i]
                    class_id = int(detections.class_id[i])
                    
                    w = x2 - x1
                    h = y2 - y1
                    cx = x1 + w / 2.0
                    cy = y1 + h / 2.0
                    
                    cx_norm = min(max(cx / w_img, 0.0), 1.0)
                    cy_norm = min(max(cy / h_img, 0.0), 1.0)
                    w_norm = min(max(w / w_img, 0.0), 1.0)
                    h_norm = min(max(h / h_img, 0.0), 1.0)
                    
                    f.write(f"{class_id} {cx_norm:.6f} {cy_norm:.6f} {w_norm:.6f} {h_norm:.6f}\n")
            elif args.task == "segment":
                if args.model_type == 'yolo' and yolo_masks_xyn is not None:
                    for i in range(len(detections)):
                        class_id = int(detections.class_id[i])
                        coords = yolo_masks_xyn[i].reshape(-1)
                        if len(coords) < 6:
                            continue
                        coords_str = " ".join([f"{c:.6f}" for c in coords])
                        f.write(f"{class_id} {coords_str}\n")
                elif args.model_type == 'rfdetr' and detections.mask is not None:
                    for i in range(len(detections)):
                        class_id = int(detections.class_id[i])
                        mask = detections.mask[i]
                        polygons = sv.mask_to_polygons(mask)
                        if not polygons:
                            continue
                        
                        largest_poly = max(polygons, key=lambda p: cv2.contourArea(p))
                        
                        if len(largest_poly) < 3:
                            continue
                            
                        normalized_poly = largest_poly.astype(float)
                        normalized_poly[:, 0] /= w_img
                        normalized_poly[:, 1] /= h_img
                        
                        normalized_poly = np.clip(normalized_poly, 0.0, 1.0)
                        
                        coords_str = " ".join([f"{c:.6f}" for c in normalized_poly.flatten()])
                        f.write(f"{class_id} {coords_str}\n")
                
        print(f"[{idx}/{total_images}] Exported: {label_file.name} ({len(detections)} detections)")

    print("\nExport Complete.")

if __name__ == "__main__":
    main()
