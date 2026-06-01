#!/usr/bin/env python3
"""
Dataset Organization Script for YOLO / COCO Training

This script:
1. Scans an input directory for images and labels (flexible structure)
2. Splits them into train/val/test sets
3. Moves them to a processed directory structure
4. Generates data.yaml (YOLO) or _annotations.coco.json (COCO)

Usage:
    python3 organize_dataset.py --input data/raw_import --output data/processed
    python3 organize_dataset.py --input data/augmented --output data/processed_aug
    python3 organize_dataset.py --input data/raw_import --output data/processed_coco --format coco
"""

import json
import os
import shutil
import random
import argparse
import yaml
from pathlib import Path
from typing import List, Tuple

import cv2
import numpy as np
from tqdm import tqdm

SCRIPT_DIR = Path(__file__).parent.resolve()

CLASSES_YAML = SCRIPT_DIR / "classes.yaml"


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp"}


def safe_relative(path: Path, base: Path) -> Path:
    try:
        return path.relative_to(base)
    except ValueError:
        return path

def get_class_names():
    """Load class names from classes.yaml (single source of truth)."""
    if CLASSES_YAML.exists():
        try:
            with open(CLASSES_YAML, 'r') as f:
                data = yaml.safe_load(f)
                names = data.get('names', [])
                if names:
                    return names
        except Exception:
            pass
    raise FileNotFoundError(
        f"Could not load class names from {CLASSES_YAML}. "
        "Make sure classes.yaml exists in the model_pipeline directory."
    )

def find_image_label_pairs(input_dir: Path) -> List[Tuple[Path, Path]]:
    """Find all image-label pairs in the input directory with flexible detection."""
    pairs = []
    
    # Potential subdirectories
    potential_img_dirs = [input_dir / "images", input_dir / "yolo_images", input_dir]
    potential_lbl_dirs = [input_dir / "labels", input_dir / "yolo_labels", input_dir]
    
    img_dir = None
    lbl_dir = None
    
    for d in potential_img_dirs:
        if d.exists() and (list(d.glob("*.jpg")) or list(d.glob("*.png"))):
            img_dir = d
            break
            
    for d in potential_lbl_dirs:
        if d.exists() and list(d.glob("*.txt")):
            lbl_dir = d
            break
            
    if not img_dir or not lbl_dir:
        return []
        
    print(f"Using images from: {safe_relative(img_dir, SCRIPT_DIR.parent.parent.parent.parent.parent.parent)}")
    print(f"Using labels from: {safe_relative(lbl_dir, SCRIPT_DIR.parent.parent.parent.parent.parent.parent)}")

    for img_path in img_dir.iterdir():
        if img_path.suffix.lower() not in IMAGE_EXTENSIONS:
            continue
        
        label_path = lbl_dir / (img_path.stem + ".txt")
        if label_path.exists():
            pairs.append((img_path, label_path))
            
    return pairs

def load_yolo_labels(label_path: Path) -> Tuple[List[List[float]], List[int]]:
    bboxes, class_labels = [], []
    if label_path.exists():
        with open(label_path, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) == 5:
                    cls = int(float(parts[0]))
                    xc, yc, w, h = [float(x) for x in parts[1:]]
                    xmin = max(0.0, xc - w/2)
                    ymin = max(0.0, yc - h/2)
                    xmax = min(1.0, xc + w/2)
                    ymax = min(1.0, yc + h/2)
                    new_w = max(0.000001, xmax - xmin)
                    new_h = max(0.000001, ymax - ymin)
                    new_xc = xmin + new_w / 2
                    new_yc = ymin + new_h / 2
                    bboxes.append([new_xc, new_yc, new_w, new_h])
                    class_labels.append(cls)
    return bboxes, class_labels

def save_yolo_labels(label_path: Path, bboxes: List[List[float]], class_labels: List[int]):
    with open(label_path, 'w') as f:
        for bbox, cls in zip(bboxes, class_labels):
            f.write(f"{int(float(cls))} {' '.join([f'{x:.6f}' for x in bbox])}\n")

def process_and_save_image(img_path: Path, lbl_path: Path, img_dest: Path, lbl_dest: Path, letterbox_size: int = None):
    """Copies or resizes the image/label. Returns the new (height, width) of the saved image."""
    if not letterbox_size:
        shutil.copy2(img_path, img_dest)
        shutil.copy2(lbl_path, lbl_dest)
        # we still need height/width for COCO, so we read it
        img = cv2.imread(str(img_dest))
        return img.shape[:2] if img is not None else (0, 0)
    
    import albumentations as A
    transform = A.Compose([
        A.LongestMaxSize(max_size=letterbox_size),
        A.PadIfNeeded(min_height=letterbox_size, min_width=letterbox_size, border_mode=cv2.BORDER_CONSTANT, fill=(114, 114, 114))
    ], bbox_params=A.BboxParams(format='yolo', label_fields=['class_labels'], min_visibility=0.0))
    
    image = cv2.imread(str(img_path))
    if image is None:
        return (0, 0)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    bboxes, class_labels = load_yolo_labels(lbl_path)
    augmented = transform(image=image, bboxes=bboxes, class_labels=class_labels)
    
    aug_img = cv2.cvtColor(augmented['image'], cv2.COLOR_RGB2BGR)
    cv2.imwrite(str(img_dest), aug_img)
    save_yolo_labels(lbl_dest, augmented['bboxes'], augmented['class_labels'])
    
    return aug_img.shape[:2]

def create_directory_structure(processed_dir: Path, fmt: str):
    """Create the train/val/test directory structure."""
    if processed_dir.exists():
        shutil.rmtree(processed_dir)
        print(f"Cleared existing {processed_dir}")
    
    if fmt == "yolo":
        for split in ["train", "val", "test"]:
            (processed_dir / split / "images").mkdir(parents=True, exist_ok=True)
            (processed_dir / split / "labels").mkdir(parents=True, exist_ok=True)
    else:
        # COCO: RF-DETR expects train/, valid/, test/ with images + JSON
        for split in ["train", "valid", "test"]:
            (processed_dir / split).mkdir(parents=True, exist_ok=True)

def split_and_move_data_yolo(pairs: List[Tuple[Path, Path]], processed_dir: Path, ratios: tuple, letterbox_size: int = None):
    """Shuffle and split data into train/val/test sets (YOLO format)."""
    random.shuffle(pairs)
    
    train_ratio, val_ratio, _ = ratios
    n = len(pairs)
    train_end = int(n * train_ratio)
    val_end = train_end + int(n * val_ratio)
    
    splits = {
        "train": pairs[:train_end],
        "val": pairs[train_end:val_end],
        "test": pairs[val_end:]
    }
    
    for split_name, split_pairs in splits.items():
        img_dest = processed_dir / split_name / "images"
        lbl_dest = processed_dir / split_name / "labels"
        
        for img_path, lbl_path in tqdm(split_pairs, desc=f"{split_name}"):
            process_and_save_image(img_path, lbl_path, img_dest / img_path.name, lbl_dest / lbl_path.name, letterbox_size)
    
    return splits

def yolo_to_coco_annotation(label_path: Path, image_id: int, img_width: int,
                            img_height: int, ann_id_start: int) -> Tuple[list, int]:
    """Convert a single YOLO label file to a list of COCO annotation dicts.

    Returns:
        (annotations_list, next_ann_id)
    """
    annotations = []
    ann_id = ann_id_start

    if not label_path.exists():
        return annotations, ann_id

    with open(label_path, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) != 5:
                continue

            cls = int(float(parts[0]))
            xc, yc, w, h = [float(x) for x in parts[1:]]

            # Convert YOLO normalised (xc, yc, w, h) → COCO absolute (x, y, w, h)
            abs_w = w * img_width
            abs_h = h * img_height
            abs_x = (xc * img_width) - abs_w / 2
            abs_y = (yc * img_height) - abs_h / 2

            annotations.append({
                "id": ann_id,
                "image_id": image_id,
                "category_id": cls,
                "bbox": [round(abs_x, 2), round(abs_y, 2),
                         round(abs_w, 2), round(abs_h, 2)],
                "area": round(abs_w * abs_h, 2),
                "iscrowd": 0
            })
            ann_id += 1

    return annotations, ann_id

def split_and_move_data_coco(pairs: List[Tuple[Path, Path]], processed_dir: Path,
                             ratios: tuple, class_names: list, letterbox_size: int = None):
    """Shuffle and split data into train/valid/test sets (COCO format)."""
    random.shuffle(pairs)

    train_ratio, val_ratio, _ = ratios
    n = len(pairs)
    train_end = int(n * train_ratio)
    val_end = train_end + int(n * val_ratio)

    # RF-DETR uses "valid" not "val"
    splits = {
        "train": pairs[:train_end],
        "valid": pairs[train_end:val_end],
        "test": pairs[val_end:]
    }

    # Build COCO categories list
    categories = [{"id": i, "name": name} for i, name in enumerate(class_names)]

    for split_name, split_pairs in splits.items():
        split_dir = processed_dir / split_name
        images_list = []
        annotations_list = []
        ann_id = 1

        for image_id, (img_path, lbl_path) in enumerate(
                tqdm(split_pairs, desc=f"{split_name}"), start=1):
            # Process image and labels
            dst_img = split_dir / img_path.name
            dst_lbl = split_dir / lbl_path.name
            img_height, img_width = process_and_save_image(img_path, lbl_path, dst_img, dst_lbl, letterbox_size)
            
            if img_height == 0:
                continue

            images_list.append({
                "id": image_id,
                "file_name": img_path.name,
                "width": img_width,
                "height": img_height
            })

            anns, ann_id = yolo_to_coco_annotation(
                dst_lbl, image_id, img_width, img_height, ann_id)
            annotations_list.extend(anns)

        coco_dict = {
            "images": images_list,
            "annotations": annotations_list,
            "categories": categories
        }

        json_path = split_dir / "_annotations.coco.json"
        with open(json_path, 'w') as f:
            json.dump(coco_dict, f, indent=2)

        print(f"{split_name}: {len(split_pairs)} images, "
              f"{len(annotations_list)} annotations → {json_path.name}")

def generate_data_yaml(processed_dir: Path, class_names: list):
    """Generate the data.yaml file."""
    yaml_path = processed_dir / "data.yaml"
    content = {
        'path': str(processed_dir.absolute()),
        'train': 'train/images',
        'val': 'val/images',
        'test': 'test/images',
        'nc': len(class_names),
        'names': class_names
    }
    
    with open(yaml_path, 'w') as f:
        yaml.dump(content, f, sort_keys=False)
    print(f"\nGenerated {yaml_path}")

def main():
    parser = argparse.ArgumentParser(description="Organize YOLO/COCO Dataset with flexible structure")
    parser.add_argument("--input", "-i", type=str, default="data/raw_import", help="Input directory")
    parser.add_argument("--output", "-o", type=str, default="data/processed", help="Output directory")
    parser.add_argument("--train", type=float, default=0.7, help="Train ratio (0-1)")
    parser.add_argument("--val", type=float, default=0.2, help="Val ratio (0-1)")
    parser.add_argument(
        "--format", "-f",
        type=str,
        choices=["yolo", "coco"],
        default="yolo",
        help="Output format: yolo (default) or coco (for RF-DETR)"
    )
    parser.add_argument(
        "--letterbox",
        type=int,
        default=None,
        help="Optional size to letterbox pad the images to (e.g., 512 for RF-DETR)."
    )
    
    args = parser.parse_args()
    
    input_dir = Path(args.input)
    output_dir = Path(args.output)
    fmt = args.format
    
    print("=" * 50)
    print(f"Dataset Organizer  [format: {fmt.upper()}]")
    print("=" * 50)
    
    if not input_dir.exists():
        print(f"Error: {input_dir} not found.")
        return
        
    pairs = find_image_label_pairs(input_dir)
    if not pairs:
        print("No image-label pairs found!")
        return
        
    print(f"Found {len(pairs)} image-label pairs.")
    
    class_names = get_class_names()

    create_directory_structure(output_dir, fmt)

    ratios = (args.train, args.val, 1.0 - args.train - args.val)

    if fmt == "yolo":
        split_and_move_data_yolo(pairs, output_dir, ratios, args.letterbox)
        generate_data_yaml(output_dir, class_names)
    else:
        split_and_move_data_coco(pairs, output_dir, ratios, class_names, args.letterbox)
    
    print("\n" + "=" * 50)
    print("Done!")
    if fmt == "yolo":
        print(f"Training command: python3 training.py --data {output_dir}/data.yaml")
    else:
        print(f"Training command: python3 training.py --model rfdetr --dataset-dir {output_dir}")
    print("=" * 50)

if __name__ == "__main__":
    main()
