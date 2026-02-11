#!/usr/bin/env python3
"""
Dataset Organization Script for YOLO Training

This script:
1. Scans an input directory for images and labels (flexible structure)
2. Splits them into train/val/test sets
3. Moves them to a processed directory structure
4. Generates data.yaml

Usage:
    python3 organize_dataset.py --input data/raw_import --output data/processed
    python3 organize_dataset.py --input data/augmented --output data/processed_aug
"""

import os
import shutil
import random
import argparse
import yaml
from pathlib import Path
from typing import List, Tuple

# Configuration
SCRIPT_DIR = Path(__file__).parent.resolve()
DATA_UNITY_YAML = SCRIPT_DIR / "data_unity.yaml"

# Default class names if YAML not found
DEFAULT_CLASS_NAMES = [
    "gate", "lane_marker", "red_pipe", "white_pipe", "octagon",
    "table", "bin", "board", "shark", "sawfish"
]

IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp"}


def safe_relative(path: Path, base: Path) -> Path:
    try:
        return path.relative_to(base)
    except ValueError:
        return path

def get_class_names():
    if DATA_UNITY_YAML.exists():
        try:
            with open(DATA_UNITY_YAML, 'r') as f:
                data = yaml.safe_load(f)
                return data.get('names', DEFAULT_CLASS_NAMES)
        except Exception:
            pass
    return DEFAULT_CLASS_NAMES

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
        
    #print(f"Using images from: {img_dir.relative_to(SCRIPT_DIR.parent.parent.parent.parent.parent.parent) if img_dir.is_relative_to(SCRIPT_DIR.parent.parent.parent.parent.parent.parent) else img_dir}")
    #print(f"Using labels from: {lbl_dir.relative_to(SCRIPT_DIR.parent.parent.parent.parent.parent.parent) if lbl_dir.is_relative_to(SCRIPT_DIR.parent.parent.parent.parent.parent.parent) else lbl_dir}")
    print(f"Using images from: {safe_relative(img_dir, SCRIPT_DIR.parent.parent.parent.parent.parent.parent)}")
    print(f"Using labels from: {safe_relative(lbl_dir, SCRIPT_DIR.parent.parent.parent.parent.parent.parent)}")

    for img_path in img_dir.iterdir():
        if img_path.suffix.lower() not in IMAGE_EXTENSIONS:
            continue
        
        label_path = lbl_dir / (img_path.stem + ".txt")
        if label_path.exists():
            pairs.append((img_path, label_path))
            
    return pairs

def create_directory_structure(processed_dir: Path):
    """Create the train/val/test directory structure."""
    if processed_dir.exists():
        shutil.rmtree(processed_dir)
        print(f"Cleared existing {processed_dir}")
    
    for split in ["train", "val", "test"]:
        (processed_dir / split / "images").mkdir(parents=True, exist_ok=True)
        (processed_dir / split / "labels").mkdir(parents=True, exist_ok=True)

def split_and_move_data(pairs: List[Tuple[Path, Path]], processed_dir: Path, ratios: tuple):
    """Shuffle and split data into train/val/test sets."""
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
        
        for img_path, lbl_path in split_pairs:
            # Using copy2 to preserve metadata, but could use move if space is an issue
            shutil.copy2(img_path, img_dest / img_path.name)
            shutil.copy2(lbl_path, lbl_dest / lbl_path.name)
        
        print(f"{split_name}: {len(split_pairs)} images")
    
    return splits

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
    parser = argparse.ArgumentParser(description="Organize YOLO Dataset with flexible structure")
    parser.add_argument("--input", "-i", type=str, default="data/raw_import", help="Input directory")
    parser.add_argument("--output", "-o", type=str, default="data/processed", help="Output directory")
    parser.add_argument("--train", type=float, default=0.7, help="Train ratio (0-1)")
    parser.add_argument("--val", type=float, default=0.2, help="Val ratio (0-1)")
    
    args = parser.parse_args()
    
    input_dir = Path(args.input)
    output_dir = Path(args.output)
    
    print("=" * 50)
    print("YOLO Dataset Organizer")
    print("=" * 50)
    
    if not input_dir.exists():
        print(f"Error: {input_dir} not found.")
        return
        
    pairs = find_image_label_pairs(input_dir)
    if not pairs:
        print("No image-label pairs found!")
        return
        
    print(f"Found {len(pairs)} image-label pairs.")
    
    create_directory_structure(output_dir)
    split_and_move_data(pairs, output_dir, (args.train, args.val, 1.0 - args.train - args.val))
    
    class_names = get_class_names()
    generate_data_yaml(output_dir, class_names)
    
    print("\n" + "=" * 50)
    print("Done!")
    print(f"Training command: python3 training-unity.py --data {output_dir}/data.yaml")
    print("=" * 50)

if __name__ == "__main__":
    main()
