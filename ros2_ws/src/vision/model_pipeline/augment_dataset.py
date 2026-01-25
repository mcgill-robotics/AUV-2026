#!/usr/bin/env python3
"""
Heavy Data Augmentation Script for Synthetic Unity Datasets

This script applies a variety of geometric, color, and noise augmentations
to increase the diversity of the synthetic dataset and improve model robustness.
Supports YOLO format labels.

Usage:
    python3 augment_dataset.py --input data/raw_import --output data/augmented --multiplier 5
"""

import os
import cv2
import yaml
import argparse
import numpy as np
import albumentations as A
from pathlib import Path
from tqdm import tqdm
from concurrent.futures import ProcessPoolExecutor
from typing import List, Tuple
import random
import time
# Class names should match data.yaml
# We try to load from data_unity.yaml if it exists
SCRIPT_DIR = Path(__file__).parent.resolve()
DEFAULT_CLASS_NAMES = ["gate", "lane_marker", "red_pipe", "white_pipe", "octagon", "table", "bin", "board", "shark", "sawfish"]

def get_class_names(config_path: Path) -> List[str]:
    if config_path.exists():
        try:
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)
                return data.get('names', DEFAULT_CLASS_NAMES)
        except Exception:
            pass
    return DEFAULT_CLASS_NAMES

def get_augmentation_pipeline(img_width: int = 640, img_height: int = 360):
    """
    Returns a heavy, "ugly" augmentation pipeline designed to bridge the
    Sim-to-Real gap for underwater robotics.
    """
    return A.Compose([
        # 1. RESIZE FIRST: Match the real camera resolution. This is crucial.
        A.Resize(height=img_height, width=img_width, p=1.0),

        # 2. GEOMETRIC (More Aggressive)
        A.HorizontalFlip(p=0.5),
        A.ShiftScaleRotate(shift_limit=0.08, scale_limit=0.15, rotate_limit=5, p=0.7, border_mode=cv2.BORDER_CONSTANT, fill=0),
        A.Perspective(scale=(0.05, 0.1), p=0.3),
        A.OpticalDistortion(distort_limit=0.1, p=0.3), # Simulates lens/water distortion

        # 3. TEXTURE ANNIHILATION (Forces focus on shape)
        A.OneOf([
            A.ToGray(p=0.2), # Much less frequent
            A.ColorJitter(brightness=0.3, contrast=0.3, saturation=0.4, hue=0.1, p=1.0), # Most common in this block
            A.Solarize(threshold_range=(0.4, 0.6), p=0.2), # Much less frequent
            A.ChannelShuffle(p=0.3),
        ], p=0.4), # Reduced overall probability from 0.6 to 0.4

        A.OneOf([
            A.RandomBrightnessContrast(brightness_limit=(-0.1, 0.3), contrast_limit=0.3, p=1.0),
            A.HueSaturationValue(hue_shift_limit=30, sat_shift_limit=40, val_shift_limit=(-10, 30), p=1.0),
            A.RGBShift(r_shift_limit=30, g_shift_limit=30, b_shift_limit=30, p=1.0),
        ], p=0.7), 

        A.RandomGamma(gamma_limit=(95, 140), p=0.4),
        A.Posterize(num_bits=(3, 5), p=0.2),
        A.ImageCompression(quality_range=(40, 80), p=0.4),

        # 4. UNDERWATER TURBIDITY SIMULATION (Blur, Noise, Backscatter)
        A.OneOf([
            A.GaussianBlur(blur_limit=3, p=1.0), # Fixed to minimum effective blur
            A.MotionBlur(blur_limit=(3, 5), p=1.0), # Reduced max motion blur
            A.RandomFog(fog_coef_range=(0.05, 0.2), alpha_coef=0.08, p=1.0), # Lightened fog
        ], p=0.5),

        A.OneOf([
            A.GaussNoise(std_range=(0.01, 0.05), p=1.0), # Reduced noise max
            A.ISONoise(color_shift=(0.01, 0.03), intensity=(0.05, 0.2), p=1.0),
        ], p=0.5),

        # Simulated water refraction/ripples
        A.ElasticTransform(alpha=1, sigma=50, p=0.2), # Removed deprecated alpha_affine
        
        # Occlusions (Forces focus on partial shapes)
        A.CoarseDropout(
            num_holes_range=(1, 8), 
            hole_height_range=(10, int(img_height*0.1)), 
            hole_width_range=(10, int(img_width*0.1)), 
            p=0.4, 
            fill=0
        ),

        # Simulate floating particles
        A.RandomRain(slant_range=(-10, 10), drop_length=2, drop_width=2, drop_color=(200, 200, 200), blur_value=3, brightness_coefficient=0.9, rain_type='drizzle', p=0.1),

        # 5. FINAL REFINEMENTS
        A.CLAHE(clip_limit=3.0, p=0.2),

    ], bbox_params=A.BboxParams(format='yolo', label_fields=['class_labels'], min_visibility=0.3))
def load_yolo_labels(label_path: Path) -> List[List[float]]:
    labels = []
    if label_path.exists():
        with open(label_path, 'r') as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) == 5:
                    cls = int(parts[0])
                    xc, yc, w, h = [float(x) for x in parts[1:]]
                    
                    # Convert to [xmin, ymin, xmax, ymax] and clip to [0, 1]
                    xmin = max(0.0, xc - w/2)
                    ymin = max(0.0, yc - h/2)
                    xmax = min(1.0, xc + w/2)
                    ymax = min(1.0, yc + h/2)
                    
                    # Convert back to YOLO center/width/height
                    # Use a very tiny epsilon to be extra safe
                    new_w = max(0.0001, xmax - xmin)
                    new_h = max(0.0001, ymax - ymin)
                    new_xc = xmin + new_w / 2
                    new_yc = ymin + new_h / 2
                    
                    # Final safety clip for center
                    new_xc = max(0.0, min(1.0, new_xc))
                    new_yc = max(0.0, min(1.0, new_yc))
                    
                    labels.append([cls, new_xc, new_yc, new_w, new_h])
    return labels

def save_yolo_labels(label_path: Path, bboxes: List[List[float]], class_labels: List[int]):
    with open(label_path, 'w') as f:
        for bbox, cls in zip(bboxes, class_labels):
            f.write(f"{cls} {' '.join([f'{x:.6f}' for x in bbox])}\n")

def process_single_image(args_tuple):
    image_path, label_path, output_dir, multiplier, transform_func, task_idx = args_tuple
    
    # Ensure each worker gets a unique seed for randomness (especially for parallel processing)
    # We use a combination of task index and current time to ensure uniqueness
    seed = (int(time.time()) + task_idx) % (2**32)
    random.seed(seed)
    np.random.seed(seed)
    
    # Re-instantiate the transform if it's not provided or to ensure fresh state
    # (Though passing it in is fine if we seed numpy correctly)
    
    # CRITICAL FIX: Instantiate the pipeline INSIDE the worker to pick up the new seed
    transform = get_augmentation_pipeline()
    
    image = cv2.imread(str(image_path))
    if image is None:
        return 0
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    yolo_data = load_yolo_labels(label_path)
    if not yolo_data:
        # If no labels, we still might want to keep the image for background,
        # but usually in synthetic we expect labels.
        bboxes = []
        class_labels = []
    else:
        # yolo format: [class, x_center, y_center, width, height]
        # Albumentations expects [x_center, y_center, width, height] and then labels
        bboxes = [item[1:] for item in yolo_data]
        class_labels = [int(item[0]) for item in yolo_data]

    img_output_dir = output_dir / "images"
    lbl_output_dir = output_dir / "labels"
    
    # Save original if requested (skipped here as we usually keep it in input)
    
    count = 0
    for i in range(multiplier):
        try:
            augmented = transform(image=image, bboxes=bboxes, class_labels=class_labels)
            aug_img = augmented['image']
            aug_bboxes = augmented['bboxes']
            aug_labels = augmented['class_labels']
            
            aug_name = f"{image_path.stem}_aug_{i}"
            
            # Save image
            aug_img_bgr = cv2.cvtColor(aug_img, cv2.COLOR_RGB2BGR)
            cv2.imwrite(str(img_output_dir / f"{aug_name}.jpg"), aug_img_bgr)
            
            # Save label
            save_yolo_labels(lbl_output_dir / f"{aug_name}.txt", aug_bboxes, aug_labels)
            count += 1
        except Exception as e:
            # print(f"Error augmenting {image_path.name}: {e}")
            continue
            
    return count

def run_visualization(img_files, lbl_input_dir, output_dir, transform, num_vis):
    vis_dir = output_dir / "visualization"
    vis_dir.mkdir(parents=True, exist_ok=True)
    
    import random
    selected = random.sample(img_files, min(len(img_files), num_vis))
    
    print(f"Generating {len(selected)} visualization samples in {vis_dir}...")
    
    for img_p in selected:
        lbl_p = lbl_input_dir / (img_p.stem + ".txt")
        if not lbl_p.exists():
            continue
            
        image = cv2.imread(str(img_p))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        yolo_data = load_yolo_labels(lbl_p)
        bboxes = [item[1:] for item in yolo_data]
        class_labels = [int(item[0]) for item in yolo_data]
        
        augmented = transform(image=image, bboxes=bboxes, class_labels=class_labels)
        aug_img = augmented['image']
        aug_bboxes = augmented['bboxes']
        
        # Draw bboxes
        aug_img_bgr = cv2.cvtColor(aug_img, cv2.COLOR_RGB2BGR)
        h, w, _ = aug_img_bgr.shape
        for bbox in aug_bboxes:
            x_c, y_c, bw, bh = bbox
            x1 = int((x_c - bw/2) * w)
            y1 = int((y_c - bh/2) * h)
            x2 = int((x_c + bw/2) * w)
            y2 = int((y_c + bh/2) * h)
            cv2.rectangle(aug_img_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
        cv2.imwrite(str(vis_dir / f"vis_{img_p.name}"), aug_img_bgr)
    
    print(f"Visualization complete. Check {vis_dir}")

def main():
    parser = argparse.ArgumentParser(description="Heavy Augmentation for YOLO Synthetic Dataset")
    parser.add_argument("--input", type=str, default="data/raw_import", help="Input directory (should contain images/ and labels/)")
    parser.add_argument("--output", type=str, default="data/augmented", help="Output directory")
    parser.add_argument("--multiplier", type=int, default=1, help="Number of augmented versions per image")
    parser.add_argument("--workers", type=int, default=8, help="Number of parallel workers")
    parser.add_argument("--visualize", action="store_true", help="Visualize few samples in a 'visualization' folder")
    parser.add_argument("--num_vis", type=int, default=10, help="Number of samples to visualize")
    
    args = parser.parse_args()
    
    input_dir = Path(args.input)
    output_dir = Path(args.output)
    
    # Create output structure (and clear existing if any)
    if output_dir.exists():
        import shutil
        print(f"Clearing existing output directory: {output_dir}")
        shutil.rmtree(output_dir)
    
    (output_dir / "images").mkdir(parents=True, exist_ok=True)
    (output_dir / "labels").mkdir(parents=True, exist_ok=True)
    
    # Detect image and label directories
    potential_img_dirs = [input_dir / "images", input_dir / "yolo_images", input_dir]
    potential_lbl_dirs = [input_dir / "labels", input_dir / "yolo_labels", input_dir]
    
    img_input_dir = None
    lbl_input_dir = None
    
    for d in potential_img_dirs:
        if d.exists() and (list(d.glob("*.jpg")) or list(d.glob("*.png"))):
            img_input_dir = d
            break
            
    for d in potential_lbl_dirs:
        if d.exists() and list(d.glob("*.txt")):
            lbl_input_dir = d
            break
            
    if not img_input_dir or not lbl_input_dir:
        print(f"Error: Could not find images or labels in {input_dir}")
        print(f"Looked in: {potential_img_dirs}")
        return
        
    img_files = list(img_input_dir.glob("*.jpg")) + list(img_input_dir.glob("*.png"))
    print(f"Found {len(img_files)} images in {img_input_dir}")
    print(f"Using labels from {lbl_input_dir}")
    
    if args.visualize:
        # Create transform here for visualization only
        vis_transform = get_augmentation_pipeline()
        run_visualization(img_files, lbl_input_dir, output_dir, vis_transform, args.num_vis)
        return

    tasks = []
    for idx, img_p in enumerate(img_files):
        lbl_p = lbl_input_dir / (img_p.stem + ".txt")
        if lbl_p.exists():
            # Pass None for transform_func since we create it inside the worker now
            tasks.append((img_p, lbl_p, output_dir, args.multiplier, None, idx))

    print(f"Processing in parallel with {args.workers} workers...")
    total_augmented = 0
    with ProcessPoolExecutor(max_workers=args.workers) as executor:
        results = list(tqdm(executor.map(process_single_image, tasks), total=len(tasks)))
        total_augmented = sum(results)
    
    print(f"\nDone! Created {total_augmented} augmented samples in {output_dir}")

    # Create a small data.yaml for the augmented dataset if needed
    data_unity_path = SCRIPT_DIR / "data_unity.yaml"
    class_names = get_class_names(data_unity_path)
    
    aug_yaml_path = output_dir / "data_aug.yaml"
    with open(aug_yaml_path, 'w') as f:
        yaml.dump({
            'path': str(output_dir.absolute()),
            'train': 'images',
            'val': 'images', # Using same for simple validation check if needed
            'nc': len(class_names),
            'names': class_names
        }, f)
    print(f"Generated helper YAML at {aug_yaml_path}")

if __name__ == "__main__":
    main()
