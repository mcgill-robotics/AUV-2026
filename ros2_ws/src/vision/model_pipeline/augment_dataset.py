#!/usr/bin/env python3
"""
Heavy Data Augmentation Script for Synthetic Unity Datasets

This script takes an organized dataset (with train/val/test splits), augments
ONLY the training split, and copies val/test as-is into a new output directory.
This prevents data leakage by keeping augmentation isolated to training data.

Supports YOLO and COCO format datasets.

Usage:
    python3 augment_dataset.py --input data/processed --output data/processed_aug --multiplier 3
    python3 augment_dataset.py --input data/processed_coco --output data/processed_coco_aug --multiplier 3 --format coco
"""

import json
import os
import cv2
import yaml
import argparse
import numpy as np
import shutil
import warnings

warnings.filterwarnings("ignore", category=UserWarning, module="albumentations")
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
CLASSES_YAML = SCRIPT_DIR / "classes.yaml"


def get_class_names() -> List[str]:
    """Load class names from classes.yaml (single source of truth)."""
    if CLASSES_YAML.exists():
        try:
            with open(CLASSES_YAML, "r") as f:
                data = yaml.safe_load(f)
                names = data.get("names", [])
                if names:
                    return names
        except Exception:
            pass
    raise FileNotFoundError(
        f"Could not load class names from {CLASSES_YAML}. "
        "Make sure classes.yaml exists in the model_pipeline directory."
    )


def get_augmentation_pipeline(task: str, imgsz: int = None):
    """
    Returns a heavy, "ugly" augmentation pipeline designed to bridge the
    Sim-to-Real gap for underwater robotics.
    """
    transforms = []

    if imgsz:
        transforms.extend(
            [
                A.LongestMaxSize(max_size=imgsz, p=1.0),
                A.PadIfNeeded(
                    min_height=imgsz,
                    min_width=imgsz,
                    border_mode=cv2.BORDER_CONSTANT,
                    fill=(114, 114, 114),
                    p=1.0,
                ),
            ]
        )
    else:
        transforms.append(A.Resize(height=360, width=640, p=1.0))

    transforms.extend(
        [
            # 2. GEOMETRIC (More Aggressive)
            A.HorizontalFlip(p=0.5),
            A.ShiftScaleRotate(
                shift_limit=0.08,
                scale_limit=0.15,
                rotate_limit=5,
                p=0.7,
                border_mode=cv2.BORDER_CONSTANT,
                fill=0,
            ),
            # 3. TEXTURE ANNIHILATION (Forces focus on shape)
            A.OneOf(
                [
                    A.ToGray(p=0.2),  # Much less frequent
                    A.ColorJitter(
                        brightness=0.3, contrast=0.3, saturation=0.4, hue=0.1, p=1.0
                    ),  # Most common in this block
                    A.Solarize(threshold_range=(0.4, 0.6), p=0.2),  # Much less frequent
                    A.ChannelShuffle(p=0.3),
                ],
                p=0.6,
            ),
            A.OneOf(
                [
                    A.RandomBrightnessContrast(
                        brightness_limit=(-0.1, 0.3), contrast_limit=0.3, p=1.0
                    ),
                    A.HueSaturationValue(
                        hue_shift_limit=30,
                        sat_shift_limit=40,
                        val_shift_limit=(-10, 30),
                        p=1.0,
                    ),
                    A.RGBShift(
                        r_shift_limit=30, g_shift_limit=30, b_shift_limit=30, p=1.0
                    ),
                ],
                p=0.7,
            ),
            A.RandomGamma(gamma_limit=(95, 140), p=0.4),
            A.Posterize(num_bits=(3, 5), p=0.2),
            A.ImageCompression(quality_range=(40, 80), p=0.4),
            # 4. UNDERWATER TURBIDITY SIMULATION (Blur, Noise, Backscatter)
            A.OneOf(
                [
                    A.GaussianBlur(
                        blur_limit=3, p=1.0
                    ),  # Fixed to minimum effective blur
                    A.MotionBlur(blur_limit=(3, 5), p=1.0),  # Reduced max motion blur
                    A.RandomFog(
                        fog_coef_range=(0.05, 0.2), alpha_coef=0.08, p=1.0
                    ),  # Lightened fog
                ],
                p=0.5,
            ),
            A.OneOf(
                [
                    A.GaussNoise(std_range=(0.01, 0.05), p=1.0),  # Reduced noise max
                    A.ISONoise(color_shift=(0.01, 0.03), intensity=(0.05, 0.2), p=1.0),
                ],
                p=0.5,
            ),
            # Simulate floating particles
            A.RandomRain(
                slant_range=(-10, 10),
                drop_length=2,
                drop_width=2,
                drop_color=(200, 200, 200),
                blur_value=3,
                brightness_coefficient=0.9,
                rain_type="drizzle",
                p=0.1,
            ),
            # 5. FINAL REFINEMENTS
            A.CLAHE(clip_limit=3.0, p=0.2),
        ]
    )

    # Occlusions (Forces focus on partial shapes)
    if task == "detect":
        transforms.insert(
            len(transforms) - 2,
            A.CoarseDropout(
                num_holes_range=(1, 8),
                hole_height_range=(10, 64 if not imgsz else int(imgsz * 0.1)),
                hole_width_range=(10, 64 if not imgsz else int(imgsz * 0.1)),
                p=0.4,
                fill_value=0,
            ),
        )

    return A.Compose(
        transforms,
        bbox_params=(
            A.BboxParams(
                format="yolo", label_fields=["class_labels"], min_visibility=0.3
            )
            if task == "detect"
            else None
        ),
        keypoint_params=(
            A.KeypointParams(
                format="xy", label_fields=["kp_mapping"], remove_invisible=False
            )
            if task == "segment"
            else None
        ),
    )


def load_yolo_labels(
    label_path: Path, task_type: str, w_orig: int = 1, h_orig: int = 1
) -> dict:
    data = {"bboxes": [], "class_labels": [], "keypoints": [], "kp_mapping": []}
    obj_idx = 0
    if label_path.exists():
        with open(label_path, "r") as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) >= 3:
                    class_id = int(parts[0])
                    coords = [float(x) for x in parts[1:]]

                    if len(coords) == 4 and task_type == "detect":
                        xc, yc, w, h = coords
                        xmin = max(0.0, xc - w / 2)
                        ymin = max(0.0, yc - h / 2)
                        xmax = min(1.0, xc + w / 2)
                        ymax = min(1.0, yc + h / 2)
                        new_w = max(0.0001, xmax - xmin)
                        new_h = max(0.0001, ymax - ymin)
                        new_xc = max(0.0, min(1.0, xmin + new_w / 2))
                        new_yc = max(0.0, min(1.0, ymin + new_h / 2))
                        data["bboxes"].append([new_xc, new_yc, new_w, new_h])
                        data["class_labels"].append(class_id)

                    elif len(coords) >= 6 and task_type == "segment":
                        for i in range(0, len(coords), 2):
                            kx = coords[i] * w_orig
                            ky = coords[i + 1] * h_orig
                            data["keypoints"].append([kx, ky])
                            data["kp_mapping"].append(obj_idx)
                        data["class_labels"].append(class_id)
                        obj_idx += 1
    return data


def save_yolo_labels(
    label_path: Path, data_items: List[List[float]], class_labels: List[int]
):
    with open(label_path, "w") as f:
        for item, cls in zip(data_items, class_labels):
            f.write(f"{cls} {' '.join([f'{x:.6f}' for x in item])}\n")


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp"}


def process_single_image(args_tuple):
    (
        image_path,
        label_path,
        output_dir,
        multiplier,
        output_fmt,
        task_type,
        imgsz,
        task_idx,
    ) = args_tuple

    seed = (int(time.time()) + task_idx) % (2**32)
    random.seed(seed)
    np.random.seed(seed)

    transform = get_augmentation_pipeline(task_type, imgsz)

    image = cv2.imread(str(image_path))
    if image is None:
        return []
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    h_orig, w_orig = image.shape[:2]
    yolo_data = load_yolo_labels(label_path, task_type, w_orig, h_orig)

    class_labels = yolo_data["class_labels"]
    bboxes = yolo_data["bboxes"]
    keypoints = yolo_data["keypoints"]
    kp_mapping = yolo_data["kp_mapping"]

    if output_fmt == "yolo":
        img_output_dir = output_dir / "images"
        lbl_output_dir = output_dir / "labels"
    else:
        img_output_dir = output_dir
        lbl_output_dir = output_dir

    results = []

    for i in range(multiplier):
        try:
            kwargs = {"image": image}
            if task_type == "detect":
                kwargs["bboxes"] = bboxes
                kwargs["class_labels"] = class_labels
            if task_type == "segment":
                kwargs["keypoints"] = keypoints
                kwargs["kp_mapping"] = kp_mapping

            augmented = transform(**kwargs)
            aug_img = augmented["image"]
            img_h, img_w = aug_img.shape[:2]

            aug_data = []
            aug_labels = []

            if task_type == "detect":
                aug_data = augmented.get("bboxes", [])
                aug_labels = augmented.get("class_labels", [])
            elif task_type == "segment":
                aug_kps = augmented.get("keypoints", [])
                aug_kp_mapping = augmented.get("kp_mapping", [])
                if aug_kps:
                    poly_dict = {}
                    for kp, item_idx_float in zip(aug_kps, aug_kp_mapping):
                        item_idx = int(item_idx_float)
                        if item_idx not in poly_dict:
                            poly_dict[item_idx] = []
                        nx = max(0.0, min(1.0, kp[0] / img_w))
                        ny = max(0.0, min(1.0, kp[1] / img_h))
                        poly_dict[item_idx].extend([nx, ny])

                    for item_idx in range(len(class_labels)):
                        if item_idx in poly_dict:
                            aug_data.append(poly_dict[item_idx])
                            aug_labels.append(class_labels[item_idx])

            aug_name = f"{image_path.stem}_aug_{i}"
            out_path = img_output_dir / f"{aug_name}.jpg"

            aug_img_bgr = cv2.cvtColor(aug_img, cv2.COLOR_RGB2BGR)
            success = cv2.imwrite(str(out_path), aug_img_bgr)
            if not success or not out_path.exists():
                continue

            save_yolo_labels(lbl_output_dir / f"{aug_name}.txt", aug_data, aug_labels)

            results.append(
                {
                    "file_name": f"{aug_name}.jpg",
                    "width": img_w,
                    "height": img_h,
                    "bboxes": list(aug_data) if task_type == "detect" else [],
                    "polygons": list(aug_data) if task_type == "segment" else [],
                    "class_labels": list(aug_labels),
                }
            )
        except Exception as e:
            continue

    return results


def run_visualization(
    img_files, lbl_input_dir, output_dir, transform, num_vis, task_type
):
    vis_dir = output_dir / "visualization"
    vis_dir.mkdir(parents=True, exist_ok=True)

    selected = random.sample(img_files, min(len(img_files), num_vis))

    print(f"Generating {len(selected)} visualization samples in {vis_dir}...")

    for img_p in selected:
        lbl_p = lbl_input_dir / (img_p.stem + ".txt")
        if not lbl_p.exists():
            continue

        image = cv2.imread(str(img_p))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        h_orig, w_orig = image.shape[:2]

        yolo_data = load_yolo_labels(lbl_p, task_type, w_orig, h_orig)
        class_labels = yolo_data["class_labels"]
        if len(class_labels) == 0:
            continue

        kwargs = {"image": image}
        if task_type == "detect":
            kwargs["bboxes"] = yolo_data["bboxes"]
            kwargs["class_labels"] = class_labels
        elif task_type == "segment":
            kwargs["keypoints"] = yolo_data["keypoints"]
            kwargs["kp_mapping"] = yolo_data["kp_mapping"]

        augmented = transform(**kwargs)
        aug_img = augmented["image"]
        aug_img_bgr = cv2.cvtColor(aug_img, cv2.COLOR_RGB2BGR)
        h, w, _ = aug_img_bgr.shape

        if task_type == "detect":
            aug_bboxes = augmented.get("bboxes", [])
            for bbox in aug_bboxes:
                x_c, y_c, bw, bh = bbox
                x1 = int((x_c - bw / 2) * w)
                y1 = int((y_c - bh / 2) * h)
                x2 = int((x_c + bw / 2) * w)
                y2 = int((y_c + bh / 2) * h)
                cv2.rectangle(aug_img_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
        else:
            aug_kps = augmented.get("keypoints", [])
            aug_kp_mapping = augmented.get("kp_mapping", [])
            if aug_kps:
                poly_dict = {}
                for kp, item_idx_float in zip(aug_kps, aug_kp_mapping):
                    item_idx = int(item_idx_float)
                    if item_idx not in poly_dict:
                        poly_dict[item_idx] = []
                    poly_dict[item_idx].extend([kp[0], kp[1]])

                for idx, pts in poly_dict.items():
                    pts_arr = np.array(pts, dtype=np.int32).reshape(-1, 2)
                    cv2.polylines(aug_img_bgr, [pts_arr], True, (0, 255, 0), 2)

        cv2.imwrite(str(vis_dir / f"vis_{img_p.name}"), aug_img_bgr)

    print(f"Visualization complete. Check {vis_dir}")


def build_coco_json(
    all_results: List[List[dict]], class_names: List[str], task: str
) -> dict:
    """Build a COCO annotation dict from the collected worker results."""
    categories = [{"id": i, "name": name} for i, name in enumerate(class_names)]
    images = []
    annotations = []
    ann_id = 1
    image_id = 1

    for worker_results in all_results:
        for entry in worker_results:
            images.append(
                {
                    "id": image_id,
                    "file_name": entry["file_name"],
                    "width": entry["width"],
                    "height": entry["height"],
                }
            )

            img_w = entry["width"]
            img_h = entry["height"]

            data_items = (
                entry["bboxes"] if task == "detect" else entry.get("polygons", [])
            )

            for item_data, cls in zip(data_items, entry["class_labels"]):
                segmentation = []

                if task == "detect":
                    xc, yc, w, h = item_data
                    abs_w = w * img_w
                    abs_h = h * img_h
                    abs_x = (xc * img_w) - abs_w / 2
                    abs_y = (yc * img_h) - abs_h / 2
                    poly_area = abs_w * abs_h
                else:  # segment
                    poly_abs = []
                    xs, ys = [], []
                    for i in range(0, len(item_data), 2):
                        px, py = item_data[i] * img_w, item_data[i + 1] * img_h
                        poly_abs.extend([px, py])
                        xs.append(px)
                        ys.append(py)

                    segmentation = [poly_abs]
                    abs_x, abs_y = min(xs), min(ys)
                    abs_w, abs_h = max(xs) - abs_x, max(ys) - abs_y

                    # Calculate true polygon area (Shoelace formula via OpenCV)
                    contour = np.array(poly_abs).reshape(-1, 2)
                    poly_area = cv2.contourArea(np.float32(contour))

                annotations.append(
                    {
                        "id": ann_id,
                        "image_id": image_id,
                        "category_id": int(cls),
                        "segmentation": segmentation,
                        "bbox": [
                            round(abs_x, 2),
                            round(abs_y, 2),
                            round(abs_w, 2),
                            round(abs_h, 2),
                        ],
                        "area": round(poly_area, 2),
                        "iscrowd": 0,
                    }
                )
                ann_id += 1

            image_id += 1

    return {"images": images, "annotations": annotations, "categories": categories}


def detect_dataset_format(input_dir: Path) -> Tuple[str, str]:
    """Detect the format (yolo/coco) and the train split name (train).

    Returns (format, val_split_name) where val_split_name is 'val' or 'valid'.
    """
    # Check for COCO format (train dir has _annotations.coco.json)
    for val_name in ["valid", "val"]:
        train_dir = input_dir / "train"
        if train_dir.exists():
            if (train_dir / "_annotations.coco.json").exists():
                return "coco", val_name
            if (train_dir / "images").exists():
                return "yolo", val_name
    return None, None


def find_images_and_labels(split_dir: Path, fmt: str):
    """Find image and label files in a split directory."""
    if fmt == "yolo":
        img_dir = split_dir / "images"
        lbl_dir = split_dir / "labels"
        if not img_dir.exists():
            return [], None, None
        img_files = [
            p for p in img_dir.iterdir() if p.suffix.lower() in IMAGE_EXTENSIONS
        ]
        return img_files, img_dir, lbl_dir
    else:
        # COCO: images are directly in the split dir
        img_files = [
            p for p in split_dir.iterdir() if p.suffix.lower() in IMAGE_EXTENSIONS
        ]
        # Labels come from YOLO .txt files alongside images (from organize step)
        # But for COCO datasets, labels are in the JSON. We need the .txt files
        # for the augmentation pipeline which uses YOLO internally.
        # Check if there are .txt label files
        lbl_dir = split_dir
        return img_files, split_dir, lbl_dir


def copy_split(src_dir: Path, dst_dir: Path):
    """Copy an entire split directory (val or test) as-is."""
    if not src_dir.exists():
        return
    if dst_dir.exists():
        shutil.rmtree(dst_dir)
    shutil.copytree(src_dir, dst_dir)


def main():
    parser = argparse.ArgumentParser(
        description="Augment training data from an organized dataset",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
Examples:
  python3 augment_dataset.py --input data/processed --output data/processed_aug --multiplier 3
  python3 augment_dataset.py --input data/processed_coco --output data/processed_coco_aug --multiplier 3 --format coco
""",
    )
    parser.add_argument(
        "--input",
        type=str,
        default="data/processed",
        help="Input dataset directory with train/val/test splits",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="data/processed_aug",
        help="Output dataset directory (complete augmented dataset)",
    )
    parser.add_argument(
        "--multiplier",
        type=int,
        default=1,
        help="Number of augmented versions per training image",
    )
    parser.add_argument(
        "--workers", type=int, default=8, help="Number of parallel workers"
    )
    parser.add_argument(
        "--visualize",
        action="store_true",
        help="Visualize few samples in a 'visualization' folder",
    )
    parser.add_argument(
        "--num_vis", type=int, default=10, help="Number of samples to visualize"
    )
    parser.add_argument(
        "--task",
        type=str,
        choices=["detect", "segment"],
        default="detect",
        help="Task type: detect (bounding boxes) or segment (polygons)",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=None,
        help="Target image size for letterboxing (e.g., 640)",
    )
    parser.add_argument(
        "--format",
        "-f",
        type=str,
        choices=["yolo", "coco"],
        default="yolo",
        help="Dataset format: yolo (default) or coco (for RF-DETR)",
    )

    args = parser.parse_args()

    if args.imgsz is None:
        if args.format == "coco":
            if args.task == "segment":
                args.imgsz = 384
            else:
                args.imgsz = 512
        else:
            args.imgsz = 640

    input_dir = Path(args.input).resolve()
    output_dir = Path(args.output).resolve()
    output_fmt = args.format

    if input_dir == output_dir:
        print("Error: input and output directories must be different.")
        return

    # --- Detect split structure ---
    train_dir = input_dir / "train"
    # Detect val split name: 'val' (YOLO) or 'valid' (COCO/RF-DETR)
    if (input_dir / "valid").exists():
        val_name = "valid"
    else:
        val_name = "val"
    val_dir = input_dir / val_name
    test_dir = input_dir / "test"

    if not train_dir.exists():
        print(f"Error: no 'train' directory found in {input_dir}")
        print("Run organize_dataset.py first to split your data.")
        return

    # --- Find training images and labels ---
    if output_fmt == "yolo":
        img_input_dir = train_dir / "images"
        lbl_input_dir = train_dir / "labels"
        if not img_input_dir.exists():
            # Fallback: try flat structure
            img_input_dir = train_dir
            lbl_input_dir = train_dir
    else:
        # COCO: images and .txt labels are both in train/
        # (organize_dataset.py copies .txt labels alongside images)
        img_input_dir = train_dir
        lbl_input_dir = train_dir

    img_files = [
        p for p in img_input_dir.iterdir() if p.suffix.lower() in IMAGE_EXTENSIONS
    ]

    if not img_files:
        print(f"Error: no images found in {img_input_dir}")
        return

    print("=" * 50)
    print(f"Dataset Augmentation  [format: {output_fmt.upper()}]")
    print("=" * 50)
    print(f"Input:      {input_dir}")
    print(f"Output:     {output_dir}")
    print(f"Train imgs: {len(img_files)}")
    print(f"Multiplier: {args.multiplier}")
    print()

    # --- Clear and create output directory ---
    if output_dir.exists():
        print(f"Clearing existing output directory: {output_dir}")
        shutil.rmtree(output_dir)

    # Create output train directory
    out_train_dir = output_dir / "train"
    if output_fmt == "yolo":
        (out_train_dir / "images").mkdir(parents=True, exist_ok=True)
        (out_train_dir / "labels").mkdir(parents=True, exist_ok=True)
    else:
        out_train_dir.mkdir(parents=True, exist_ok=True)

    # --- Copy val and test as-is ---
    if val_dir.exists():
        out_val_name = val_name
        print(f"Copying {val_name}/ as-is...")
        copy_split(val_dir, output_dir / out_val_name)
    if test_dir.exists():
        print(f"Copying test/ as-is...")
        copy_split(test_dir, output_dir / "test")

    # --- Visualization mode ---
    if args.visualize:
        vis_transform = get_augmentation_pipeline(args.task, args.imgsz)
        run_visualization(
            img_files, lbl_input_dir, output_dir, vis_transform, args.num_vis, args.task
        )
        return

    # --- Augment training split ---
    tasks = []
    for idx, img_p in enumerate(img_files):
        lbl_p = lbl_input_dir / (img_p.stem + ".txt")
        if lbl_p.exists():
            tasks.append(
                (
                    img_p,
                    lbl_p,
                    out_train_dir,
                    args.multiplier,
                    output_fmt,
                    args.task,
                    args.imgsz,
                    idx,
                )
            )

    print(f"Augmenting {len(tasks)} training images with {args.workers} workers...")
    all_results = []
    total_augmented = 0
    with ProcessPoolExecutor(max_workers=args.workers) as executor:
        results = list(
            tqdm(executor.map(process_single_image, tasks), total=len(tasks))
        )
        for worker_result in results:
            total_augmented += len(worker_result)
            all_results.append(worker_result)

    print(f"\nCreated {total_augmented} augmented training samples")

    # --- Generate annotation / config files ---
    class_names = get_class_names()

    if output_fmt == "coco":
        # Write COCO JSON for the augmented train split
        coco_dict = build_coco_json(all_results, class_names, args.task)
        json_path = out_train_dir / "_annotations.coco.json"
        with open(json_path, "w") as f:
            json.dump(coco_dict, f, indent=2)
        print(
            f"Generated {json_path.name}: {len(coco_dict['images'])} images, "
            f"{len(coco_dict['annotations'])} annotations"
        )
    else:
        # Generate data.yaml for the augmented YOLO dataset
        yaml_path = output_dir / "data.yaml"
        with open(yaml_path, "w") as f:
            yaml.dump(
                {
                    "path": str(output_dir.absolute()),
                    "train": "train/images",
                    "val": f"{val_name}/images",
                    "test": "test/images",
                    "nc": len(class_names),
                    "names": class_names,
                },
                f,
                sort_keys=False,
            )
        print(f"Generated {yaml_path}")

    print()
    print("=" * 50)
    print("Done! Augmented dataset ready at:")
    print(f"  {output_dir}")
    if output_fmt == "yolo":
        print(f"\nTrain: python3 training.py --data {output_dir}/data.yaml --unity")
    else:
        print(f"\nTrain: python3 training.py --model rfdetr --dataset-dir {output_dir}")
    print("=" * 50)


if __name__ == "__main__":
    main()
