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
from typing import List, Tuple, Dict, Union
import albumentations as A

LabelItem = Dict[str, Union[int, List[float]]]

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

    print(
        f"Using images from: {safe_relative(img_dir, SCRIPT_DIR.parent.parent.parent.parent.parent.parent)}"
    )
    print(
        f"Using labels from: {safe_relative(lbl_dir, SCRIPT_DIR.parent.parent.parent.parent.parent.parent)}"
    )

    for img_path in img_dir.iterdir():
        if img_path.suffix.lower() not in IMAGE_EXTENSIONS:
            continue

        label_path = lbl_dir / (img_path.stem + ".txt")
        if label_path.exists():
            pairs.append((img_path, label_path))

    return pairs


def load_labels(label_path: Path, task: str) -> List[LabelItem]:
    """Unified label loader for both detection and segmentation."""
    items = []
    if not label_path.exists():
        return items

    with open(label_path, "r") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 3:
                continue

            cls = int(float(parts[0]))

            if task == "detect" and len(parts) == 5:
                xc, yc, w, h = [float(x) for x in parts[1:]]
                xmin = max(0.0, xc - w / 2)
                ymin = max(0.0, yc - h / 2)
                xmax = min(1.0, xc + w / 2)
                ymax = min(1.0, yc + h / 2)
                new_w = max(0.000001, xmax - xmin)
                new_h = max(0.000001, ymax - ymin)
                new_xc = max(0.0, min(1.0, xmin + new_w / 2))
                new_yc = max(0.0, min(1.0, ymin + new_h / 2))
                items.append({"class_id": cls, "data": [new_xc, new_yc, new_w, new_h]})

            elif task == "segment" and len(parts) >= 7:
                coords = [float(x) for x in parts[1:]]
                items.append({"class_id": cls, "data": coords})

    return items


def save_yolo_labels(label_path: Path, items: list):
    with open(label_path, "w") as f:
        for item in items:
            cls = item["class_id"]
            data = item["data"]
            if len(data) == 4:
                line = (
                    f"{cls} {data[0]:.6f} {data[1]:.6f} {data[2]:.6f} {data[3]:.6f}\n"
                )
            else:
                coords_str = " ".join([f"{x:.6f}" for x in data])
                line = f"{cls} {coords_str}\n"
            f.write(line)


def process_and_save_image(
    img_path: Path,
    lbl_path: Path,
    items: List[LabelItem],
    img_dest: Path,
    lbl_dest: Path,
    task: str,
    letterbox_size: int = None,
):
    """Copies or resizes the image/label. Returns the new (height, width) of the saved image."""
    if not letterbox_size:
        shutil.copy2(img_path, img_dest)
        if lbl_path.exists():
            shutil.copy2(lbl_path, lbl_dest)
        else:
            save_yolo_labels(lbl_dest, items)
        # we still need height/width for COCO, so we read it
        img = cv2.imread(str(img_dest))
        return img.shape[:2] if img is not None else (0, 0)

    transform = None
    if letterbox_size:
        transform = A.Compose(
            [
                A.LongestMaxSize(max_size=letterbox_size),
                A.PadIfNeeded(
                    min_height=letterbox_size,
                    min_width=letterbox_size,
                    border_mode=cv2.BORDER_CONSTANT,
                    fill=(114, 114, 114),
                ),
            ],
            bbox_params=(
                A.BboxParams(
                    format="yolo",
                    label_fields=["class_labels_bbox"],
                    min_visibility=0.0,
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

    image = cv2.imread(str(img_path))
    if image is None:
        return (0, 0)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    img_h, img_w = image.shape[:2]

    bboxes = []
    class_labels_bbox = []
    kps = []
    kp_mapping = []

    if task == "detect":
        for item in items:
            bboxes.append(item["data"])
            class_labels_bbox.append(item["class_id"])
    if task == "segment":
        for idx, item in enumerate(items):
            poly = item["data"]
            for i in range(0, len(poly), 2):
                kx = poly[i] * img_w
                ky = poly[i + 1] * img_h
                kps.append([kx, ky])
                kp_mapping.append(idx)

    new_h, new_w = img_h, img_w
    if transform:
        kwargs = {"image": image}
        if task == "detect":
            kwargs["bboxes"] = bboxes
            kwargs["class_labels_bbox"] = class_labels_bbox
        if task == "segment":
            kwargs["keypoints"] = kps
            kwargs["kp_mapping"] = kp_mapping

        aug = transform(**kwargs)
        image = aug["image"]
        new_h, new_w = image.shape[:2]

        if task == "detect":
            bboxes = aug.get("bboxes", [])
            class_labels_bbox = aug.get("class_labels_bbox", [])
        if task == "segment":
            augmented = aug

    out_items = []
    if task == "detect":
        for bbox, cls in zip(bboxes, class_labels_bbox):
            out_items.append({"class_id": cls, "data": bbox})

    if task == "segment" and transform and "keypoints" in augmented:
        aug_kps = augmented["keypoints"]
        aug_kp_mapping = augmented.get("kp_mapping", [])
        poly_dict = {}
        for kp, item_idx_float in zip(aug_kps, aug_kp_mapping):
            item_idx = int(item_idx_float)
            if item_idx not in poly_dict:
                poly_dict[item_idx] = []
            nx = max(0.0, min(1.0, kp[0] / new_w))
            ny = max(0.0, min(1.0, kp[1] / new_h))
            poly_dict[item_idx].extend([nx, ny])

        for item_idx, coords in poly_dict.items():
            cls = items[item_idx]["class_id"]
            out_items.append({"class_id": cls, "data": coords})
    elif task == "segment" and not transform:
        out_items = items

    aug_img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    cv2.imwrite(str(img_dest), aug_img)
    save_yolo_labels(lbl_dest, out_items)

    return new_h, new_w


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


def split_and_move_data_yolo(
    pairs: List[Tuple[Path, Path]],
    processed_dir: Path,
    ratios: tuple,
    task: str,
    letterbox_size: int = None,
):
    """Shuffle and split data into train/val/test sets (YOLO format)."""
    random.shuffle(pairs)

    train_ratio, val_ratio, _ = ratios
    n = len(pairs)
    train_end = int(n * train_ratio)
    val_end = train_end + int(n * val_ratio)

    splits = {
        "train": pairs[:train_end],
        "val": pairs[train_end:val_end],
        "test": pairs[val_end:],
    }

    for split_name, split_pairs in splits.items():
        img_dest = processed_dir / split_name / "images"
        lbl_dest = processed_dir / split_name / "labels"

        for img_path, lbl_path in tqdm(split_pairs, desc=f"{split_name}"):
            items = load_labels(lbl_path, task)
            process_and_save_image(
                img_path,
                lbl_path,
                items,
                img_dest / img_path.name,
                lbl_dest / lbl_path.name,
                task,
                letterbox_size,
            )

    return splits


def yolo_to_coco_annotation(
    label_path: Path,
    image_id: int,
    img_width: int,
    img_height: int,
    ann_id_start: int,
    task: str,
) -> Tuple[list, int]:
    """Convert a single YOLO label file to a list of COCO annotation dicts.

    Returns:
        (annotations_list, next_ann_id)
    """
    annotations = []
    ann_id = ann_id_start

    if not label_path.exists():
        return annotations, ann_id

    with open(label_path, "r") as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 3:
                continue

            cls = int(float(parts[0]))

            if task == "detect" and len(parts) == 5:
                xc, yc, w, h = [float(x) for x in parts[1:]]

                abs_w = w * img_width
                abs_h = h * img_height
                abs_x = (xc * img_width) - abs_w / 2
                abs_y = (yc * img_height) - abs_h / 2

                # Clamp bbox to image boundaries
                abs_x = max(0.0, min(img_width - 1.0, abs_x))
                abs_y = max(0.0, min(img_height - 1.0, abs_y))
                abs_w = max(0.0, min(img_width - abs_x, abs_w))
                abs_h = max(0.0, min(img_height - abs_y, abs_h))

                segmentation = []
                area = round(abs_w * abs_h, 2)

            elif task == "segment" and len(parts) >= 7:
                coords = [float(x) for x in parts[1:]]
                poly_abs = []
                xs = []
                ys = []
                for i in range(0, len(coords), 2):
                    px = coords[i] * img_width
                    py = coords[i + 1] * img_height
                    poly_abs.extend([px, py])
                    xs.append(px)
                    ys.append(py)

                abs_x = min(xs)
                abs_y = min(ys)
                abs_w = max(xs) - abs_x
                abs_h = max(ys) - abs_y
                segmentation = [poly_abs]

                contour = np.array(poly_abs).reshape(-1, 2)
                area = round(cv2.contourArea(np.float32(contour)), 2)
            else:
                continue

            annotations.append(
                {
                    "id": ann_id,
                    "image_id": image_id,
                    "category_id": cls,
                    "segmentation": segmentation,
                    "bbox": [
                        round(abs_x, 2),
                        round(abs_y, 2),
                        round(abs_w, 2),
                        round(abs_h, 2),
                    ],
                    "area": area,
                    "iscrowd": 0,
                }
            )
            ann_id += 1

    return annotations, ann_id


def split_and_move_data_coco(
    pairs: List[Tuple[Path, Path]],
    processed_dir: Path,
    ratios: tuple,
    class_names: list,
    task: str,
    letterbox_size: int = None,
):
    """Shuffle and split data into train/valid/test sets (COCO format)."""
    random.shuffle(pairs)

    train_ratio, val_ratio, _ = ratios
    n = len(pairs)
    train_end = int(n * train_ratio)
    val_end = train_end + int(n * val_ratio)

    splits = {
        "train": pairs[:train_end],
        "valid": pairs[train_end:val_end],
        "test": pairs[val_end:],
    }

    categories = [{"id": i, "name": name} for i, name in enumerate(class_names)]

    for split_name, split_pairs in splits.items():
        split_dir = processed_dir / split_name
        images_list = []
        annotations_list = []
        ann_id = 1

        for image_id, (img_path, lbl_path) in enumerate(
            tqdm(split_pairs, desc=f"{split_name}"), start=1
        ):
            items = load_labels(lbl_path, task)

            dst_img = split_dir / img_path.name
            dst_lbl = split_dir / lbl_path.name

            img_height, img_width = process_and_save_image(
                img_path, lbl_path, items, dst_img, dst_lbl, task, letterbox_size
            )

            if img_height == 0:
                continue

            images_list.append(
                {
                    "id": image_id,
                    "file_name": img_path.name,
                    "width": img_width,
                    "height": img_height,
                }
            )

            anns, ann_id = yolo_to_coco_annotation(
                dst_lbl, image_id, img_width, img_height, ann_id, task
            )
            annotations_list.extend(anns)

        coco_dict = {
            "images": images_list,
            "annotations": annotations_list,
            "categories": categories,
        }

        json_path = split_dir / "_annotations.coco.json"
        with open(json_path, "w") as f:
            json.dump(coco_dict, f, indent=2)

        print(
            f"{split_name}: {len(split_pairs)} images, "
            f"{len(annotations_list)} annotations → {json_path.name}"
        )


def generate_data_yaml(processed_dir: Path, class_names: list):
    """Generate the data.yaml file."""
    yaml_path = processed_dir / "data.yaml"
    content = {
        "path": str(processed_dir.absolute()),
        "train": "train/images",
        "val": "val/images",
        "test": "test/images",
        "nc": len(class_names),
        "names": class_names,
    }

    with open(yaml_path, "w") as f:
        yaml.dump(content, f, sort_keys=False)
    print(f"\nGenerated {yaml_path}")


def main():
    parser = argparse.ArgumentParser(
        description="Organize YOLO/COCO Dataset with flexible structure"
    )
    parser.add_argument(
        "--input", "-i", type=str, default="data/raw_import", help="Input directory"
    )
    parser.add_argument(
        "--output", "-o", type=str, default="data/processed", help="Output directory"
    )
    parser.add_argument("--train", type=float, default=0.7, help="Train ratio (0-1)")
    parser.add_argument("--val", type=float, default=0.2, help="Val ratio (0-1)")
    parser.add_argument(
        "--task",
        type=str,
        choices=["detect", "segment"],
        default="detect",
        help="Task type: detect (bounding boxes) or segment (polygons)",
    )
    parser.add_argument(
        "--format",
        "-f",
        type=str,
        choices=["yolo", "coco"],
        default="yolo",
        help="Output format: yolo (default) or coco (for RF-DETR)",
    )
    parser.add_argument(
        "--letterbox",
        type=int,
        default=None,
        help="Optional size to letterbox pad the images to (e.g., 512 for RF-DETR).",
    )

    args = parser.parse_args()

    if args.letterbox is None:
        if args.format == "coco":
            if args.task == "segment":
                args.letterbox = 384
            else:
                args.letterbox = 512
        else:
            args.letterbox = 640

    input_dir = Path(args.input)
    output_dir = Path(args.output)
    fmt = args.format

    print("=" * 50)
    print(f"Dataset Organizer  [format: {fmt.upper()}]")
    print("=" * 50)

    if not input_dir.exists():
        print(f"Error: {input_dir} not found.")
        return

    class_names = get_class_names()
    ratios = (args.train, args.val, 1.0 - args.train - args.val)

    pairs = find_image_label_pairs(input_dir)
    if not pairs:
        print("No image-label pairs found!")
        return

    print(f"Found {len(pairs)} image-label pairs.")
    create_directory_structure(output_dir, fmt)

    if fmt == "yolo":
        split_and_move_data_yolo(pairs, output_dir, ratios, args.task, args.letterbox)
        generate_data_yaml(output_dir, class_names)
    else:
        split_and_move_data_coco(
            pairs, output_dir, ratios, class_names, args.task, args.letterbox
        )

    print("\n" + "=" * 50)
    print("Done!")
    if fmt == "yolo":
        print(f"Training command: python3 training.py --data {output_dir}/data.yaml")
    else:
        print(
            f"Training command: python3 training.py --model rfdetr --dataset-dir {output_dir}"
        )
    print("=" * 50)


if __name__ == "__main__":
    main()
