#!/usr/bin/env python3
"""
Random Image Inference Viewer

Load random images from a folder, run YOLO inference, and display results.

Usage:
    python3 random_inference.py --folder data/processed/test/images
    python3 random_inference.py --folder /path/to/images --model best_yolov11s_model.pt

Controls:
    - Press any key to load the next random image
    - Press 'q' to quit
"""

import argparse
import random
from pathlib import Path

import cv2
from ultralytics import YOLO

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


def main():
    parser = argparse.ArgumentParser(description="View YOLO predictions on random images")
    parser.add_argument(
        "--folder", "-f",
        type=str,
        required=True,
        help="Path to folder containing images"
    )
    parser.add_argument(
        "--model", "-m",
        type=str,
        default=str(BASE_DIR / "best_yolov11s_model.pt"),
        help="Path to YOLO model weights"
    )
    parser.add_argument(
        "--conf", "-c",
        type=float,
        default=0.5,
        help="Confidence threshold (default: 0.5)"
    )
    args = parser.parse_args()

    # Load model
    model = YOLO(args.model)
    print(f"Loaded model: {args.model}")

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
        results = model.predict(img, verbose=False, conf=args.conf)

        # Draw results
        annotated = results[0].plot()

        # Show
        cv2.imshow("YOLO Inference - Random Image", annotated)

        # Wait for keypress
        key = cv2.waitKey(0) & 0xFF
        if key == ord('q'):
            break

    cv2.destroyAllWindows()
    print("Done.")


if __name__ == "__main__":
    main()
