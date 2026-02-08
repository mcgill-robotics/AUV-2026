#!/usr/bin/env python3
"""
Unified YOLO Training Script

Supports training YOLOv8 and YOLOv11 models with configurable parameters.

Usage:
    python3 training.py --model v8   # Train YOLOv8
    python3 training.py --model v11  # Train YOLOv11
    python3 training.py --model v8 --epochs 100 --batch 16
"""

import argparse
import os
import shutil
from pathlib import Path

from ultralytics import YOLO

# Script directory
SCRIPT_DIR = Path(__file__).parent.resolve()


def get_model_weights(model_version: str, size: str) -> str:
    """Get the pretrained weights file for the specified YOLO version and size."""
    if model_version == "v8":
        return f"yolov8{size}.pt"
    elif model_version == "v11":
        # v11 uses 'yolo11' prefix
        return f"yolo11{size}.pt"
    else:
        raise ValueError(f"Unknown model version: {model_version}. Use 'v8' or 'v11'.")


def train(args):
    """Run YOLO training with the specified configuration."""
    
    # Paths
    data_yaml = SCRIPT_DIR / "data/processed/data.yaml"
    
    if not data_yaml.exists():
        print(f"Error: {data_yaml} not found.")
        print("Run 'python3 organize_dataset.py' first to generate it.")
        return
    
    # Model setup
    if (args.custom_model == "False"):
        weights = get_model_weights(args.model, args.size)
        print(f"\n{'='*50}")
        print(f"Training YOLO{args.model}{args.size} ({weights})")
        print(f"{'='*50}")
        
        model = YOLO(weights)

    else:
        model = YOLO(args.custom_model)
    
    # Training run name
    run_name = f"yolo{args.model}{args.size}_sim_dataset"
    
    # Start training
    model.train(
        data=str(data_yaml),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        pretrained=True,
        task="detect",
        cache=args.cache,
        workers=args.workers,
        name=run_name,
        lr0=args.learning_rate,
        # Augmentation parameters (tuned for Unity-generated data)
        # Unity already handled color randomization, so disable HSV augs
        hsv_h=0.01,
        hsv_s=0.3,
        hsv_v=0.3,
        translate=0.2,
        scale=0.1,
        # Left/right flip is fine (symmetry), but no vertical flip (gravity matters)
        fliplr=0.5,
        flipud=0.0,
        # Mosaic helps with detection stability
        mosaic=1.0,
        copy_paste=0.1,
        erasing=0.1,
        # Slight rotation is okay, but not upside-down (AUV rarely sees that)
        degrees=20.0,
    )
    
    # Copy best weights to a convenient location
    best_weights_src = SCRIPT_DIR / "runs" / "detect" / run_name / "weights" / "best.pt"
    best_weights_dst = SCRIPT_DIR / f"best_yolo{args.model}{args.size}_model.pt"
    
    if best_weights_src.exists():
        shutil.copy2(best_weights_src, best_weights_dst)
        print(f"\nBest model saved to: {best_weights_dst}")
    
    print(f"\n{'='*50}")
    print("Training complete!")
    print(f"{'='*50}")


def main():
    parser = argparse.ArgumentParser(
        description="Train YOLO models on your custom dataset",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 training.py --model v8 --size n    # Train YOLOv8-Nano
  python3 training.py --model v11 --size s   # Train YOLOv11-Small
  python3 training.py --epochs 50 --batch 32
        """
    )
    
    parser.add_argument(
        "--model", "-m",
        type=str,
        choices=["v8", "v11"],
        default="v8",
        help="YOLO version to train (default: v8)"
    )
    parser.add_argument(
        "--size", "-s",
        type=str,
        choices=["n", "s", "m", "l", "x"],
        default="n",
        help="Model size: nano, small, medium, large, xlarge (default: n)"
    )
    parser.add_argument(
        "--custom-model", "-c",
        type=str,
        default="False",
        help="Train from an existing YOLO model file (default: False)"
    )
    parser.add_argument(
        "--epochs", "-e",
        type=int,
        default=200,
        help="Number of training epochs (default: 200)"
    )
    parser.add_argument(
        "--batch", "-b",
        type=int,
        default=-1,
        help="Batch size. -1 for auto (60%% GPU memory). (default: -1)"
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=640,
        help="Image size for training (default: 640)"
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=2,
        help="Number of dataloader workers (default: 2)"
    )
    parser.add_argument(
        "--cache",
        action="store_true",
        help="Cache images for faster training"
    )
    parser.add_argument(
        "--learning-rate", "-lr0",
        type=float,
        default=0.01,
        help="Initial learning rate"
    )
    
    args = parser.parse_args()
    train(args)


if __name__ == "__main__":
    main()
