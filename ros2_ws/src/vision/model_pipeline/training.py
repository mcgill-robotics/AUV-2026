#!/usr/bin/env python3
"""
Unified Training Script

Supports training YOLOv8, YOLOv11, and RF-DETR models with configurable parameters.

Usage:
    python3 training.py --model v8   # Train YOLOv8
    python3 training.py --model v11  # Train YOLOv11
    python3 training.py --model rfdetr  # Train RF-DETR Base
    python3 training.py --model rfdetr --size l  # Train RF-DETR Large
    python3 training.py --model v8 --epochs 100 --batch 16
"""

import argparse
from dataclasses import dataclass
import os
import shutil
from pathlib import Path

# Script directory
SCRIPT_DIR = Path(__file__).parent.resolve()

@dataclass
class TrainingParameters:
    hsv_h: float
    hsv_s: float
    hsv_v: float
    translate: float
    scale: float
    fliplr: float
    flipud: float
    mosaic: float
    copy_paste: float
    erasing: float
    degrees: float

#Unity handles training paramaters, so most are disabled
unity_parameters = TrainingParameters (
    hsv_h = 0.0, 
    hsv_s = 0.0, 
    hsv_v = 0.0, 
    translate = 0.2, 
    scale = 0.0, 
    fliplr = 0.5, 
    flipud = 0.0, 
    mosaic = 1.0, 
    copy_paste = 0.1, 
    erasing = 0.1, 
    degrees = 0.0
)
real_parameters = TrainingParameters (
    hsv_h = 0.01,
    hsv_s = 0.3,
    hsv_v = 0.3,
    translate = 0.2,
    scale = 0.1,
    fliplr = 0.5,
    flipud = 0.0,
    mosaic = 1.0,
    copy_paste = 0.1,
    erasing = 0.1,
    degrees = 20.0
)

def get_model_weights(model_version: str, size: str) -> str:
    """Get the pretrained weights file for the specified YOLO version and size."""
    if model_version == "v8":
        return f"yolov8{size}.pt"
    elif model_version == "v11":
        # v11 uses 'yolo11' prefix
        return f"yolo11{size}.pt"
    else:
        raise ValueError(f"Unknown model version: {model_version}. Use 'v8' or 'v11'.")


def train_yolo(args):
    """Run YOLO training with the specified configuration."""
    from ultralytics import YOLO

    # Paths
    data_yaml = args.data
    
    if not data_yaml.exists():
        print(f"Error: {data_yaml} not found.")
        print("Run 'python3 organize_dataset.py' first to generate it.")
        return
    
    # Model setup
    if (args.custom_model):
        model = YOLO(args.custom_model)
        run_name = "yolo11s"
    else:
        weights = get_model_weights(args.model, args.size)
        print(f"\n{'='*50}")
        print(f"Training YOLO{args.model}{args.size} ({weights})")
        print(f"{'='*50}")
        
        run_name = f"yolo{args.model}{args.size}"
        model = YOLO(weights)
    
    # Training parameters
    if (args.unity):
        training_parameters = unity_parameters
    else:
        training_parameters = real_parameters
    
    # Start training
    
    model.train(
        data = str(data_yaml),
        epochs = args.epochs,
        imgsz = args.imgsz,
        batch = args.batch,
        pretrained = True,
        task = "detect",
        cache = args.cache,
        workers = args.workers,
        name = run_name,
        lr0 = args.learning_rate,
        # Augmentation parameters
        hsv_h = training_parameters.hsv_h,
        hsv_s = training_parameters.hsv_s,
        hsv_v = training_parameters.hsv_v,
        translate = training_parameters.translate,
        scale = training_parameters.scale,
        fliplr = training_parameters.fliplr,
        flipud = training_parameters.flipud,
        mosaic = training_parameters.mosaic,
        copy_paste = training_parameters.copy_paste,
        erasing = training_parameters.erasing,
        degrees = training_parameters.degrees,
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


def train_rfdetr(args):
    """Run RF-DETR training with the specified configuration."""

    # Dataset path — RF-DETR expects a COCO-format directory with
    # train/, valid/, and test/ subdirectories each containing
    # _annotations.coco.json plus images.
    dataset_dir = args.dataset_dir or str(SCRIPT_DIR / "data" / "processed")

    if not Path(dataset_dir).exists():
        print(f"Error: dataset directory '{dataset_dir}' not found.")
        print("Provide a COCO-format dataset with train/valid/test splits.")
        return

    # Output directory
    output_dir = args.output_dir or str(SCRIPT_DIR / "runs" / "rfdetr")

    # Determine model class based on size
    rfdetr_models = {
        "n": ("rfdetr", "RFDETRNano", "RF-DETR-Nano"),
        "s": ("rfdetr", "RFDETRSmall", "RF-DETR-Small"),
        "m": ("rfdetr", "RFDETRMedium", "RF-DETR-Medium"),
        "l": ("rfdetr", "RFDETRLarge", "RF-DETR-Large"),
    }

    size = args.size
    if size not in rfdetr_models:
        print(f"Error: RF-DETR does not support size '{size}'. Use n/s/m/l.")
        return

    module_name, class_name, model_label = rfdetr_models[size]
    import importlib
    rfdetr_module = importlib.import_module(module_name)
    RFDETRModel = getattr(rfdetr_module, class_name)

    print(f"\n{'='*50}")
    print(f"Training {model_label}")
    print(f"  Dataset:       {dataset_dir}")
    print(f"  Output:        {output_dir}")
    print(f"  Epochs:        {args.epochs}")
    print(f"  Batch size:    {args.batch}")
    print(f"  Grad accum:    {args.grad_accum_steps}")
    print(f"  Learning rate: {args.learning_rate}")
    if args.resume:
        print(f"  Resume from:   {args.resume}")
    print(f"{'='*50}")

    # Create model (optionally from a checkpoint)
    model_kwargs = {}
    if args.custom_model:
        model_kwargs["pretrain_weights"] = args.custom_model

    model = RFDETRModel(**model_kwargs)

    # Build training kwargs
    train_kwargs = dict(
        dataset_dir=dataset_dir,
        epochs=args.epochs,
        batch_size=args.batch,
        grad_accum_steps=args.grad_accum_steps,
        lr=args.learning_rate,
        output_dir=output_dir,
        progress_bar="tqdm",          # Show per-step progress in terminal
        tensorboard=not args.no_tensorboard,  # TensorBoard ON by default
    )

    if args.resume:
        train_kwargs["resume"] = args.resume

    if args.early_stopping:
        train_kwargs["early_stopping"] = True

    if args.wandb:
        train_kwargs["wandb"] = True

    model.train(**train_kwargs)

    # Print TensorBoard instructions
    if not args.no_tensorboard:
        print(f"\nTensorBoard logs saved to: {output_dir}")
        print(f"  View with: python3 -m tensorboard.main --logdir {output_dir}")

    # Copy best weights to a convenient location
    best_weights_src = Path(output_dir) / "checkpoint_best_total.pth"
    best_weights_dst = SCRIPT_DIR / f"best_{model_label.lower().replace('-', '_')}_model.pth"

    if best_weights_src.exists():
        shutil.copy2(best_weights_src, best_weights_dst)
        print(f"\nBest model saved to: {best_weights_dst}")

    print(f"\n{'='*50}")
    print("Training complete!")
    print(f"{'='*50}")


def train(args):
    """Dispatch to the appropriate training function."""
    if args.model == "rfdetr":
        train_rfdetr(args)
    else:
        train_yolo(args)


def main():
    parser = argparse.ArgumentParser(
        description="Train YOLO or RF-DETR models on your custom dataset",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
Examples:
  python3 training.py --model v8 --size n        # Train YOLOv8-Nano
  python3 training.py --model v11 --size s        # Train YOLOv11-Small
  python3 training.py --model rfdetr              # Train RF-DETR Base
  python3 training.py --model rfdetr --size l     # Train RF-DETR Large
  python3 training.py --model rfdetr --batch 4 --grad-accum-steps 4
  python3 training.py --epochs 50 --batch 32
        """
    )
    
    parser.add_argument(
        "--model", "-m",
        type=str,
        choices=["v8", "v11", "rfdetr"],
        default="v8",
        help="Model to train: v8, v11, or rfdetr (default: v8)"
    )
    parser.add_argument(
        "--size", "-s",
        type=str,
        choices=["n", "s", "m", "l", "x"],
        default="s",
        help="Model size. YOLO: n/s/m/l/x. RF-DETR: n/s/m/l. (default: s)"
    )
    parser.add_argument(
        "--custom-model", "-c",
        type=str,
        help="Train from an existing model file (YOLO .pt or RF-DETR .pth)"
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
        help="Batch size. YOLO: -1 for auto. RF-DETR: set explicitly (default: -1)"
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=640,
        help="Image size for training — YOLO only (default: 640)"
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=2,
        help="Number of dataloader workers — YOLO only (default: 2)"
    )
    parser.add_argument(
        "--cache",
        action="store_true",
        help="Cache images for faster training — YOLO only"
    )
    parser.add_argument(
        "--learning-rate", "-lr0",
        type=float,
        default=None,
        help="Initial learning rate (default: 0.0003 for YOLO, 1e-4 for RF-DETR)"
    )
    parser.add_argument(
        "--unity",
        action="store_true",
        help="Use training parameters for unity dataset — YOLO only"
    )

    # --- RF-DETR specific arguments ---
    rfdetr_group = parser.add_argument_group("RF-DETR options")
    rfdetr_group.add_argument(
        "--dataset-dir",
        type=str,
        default=None,
        help="Path to COCO-format dataset (train/valid/test splits). "
             "Default: data/processed"
    )
    rfdetr_group.add_argument(
        "--output-dir",
        type=str,
        default=None,
        help="Directory for RF-DETR output checkpoints. Default: runs/rfdetr"
    )
    rfdetr_group.add_argument(
        "--grad-accum-steps",
        type=int,
        default=4,
        help="Gradient accumulation steps (default: 4). "
             "Adjust with --batch to keep effective batch size at 16."
    )
    rfdetr_group.add_argument(
        "--resume",
        type=str,
        default=None,
        help="Path to a checkpoint.pth to resume training from"
    )
    rfdetr_group.add_argument(
        "--early-stopping",
        action="store_true",
        help="Enable early stopping based on validation mAP"
    )
    rfdetr_group.add_argument(
        "--no-tensorboard",
        action="store_true",
        default=False,
        help="Disable TensorBoard logging (enabled by default)"
    )
    rfdetr_group.add_argument(
        "--wandb",
        action="store_true",
        help="Enable Weights & Biases logging"
    )
    parser.add_argument(
        "--data",
        type=Path,
        default=SCRIPT_DIR / "data" / "processed" / "data.yaml",
        help="data.yaml location"
    )
    
    args = parser.parse_args()

    # Apply model-specific learning rate defaults
    if args.learning_rate is None:
        args.learning_rate = 1e-4 if args.model == "rfdetr" else 0.0003

    # Apply sensible RF-DETR batch default (instead of YOLO's -1 auto)
    if args.model == "rfdetr" and args.batch == -1:
        args.batch = 4

    train(args)


if __name__ == "__main__":
    main()
