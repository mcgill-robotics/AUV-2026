#!/usr/bin/env python3
import argparse
import importlib

def main():
    parser = argparse.ArgumentParser(description="Export RF-DETR model to ONNX/TensorRT")
    parser.add_argument(
        "--model", "-m",
        type=str,
        required=True,
        help="Path to pre-trained weights (.pth)"
    )
    parser.add_argument(
        "--size", "-s",
        type=str,
        choices=["n", "s", "m", "l"],
        default="s",
        help="Model size: n/s/m/l (default: s)"
    )
    parser.add_argument(
        "--task", "-t",
        type=str,
        choices=["detect", "segment"],
        default="detect",
        help="Task type: detect or segment (default: detect)"
    )
    args = parser.parse_args()

    if args.task == "segment":
        rfdetr_models = {
            "n": ("rfdetr", "RFDETRSegNano"),
            "s": ("rfdetr", "RFDETRSegSmall"),
            "m": ("rfdetr", "RFDETRSegMedium"),
            "l": ("rfdetr", "RFDETRSegLarge"),
        }
    else:
        rfdetr_models = {
            "n": ("rfdetr", "RFDETRNano"),
            "s": ("rfdetr", "RFDETRSmall"),
            "m": ("rfdetr", "RFDETRMedium"),
            "l": ("rfdetr", "RFDETRLarge"),
        }

    module_name, class_name = rfdetr_models[args.size]
    rfdetr_module = importlib.import_module(module_name)
    RFDETRModel = getattr(rfdetr_module, class_name)

    print(f"Loading {class_name} with weights from {args.model}")
    model = RFDETRModel(pretrain_weights=args.model)
    
    print("Exporting model...")
    model.export()
    print("Export complete!")

if __name__ == "__main__":
    main()
