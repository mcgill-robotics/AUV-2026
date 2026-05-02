#!/bin/bash

usage() {
    echo ""
    echo "Usage:"
    echo "  Fine-tuning : ./training.sh <model_path> [--model-type yolo|rfdetr] [--organize-args \"--flag value\"] [--training-args \"--flag value\"]"
    echo "  Synthetic   : ./training.sh --mode synthetic [--model-type yolo|rfdetr] [--organize-args \"--flag value\"] [--augment-args \"--flag value\"] [--training-args \"--flag value\"]"
    echo ""
    echo "Options:"
    echo "  --model-type   Model architecture to use: 'yolo' (default) or 'rfdetr'"
    echo "  --mode         Training mode: 'finetune' (default) or 'synthetic'"
    echo "  --organize-args  Quoted string of extra flags forwarded to organize_dataset.py"
    echo "  --augment-args   Quoted string of extra flags forwarded to augment_dataset.py (synthetic mode only)"
    echo "  --training-args  Quoted string of extra flags forwarded to training.py"
    echo ""
    echo "See README.md for detailed documentation and examples."
    exit 1
}

# ── Defaults ─────────────────────────────────────────────────────────────────
MODEL_PATH=""
MODEL_TYPE="yolo"
MODE="finetune"
ORGANIZE_ARGS=""
AUGMENT_ARGS=""
TRAINING_ARGS=""

# ── Parse arguments ───────────────────────────────────────────────────────────
# First positional arg (if not a flag) is the model path
if [ -n "$1" ] && [[ "$1" != --* ]]; then
    MODEL_PATH="$1"
    shift
fi

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --model-type)
            MODEL_TYPE="$2"
            shift 2
            ;;
        --mode)
            MODE="$2"
            shift 2
            ;;
        --organize-args)
            ORGANIZE_ARGS="$2"
            shift 2
            ;;
        --augment-args)
            AUGMENT_ARGS="$2"
            shift 2
            ;;
        --training-args)
            TRAINING_ARGS="$2"
            shift 2
            ;;
        -h|--help)
            usage
            ;;
        *)
            echo "Error: Unknown parameter '$1'"
            usage
            ;;
    esac
done

# ── Validate ──────────────────────────────────────────────────────────────────
if [[ "$MODEL_TYPE" != "yolo" && "$MODEL_TYPE" != "rfdetr" ]]; then
    echo "Error: --model-type must be 'yolo' or 'rfdetr' (got: '$MODEL_TYPE')"
    usage
fi

if [[ "$MODE" != "finetune" && "$MODE" != "synthetic" ]]; then
    echo "Error: --mode must be 'finetune' or 'synthetic' (got: '$MODE')"
    usage
fi

if [ "$MODE" = "finetune" ] && [ -z "$MODEL_PATH" ]; then
    echo "Error: Fine-tuning mode requires a model filepath as the first argument."
    usage
fi

# ── Resolve paths & format flags per model type ───────────────────────────────
if [ "$MODEL_TYPE" = "rfdetr" ]; then
    PROCESSED_DIR="data/processed_coco"
    AUG_DIR="data/processed_coco_aug"
    FORMAT_ARG="--format coco"
    DEFAULT_MULTIPLIER="1"
else
    PROCESSED_DIR="data/processed"
    AUG_DIR="data/processed_aug"
    FORMAT_ARG=""
    DEFAULT_MULTIPLIER="3"
fi

# ── Run pipeline ──────────────────────────────────────────────────────────────
echo ""
echo "╔══════════════════════════════════════════════╗"
echo "  Training Pipeline"
echo "  Mode       : $MODE"
echo "  Model type : $MODEL_TYPE"
[ -n "$MODEL_PATH" ] && echo "  Model path : $MODEL_PATH"
echo "╚══════════════════════════════════════════════╝"
echo ""

if [ "$MODE" = "finetune" ]; then
    # ── Fine-tuning pipeline ──────────────────────────────────────────────────
    echo "==> [1/3] Fixing labels..."
    python3 fix_labels.py

    echo ""
    echo "==> [2/3] Organizing dataset (format: ${MODEL_TYPE})..."
    python3 organize_dataset.py $FORMAT_ARG $ORGANIZE_ARGS

    echo ""
    echo "==> [3/3] Fine-tuning from: $MODEL_PATH"
    if [ "$MODEL_TYPE" = "rfdetr" ]; then
        python3 training.py \
            --model rfdetr \
            --custom-model "$MODEL_PATH" \
            --dataset-dir "$PROCESSED_DIR" \
            $TRAINING_ARGS
    else
        python3 training.py \
            --custom-model "$MODEL_PATH" \
            $TRAINING_ARGS
    fi

else
    # ── Synthetic training pipeline ───────────────────────────────────────────
    echo "==> [1/3] Organizing dataset (format: ${MODEL_TYPE})..."
    python3 organize_dataset.py $FORMAT_ARG $ORGANIZE_ARGS

    echo ""
    echo "==> [2/3] Augmenting training split (multiplier: ${DEFAULT_MULTIPLIER}x)..."
    python3 augment_dataset.py \
        --input  "$PROCESSED_DIR" \
        --output "$AUG_DIR" \
        --multiplier "$DEFAULT_MULTIPLIER" \
        $FORMAT_ARG \
        $AUGMENT_ARGS

    echo ""
    echo "==> [3/3] Training on synthetic data..."
    if [ "$MODEL_TYPE" = "rfdetr" ]; then
        python3 training.py \
            --model rfdetr \
            --dataset-dir "$AUG_DIR" \
            $TRAINING_ARGS
    else
        python3 training.py \
            --model v11 \
            --size s \
            --data "$AUG_DIR/data.yaml" \
            --unity \
            $TRAINING_ARGS
    fi
fi

echo ""
echo "✓ Pipeline complete."