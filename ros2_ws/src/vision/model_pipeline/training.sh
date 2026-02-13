#!/bin/bash
if [ -z "$1" ]; then
        echo "Usage: ./training.sh <path_to_synthetic_model.pt>"
        exit 1
fi
python3 fix_labels.py
python3 organize_dataset.py
python3 training.py --custom-model "$1" --learning-rate 0.0003
