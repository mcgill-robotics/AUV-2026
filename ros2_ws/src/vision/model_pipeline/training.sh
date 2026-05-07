#!/bin/bash
# check if script is passed without model argument or if first argument start with -- (i.e. trying to pass organize or training flags without model argument)
if [ -z "$1" ] || [[ "$1" == --* ]]; then
    echo "Error: Missing model filepath."
    echo "Usage: ./training.sh <path_to_synthetic_model.pt> [--organize-args \"--flag value\"][--training-args  \"--flag value\"]"
    exit 1
fi

shift # shift past the first argument (model filepath)

while [[ "$#" -gt 0 ]]; do
    case $1 in
        --organize-args) 
            ORGANIZE_ARGS="$2"
            shift 2 # shift past argument flag and its value
            ;;
        --training-args)
            TRAINING_ARGS="$2"
            shift 2 # shift past argument flag and its value
            ;;
        *)
            echo "Unknown parameter passed: $1"
            echo "Usage: ./training.sh <path_to_synthetic_model.pt> [--organize-args \"--flag value\"][--training-args  \"--flag value\"]"
            exit 1
            ;;
    esac
done

if [ -z "$ORGANIZE_ARGS" ]; then
    ORGANIZE_ARGS=""
fi

python3 fix_labels.py
python3 organize_dataset.py $ORGANIZE_ARGS
python3 training.py --custom-model "$1" $TRAINING_ARGS
