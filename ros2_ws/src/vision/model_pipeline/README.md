# Steps to train a model
## Unity Synthetic Data Pipeline (Recommended for Sim Training)

For training on synthetic data generated from Unity, follow this pipeline to ensure robust performance (sim-to-real gap bridge).

1. **Collect Raw Data**: Put your Unity exports in `data/raw_import`. It should have `yolo_images/` and `yolo_labels/` folders.
2. **Augment Data**: Apply heavy underwater augmentations (geometric, texture annihilation, turbidity).
   ```bash
   python3 augment_dataset.py --input data/raw_import --output data/augmented --multiplier 1
   ```
3. **Organize Dataset**: Split the data into train/val/test and generate `data.yaml`.
   ```bash
   python3 organize_dataset.py --input data/augmented --output data/processed_aug
   ```
4. **Train Model**: Run the training script on the processed dataset.
   ```bash
   python3 training.py --model v11 --size s --data data/processed_aug/data.yaml --unity
   ```

## Fine-tuning Pipeline (Local)
1. Collect real data
    1. Ideally, you should collect at least ~50 raw images for each class. 
2. Upload and Label Data
    1. Use [this slideshow](https://docs.google.com/presentation/d/12WbaJi48y1edtK-ood4Iyu_NI00ZPQlZVDVObNEsrkc/edit?slide=id.p#slide=id.p) to sign in to roboflow
    2. Follow the guide in the slideshow on how to label images
3. Generate a New Version of the Dataset
    1. Do **NOT** add any pre-processing/augmentation; this is handled in the training notebook.
    2. The name of the dataset version does not matter.
4. On the new dataset version, click download dataset
5. Move the data into `AUV-2026/ros2_ws/src/vision/model_pipeline/data/raw_import/{images, labels, data.yaml}` (create directories if necessary)
6. Download the yolo model trained on simulation data from the [Drive](https://drive.google.com/drive/folders/1gnvU_EYM1NlcfZJZPfqre9U-cagVTcNy), the file is called `yolov11s_augmented_synthetic_best_model.pt`
7. Move the synthetic model into `AUV-2026/ros2_ws/src/vision/model_pipeline`
6. Run `./training.sh yolov11s_augmented_synthetic_best_model.pt` inside the docker container (synthetic model is the one you downloaded in the previous step).
7. The pytorch model will be at `runs/detect/yolo11s/weights/best.pt`.

### Fine-tuning Advanced Usage

The default parameters for the training script are set to values that we found to work well for training on underwater annotated images, but these can be changed by passing arguments to the `training.sh`. More specifically, changes can be made at:

- The organization of the dataset (handled by `organize_dataset.py`)

| Flag | Long-form | Description | Default Value |
| --- | --- | --- | --- |
| `-i` | `--input` | Directory containing the raw dataset (images, labels, data.yaml) | `data/raw_import` |
| `-o` | `--output` | Directory to save the processed dataset (train/val/test splits and new data.yaml) | `data/processed` |
|  | `--train` | Proportion of the entire dataset to use for training (between 0 and 1) | `0.7` |
|  | `--val` | Proportion of the entire dataset to use for validation (between 0 and 1) | `0.2` |

- The fine-tuning process (handled by `training.py`)

| Flag | Long-form | Description | Default Value |
| --- | --- | --- | --- |
| `-m` | `--model` | YOLO version to train (v8 or v11) | `v8` |
| `-s` | `--size` | Model size: `'n', 's', 'm', 'l', 'x`$\to$ normal, small, medium large xlarge  | `n` |
| `-e` | `--epochs` | Number of training epochs | `200` |
| `-b` | `--batch` | Batch size. -1 for auto | `-1` |
| `-lr0` | `--learning-rate` | Initial learning rate | `0.0003` |
|  | `--imgsz` | Image size for training | `640` |
|  | `--workers` | Number of dataloader workers | `2` |
|  | `--cache` | Cache images for faster training | `False` |

To pass any of these arguments, simply add them to the end of the `training.sh` command. For example, if you wanted to train a medium model with 90% training and 5% validation for 1 epoch, you would run the following command: `
```bash
./training.sh yolov11s_augmented_synthetic_best_model.pt "--size m --train 0.9 --val 0.05" --training-args "--epochs 1"
```
## Optimize the model
Run this **ON THE JETSON**: 

```bash
yolo export model=<path_to_model.pt> format=engine half=True imgsz=640 nms=True
```

Running this inside the jetson docker container will optimize the model to run on the jetson's GPU:
- `model`: path to the pytorch model you want to optimize
- `format`: set to engine for TensorRT engine format
- `half`: use half precision (FP16) for which the jetson has optimized performance
- `imgsz`: image size for the model (should be the same as what you trained on)
- `nms`: whether to include non-max suppression in the exported model (should be True for deployment to offload CPU work to the GPU)

The model is now ready for the vision pipeline!