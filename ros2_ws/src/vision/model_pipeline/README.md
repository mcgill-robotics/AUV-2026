# Steps to train a model
## Unity Synthetic Data Pipeline (Recommended for Sim Training)

For training on synthetic data generated from Unity, follow this pipeline to ensure robust performance (sim-to-real gap bridge).

> [!IMPORTANT]
> Always **split first**, then **augment only the training set**. Augmenting before splitting causes data leakage - augmented copies of the same source image may end up in both train and validation, inflating metrics.

### YOLO Workflow

1. **Collect Raw Data**: Put your Unity exports in `data/raw_import`. It should have `yolo_images/` and `yolo_labels/` folders.
2. **Organize Dataset**: Split the data into train/val/test and generate `data.yaml`.
   ```bash
   python3 organize_dataset.py --input data/raw_import --output data/processed
   ```
3. **Augment Training Data**: Creates a new dataset with augmented train + original val/test.
   ```bash
   python3 augment_dataset.py --input data/processed --output data/processed_aug --multiplier 3
   ```
4. **Train Model**: Run the training script on the augmented dataset.
   ```bash
   python3 training.py --model v11 --size s --data data/processed_aug/data.yaml --unity
   ```

### RF-DETR Workflow

1. **Collect Raw Data**: Same as above.
2. **Organize Dataset** in COCO format:
   ```bash
   python3 organize_dataset.py --input data/raw_import --output data/processed_coco --format coco
   ```
3. **Augment Training Data**: Creates a new dataset with augmented train + original valid/test.
   ```bash
   python3 augment_dataset.py --input data/processed_coco --output data/processed_coco_aug --multiplier 1 --format coco
   ```
4. **Train Model**:
   ```bash
   python3 training.py --model rfdetr --dataset-dir data/processed_coco_aug --epochs 25
   ```
   ```bash
   python3 -m tensorboard.main --logdir /home/douglas/AUV-2026/ros2_ws/src/vision/model_pipeline/runs/rfdetr
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
    1. Select export format YOLOv11
    2. Choose download zip to computer
5. Move the data into `AUV-2026/ros2_ws/src/vision/model_pipeline/data/raw_import/{images, labels, data.yaml}` (create directories if necessary)
6. Download the yolo model trained on simulation data from the [Drive](https://drive.google.com/drive/folders/1gnvU_EYM1NlcfZJZPfqre9U-cagVTcNy), the file is called `yolov11s_augmented_synthetic_best_model.pt`
7. Move the synthetic model into `AUV-2026/ros2_ws/src/vision/model_pipeline`
8. Run `./training.sh yolov11s_augmented_synthetic_best_model.pt` inside the docker container (synthetic model is the one you downloaded in the previous step).
9. The pytorch model will be at `runs/detect/yolo11s/weights/best.pt`.

### Fine-tuning Advanced Usage

The default parameters for the training script are set to values that we found to work well for training on underwater annotated images, but these can be changed by passing arguments to the `training.sh`. More specifically, changes can be made at:

- The organization of the dataset (handled by `organize_dataset.py`)

| Flag | Long-form | Description | Default Value |
| --- | --- | --- | --- |
| `-i` | `--input` | Directory containing the raw dataset (images, labels, data.yaml) | `data/raw_import` |
| `-o` | `--output` | Directory to save the processed dataset (train/val/test splits and new data.yaml) | `data/processed` |
|  | `--train` | Proportion of the entire dataset to use for training (between 0 and 1) | `0.7` |
|  | `--val` | Proportion of the entire dataset to use for validation (between 0 and 1) | `0.2` |
| `-f` | `--format` | Output format: `yolo` or `coco` (for RF-DETR) | `yolo` |

- The fine-tuning process (handled by `training.py`)

| Flag | Long-form | Description | Default Value |
| --- | --- | --- | --- |
| `-m` | `--model` | Model to train: `v8`, `v11`, or `rfdetr` | `v8` |
| `-s` | `--size` | Model size: `n, s, m, l, x` (YOLO) or `n` (Base) / `l` (Large) for RF-DETR | `n` |
| `-c` | `--custom-model` | Train from an existing model file (YOLO `.pt` or RF-DETR `.pth`) | |
| `-e` | `--epochs` | Number of training epochs | `200` |
| `-b` | `--batch` | Batch size. -1 for auto (YOLO only) | `-1` |
| `-lr0` | `--learning-rate` | Initial learning rate | `0.0003` (YOLO), `1e-4` (RF-DETR) |
|  | `--data` | Path to `data.yaml` (YOLO only) | `data/processed/data.yaml` |
|  | `--imgsz` | Image size for training (YOLO only) | `640` |
|  | `--workers` | Number of dataloader workers (YOLO only) | `2` |
|  | `--cache` | Cache images for faster training (YOLO only) | `False` |
|  | `--unity` | Use augmentation parameters tuned for Unity synthetic data (YOLO only) | `False` |
|  | `--dataset-dir` | COCO dataset path (RF-DETR only) | `data/processed` |
|  | `--grad-accum-steps` | Gradient accumulation steps (RF-DETR only) | `4` |

To pass any of these arguments, simply add them to the end of the `training.sh` command. For example, if you wanted to train a medium model for 1 epoch with input data `./data` you would run the following command:
```bash
./training.sh yolov11s_augmented_synthetic_best_model.pt --training-args "--size m --epochs 1" --organize-args "--input ./data"
```
To train a model from scratch (no fine tuning), specify one of the base models (for example yolov8n.pt) instead of yolov11s\_augmented\_synthetic\_best\_model.pt

## Visualize Predictions

You can view inference results on random images from your dataset using `visualize_label.py` to verify the model predictions before deployment:

**For YOLO:**
```bash
python3 visualize_label.py --folder data/processed/test/images --model-type yolo --model runs/detect/yolov11s/weights/best.pt
```

**For RF-DETR:**
```bash
python3 visualize_label.py --folder data/processed_coco/test/images --model-type rfdetr --model best_rf_detr_small_model.pth
```

## Pre-labeling for Roboflow

You can use the `export_labels.py` script to run inference on a folder of raw images and automatically export `.txt` YOLO-format bounding box annotations perfectly matched to your dataset. This allows you to bulk upload pre-annotated data directly into Roboflow for manual refinement!

By default, the script creates a duplicate folder named `[folder]_prelabeled` ensuring your original raw image dataset remains untouched! It places all generated `.txt` files and an intelligently synced `classes.txt` within this newly generated folder.

**For RF-DETR:**
```bash
python3 export_labels.py --folder data/my_raw_images --model-type rfdetr --model best_rf_detr_small_model.pth
```

**For YOLO:**
```bash
python3 export_labels.py --folder data/my_raw_images --model-type yolo --model best_yolov11s_model.pt
```

After running the command, simply drag the generated `my_raw_images_prelabeled` folder securely straight into Roboflow's web interface.

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
