# Vision Model Pipeline

## 1. Complete End-to-End Workflow (Sim to Real)
This is the recommended comprehensive workflow for bridging the gap from our Unity synthetic environments to real-world deployment:

1. **Train on Synthetic Data (Sim):** Generate a dataset directly from Unity and train a baseline model purely on synthetic data. *(See [2. Unity Synthetic Data Pipeline](#2-unity-synthetic-data-pipeline-recommended-for-sim-training))*
2. **Pre-label Real Data (Synth Model):** Use the synthetically-trained baseline model to automatically draw preliminary bounding boxes on all your raw real-world images. *(See [3. Pre-labeling for Roboflow](#3-pre-labeling-for-roboflow))*
3. **Upload to Roboflow & Manual Verification:** Bulk upload the newly generated `[folder]_prelabeled` directory directly into Roboflow. Open Roboflow Annotate and effortlessly fix the minor mistakes made by the synthetic model instead of drawing completely from scratch.
4. **Export & Final Training:** Export the perfectly verified dataset back from Roboflow dynamically. Run `organize_dataset.py` to split it, then run `training.py` on this refined dataset to confidently deploy! *(See [4. Fine-tuning Pipeline (Local)](#4-fine-tuning-pipeline-local))*
   > **Note on Augmentation:** Unlike the synthetic pipeline, which strictly relies on our manual `augment_dataset.py` script to forcefully overcome rigid simulation artifacts, for real-world images it is usually better to **skip** `augment_dataset.py`! Native YOLO and RF-DETR PyTorch data-loaders already apply dynamic augmentations behind-the-scenes (like mosaic, scaling, or color jitter) during training, which is more than enough for real-world images without risking aggressive corruption.
## 2. Unity Synthetic Data Pipeline (Recommended for Sim Training)

For training on synthetic data generated from Unity, follow this pipeline to ensure robust performance (sim-to-real gap bridge).

> [!IMPORTANT]
> Always **split first**, then **augment only the training set**. Augmenting before splitting causes data leakage - augmented copies of the same source image may end up in both train and validation, inflating metrics.

### 2.1 YOLO Workflow

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

### 2.2 RF-DETR Workflow

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
   python3 training.py --model rfdetr --dataset-dir data/processed_coco_aug --epochs 50
   ```
   ```bash
   python3 -m tensorboard.main --logdir /home/douglas/AUV-2026/ros2_ws/src/vision/model_pipeline/runs/rfdetr
   ```



## 3. Pre-labeling for Roboflow

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

## 4. Fine-tuning Pipeline (Local)
1. **Prepare Data Locally**
    1. Run `export_labels.py` (see [3. Pre-labeling for Roboflow](#3-pre-labeling-for-roboflow)) on your newly collected raw images. This saves you from annotating hundreds of images from zero by utilizing your synthetic baseline model.
2. **Upload and Verify on Roboflow**
    1. Drag and drop the generated `_prelabeled` folder into your Roboflow workspace.
    2. Using Roboflow Annotate, manually review the images and correct any inaccuracies made by the synthetic model.
3. **Generate a New Dataset Version**
    1. Do **NOT** add any pre-processing or augmentation steps in Roboflow; native PyTorch dataloaders heavily handle mosaic, jitter, and scaling dynamically during actual training.
4. **Download Dataset to Machine**
    1. Select export format **YOLOv11** (even if using RF-DETR, because our script organizes everything automatically!).
    2. Download the zip and extract its contents (`images/`, `labels/`, `data.yaml`) into `AUV-2026/ros2_ws/src/vision/model_pipeline/data/raw_import/`.
5. **Run the Pre-processing Script**
    1. For **YOLO**: `python3 organize_dataset.py --input data/raw_import --output data/processed`
    2. For **RF-DETR**: `python3 organize_dataset.py --input data/raw_import --output data/processed_coco --format coco`
6. **Execute Fine-tuning Process**
    1. Download or locate your previously trained synthetic baseline model (`.pt` or `.pth`) and place it inside `model_pipeline`.
    2. For **YOLO**: Run `./training.sh <your_synthetic_model>.pt` inside the Docker container.
    3. For **RF-DETR**: Run `python3 training.py --model rfdetr --custom-model <your_synthetic_model>.pth --dataset-dir data/processed_coco --epochs 50`
7. **Locate Your Fine-Tuned Weights**
    1. YOLO outputs generally save to: `runs/detect/yolo11s/weights/best.pt`
    2. RF-DETR outputs generally save to: `runs/rfdetr/best_rf_detr_small_model.pth`

### 4.1 Fine-tuning Advanced Usage

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

## 5. Visualize Predictions

You can view inference results on random images from your dataset using `visualize_label.py` to verify the model predictions before deployment:

**For YOLO:**
```bash
python3 visualize_label.py --folder data/processed/test/images --model-type yolo --model runs/detect/yolov11s/weights/best.pt
```

**For RF-DETR:**
```bash
python3 visualize_label.py --folder data/processed_coco/test/images --model-type rfdetr --model best_rf_detr_small_model.pth
```

## 6. Optimize the model
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
