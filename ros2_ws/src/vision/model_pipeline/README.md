# Vision Model Pipeline

> [!IMPORTANT]
> **Docker Environment:** All python scripts and commands listed in this document must be executed from inside the Docker container.

## Complete End-to-End Workflow (Sim to Real)
This is the recommended comprehensive workflow for bridging the gap from our Unity synthetic environments to real-world deployment:

1. **Train on Synthetic Data (Sim):** Generate a dataset directly from Unity and train a baseline model purely on synthetic data. *(See [1. Unity Synthetic Data Pipeline](#1-unity-synthetic-data-pipeline-recommended-for-sim-training))*
2. **Pre-label Real Data (Synth Model):** Use the synthetically-trained baseline model to automatically draw preliminary bounding boxes on all your raw real-world images. *(See [2. Pre-labeling for Roboflow](#2-pre-labeling-for-roboflow))*
3. **Upload to Roboflow & Manual Verification:** Bulk upload the newly generated `[folder]_prelabeled` directory directly into Roboflow. Open Roboflow Annotate and effortlessly fix the minor mistakes made by the synthetic model instead of drawing completely from scratch.
4. **Export & Final Training:** Export the perfectly verified dataset back from Roboflow dynamically. Run `organize_dataset.py` to split it, then run `training.py` on this refined dataset to confidently deploy! *(See [3. Fine-tuning Pipeline (Local)](#3-fine-tuning-pipeline-local))*
   > **Note on Augmentation:** Unlike the synthetic pipeline, which strictly relies on our manual `augment_dataset.py` script to forcefully overcome rigid simulation artifacts, for real-world images it is usually better to **skip** `augment_dataset.py`! Native YOLO and RF-DETR PyTorch data-loaders already apply dynamic augmentations behind-the-scenes (like mosaic, scaling, or color jitter) during training, which is more than enough for real-world images without risking aggressive corruption.

## 1. Unity Synthetic Data Pipeline (Recommended for Sim Training)

For training on synthetic data generated from Unity, follow this pipeline to ensure robust performance (sim-to-real gap bridge).

> [!IMPORTANT]
> Always **split first**, then **augment only the training set**. Augmenting before splitting causes data leakage - augmented copies of the same source image may end up in both train and validation, inflating metrics.

### 1.1 YOLO Workflow

Running the Python scripts directly provides maximum modularity and makes debugging much easier if any step fails.

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

*(Alternatively, you can run the automated script: `./training.sh --mode synthetic`)*

### 1.2 RF-DETR Workflow

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
   python3 training.py --model rfdetr --dataset-dir data/processed_coco_aug --epochs 50 --unity
   ```
   Not needed, good to monitor training metrics.
   ```bash
   python3 -m tensorboard.main --logdir /home/douglas/AUV-2026/ros2_ws/src/vision/model_pipeline/runs/rfdetr
   ```

*(Alternatively, you can run the automated script: `./training.sh --mode synthetic --model-type rfdetr`)*


## 2. Pre-labeling for Roboflow

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

## 3. Fine-tuning Pipeline (Local)
1. **Prepare Data Locally**
    1. Run `export_labels.py` (see [2. Pre-labeling for Roboflow](#2-pre-labeling-for-roboflow)) on your newly collected raw images. This saves you from annotating hundreds of images from zero by utilizing your synthetic baseline model.
2. **Upload and Verify on Roboflow**
    1. Drag and drop the generated `_prelabeled` folder into your Roboflow workspace.
    2. Using Roboflow Annotate, manually review the images and correct any inaccuracies made by the synthetic model.
3. **Generate a New Dataset Version**
    1. Do **NOT** add any pre-processing or augmentation steps in Roboflow; native PyTorch dataloaders heavily handle mosaic, jitter, and scaling dynamically during actual training.
4. **Download Dataset to Machine**
    1. Select export format **YOLOv11** (even if using RF-DETR, because our script organizes everything automatically!).
    2. Download the zip and extract its contents (`images/`, `labels/`, `data.yaml`) into `AUV-2026/ros2_ws/src/vision/model_pipeline/data/raw_import/`.
5. **Undo Roboflow Shenanigans**
    1. Roboflow sometimes re-orders the labels in alphabetical order. To undo this and perfectly map everything to the correct targets (as defined in `classes.yaml`), simply run the `fix_labels.py` script.
    2. Run `python3 fix_labels.py --data-dir data/raw_import`. The script will automatically read the target labels from `classes.yaml` and the current labels from your Roboflow `data.yaml`, then correctly remap your dataset indices
6. **Run the Pre-processing Script**
    1. For **YOLO**: `python3 organize_dataset.py --input data/raw_import --output data/processed`
    2. For **RF-DETR**: `python3 organize_dataset.py --input data/raw_import --output data/processed_coco --format coco --letterbox 512`
7. **Execute Fine-tuning Process**
    1. Download or locate your previously trained synthetic baseline model (`.pt` or `.pth`) and place it inside `model_pipeline`.
    2. For **YOLO**: Run `python3 training.py --model v11 --size s --data data/processed/data.yaml --custom-model <your_synthetic_model>.pt`
    3. For **RF-DETR**: Run `python3 training.py --model rfdetr --dataset-dir data/processed_coco --custom-model <your_synthetic_model>.pth`
8. **Locate Your Fine-Tuned Weights**
    1. YOLO outputs generally save to: `runs/detect/yolo11s/weights/best.pt`
    2. RF-DETR outputs generally save to: `runs/rfdetr/best_rf_detr_small_model.pth`


## 4. Optimize the model
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

## Training Advanced Usage

`training.sh` is a unified entrypoint for both fine-tuning and synthetic training, for both YOLO and RF-DETR.

### Usage Signature
```bash
./training.sh [<model_path>] [--model-type yolo|rfdetr] [--mode finetune|synthetic] [--organize-args "..."] [--augment-args "..."] [--training-args "..."]
```

| Argument | Description | Default |
| --- | --- | --- |
| `<model_path>` | Path to the model to fine-tune from (`.pt` for YOLO, `.pth` for RF-DETR). Required in `finetune` mode; omit in `synthetic` mode. | — |
| `--model-type` | Model architecture: `yolo` or `rfdetr` | `yolo` |
| `--mode` | Training mode: `finetune` (real-world data) or `synthetic` (Unity sim data) | `finetune` |
| `--organize-args` | Quoted extra flags forwarded to `organize_dataset.py` | — |
| `--augment-args` | Quoted extra flags forwarded to `augment_dataset.py` (`synthetic` mode only) | — |
| `--training-args` | Quoted extra flags forwarded to `training.py` | — |

#### Mode description

| Step | `finetune` | `synthetic` |
| --- | --- | --- |
| 1 | `fix_labels.py` | `organize_dataset.py` |
| 2 | `organize_dataset.py` | `augment_dataset.py` |
| 3 | `training.py --custom-model <model>` | `training.py` (from scratch) |

The format flag (`--format coco`) and dataset paths (`data/processed_coco`, `data/processed_coco_aug`) are automatically set when `--model-type rfdetr` is used. The `--letterbox 512` flag should be passed via `--organize-args` for RF-DETR to maintain aspect ratio during training.

### Examples for Python Pipeline
As always keep in mind these scripts assume the defaults, notably that the input dataset for training is `data/raw_import`.

#### 1. Fine-tune YOLO (default)
```bash
python3 fix_labels.py --data-dir data/raw_import
python3 organize_dataset.py --input data/raw_import --output data/processed
python3 training.py --model v11 --size s --data data/processed/data.yaml --custom-model yolov11s_synthetic_best.pt
```

#### 2. Fine-tune RF-DETR
```bash
python3 fix_labels.py --data-dir data/raw_import
python3 organize_dataset.py --input data/raw_import --output data/processed_coco --format coco --letterbox 512
python3 training.py --model rfdetr --dataset-dir data/processed_coco --custom-model best_rf_detr_small_model.pth
```

#### 3. Train YOLO from scratch on synthetic Unity data
```bash
python3 organize_dataset.py --input data/raw_import --output data/processed
python3 augment_dataset.py --input data/processed --output data/processed_aug --multiplier 3
python3 training.py --model v11 --size s --data data/processed_aug/data.yaml --unity
```

#### 4. Train YOLO model from scratch using alternate input folder
Say you have a different folder of synthetic data (e.g. `data/my_unity_data`) that you want to train on instead of the default `data/raw_import`:
```bash
python3 organize_dataset.py --input ./data/my_unity_data --output data/processed
python3 augment_dataset.py --input data/processed --output data/processed_aug --multiplier 3
python3 training.py --model v11 --size s --data data/processed_aug/data.yaml --unity
```

#### 5. Train RF-DETR from scratch on synthetic Unity data
```bash
python3 organize_dataset.py --input data/raw_import --output data/processed_coco --format coco --letterbox 512
python3 augment_dataset.py --input data/processed_coco --output data/processed_coco_aug --multiplier 1 --format coco
python3 training.py --model rfdetr --dataset-dir data/processed_coco_aug --epochs 50 --unity
```

#### 6. Fine-tune YOLO, medium model, 1 epoch, custom input folder
```bash
python3 fix_labels.py --data-dir ./data
python3 organize_dataset.py --input ./data --output data/processed
python3 training.py --model v11 --size m --epochs 1 --data data/processed/data.yaml --custom-model yolov11s_synthetic_best.pt
```

#### 7. Train RF-DETR synthetic with extra epochs
```bash
python3 organize_dataset.py --input data/raw_import --output data/processed_coco --format coco --letterbox 512
python3 augment_dataset.py --input data/processed_coco --output data/processed_coco_aug --multiplier 1 --format coco
python3 training.py --model rfdetr --dataset-dir data/processed_coco_aug --epochs 100 --unity
```

*(Note: While the explicit Python commands above are recommended for full modularity, you can also use the `./training.sh` wrapper script which acts as a unified entrypoint combining these steps.)*

### Details on underlying scripts and parameters

To train from a base YOLO model with no fine-tuning starting point, pass a stock model (e.g. `yolov8n.pt`) instead of a custom checkpoint.

The default parameters for the underlying scripts are set to values that we found to work well for training on underwater annotated images, but these can be changed by passing arguments via `--organize-args`, `--augment-args`, and `--training-args`. More specifically, changes can be made at:

- The organization of the dataset (handled by `organize_dataset.py`)

| Flag | Long-form | Description | Default Value |
| --- | --- | --- | --- |
| `-i` | `--input` | Directory containing the raw dataset (images, labels, data.yaml) | `data/raw_import` |
| `-o` | `--output` | Directory to save the processed dataset (train/val/test splits and new data.yaml) | `data/processed` |
|  | `--train` | Proportion of the entire dataset to use for training (between 0 and 1) | `0.7` |
|  | `--val` | Proportion of the entire dataset to use for validation (between 0 and 1) | `0.2` |
| `-f` | `--format` | Output format: `yolo` or `coco` (for RF-DETR) | `yolo` |
|  | `--letterbox`| Optional size to letterbox pad images (e.g. 512). Essential for maintaining aspect ratio for RF-DETR | `None` |

- The augmentation process (handled by `augment_dataset.py`, only used for synthetic training)

| Flag | Long-form | Description | Default Value |
| --- | --- | --- | --- |
| `-i` | `--input` | Directory containing the organized dataset (train/val/test splits and data.yaml) | `data/processed` |
| `-o` | `--output` | Directory to save the augmented dataset (augmented train + original val/test) | `data/processed_aug` |
| `-m` | `--multiplier` | How many augmented copies to create for each original training image (integer >= 1) | `3` (YOLO) `1` (RF-DETR) |
|  | `--workers` | Number of parallel workers to use for augmentation | `8` |
|  | `--visualize` | Whether to save example augmented images for visualization (in `visualizations/`) | `False` |
|  | `--num_vis` | Number of augmented images to save for visualization (if `--visualize` is True) | `10` |
| `-f` | `--format` | Input/output format: `yolo` or `coco` (for RF-DETR) | `yolo` |

- The fine-tuning process (handled by `training.py`)

| Flag | Long-form | Description | Default Value |
| --- | --- | --- | --- |
| `-m` | `--model` | Model to train: `v8`, `v11`, or `rfdetr` | `v8` |
| `-s` | `--size` | Model size: `n, s, m, l, x` (YOLO) or `n` (Base) / `l` (Large) for RF-DETR | `n` |
| `-c` | `--custom-model` | Train from an existing model file (YOLO `.pt` or RF-DETR `.pth`) | |
| `-e` | `--epochs` | Number of training epochs | `200` |
| `-b` | `--batch` | Batch size. -1 for auto (YOLO only) | `-1` (automatically determined for YOLO, set to `4` for RF-DETR) |
| `-lr0` | `--learning-rate` | Initial learning rate | `0.0003` (YOLO), `1e-4` (RF-DETR) |
|  | `--data` | Path to `data.yaml` (YOLO only) | `data/processed/data.yaml` |
|  | `--imgsz` | Image size for training (YOLO only) | `640` |
|  | `--workers` | Number of dataloader workers (YOLO only) | `2` |
|  | `--cache` | Cache images for faster training (YOLO only) | `False` |
|  | `--unity` | Use training parameters tailored for synthetic Unity datasets (avoids double-augmenting for RF-DETR) | `False` |
|  | `--dataset-dir` | COCO dataset path (RF-DETR only) | `data/processed` |
|  | `--grad-accum-steps` | Gradient accumulation steps (RF-DETR only) | `4` |

- Undoing roboflow shenanigans (handled by `fix_labels.py`)

| Argument / Flag | Description | Default Value |
| --- | --- | --- |
| `--data-dir` | Root directory containing `data.yaml` and any subfolders named `labels` | `None` |
| `--label-dir` | Directory containing label .txt files to remap | `data/raw_import/labels` or folder named `labels` inside `--data-dir` path if specified |
| `--data-yaml` | Path to `data.yaml` to update with target class names | `data/raw_import/data.yaml` or file named `data.yaml` inside `--data-dir` path if specified |
| `--skip-yaml` | Do not update the `data.yaml` file | `False` |

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

**Visualize from existing label**
```bash
python3 visualize_label.py --folder data/raw_import/images --from-label
```
