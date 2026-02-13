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
6. Run `./training.sh yolov11s_augmented_synthetic_best_model.pt` inside the docker container (synthetic model is the one you downloaded in the previous step)
7. The pytorch model will be at `runs/detect/yolo11s/weights/best.pt`

## Optimize the model
Run this **ON THE JETSON**: `yolo export model=<path_to_model.pt> format=engine half=True` inside the docker container
this optimizes the model to run on the jetson gpu.
