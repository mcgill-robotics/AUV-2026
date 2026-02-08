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
   python3 training-unity.py --model v11 --size s --data data/processed_aug/data.yaml
   ```

---

## Real World Data Pipeline (Roboflow)
If you only want to train an existing dataset and not add any new images to it, skip to step 6.
1. Collect Image Samples (Sim or Real)
    1. Ideally, you should collect at least ~50 raw images for each class. 
2. Upload and Label Data
    1. Use [this slideshow](https://docs.google.com/presentation/d/12WbaJi48y1edtK-ood4Iyu_NI00ZPQlZVDVObNEsrkc/edit?slide=id.p#slide=id.p) to sign in to roboflow
    2. Follow the guide in the slideshow on how to label images
3. Generate a New Version of the Dataset
    1. Do **NOT** add any pre-processing/augmentation; this is handled in the training notebook.
    2. The name of the dataset version does not matter.
4. Adjust Parameters in the Training Notebook
    1. Ensure the order of classes in the script is _**alphabetical**_ (same as Roboflow).
    2. As of [28/06/2024], Ultralytics only stably supports YOLOv8 and YOLO11. YOLOv9 is available but not fully reliable.
    3. The script automatically chooses the latest dataset version. To train an older version, follow these steps:
        1. Open the script,
        2. Go to the "Download dataset from Roboflow" section,
        3. Change the value of `latest_version` to the version you want,
        4. If you make other changes in the script and would like to commit them, make sure you undo the `lastest_version` change. The latest dataset version should be the default.
5. Run the Training Notebook
    1. Note: I _**HIGHLY**_ recommend training a model only if you have a **_GPU_** available.
        1. Without a GPU, training can take days. Ask someone with a GPU to let the model train overnight.
    2. [Optional] If you don't like Jupyter's output (as I don't), you can execute all cells EXCEPT the final one (which trains the model). Instead, use the `training_real.py` file to train the model, which displays the training progress in the terminal.
        1. Make sure to update the parameters in the parameter boxes.
        2. There is no need to change the rest of the code.
    3. This makes a new directory inside runs/detect, the pytorch model is runs/detect/{newest directory}/weights/best.pt

## Fine-tuning Pipeline (Local)
0. First download the yolo model trained on simulation data from the [Drive](https://drive.google.com/drive/folders/1gnvU_EYM1NlcfZJZPfqre9U-cagVTcNy), the file is called yolov11s\_augmented\_synthetic\_best\_model.pt
1. Collect real data, same as Real-world data pipeline
2. Either follow step 2 of real-world pipeline to label the data in roboflow, and then create a version and download the dataset, or use local labeling tools to label the images
3. If the data has both cameras captured in the image, you need to separate the left part of the images and labels and keep only the left part. To do this:
    1. Run `split_data.py {path_to_your_data}` (path_to_your_data should be the folder containing images and labels folders)
4. Move the data into AUV-2026/ros2\_ws/src/vision/model\_pipeline/data/raw\_import/{images, labels, data.yaml} (create directories if necessary)
5. Run `python3 fix_labels.py`
6. Run `python3 organize_dataset.py`
7. Run `python3 training_real.py --custom-model {path_to_sim_model.pt} --learning-rate 0.0003`
    1. This makes a new directory inside runs/detect, the pytorch model is runs/detect/{newest directory}/weights/best.pt

## Optimize the model
Run this **ON THE JETSON**: `yolo export model=yolov8n.pt format=engine half=True` inside the docker container
this optimizes the model to run on the jetson gpu.
