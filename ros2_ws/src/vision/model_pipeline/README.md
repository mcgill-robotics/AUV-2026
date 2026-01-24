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
3. Upload and Label Data
    1. Go to [Roboflow](https://app.roboflow.com/auv2024) and open the desired project 
    #(TODO: change Roboflow name. used to be auv2024)
    2. Ask a lead for the McGill Robotics account information.
    3. Upload your data and label it using bounding boxes.
        1. You can keep the default train/valid/test split suggested by Roboflow.
        2. Ensure proper labeling of objects, as it affects training performance even if not perfect.
6. Generate a New Version of the Dataset
    1. Do **NOT** add any pre-processing/augmentation; this is handled in the training notebook.
    2. The name of the dataset version does not matter.
7. Adjust Parameters in the Training Notebook
    1. Ensure the order of classes in the script is _**alphabetical**_ (same as Roboflow).
    2. As of [28/06/2024], Ultralytics only stably supports YOLOv8. YOLOv9 is available but not fully reliable.
    3. The script automatically chooses the latest dataset version. To train an older version, follow these steps:
        1. Open the script,
        2. Go to the "Download dataset from Roboflow" section,
        3. Change the value of `latest_version` to the version you want,
        4. If you make other changes in the script and would like to commit them, make sure you undo the `lastest_version` change. The latest dataset version should be the default.
8. Run the Training Notebook
    1. Note: I _**HIGHLY**_ recommend training a model only if you have a **_GPU_** available.
        1. Without a GPU, training can take days. Ask someone with a GPU to let the model train overnight.
    3. [Optional] If you don't like Jupyter's output (as I don't), you can execute all cells EXCEPT the final one (which trains the model). Instead, use the `training.py` file to train the model, which displays the training progress in the terminal.
        1. Make sure to update the parameters in the parameter boxes.
        2. There is no need to change the rest of the code.
