import cv2
import os
from pathlib import Path

datasets = ["raw_data/test", "raw_data/train", "raw_data/valid"]

save_dir_path = "raw_data_split"
os.makedirs(save_dir_path, exist_ok=True)

image_write_path = f"{save_dir_path}/images"
label_write_path = f"{save_dir_path}/labels"
os.makedirs(image_write_path, exist_ok=True)
os.makedirs(label_write_path, exist_ok=True)

for dataset in datasets:
    #for image in os.listdir(dataset + "/")
    for image_name in os.listdir(f"{dataset}/images"):
        image_name_no_extension = image_name[:-4] # discard .jpg or .png
        image_path = f"{dataset}/images/{image_name}"
        image = cv2.imread(image_path)
        height, width, channels = image.shape
        width_cutoff = width // 2
        left_image = image[:, :(width_cutoff)]
        right_image = image[:, (width_cutoff):]

        cv2.imwrite(f"{image_write_path}/{image_name_no_extension}_left.jpg",left_image)
        cv2.imwrite(f"{image_write_path}/{image_name_no_extension}_right.jpg", right_image)
    for label_name in os.listdir(f"{dataset}/labels"):
        label_name_no_extension = label_name[:-4] # discard .txt
        label_path = f"{dataset}/labels/{label_name}"
        
        base_path = f"{label_write_path}/{label_name_no_extension}"
        with open(label_path, 'r') as label_file, open(f"{base_path}_left.txt", 'w') as left_output_file, open(f"{base_path}_right.txt", 'w') as right_output_file:
            for line in label_file:
                bounding_box = line.strip().split(" ")
                if (float(bounding_box[1]) < 0.5): # second parameter is x-center
                    x_center = 2 * float(bounding_box[1])
                    label_width = 2 * float(bounding_box[3]) # third parameter is width
                    if (x_center + label_width / 2 > 1.0):
                        label_width = 2 * (1.0 - x_center)
                    if (x_center - label_width / 2 < 0.0):
                        label_width = 2 * x_center
                    bounding_box[1] = str(x_center)
                    bounding_box[3] = str(label_width)
                    new_bb_string = " ".join(bounding_box)
                    left_output_file.write(new_bb_string + "\n")
                else:
                    x_center = 2 * float(bounding_box[1]) - 1
                    label_width = 2 * float(bounding_box[3])
                    if (x_center + label_width / 2 > 1.0):
                        label_width = 2 * (1.0 - x_center)
                    if (x_center - label_width / 2 < 0.0):
                        label_width = 2 * x_center
                    bounding_box[1] = str(x_center)
                    bounding_box[3] = str(label_width)
                    new_bb_string = " ".join(bounding_box)
                    right_output_file.write(new_bb_string + "\n")

