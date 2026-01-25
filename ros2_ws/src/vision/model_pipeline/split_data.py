import sys, os, cv2

if (len(sys.argv)) < 2:
    print("Usage: python3 split_data.py [dataset_path]")
    exit(1)
else:
    dataset_path: str = sys.argv[1]

left_data_path: str = os.path.join(dataset_path, "left")
right_data_path: str = os.path.join(dataset_path, "right")

os.makedirs(os.path.join(left_data_path, "images"), exist_ok=True)
os.makedirs(os.path.join(left_data_path, "labels"), exist_ok=True)
os.makedirs(os.path.join(right_data_path, "images"), exist_ok=True)
os.makedirs(os.path.join(right_data_path, "labels"), exist_ok=True)

for image_name in os.listdir(os.path.join(dataset_path, "images")):
    image_path = os.path.join(dataset_path, "images", image_name)
    image = cv2.imread(image_path)
    height, width, channels = image.shape
    width_cutoff = width // 2
    left_image = image[:, :(width_cutoff)]
    right_image = image[:, (width_cutoff):]
    
    cv2.imwrite(os.path.join(left_data_path, "images", image_name), left_image)
    cv2.imwrite(os.path.join(right_data_path, "images", image_name), right_image)

for label_name in os.listdir(os.path.join(dataset_path, "labels")):
    label_path = os.path.join(dataset_path, "labels", label_name)

    left_path = os.path.join(left_data_path, "labels", label_name)
    right_path = os.path.join(right_data_path, "labels", label_name)

    with open(label_path, 'r') as label_file, open(left_path, 'w') as left_output_file, open(right_path, 'w') as right_output_file:
        for line in label_file:
            bounding_box = line.strip().split(" ")
            if (float(bounding_box[1]) < 0.5):
                x_center = 2 * float(bounding_box[1])
                label_width = 2 * float(bounding_box[3])
                if (x_center + label_width / 2 > 1.0):
                    label_width = 2 * (1.0 - x_center)
                if (x_center - label_width / 2 < 0.0):
                    label_width = 2 * x_center

                bounding_box[1] = str(x_center)
                bounding_box[3] = str(label_width)
                new_bb_string = " ".join(bounding_box)
                left_output_file.write(new_bb_string + "\n")

            else:
                x_center = 2 * float(bounding_box[1]) - 1.0
                label_width = 2 * float(bounding_box[3])
                if (x_center + label_width / 2 > 1.0):
                    label_width = 2 * (1.0 - x_center)
                if (x_center - label_width / 2 < 0.0):
                    label_width = 2 * x_center
                bounding_box[1] = str(x_center)
                bounding_box[3] = str(label_width)
                new_bb_string = " ".join(bounding_box)
                right_output_file.write(new_bb_string + "\n")

