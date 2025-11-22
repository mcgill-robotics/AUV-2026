import os
import random
import cv2

# Root paths
#img_dir = "data/augmented/val/images"
#label_dir = "data/augmented/val/labels"
img_dir = "data/augmented/train/images"
label_dir = "data/augmented/train/labels"
#img_dir = "data/split_raw/left/val/images"
#label_dir = "data/split_raw/left/val/labels"

# Pick a random image file
img_file = random.choice(os.listdir(img_dir))
img_path = os.path.join(img_dir, img_file)

# Matching YOLO label file (same base name, .txt extension)
label_file = os.path.splitext(img_file)[0] + ".txt"
label_path = os.path.join(label_dir, label_file)

print(f"Selected image: {img_file}")
print(f"Label file: {label_file}")

# Load image
img = cv2.imread(img_path)
img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
h, w, _ = img.shape

# Read YOLO labels if file exists
if os.path.exists(label_path):
    with open(label_path, "r") as f:
        labels = f.readlines()
else:
    labels = []
    print("⚠️ No label file found for this image!")

# Draw bounding boxes
for label in labels:
    cls, x_center, y_center, bw, bh = map(float, label.split())
    
    # Convert YOLO normalized coords → pixel coords
    x_center *= w
    y_center *= h
    bw *= w
    bh *= h
    
    x1 = int(x_center - bw / 2)
    y1 = int(y_center - bh / 2)
    x2 = int(x_center + bw / 2)
    y2 = int(y_center + bh / 2)
    
    # Draw rectangle and class id
    cv2.rectangle(img, (x1, y1), (x2, y2), (255, 0, 0), 2)
    cv2.putText(img, str(int(cls)), (x1, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

# === Save result to /dev/shm/ ===
output_path = "/dev/shm/image_labeled.jpg"
cv2.imwrite(output_path, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))

print(f"✅ Saved labeled image to: {output_path}")

