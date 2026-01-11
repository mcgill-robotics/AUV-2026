from pathlib import Path
import yaml

# Paths
LABEL_DIR = Path("data/raw_import/labels")
DATA_YAML = Path("data.yaml")

# Old index -> New index mapping
CLASS_MAP = {
    0: 0,  # gate
    1: 3,  # octagon_table
    2: 2,  # octagon_top
    3: 1,  # path_marker
    4: 5,  # sawfish
    5: 4,  # shark
}

# Final class names in NEW index order
NEW_CLASS_NAMES = [
    "gate",           # 0
    "path_marker",    # 1
    "octagon_top",    # 2
    "octagon_table", # 3
    "shark",          # 4
    "sawfish",        # 5
]

def remap_label_file(label_path: Path):
    new_lines = []

    with label_path.open("r") as f:
        for line in f:
            parts = line.strip().split()
            if not parts:
                continue

            old_class = int(parts[0])

            # DROP labels not in CLASS_MAP
            if old_class not in CLASS_MAP:
                continue

            new_class = CLASS_MAP[old_class]
            parts[0] = str(new_class)
            new_lines.append(" ".join(parts))

    # Overwrite file (or delete if empty)
    if new_lines:
        with label_path.open("w") as f:
            f.write("\n".join(new_lines) + "\n")
    else:
        label_path.unlink()  # remove empty label files

def update_data_yaml():
    if not DATA_YAML.exists():
        print("data.yaml not found, skipping update.")
        return

    with DATA_YAML.open("r") as f:
        data = yaml.safe_load(f)

    data["nc"] = len(NEW_CLASS_NAMES)
    data["names"] = NEW_CLASS_NAMES

    with DATA_YAML.open("w") as f:
        yaml.safe_dump(data, f, sort_keys=False)

def main():
    label_files = sorted(LABEL_DIR.glob("*.txt"))

    if not label_files:
        print("No label files found.")
        return

    for label_file in label_files:
        remap_label_file(label_file)

    update_data_yaml()

    print(f"Remapped labels and removed unused classes.")
    print(f"Remaining classes: {NEW_CLASS_NAMES}")

if __name__ == "__main__":
    main()

