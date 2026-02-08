from pathlib import Path
import yaml

# Paths
LABEL_DIR = Path("data/raw_import/labels")
DATA_YAML = Path("data/raw_import/data.yaml")

# Real Dataset (Source) -> Synthetic Dataset (Target)
# 0: gate -> 0: gate
# 1: octagon_table -> 5: table
# 2: octagon_top -> 4: octagon
# 3: path_marker -> 1: lane_marker
# 4: sawfish -> 9: sawfish
# 5: shark -> 8: shark

CLASS_MAP = {
    0: 0,  # gate -> gate
    1: 5,  # octagon_table -> table
    2: 4,  # octagon_top -> octagon
    3: 1,  # path_marker -> lane_marker
    4: 9,  # sawfish -> sawfish
    5: 8,  # shark -> shark
}

# Final class names (Must match synthetic dataset exactly)
NEW_CLASS_NAMES = [
    "gate",         # 0
    "lane_marker",  # 1
    "red_pipe",     # 2
    "white_pipe",   # 3
    "octagon",      # 4
    "table",        # 5
    "bin",          # 6
    "board",        # 7
    "shark",        # 8
    "sawfish"       # 9
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

