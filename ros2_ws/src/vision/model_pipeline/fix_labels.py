#!/usr/bin/env python3
"""
Label Remapper

Remaps label class ids using classes.yaml as the target, and reads the 
current class names directly from the dataset's data.yaml file.
Useful when a dataset source (like Roboflow) reorders or omits classes,
ensuring the class indices match the target model's expected order.

Usage:
    python3 fix_labels.py --data-dir data/raw_import
"""
from pathlib import Path
import argparse
import yaml

SCRIPT_DIR = Path(__file__).parent.resolve()
CLASSES_YAML = SCRIPT_DIR / "classes.yaml"

DEFAULT_LABEL_DIR = Path("data/raw_import/labels")
DEFAULT_DATA_YAML = Path("data/raw_import/data.yaml")


def get_target_class_names() -> list[str]:
    if not CLASSES_YAML.exists():
        raise FileNotFoundError(f"Target classes file not found: {CLASSES_YAML}")

    with CLASSES_YAML.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
        return data.get("names", [])


def get_current_class_names(data_yaml: Path) -> list[str]:
    if not data_yaml.exists():
        raise FileNotFoundError(f"Current data.yaml not found: {data_yaml}")

    with data_yaml.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
        names = data.get("names", [])
        
        # YOLO format can sometimes be a dict {0: 'class1', 1: 'class2'}
        if isinstance(names, dict):
            names = [names[k] for k in sorted(names.keys())]
            
        return names


def build_class_map(target_names: list[str], current_names: list[str]) -> dict[int, int]:
    class_map = {}
    for current_idx, name in enumerate(current_names):
        try:
            target_idx = target_names.index(name)
            class_map[current_idx] = target_idx
        except ValueError:
            print(f"Warning: class '{name}' in current_names not found in target_names. Its labels will be removed.")

    return class_map


def remap_label_file(label_path: Path, class_map: dict[int, int]) -> None:
    new_lines: list[str] = []

    with label_path.open("r", encoding="utf-8") as f:
        for line in f:
            parts = line.strip().split()
            if not parts:
                continue

            try:
                old_class = int(parts[0])
            except ValueError:
                print(f"Skipping invalid label in {label_path}: {line.strip()}")
                continue

            if old_class not in class_map:
                continue

            parts[0] = str(class_map[old_class])
            new_lines.append(" ".join(parts))

    with label_path.open("w", encoding="utf-8") as f:
        if new_lines:
            f.write("\n".join(new_lines) + "\n")
        else:
            f.write("")


def update_data_yaml(data_yaml: Path, target_names: list[str]) -> None:
    if not data_yaml.exists():
        print(f"data.yaml not found at {data_yaml}, skipping update.")
        return

    with data_yaml.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    data["nc"] = len(target_names)
    # Ensure it's a list even if it was a dict before
    data["names"] = target_names

    with data_yaml.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)


def find_label_dirs(data_dir: Path) -> list[Path]:
    return sorted({path for path in data_dir.rglob("labels") if path.is_dir()})


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Remap label class ids using classes.yaml and the dataset's data.yaml."
    )

    parser.add_argument(
        "--data-dir",
        type=Path,
        default=None,
        help="Root directory containing data.yaml and any subfolders named labels",
    )
    parser.add_argument(
        "--label-dir",
        type=Path,
        default=DEFAULT_LABEL_DIR,
        help="Directory containing label .txt files to remap",
    )
    parser.add_argument(
        "--data-yaml",
        type=Path,
        default=DEFAULT_DATA_YAML,
        help="Path to data.yaml to update with target class names",
    )
    parser.add_argument(
        "--skip-yaml",
        action="store_true",
        help="Do not update the data.yaml file",
    )

    return parser.parse_args()


def main() -> None:
    args = parse_args()

    try:
        target_names = get_target_class_names()
    except FileNotFoundError as e:
        print(e)
        return

    if not target_names:
        print(f"No target classes found in {CLASSES_YAML}.")
        return

    if args.data_dir is not None:
        if not args.data_dir.exists():
            print(f"Data directory not found: {args.data_dir}")
            return
        data_yaml = args.data_dir / "data.yaml"
        label_dirs = find_label_dirs(args.data_dir)
        if not label_dirs:
            print(f"No label directories named 'labels' found under {args.data_dir}.")
            return
        print(f"Using data dir {args.data_dir}; found label dirs: {', '.join(str(p) for p in label_dirs)}")
    else:
        data_yaml = args.data_yaml
        label_dirs = [args.label_dir]

    try:
        current_names = get_current_class_names(data_yaml)
    except FileNotFoundError as e:
        print(e)
        return

    class_map = build_class_map(target_names, current_names)

    label_files = []
    for label_dir in label_dirs:
        label_files.extend(sorted(label_dir.glob("*.txt")))

    if not label_files:
        print(f"No label files found in {', '.join(str(d) for d in label_dirs)}.")
        return

    for label_file in label_files:
        remap_label_file(label_file, class_map)

    if not args.skip_yaml:
        update_data_yaml(data_yaml, target_names)

    print(f"Remapped labels successfully. Mapped {len(current_names)} current classes to {len(target_names)} target classes.")
    print(f"Updated {len(label_files)} label files.")


if __name__ == "__main__":
    main()
