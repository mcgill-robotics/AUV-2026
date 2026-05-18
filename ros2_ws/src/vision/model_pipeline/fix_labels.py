#!/usr/bin/env python3
from pathlib import Path
import argparse
import yaml

DEFAULT_LABEL_DIR = Path("data/raw_import/labels")
DEFAULT_DATA_YAML = Path("data/raw_import/data.yaml")
DEFAULT_DATA_DIR = Path("data/raw_import")


def load_class_names(path: Path) -> list[str]:
    if not path.exists():
        raise FileNotFoundError(f"Class names file not found: {path}")

    with path.open("r", encoding="utf-8") as f:
        lines = [line.strip() for line in f if line.strip()]

    return lines


def build_class_map(target_names: list[str], current_names: list[str]) -> dict[int, int]:
    if len(target_names) != len(current_names):
        print(
            f"Warning: target has {len(target_names)} classes, current has {len(current_names)} classes."
            " Only the shared line positions will be mapped."
        )

    mapping_size = min(len(target_names), len(current_names))
    return {index: index for index in range(mapping_size)}


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

    if new_lines:
        with label_path.open("w", encoding="utf-8") as f:
            f.write("\n".join(new_lines) + "\n")


def update_data_yaml(data_yaml: Path, target_names: list[str]) -> None:
    if not data_yaml.exists():
        print(f"data.yaml not found at {data_yaml}, skipping update.")
        return

    with data_yaml.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    data["nc"] = len(target_names)
    data["names"] = target_names

    with data_yaml.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)


def find_label_dirs(data_dir: Path) -> list[Path]:
    return sorted({path for path in data_dir.rglob("labels") if path.is_dir()})


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Remap label class ids using target and current class_names.txt files."
    )

    # positional arguments
    parser.add_argument(
        "target_class_names",
        type=Path,
        help="Path to the target class_names.txt file",
    )
    parser.add_argument(
        "current_class_names",
        type=Path,
        help="Path to the current class_names.txt file",
    )

    # flags
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

    target_names = load_class_names(args.target_class_names)
    current_names = load_class_names(args.current_class_names)
    class_map = build_class_map(target_names, current_names)

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

    print(f"Remapped labels using {args.current_class_names} -> {args.target_class_names}.")
    print(f"Updated {len(label_files)} label files.")
    print(f"Target classes: {len(target_names)}")


if __name__ == "__main__":
    main()

