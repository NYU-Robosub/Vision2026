
from pathlib import Path
from collections import Counter
import hashlib
import shutil

root = Path.home() / "Vision2026" / "detector"

sources = [
    ("nyu", root / "dataset_gate_only"),
    ("sailfish", root / "external_datasets" / "sailfish"),
]

output = root / "dataset_combined"

# Stop if the output already exists to avoid mixing old files.
if output.exists():
    raise SystemExit(
        f"{output} already exists. Inspect it before rebuilding."
    )

# Process test first so exact duplicate images cannot
# enter the training or validation splits.
splits = ["test", "valid", "train"]

seen_hashes = {}
counts = Counter()
invalid = []
image_extensions = {".jpg", ".jpeg", ".png"}

for split in splits:
    for folder in ("images", "labels"):
        (output / split / folder).mkdir(parents=True, exist_ok=True)

for split in splits:
    for source_name, source_root in sources:
        image_dir = source_root / split / "images"
        label_dir = source_root / split / "labels"

        if not image_dir.is_dir():
            raise SystemExit(f"Missing directory: {image_dir}")

        for img in sorted(image_dir.iterdir()):
            if img.suffix.lower() not in image_extensions:
                continue

            label = label_dir / (img.stem + ".txt")

            if not label.exists():
                invalid.append((str(img), "Missing label"))
                continue

            try:
                content = label.read_text().strip()

                for line in content.splitlines():
                    fields = line.split()

                    if len(fields) != 5:
                        raise ValueError("Invalid YOLO label format")

                    values = [float(x) for x in fields]

                    if values[0] != 0:
                        raise ValueError("Unexpected class ID")

                    x, y, w, h = values[1:]

                    if not (
                        0 <= x <= 1 and
                        0 <= y <= 1 and
                        0 < w <= 1 and
                        0 < h <= 1 and
                        x - w / 2 >= -0.001 and
                        x + w / 2 <= 1.001 and
                        y - h / 2 >= -0.001 and
                        y + h / 2 <= 1.001
                    ):
                        raise ValueError("Invalid bounding box")

            except (ValueError, UnicodeError) as e:
                invalid.append((str(label), str(e)))
                continue

            digest = hashlib.sha256(img.read_bytes()).hexdigest()

            if digest in seen_hashes:
                counts["exact_duplicates_skipped"] += 1
                continue

            seen_hashes[digest] = split

            new_name = f"{source_name}_{img.name}"
            new_label = f"{source_name}_{img.stem}.txt"

            shutil.copy2(
                img, output / split / "images" / new_name
            )
            shutil.copy2(
                label, output / split / "labels" / new_label
            )

            counts[f"{source_name}_{split}"] += 1

yaml_text = f"""path: {output}
train: train/images
val: valid/images
test: test/images

nc: 1
names:
  0: gate
"""

(output / "data.yaml").write_text(yaml_text)

print("\nCOMBINED DATASET RESULTS")
for key, value in sorted(counts.items()):
    print(f"{key}: {value}")

print(f"\nInvalid/missing labels: {len(invalid)}")
for item in invalid[:20]:
    print(item)

print(f"\nDataset saved to: {output}")
