from pathlib import Path
from ultralytics import YOLO


def main():
    detector_dir = Path(__file__).resolve().parent
    data_yaml = detector_dir / "dataset" / "data.yaml"
    runs_dir = detector_dir / "runs"

    model = YOLO("yolov8n.pt")

    model.train(
        data=str(data_yaml),
        epochs=50,
        imgsz=640,
        batch=16,
        seed=42,
        device=0,
        project=str(runs_dir),
        name="baseline_v3"
    )


if __name__ == "__main__":
    main()

