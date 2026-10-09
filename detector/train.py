from ultralytics import YOLO

def main():
    model = YOLO("yolov8n.pt")

    model.train(
        data="dataset/data.yaml",
        epochs=3,
        imgsz=640,
        batch=16,
        seed=42,
        project="runs",
        name="sanity_check"
    )

if __name__ == "__main__":
    main()

