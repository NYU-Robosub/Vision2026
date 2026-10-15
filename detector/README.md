NYU RoboSub YOLO Baseline
Reproducible baseline training setup for the NYU RoboSub object detection dataset using Ultralytics YOLO.
Dataset
This setup uses RoboSub dataset version 3 from the NYU RoboSub Roboflow workspace.
- Roboflow project: robosub-ohluf
- Dataset version: 3
- Format: YOLOv8
- Train: 128 images
- Validation: 41 images
- Test: 13 images
- License: CC BY 4.0
Classes
1. gate
2. gate_blue
3. gate_leg_l
4. gate_leg_r
Project Structure
robosub-baseline/
├── dataset/
│   ├── train/
│   ├── valid/
│   ├── test/
│   └── data.yaml
├── train.py
├── requirements.txt
└── README.md
The Roboflow dataset should be extracted into dataset/.
The paths at the top of dataset/data.yaml should be:
train: train/images
val: valid/images
test: test/images
Environment
The initial setup was verified with:
- Python 3.10.12
- Ultralytics 8.4.174
- NVIDIA GPU with CUDA support
A GPU is recommended for training but is not required for setting up the project.
Setup
Create a virtual environment:
python3 -m venv .venv
source .venv/bin/activate
Install dependencies:
pip install -r requirements.txt
requirements.txt should contain:
ultralytics==8.4.174
Verify the environment:
yolo checks
Sanity Check Training
The current train.py is intended to verify that the dataset and training environment work end-to-end.
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
Run:
python train.py
Ultralytics will download the pretrained yolov8n.pt weights automatically if needed.
A successful run should complete all three epochs and produce trained weights including best.pt and last.pt. Ultralytics prints the exact results directory when training finishes.
Reproducibility Goal
This task is complete when another teammate can:
1. Get the same RoboSub v3 dataset.
2. Clone or copy this project.
3. Create the virtual environment.
4. Install the pinned dependencies.
5. Run python train.py without modifying the code or dataset paths.
6. Complete training successfully.
7. Obtain a working trained model and run inference with it.
The reproduced model does not need to have numerically identical metrics across different machines or GPUs, but it should train successfully and achieve comparable behavior.
Next Steps
After the sanity check is confirmed:
- choose the final baseline training settings
- run the full baseline training
- save the final best.pt
- evaluate on the validation/test sets
- record precision, recall, mAP50, and mAP50-95
- save example prediction images
- have another teammate reproduce the training from this repository
Team Workflow
Suggested ownership:
- Dataset / setup: fixed dataset version, environment, dependencies, and reproducibility setup
- Baseline training: final training configuration and trained weights
- Evaluation / reproduction: metrics, predictions, and independent reproduction test
## Baseline Results

The first baseline detector was trained on RoboSub v3 using YOLOv8n.

Training configuration:

- Model: YOLOv8n
- Epochs: 50
- Image size: 640
- Batch size: 16
- Seed: 42
- Ultralytics: 8.4.174

Test set:

- 13 images
- 39 labeled instances

Results:

- Precision: 0.994
- Recall: 1.000
- mAP50: 0.995
- mAP50-95: 0.909

Per-class mAP50-95:

- `gate`: 0.977
- `gate_leg_l`: 0.819
- `gate_leg_r`: 0.932

The test set did not contain ground-truth examples of `gate_blue`, so that class was not independently evaluated.

Qualitative inspection of the test predictions showed that the model consistently detected the full gate and both gate legs across the available test images.

This model is intended as a simple first baseline rather than a final production detector. The goal is to establish a working, reproducible training pipeline that can be improved with additional data, labeling cleanup, and broader underwater testing.
