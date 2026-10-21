# RoboSub Gate Detector Evaluation

## New YOLO Baseline

Model: YOLOv8n (baseline_v3)

Dataset: RoboSub Roboflow v3

Test dataset: 13 images, 39 labeled objects

### Results

| Metric | Result |
|---|---|
| Precision | 1.0000 |
| Recall | 1.0000 |
| mAP@50 | 0.9950 |
| mAP@50-95 | 0.9093 |

Evaluation settings:
- Image size: 640
- Confidence threshold: 0.25
- Dataset split: test

### Evaluated Classes

- gate
- gate_leg_l
- gate_leg_r

The test set did not contain labeled gate_blue objects.

### Evaluation Images

- PR_curve.png: Precision-recall curve
- confusion_matrix.png: Detection confusion matrix
- examples/: Sample predictions with bounding boxes

### Limitations

The evaluation used only 13 test images. Additional testing on
unseen underwater footage is needed to verify generalization.

## Old vs New Detector Comparison

The previous RoboSub detector weights are not yet available locally.

Once obtained, both detectors will be evaluated on the same images
using compatible class definitions and consistent evaluation settings.
