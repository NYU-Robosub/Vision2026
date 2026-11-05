#!/usr/bin/env python3
"""Prepare gate-only YOLO dataset, add clipped examples, train and evaluate.
Run from the Vision2026 repository root. Requires ultralytics, opencv-python, pyyaml.
"""
import argparse, random, shutil
from pathlib import Path
import cv2
import yaml

ROOT = Path(__file__).resolve().parents[2]  # script lives at detector/model_comparison/gate_only.py
SOURCE = ROOT / 'detector/dataset'
DEST = ROOT / 'detector/dataset_gate_only'
IMAGE_EXT = {'.jpg', '.jpeg', '.png', '.bmp', '.webp'}


def annotations(path):
    items = []
    if not path.exists():
        return items
    for line in path.read_text().splitlines():
        fields = line.split()
        if len(fields) != 5:
            continue
        try:
            cls = int(fields[0]); x, y, w, h = map(float, fields[1:])
        except ValueError:
            continue
        if cls in (0, 1) and 0 < w <= 1 and 0 < h <= 1:
            items.append((x,y,w,h))
    return items


def label_text(boxes):
    return ''.join('0 ' + ' '.join(f'{v:.6f}' for v in b) + '\n' for b in boxes)


def cropped_labels(boxes, width, height, rect, min_retained=.35):
    l, t, r, b = rect
    output = []
    for x,y,w,h in boxes:
        x1, y1 = (x-w/2)*width, (y-h/2)*height
        x2, y2 = (x+w/2)*width, (y+h/2)*height
        a, c = max(x1,l), min(x2,r)
        d, e = max(y1,t), min(y2,b)
        area = max(0,c-a)*max(0,e-d)
        if area <= 0 or area/((x2-x1)*(y2-y1)) < min_retained:
            continue
        cw, ch = r-l, b-t
        output.append(((a+c-2*l)/(2*cw), (d+e-2*t)/(2*ch), (c-a)/cw, (e-d)/ch))
    return output


def prepare(args):
    random.seed(args.seed)
    if not SOURCE.joinpath('data.yaml').exists():
        raise SystemExit(f'Missing {SOURCE}/data.yaml; run from Vision2026 root')
    if DEST.exists() and not args.overwrite:
        raise SystemExit(f'{DEST} exists; use --overwrite to rebuild it')
    if DEST.exists():
        shutil.rmtree(DEST)
    counts = {}
    for split in ('train','valid','test'):
        image_dir = SOURCE / split / 'images'
        label_dir = SOURCE / split / 'labels'
        (DEST/split/'images').mkdir(parents=True, exist_ok=True)
        (DEST/split/'labels').mkdir(parents=True, exist_ok=True)
        images = sorted(p for p in image_dir.iterdir() if p.suffix.lower() in IMAGE_EXT)
        n_crops = 0; n_with_gates = 0
        for image in images:
            boxes = annotations(label_dir / f'{image.stem}.txt')
            if boxes:
                n_with_gates += 1
            shutil.copy2(image, DEST/split/'images'/image.name)
            (DEST/split/'labels'/f'{image.stem}.txt').write_text(label_text(boxes))
            if split != 'train' or not boxes:
                continue
            im = cv2.imread(str(image))
            if im is None:
                continue
            H,W = im.shape[:2]
            for j in range(args.crops_per_image):
                target = random.choice(boxes)
                cx,cy,bw,bh = target
                # Crop through a target gate with a random edge, keeping ~55%-85%
                # of its bounding box in view. Dataset labels are clipped to crop.
                x1, y1 = (cx-bw/2)*W, (cy-bh/2)*H
                x2, y2 = (cx+bw/2)*W, (cy+bh/2)*H
                side = random.choice(('left','right','top','bottom'))
                retain = random.uniform(.55,.85)
                l,t,r,b = 0,0,W,H
                if side == 'left': l = round(x2 - (x2-x1)*retain)
                if side == 'right': r = round(x1 + (x2-x1)*retain)
                if side == 'top': t = round(y2 - (y2-y1)*retain)
                if side == 'bottom': b = round(y1 + (y2-y1)*retain)
                l,t,r,b = max(0,l),max(0,t),min(W,r),min(H,b)
                if r-l < 96 or b-t < 96:
                    continue
                updated = cropped_labels(boxes,W,H,(l,t,r,b))
                if not updated:
                    continue
                stem = f'{image.stem}_clip_{j:02d}'
                suffix = image.suffix
                cv2.imwrite(str(DEST/split/'images'/f'{stem}{suffix}'), im[t:b,l:r])
                (DEST/split/'labels'/f'{stem}.txt').write_text(label_text(updated))
                n_crops += 1
        counts[split] = (len(images), n_with_gates, n_crops)
    config = {'path': str(DEST.resolve()), 'train':'train/images', 'val':'valid/images', 'test':'test/images', 'nc':1, 'names':['gate']}
    (DEST/'data.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    print('Created:', DEST)
    print('Old classes 0=gate and 1=gate_blue merged -> new class 0=gate; legs ignored')
    for split,(n,p,c) in counts.items():
        print(f'{split}: {n} original images, {p} with full-gate annotations, {c} new clipped crops')
    print('IMPORTANT: review sampled crops/labels before training. Partial visible gates not already labeled as entire gates will need manual annotations.')


def train(args):
    from ultralytics import YOLO
    checkpoint = ROOT/'detector/runs/baseline_v3/weights/best.pt'
    if not checkpoint.exists():
        raise SystemExit(f'Cannot find {checkpoint}')
    model = YOLO(str(checkpoint))
    model.train(data=str(DEST/'data.yaml'), epochs=args.epochs, imgsz=640, batch=args.batch,
                project=str(ROOT/'detector/runs'), name='gate_only_clipped', exist_ok=True,
                device=args.device)


def evaluate(args):
    from ultralytics import YOLO
    weight = ROOT/'detector/runs/gate_only_clipped/weights/best.pt'
    if not weight.exists():
        raise SystemExit(f'Missing {weight}; run train first')
    model = YOLO(str(weight))
    output = ROOT/'detector/model_comparison'
    for split in ('valid','test'):
        metrics = model.val(data=str(DEST/'data.yaml'), split='val' if split == 'valid' else 'test',
                            imgsz=640, project=str(output), name=f'gate_only_{split}', exist_ok=True, plots=True,
                            device=args.device)
        print(split, 'Precision:',metrics.box.mp, 'Recall:',metrics.box.mr,
              'mAP50:',metrics.box.map50,'mAP50-95:',metrics.box.map)
    model.predict(source=str(DEST/'test/images'), save=True, conf=.25,
                  project=str(output), name='gate_only_predictions', exist_ok=True,device=args.device)
    print('Results:', output)


if __name__ == '__main__':
    p = argparse.ArgumentParser()
    p.add_argument('action', choices=['prepare','train','evaluate'])
    p.add_argument('--overwrite', action='store_true')
    p.add_argument('--seed', type=int, default=26)
    p.add_argument('--crops-per-image', type=int, default=2)
    p.add_argument('--epochs', type=int, default=35)
    p.add_argument('--batch', type=int, default=8)
    p.add_argument('--device', default='cpu', help='cpu or 0 for supported Nvidia GPU')
    a=p.parse_args()
    {'prepare':prepare,'train':train,'evaluate':evaluate}[a.action](a)
