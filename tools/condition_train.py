#!/usr/bin/env python3
"""Train the parcel condition classifier (intact / damaged) on ground-truth crops. Run in the training venv:

    ~/venvs/parcel_cls/bin/python tools/condition_train.py --dataset logs/condition/dataset \
        --splits logs/condition/splits.json --out logs/condition/runs/r1

Stop Gazebo first (GTX 1650, 4 GB). Splits are by WHOLE RECORDING; the test split is never read here.
"""
import argparse
import csv
import json
import math
import random
import time
from pathlib import Path

import cv2
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torchvision

CLASSES = ('intact', 'damaged')
MEAN = torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1)
STD = torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1)


def load_split(dataset, splits, split, column='gt_crop'):
    xs, ys, names = [], [], []
    with open(Path(dataset) / 'index.csv') as handle:
        for row in csv.DictReader(handle):
            if splits.get(row['recording']) != split or not row[column]:
                continue
            img = cv2.cvtColor(cv2.imread(str(Path(dataset) / row[column])), cv2.COLOR_BGR2RGB)
            xs.append(img)
            ys.append(CLASSES.index(row['label']))
            names.append(f"{row['recording']}/{row['parcel']}")
    return np.stack(xs), np.array(ys), names


def to_tensor(batch_u8, device, size):
    x = torch.from_numpy(batch_u8).to(device).permute(0, 3, 1, 2).float() / 255.0
    return F.interpolate(x, size=(size, size), mode='bilinear', align_corners=False)


def augment(x):
    n = x.shape[0]
    dev = x.device
    gain = 1 + (torch.rand(n, 1, 1, 1, device=dev) - 0.5) * 0.3          # brightness +-15 %
    contrast = 1 + (torch.rand(n, 1, 1, 1, device=dev) - 0.5) * 0.3
    mean = x.mean(dim=(1, 2, 3), keepdim=True)
    x = ((x - mean) * contrast + mean) * gain
    x = x + torch.randn_like(x) * 0.02                                   # sensor-like noise
    # small shift / rotation / scale (crop centring error of the detector)
    ang = (torch.rand(n, device=dev) - 0.5) * math.radians(16)
    scale = 1 + (torch.rand(n, device=dev) - 0.5) * 0.12
    shift = (torch.rand(n, 2, device=dev) - 0.5) * 0.14
    cos, sin = torch.cos(ang) / scale, torch.sin(ang) / scale
    theta = torch.stack([torch.stack([cos, -sin, shift[:, 0]], 1), torch.stack([sin, cos, shift[:, 1]], 1)], 1)
    grid = F.affine_grid(theta, x.shape, align_corners=False)
    x = F.grid_sample(x, grid, padding_mode='border', align_corners=False)
    return x.clamp(0, 1)


def build(arch):
    if arch == 'resnet18':
        m = torchvision.models.resnet18(weights=torchvision.models.ResNet18_Weights.IMAGENET1K_V1)
        m.fc = nn.Linear(m.fc.in_features, 2)
        head = list(m.fc.parameters())
    elif arch == 'mobilenet_v3_small':
        m = torchvision.models.mobilenet_v3_small(weights=torchvision.models.MobileNet_V3_Small_Weights.IMAGENET1K_V1)
        m.classifier[-1] = nn.Linear(m.classifier[-1].in_features, 2)
        head = list(m.classifier.parameters())
    else:
        raise ValueError(arch)
    return m, head


@torch.no_grad()
def evaluate(model, xs, ys, device, size, batch=128):
    model.eval()
    logits = []
    for i in range(0, len(xs), batch):
        x = (to_tensor(xs[i:i + batch], device, size) - MEAN.to(device)) / STD.to(device)
        logits.append(model(x).float().cpu())
    logits = torch.cat(logits)
    pred = logits.argmax(1).numpy()
    loss = F.cross_entropy(logits, torch.from_numpy(ys)).item()
    tp = int(((pred == 1) & (ys == 1)).sum())
    fp = int(((pred == 1) & (ys == 0)).sum())
    fn = int(((pred == 0) & (ys == 1)).sum())
    tn = int(((pred == 0) & (ys == 0)).sum())
    return dict(loss=loss, acc=(tp + tn) / len(ys), damaged_recall=tp / max(1, tp + fn),
                damaged_precision=tp / max(1, tp + fp), intact_recall=tn / max(1, tn + fp),
                balanced_acc=0.5 * (tp / max(1, tp + fn) + tn / max(1, tn + fp)),
                confusion=[[tn, fp], [fn, tp]])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dataset', required=True)
    ap.add_argument('--splits', required=True, help='json {recording: train|val|test}')
    ap.add_argument('--out', required=True)
    ap.add_argument('--arch', default='resnet18', choices=['resnet18', 'mobilenet_v3_small'])
    ap.add_argument('--size', type=int, default=128)
    ap.add_argument('--batch', type=int, default=64)
    ap.add_argument('--head-epochs', type=int, default=3)
    ap.add_argument('--epochs', type=int, default=20)
    ap.add_argument('--lr-head', type=float, default=1e-3)
    ap.add_argument('--lr', type=float, default=1e-4)
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args()
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    splits = json.loads(Path(args.splits).read_text())
    xtr, ytr, _ = load_split(args.dataset, splits, 'train')
    xva, yva, _ = load_split(args.dataset, splits, 'val')
    print(f'device={device} train={len(ytr)} (damaged {int(ytr.sum())}) val={len(yva)} (damaged {int(yva.sum())})')
    model, head = build(args.arch)
    model = model.to(device)
    freq = np.bincount(ytr, minlength=2).astype(np.float32)
    weight = torch.tensor(freq.sum() / (2 * np.maximum(freq, 1)), device=device)
    scaler = torch.amp.GradScaler(enabled=device == 'cuda')
    writer = None
    try:
        from torch.utils.tensorboard import SummaryWriter
        writer = SummaryWriter(str(out / 'tb'))
    except Exception as exc:  # noqa: BLE001
        print('tensorboard unavailable:', exc)
    config = dict(vars(args), device=device, torch=torch.__version__, class_weight=weight.tolist(),
                  train_crops=int(len(ytr)), val_crops=int(len(yva)),
                  weights='torchvision IMAGENET1K_V1 (BSD-3-Clause code; ImageNet-pretrained weights)')
    (out / 'config.json').write_text(json.dumps(config, indent=1))
    history, best = [], None
    total = args.head_epochs + args.epochs
    opt = None
    for epoch in range(total):
        phase = 'head' if epoch < args.head_epochs else 'full'
        if epoch == 0:
            for p in model.parameters():
                p.requires_grad = False
            for p in head:
                p.requires_grad = True
            opt = torch.optim.AdamW(head, lr=args.lr_head, weight_decay=1e-4)
        elif epoch == args.head_epochs:
            for p in model.parameters():
                p.requires_grad = True
            opt = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=1e-4)
            sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=max(1, args.epochs))
        elif phase == 'full':
            sched.step()
        model.train()
        order = np.random.permutation(len(ytr))
        running, t0 = 0.0, time.time()
        for i in range(0, len(order), args.batch):
            idx = order[i:i + args.batch]
            if len(idx) < 2:
                continue
            x = augment(to_tensor(xtr[idx], device, args.size))
            x = (x - MEAN.to(device)) / STD.to(device)
            y = torch.from_numpy(ytr[idx]).to(device)
            with torch.autocast(device_type='cuda', dtype=torch.float16, enabled=device == 'cuda'):
                loss = F.cross_entropy(model(x), y, weight=weight)
            opt.zero_grad(set_to_none=True)
            scaler.scale(loss).backward()
            scaler.step(opt)
            scaler.update()
            running += loss.item() * len(idx)
        val = evaluate(model, xva, yva, device, args.size)
        row = dict(epoch=epoch + 1, phase=phase, train_loss=running / len(ytr), seconds=round(time.time() - t0, 1),
                   **{f'val_{k}': v for k, v in val.items()})
        history.append(row)
        if writer:
            writer.add_scalar('loss/train', row['train_loss'], epoch)
            writer.add_scalar('loss/val', val['loss'], epoch)
            writer.add_scalar('val/balanced_acc', val['balanced_acc'], epoch)
            writer.add_scalar('val/damaged_recall', val['damaged_recall'], epoch)
        print(f"epoch {epoch + 1:02d} {phase:4s} train_loss {row['train_loss']:.4f} val_loss {val['loss']:.4f} "
              f"bal_acc {val['balanced_acc']:.3f} dmg_recall {val['damaged_recall']:.3f} "
              f"dmg_prec {val['damaged_precision']:.3f} ({row['seconds']}s)")
        key = (val['balanced_acc'], -val['loss'])
        if best is None or key > best[0]:
            best = (key, epoch + 1)
            torch.save({'model': model.state_dict(), 'arch': args.arch, 'size': args.size, 'epoch': epoch + 1},
                       out / 'best.pt')
    (out / 'history.json').write_text(json.dumps(history, indent=1))
    print(f'best epoch {best[1]} (val balanced acc {best[0][0]:.3f}); checkpoint {out / "best.pt"}')
    ckpt = torch.load(out / 'best.pt', map_location='cpu')
    model.load_state_dict(ckpt['model'])
    model.cpu().eval()
    dummy = torch.zeros(1, 3, args.size, args.size)
    torch.onnx.export(model, dummy, str(out / 'model.onnx'), input_names=['input'], output_names=['logits'],
                      dynamic_axes={'input': {0: 'n'}, 'logits': {0: 'n'}}, opset_version=17, dynamo=False)
    print('exported', out / 'model.onnx')
    if torch.cuda.is_available():
        print(f'peak VRAM {torch.cuda.max_memory_allocated() / 2**20:.0f} MiB')


if __name__ == '__main__':
    main()
