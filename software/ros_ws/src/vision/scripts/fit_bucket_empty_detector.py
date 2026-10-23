#!/usr/bin/env python3
"""Build the empty-bucket memory bank and pick a threshold.

usage: fit_empty_model.py IMAGE_DIR OUT.pt [--roi x y w h] [--bank-size 10000]

Images are split 80/20. The 80% builds the bank; the 20% (never seen by the
bank) sets the threshold, so it reflects real empty-vs-empty variation.
Use frames from many lighting conditions / arm poses / dirt states.
"""

import argparse
import glob
import os
import random

import cv2
import numpy as np
import torch

from bucket_empty_detector.bucket_empty_detector import BucketDetector


def load(path, roi):
    img = cv2.cvtColor(cv2.imread(path), cv2.COLOR_BGR2RGB)
    if roi:
        x, y, w, h = roi
        img = img[y : y + h, x : x + w]
    return img


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("image_dir")
    ap.add_argument("out")
    ap.add_argument("--roi", nargs=4, type=int, default=None)
    ap.add_argument(
        "--bank-size", type=int, default=5000, help="smaller = faster on a Raspberry Pi"
    )
    ap.add_argument(
        "--input-size",
        type=int,
        default=160,
        help="network input side in px (saved into the model)",
    )
    ap.add_argument(
        "--margin",
        type=float,
        default=1.15,
        help="threshold = margin * max held-out empty score",
    )
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    a = ap.parse_args()

    paths = sorted(
        p
        for e in ("jpg", "jpeg", "png")
        for p in glob.glob(os.path.join(a.image_dir, f"*.{e}"))
    )
    random.seed(0)
    random.shuffle(paths)
    n_val = max(5, len(paths) // 5)
    val, train = paths[:n_val], paths[n_val:]
    print(f"{len(train)} train / {len(val)} held-out images")

    det = BucketDetector(device=a.device, input_size=a.input_size)
    feats = []
    for p in train:
        feats.append(det.features(load(p, a.roi)[None])[0].cpu())
    bank = torch.cat(feats)
    if bank.shape[0] > a.bank_size:
        idx = torch.randperm(bank.shape[0])[: a.bank_size]
        bank = bank[idx]
    det.bank = bank.to(det.device)
    print("memory bank:", tuple(det.bank.shape))

    scores = np.array([det.score(load(p, a.roi)) for p in val])
    print(
        f"held-out empty scores: mean={scores.mean():.3f} "
        f"max={scores.max():.3f} std={scores.std():.3f}"
    )
    det.threshold = float(scores.max() * a.margin)
    print(f"threshold = {det.threshold:.3f}")
    det.save(a.out)


if __name__ == "__main__":
    main()
