"""PatchCore-style "is the bucket empty?" detector.

Frozen pretrained ResNet18 -> patch features -> nearest-neighbour distance to a
memory bank built from empty-bucket images only. High distance == something
in the bucket that has never been seen in the "empty" set.
"""

import numpy as np
import torch
import torch.nn.functional as F
import torchvision
from torchvision.models import ResNet18_Weights

MEAN = torch.tensor([0.485, 0.456, 0.406]).view(1, 3, 1, 1)
STD = torch.tensor([0.229, 0.224, 0.225]).view(1, 3, 1, 1)


class BucketDetector:
    def __init__(self, device="cpu", input_size=224):
        self.device = torch.device(device)
        self.input_size = input_size
        net = torchvision.models.resnet18(weights=ResNet18_Weights.DEFAULT)
        self.stem = torch.nn.Sequential(
            net.conv1, net.bn1, net.relu, net.maxpool, net.layer1
        )
        self.layer2, self.layer3 = net.layer2, net.layer3
        for m in (self.stem, self.layer2, self.layer3):
            m.eval().to(self.device)
            for p in m.parameters():
                p.requires_grad = False
        self.bank = None  # (N, D) tensor
        self.threshold = None  # float

    @torch.no_grad()
    def features(self, rgb_batch):
        """rgb_batch: (B,H,W,3) uint8 numpy -> (B, P, D) patch features."""
        x = torch.from_numpy(np.ascontiguousarray(rgb_batch)).to(self.device)
        x = x.permute(0, 3, 1, 2).float() / 255.0
        x = F.interpolate(
            x,
            size=(self.input_size, self.input_size),
            mode="bilinear",
            align_corners=False,
        )
        x = (x - MEAN.to(self.device)) / STD.to(self.device)
        f1 = self.layer2(self.stem(x))
        f2 = self.layer3(f1)
        # local neighbourhood aggregation, then align layer3 to layer2 resolution
        f1 = F.avg_pool2d(f1, 3, 1, 1)
        f2 = F.avg_pool2d(f2, 3, 1, 1)
        f2 = F.interpolate(f2, size=f1.shape[-2:], mode="bilinear", align_corners=False)
        f = torch.cat([f1, f2], dim=1)  # (B, 384, h, w)
        return f.flatten(2).permute(0, 2, 1)

    @torch.no_grad()
    def score(self, rgb):
        """rgb: (H,W,3) uint8. Returns anomaly score (higher = less empty)."""
        f = self.features(rgb[None])[0]  # (P, D)
        d = torch.cdist(f, self.bank).min(dim=1).values  # (P,)
        k = max(1, int(0.01 * d.numel()))  # mean of worst 1% of patches
        return d.topk(k).values.mean().item()

    def is_empty(self, rgb):
        s = self.score(rgb)
        return s < self.threshold, s

    def save(self, path):
        torch.save(
            {
                "bank": self.bank.cpu(),
                "threshold": self.threshold,
                "input_size": self.input_size,
            },
            path,
        )

    def load(self, path):
        d = torch.load(path, map_location=self.device)
        self.bank = d["bank"].to(self.device)
        self.threshold = d["threshold"]
        self.input_size = d["input_size"]
