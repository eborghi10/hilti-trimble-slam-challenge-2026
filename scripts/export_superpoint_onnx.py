#!/usr/bin/env python3
"""
Export SuperPoint backbone to ONNX for C++ inference via ONNX Runtime.

Produces a model that outputs:
  - scores: (1, 1, H, W) — detection confidence map (full resolution)
  - descriptors: (1, 256, H/8, W/8) — dense L2-normalized descriptor map

Post-processing (NMS, top-K, descriptor sampling) is done in C++.
"""

import argparse
import os
import torch
import torch.nn as nn
import torch.nn.functional as F
from lightglue import SuperPoint


class SuperPointBackbone(nn.Module):
    """SuperPoint backbone export: produces dense score map + descriptor map."""

    def __init__(self, sp_model):
        super().__init__()
        self.relu = nn.ReLU(inplace=False)
        self.pool = nn.MaxPool2d(kernel_size=2, stride=2)
        self.conv1a = sp_model.conv1a
        self.conv1b = sp_model.conv1b
        self.conv2a = sp_model.conv2a
        self.conv2b = sp_model.conv2b
        self.conv3a = sp_model.conv3a
        self.conv3b = sp_model.conv3b
        self.conv4a = sp_model.conv4a
        self.conv4b = sp_model.conv4b
        self.convPa = sp_model.convPa
        self.convPb = sp_model.convPb
        self.convDa = sp_model.convDa
        self.convDb = sp_model.convDb

    def forward(self, image):
        """
        Args:
            image: (1, 1, H, W) grayscale float32 in [0, 1]. H, W must be divisible by 8.
        Returns:
            scores: (1, 1, H, W) detection score map
            descriptors: (1, 256, H/8, W/8) L2-normalized dense descriptor map
        """
        x = self.relu(self.conv1a(image))
        x = self.relu(self.conv1b(x))
        x = self.pool(x)
        x = self.relu(self.conv2a(x))
        x = self.relu(self.conv2b(x))
        x = self.pool(x)
        x = self.relu(self.conv3a(x))
        x = self.relu(self.conv3b(x))
        x = self.pool(x)
        x = self.relu(self.conv4a(x))
        x = self.relu(self.conv4b(x))

        # Score head: 65-channel (64 cells + 1 dustbin) -> full-res score map
        cPa = self.relu(self.convPa(x))
        score_logits = self.convPb(cPa)  # (1, 65, H/8, W/8)
        B, _, Hc, Wc = score_logits.shape
        scores = torch.softmax(score_logits, dim=1)[:, :-1]  # (1, 64, H/8, W/8)
        scores = scores.reshape(B, 8, 8, Hc, Wc)
        scores = scores.permute(0, 1, 3, 2, 4)  # (1, 8, H/8, 8, W/8)
        scores = scores.reshape(B, 1, Hc * 8, Wc * 8)  # (1, 1, H, W)

        # Descriptor head
        cDa = self.relu(self.convDa(x))
        descriptors = self.convDb(cDa)  # (1, 256, H/8, W/8)
        descriptors = F.normalize(descriptors, p=2, dim=1)

        return scores, descriptors


def main():
    parser = argparse.ArgumentParser(description="Export SuperPoint to ONNX")
    parser.add_argument("--output", type=str, default="/ros2_ws/support_files/superpoint.onnx",
                        help="Output ONNX file path")
    parser.add_argument("--opset", type=int, default=17, help="ONNX opset version")
    args = parser.parse_args()

    os.makedirs(os.path.dirname(args.output), exist_ok=True)

    # Load pre-trained SuperPoint
    sp = SuperPoint(max_num_keypoints=1024).eval().cpu()
    model = SuperPointBackbone(sp)
    model.eval()

    # Dummy input (divisible by 8)
    dummy = torch.randn(1, 1, 480, 480)
    with torch.no_grad():
        scores, desc = model(dummy)
    print(f"Test output — scores: {scores.shape}, descriptors: {desc.shape}")

    # Export
    torch.onnx.export(
        model,
        dummy,
        args.output,
        input_names=["image"],
        output_names=["scores", "descriptors"],
        dynamic_axes={
            "image": {2: "height", 3: "width"},
            "scores": {2: "height_full", 3: "width_full"},
            "descriptors": {2: "height_8", 3: "width_8"},
        },
        opset_version=args.opset,
        do_constant_folding=True,
    )

    size_mb = os.path.getsize(args.output) / 1024 / 1024
    print(f"Exported: {args.output} ({size_mb:.1f} MB)")


if __name__ == "__main__":
    main()
