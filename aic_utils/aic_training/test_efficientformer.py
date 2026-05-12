"""
Quick inference test for a local EfficientFormerV2-S1 checkpoint.

Usage:
    python test_efficientformer.py
    python test_efficientformer.py --batch 4 --img 224
    python test_efficientformer.py --cpu
"""

import argparse
import re
import time

import requests
import torch
import timm
from PIL import Image
from timm.data import create_transform, resolve_data_config

LOCAL_WEIGHTS = "/home/lhphanto/ws_aic/src/aic/efficientformerv2_s1/pytorch_model.bin"
MODEL_NAME    = "efficientformerv2_s1"
IMAGE_URL     = "http://images.cocodataset.org/val2017/000000039769.jpg"


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--batch", type=int, default=1)
    p.add_argument("--img",   type=int, default=224)
    p.add_argument("--cpu",   action="store_true", help="Force CPU even if CUDA is available")
    p.add_argument("--warmup", type=int, default=3)
    p.add_argument("--runs",   type=int, default=10)
    args = p.parse_args()

    device = torch.device("cpu" if args.cpu or not torch.cuda.is_available() else "cuda")

    print(f"Model   : {MODEL_NAME}")
    print(f"Weights : {LOCAL_WEIGHTS}")
    print(f"Input   : ({args.batch}, 3, {args.img}, {args.img})  device={device}")

    model = timm.create_model(MODEL_NAME, pretrained=False, features_only=True)

    # The checkpoint uses dot-separated names (stem.conv1, stages.0.*)
    # while timm's FeatureListNet wrapper expects underscore-separated names
    # (stem_conv1, stages_0.*).  Remap and load with strict=False to drop
    # the classifier head keys (norm, head, head_dist) absent in features_only mode.
    raw_sd = torch.load(LOCAL_WEIGHTS, map_location="cpu", weights_only=True)
    remapped = {}
    for k, v in raw_sd.items():
        k = k.replace("stem.conv1", "stem_conv1").replace("stem.conv2", "stem_conv2")
        k = re.sub(r"stages\.(\d+)", r"stages_\1", k)
        remapped[k] = v
    missing, unexpected = model.load_state_dict(remapped, strict=False)
    if missing:
        print(f"[warn] missing keys after remap ({len(missing)}): {missing[:3]} ...")
    print(f"Checkpoint loaded  (unexpected/dropped: {len(unexpected)} head/norm keys)")

    model.eval().to(device)

    print(f"\nFetching image from {IMAGE_URL} ...")
    image = Image.open(requests.get(IMAGE_URL, stream=True).raw).convert("RGB")
    config    = resolve_data_config({}, model=model)
    transform = create_transform(**config)
    print("LXH:", transform)
    x = transform(image).unsqueeze(0).expand(args.batch, -1, -1, -1).to(device)
    print(f"Image size: {image.size}  →  tensor: {tuple(x.shape)}")

    with torch.no_grad():
        stages = model(x)

    print(f"\nStages : {len(stages)}")
    for i, s in enumerate(stages):
        B, C, H, W = s.shape
        print(f"  stage {i}: shape={tuple(s.shape)}  patches={H}x{W}={H*W}  channels={C}")

    B, C, H, W = stages[-1].shape
    flat = stages[-1].permute(0, 2, 3, 1).reshape(B, H * W, C)
    print(f"\nFinal stage flattened (B, N, C): {tuple(flat.shape)}")

    # Timing
    if device.type == "cuda":
        torch.cuda.synchronize(device)
    for _ in range(args.warmup):
        with torch.no_grad():
            model(x)
    if device.type == "cuda":
        torch.cuda.synchronize(device)

    t0 = time.perf_counter()
    for _ in range(args.runs):
        with torch.no_grad():
            model(x)
    if device.type == "cuda":
        torch.cuda.synchronize(device)
    elapsed_ms = (time.perf_counter() - t0) * 1e3 / args.runs
    print(f"Avg inference : {elapsed_ms:.2f} ms  ({args.runs} runs, batch={args.batch})")


if __name__ == "__main__":
    main()
