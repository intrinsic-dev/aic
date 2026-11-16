#!/usr/bin/env python3
"""Populate aic_example_policies/resource: ResNet18 hub weights + grkw/aic_act_policy from Hugging Face."""

from __future__ import annotations

import os
import sys

HF_REPO_ID = "grkw/aic_act_policy"


def download_resnet(root: str, rel: str) -> None:
    """Fetch ResNet18 ImageNet weights under ``{root}/{rel}/hub`` (torch.hub layout)."""
    import torch
    from torchvision.models import ResNet18_Weights

    hub_dir = os.path.join(root, rel, "hub")

    os.makedirs(os.path.join(hub_dir, "checkpoints"), exist_ok=True)
    torch.hub.set_dir(hub_dir)
    url = ResNet18_Weights.IMAGENET1K_V1.url
    torch.hub.load_state_dict_from_url(url, map_location="cpu")


def download_hub_model(root: str, rel: str, repo_id: str = HF_REPO_ID) -> None:
    """Download a Hugging Face snapshot into ``{root}/{rel}/aic_act_policy``."""
    from huggingface_hub import snapshot_download

    policy_dir = os.path.join(root, rel, "aic_act_policy")
    cache_dir = os.environ.get("HF_HUB_CACHE")

    os.makedirs(policy_dir, exist_ok=True)
    if cache_dir:
        snapshot_download(
            repo_id=repo_id,
            local_dir=policy_dir,
            cache_dir=cache_dir,
        )
    else:
        snapshot_download(repo_id=repo_id, local_dir=policy_dir)


def main() -> int:
    root = os.path.abspath(os.environ.get("AIC_ROOT", os.getcwd()))
    rel = os.path.join("aic_example_policies", "resource")
    download_resnet(root, rel)
    download_hub_model(root, rel)
    return 0


if __name__ == "__main__":
    sys.exit(main())
