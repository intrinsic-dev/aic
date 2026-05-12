"""
PortKeypointNet — ViT-based keypoint detector for AIC port labelling.

Strategy: the three camera images are concatenated side-by-side along the
width axis before being fed into a single ViT backbone.  This means every
patch token from every camera can attend to every other patch, giving the
model full cross-view context with no explicit fusion module.

Concatenated layout (left | center | right):

    width  = 3 × W_single  (e.g. 3 × 288 = 864)
    height = H_single       (e.g. 256)

With patch size 16 and the default 288 × 256 camera resolution:
    patches per camera = 18 × 16 = 288
    total patches      = 3 × 288 = 864   (all attending to each other)

Output: per-(entity, port, camera) predictions for all 36 combinations
    conf_visible : P(projected point inside image FOV)
    conf_present : P(port in front of camera, Z > 0)
    xy           : normalised pixel coords in [0, 1]
    log_dist     : log(distance in metres) from camera to port
"""

import torch
import torch.nn as nn
import timm

from .constants import NUM_CAMERAS, NUM_OUTPUTS

# Default backbone — ViT-S/16, ~22 M params
# Use 'vit_small_patch14_dinov2' for DINOv2 features (same size, stronger but
# requires images resized so H and W are multiples of 14).
DEFAULT_BACKBONE = "vit_small_patch16_224"


class PortKeypointNet(nn.Module):
    NUM_OUTPUTS = NUM_OUTPUTS   # 36
    OUTPUT_DIM  = 5             # conf_visible, conf_present, x, y, log_dist

    def __init__(
        self,
        backbone: str = DEFAULT_BACKBONE,
        pretrained: bool = True,
        dropout: float = 0.3,
    ):
        super().__init__()

        # dynamic_img_size=True → positional embeddings are interpolated for any
        # (H, W) at runtime, so the concatenated 3×W input works without retraining.
        self.vit = timm.create_model(
            backbone,
            pretrained=pretrained,
            num_classes=0,          # remove classifier head → returns feature tensor
            dynamic_img_size=True,  # allow non-standard spatial sizes
        )

        feat_dim = self.vit.embed_dim  # 384 for ViT-S

        self.head = nn.Sequential(
            nn.LayerNorm(feat_dim),
            nn.Linear(feat_dim, 512),
            nn.SiLU(),
            nn.Dropout(dropout),
            nn.Linear(512, 256),
            nn.SiLU(),
            nn.Dropout(dropout * 0.67),
            nn.Linear(256, self.NUM_OUTPUTS * self.OUTPUT_DIM),
        )

    # ------------------------------------------------------------------

    @staticmethod
    def concat_images(imgs: list[torch.Tensor]) -> torch.Tensor:
        """
        imgs: list of NUM_CAMERAS tensors, each (B, 3, H, W)
        returns: (B, 3, H, NUM_CAMERAS * W)
        """
        assert len(imgs) == NUM_CAMERAS
        return torch.cat(imgs, dim=3)  # concat along width

    def forward(self, imgs: list[torch.Tensor]) -> dict[str, torch.Tensor]:
        """
        Args:
            imgs: list of 3 tensors (B, 3, H, W) — ordered [left, center, right]

        Returns:
            conf_visible : (B, 36)    sigmoid in [0, 1]
            conf_present : (B, 36)    sigmoid in [0, 1]
            xy           : (B, 36, 2) sigmoid in [0, 1]; dim-2 is (x_norm, y_norm)
            log_dist     : (B, 36)    raw logits for log(dist in metres)
        """
        x = self.concat_images(imgs)           # (B, 3, H, 3W)
        feat = self.vit(x)                     # (B, embed_dim) — CLS token
        raw = self.head(feat)                  # (B, 36 * 5)
        raw = raw.view(-1, self.NUM_OUTPUTS, self.OUTPUT_DIM)  # (B, 36, 5)

        return {
            "conf_visible": torch.sigmoid(raw[..., 0]),   # (B, 36)
            "conf_present": torch.sigmoid(raw[..., 1]),   # (B, 36)
            "xy":           torch.sigmoid(raw[..., 2:4]), # (B, 36, 2)
            "log_dist":     raw[..., 4],                  # (B, 36)
        }

    # ------------------------------------------------------------------
    # Convenience helpers for inference

    @torch.no_grad()
    def predict(
        self,
        imgs: list[torch.Tensor],
        vis_threshold: float = 0.5,
        pres_threshold: float = 0.5,
    ) -> list[dict]:
        """
        Returns human-readable predictions for a batch.
        xy / dist are None when the corresponding confidence is below threshold.
        """
        import math
        from .constants import OUTPUT_KEYS

        out = self.forward(imgs)
        B = imgs[0].shape[0]
        results = []
        for b in range(B):
            row = {}
            for i, (entity, port, cam) in enumerate(OUTPUT_KEYS):
                cv   = out["conf_visible"][b, i].item()
                cp   = out["conf_present"][b, i].item()
                xy   = out["xy"][b, i].tolist() if cv >= vis_threshold else None
                dist = math.exp(out["log_dist"][b, i].item()) if cp >= pres_threshold else None
                row[(entity, port, cam)] = {
                    "conf_visible": cv,
                    "conf_present": cp,
                    "xy": xy,
                    "dist_m": dist,
                }
            results.append(row)
        return results
