"""Image preprocessing utilities shared between training environments and inference policy."""

import cv2
import numpy as np
import torch

IMG_SIZE = 84  # output image size after crop+resize
WIDE_CROP_PX = 400  # half-side of the initial wide crop (pixels in source image)
NARROW_CROP_PX = 150  # half-side used once xy_error is small


def ros_image_to_rgb(msg) -> np.ndarray:
    """Decode a sensor_msgs/Image to an H×W×3 uint8 RGB array."""
    data = np.frombuffer(msg.data, dtype=np.uint8)
    img = data.reshape(msg.height, msg.width, -1)
    enc = getattr(msg, "encoding", "rgb8")
    if enc in ("bgr8", "bgra8"):
        img = img[:, :, :3][:, :, ::-1].copy()
    elif enc in ("rgb8", "rgba8"):
        img = img[:, :, :3].copy()
    return img


def add_canny_channel(rgb: np.ndarray) -> np.ndarray:
    """Append a Canny-edge channel to an H×W×3 RGB image → H×W×4 uint8."""
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    edges = cv2.Canny(gray, threshold1=50, threshold2=150)  # uint8 0/255
    return np.concatenate([rgb, edges[:, :, np.newaxis]], axis=2)


def crop_and_resize(
    img: np.ndarray,
    cx: int,
    cy: int,
    half_size: int,
    out_size: int = IMG_SIZE,
) -> np.ndarray:
    """Crop a square region around (cx, cy) and resize to out_size×out_size.

    Clamps the crop window to image boundaries; pads with zeros if needed.
    """
    h, w = img.shape[:2]
    x0 = max(0, cx - half_size)
    x1 = min(w, cx + half_size)
    y0 = max(0, cy - half_size)
    y1 = min(h, cy + half_size)
    crop = img[y0:y1, x0:x1]
    if crop.size == 0:
        return np.zeros((out_size, out_size, img.shape[2]), dtype=np.uint8)
    return cv2.resize(crop, (out_size, out_size), interpolation=cv2.INTER_AREA)


def center_crop_and_resize(
    img: np.ndarray, half_size: int, out_size: int = IMG_SIZE
) -> np.ndarray:
    """Crop around the image center."""
    h, w = img.shape[:2]
    return crop_and_resize(img, w // 2, h // 2, half_size, out_size)


def to_tensor(img_np: np.ndarray) -> torch.Tensor:
    """Convert H×W×C uint8 numpy array to C×H×W float32 tensor in [0, 1]."""
    t = torch.from_numpy(img_np.transpose(2, 0, 1).copy())
    return t.float() / 255.0


def project_point_to_image(
    p_world: np.ndarray,
    cam_xpos: np.ndarray,
    cam_xmat: np.ndarray,
    fovy_deg: float,
    img_w: int,
    img_h: int,
) -> tuple[float, float]:
    """Project a 3-D world point to (u, v) pixel coordinates using a pinhole model.

    cam_xpos: (3,) camera position in world
    cam_xmat: (9,) or (3,3) camera rotation matrix (world-to-camera rows)
    fovy_deg: vertical field of view in degrees
    Returns (u, v) in pixel coordinates.
    """
    R = np.array(cam_xmat).reshape(3, 3)
    p_cam = R @ (p_world - cam_xpos)
    if p_cam[2] <= 0:
        return img_w / 2.0, img_h / 2.0  # behind camera
    f = img_h / (2.0 * np.tan(np.radians(fovy_deg) / 2.0))
    u = f * p_cam[0] / p_cam[2] + img_w / 2.0
    v = f * p_cam[1] / p_cam[2] + img_h / 2.0
    return float(u), float(v)


def normalize_uv(u: float, v: float, w: int, h: int) -> tuple[float, float]:
    """Normalize pixel (u,v) to [-1, 1]."""
    return 2.0 * u / w - 1.0, 2.0 * v / h - 1.0
