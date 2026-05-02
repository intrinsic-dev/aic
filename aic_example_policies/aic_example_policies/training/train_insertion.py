"""Train the Phase 2 (insertion) SAC policy.

Usage:
    python -m aic_example_policies.training.train_insertion \\
        --scene /path/to/scene.xml \\
        --port_type sfp \\
        --total_steps 500000 \\
        --save_dir checkpoints/phase2

The Phase 2 policy consumes all 3 cameras + F/T sensor to guide descent.
The detection auxiliary loss uses only the center camera's projected port center.
"""

import argparse
import os
import time

import numpy as np
import torch

from .insertion_env import InsertionConfig, InsertionEnv
from .image_utils import project_point_to_image, normalize_uv, IMG_SIZE
from .sac_trainer import ReplayBuffer, SACTrainer, DEVICE


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--scene",         required=True, help="Path to scene.xml")
    p.add_argument("--port_type",     default="sfp", choices=["sfp", "sc"])
    p.add_argument("--total_steps",   type=int, default=500_000)
    p.add_argument("--buffer_size",   type=int, default=20_000)
    p.add_argument("--batch_size",    type=int, default=256)
    p.add_argument("--warmup_steps",  type=int, default=1_000)
    p.add_argument("--save_dir",      default="checkpoints/phase2")
    p.add_argument("--save_interval", type=int, default=50_000)
    p.add_argument("--log_interval",  type=int, default=1_000)
    p.add_argument("--det_loss_weight", type=float, default=0.1)
    p.add_argument("--load",          default=None, help="Resume from checkpoint")
    p.add_argument("--render_w",      type=int, default=320)
    p.add_argument("--render_h",      type=int, default=240)
    return p.parse_args()


def compute_gt_uv(env: InsertionEnv) -> np.ndarray:
    """GT detection label from center camera."""
    port_pos = env._get_port_pos()
    cam_id = env._cam_ids["center"]
    cam_xpos = env.data.cam_xpos[cam_id]
    cam_xmat = env.data.cam_xmat[cam_id]
    fovy = env.model.cam_fovy[cam_id]
    W, H = env.cfg.render_width, env.cfg.render_height
    u, v = project_point_to_image(port_pos, cam_xpos, cam_xmat, fovy, W, H)
    return np.array(normalize_uv(u, v, W, H), dtype=np.float32)


def _to_device_obs(obs: dict, device: torch.device) -> dict:
    """Convert numpy obs dict to torch tensors (unbatched → add batch dim)."""
    out = {}
    for k, v in obs.items():
        if k == "proprio":
            out[k] = torch.from_numpy(v).unsqueeze(0).float().to(device)
        else:
            # image: H×W×C → 1×C×H×W float [0,1]
            t = torch.from_numpy(v.transpose(2, 0, 1)).unsqueeze(0).float().to(device) / 255.0
            out[k] = t
    return out


def main():
    args = parse_args()

    cfg = InsertionConfig(
        scene_path=args.scene,
        port_type=args.port_type,
        render_width=args.render_w,
        render_height=args.render_h,
    )
    env = InsertionEnv(cfg)

    buffer = ReplayBuffer(
        capacity=args.buffer_size,
        phase=2,
        img_shape=(IMG_SIZE, IMG_SIZE, 4),
        proprio_dim=13,
    )
    trainer = SACTrainer(
        phase=2,
        det_loss_weight=args.det_loss_weight,
        device=DEVICE,
    )
    if args.load:
        trainer.load(args.load)
        print(f"Resumed from {args.load} (step {trainer._train_steps})")

    os.makedirs(args.save_dir, exist_ok=True)

    obs, _ = env.reset()
    total_steps = 0
    episode_reward = 0.0
    episode_count = 0
    ep_rewards: list = []
    loss_log: dict = {}
    t0 = time.time()

    print(f"Training Phase 2 on {DEVICE}. Total steps: {args.total_steps}")

    while total_steps < args.total_steps:
        if total_steps < args.warmup_steps:
            action = env.action_space.sample()
        else:
            obs_t = _to_device_obs(obs, DEVICE)
            action = trainer.actor.get_action(obs_t).squeeze(0).cpu().numpy()

        next_obs, reward, terminated, truncated, info = env.step(action)
        done = terminated or truncated

        gt_uv = compute_gt_uv(env)
        buffer.add(obs, action, reward, next_obs, float(done), gt_uv=gt_uv)
        obs = next_obs
        episode_reward += reward
        total_steps += 1

        if done:
            ep_rewards.append(episode_reward)
            episode_count += 1
            episode_reward = 0.0
            obs, _ = env.reset()

        if total_steps >= args.warmup_steps and len(buffer) >= args.batch_size:
            loss_log = trainer.update(buffer, args.batch_size)

        if total_steps % args.log_interval == 0 and ep_rewards:
            elapsed = time.time() - t0
            mean_r = np.mean(ep_rewards[-20:])
            print(
                f"[{total_steps:>7d}] ep={episode_count:>5d} "
                f"mean_r={mean_r:+7.2f} "
                f"critic={loss_log.get('critic_loss', 0):.4f} "
                f"actor={loss_log.get('actor_loss', 0):.4f} "
                f"det={loss_log.get('det_loss', 0):.4f} "
                f"alpha={loss_log.get('alpha', 0):.3f} "
                f"fps={total_steps / elapsed:.0f}"
            )

        if total_steps % args.save_interval == 0:
            ckpt_path = os.path.join(args.save_dir, f"step_{total_steps}.pt")
            trainer.save(ckpt_path)
            print(f"  Saved checkpoint: {ckpt_path}")

    trainer.save(os.path.join(args.save_dir, "final.pt"))
    print("Training complete.")
    env.close()


if __name__ == "__main__":
    main()
