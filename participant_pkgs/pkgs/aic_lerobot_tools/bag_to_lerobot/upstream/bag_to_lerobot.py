#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 2 bag → LeRobot v3.0 exporter.

Overview
--------
This script converts one or more ROS 2 bags into a LeRobot v3 dataset using
the *same* contract-aware processing utilities used for live
inference. That keeps train/serve paths aligned and minimizes skew.

The conversion pipeline:

1) Load a "contract" describing topics/types/QoS/rate and feature shapes.
2) Scan a bag once; decode each contract topic using shared `decode_value`.
3) Select timestamps per a policy (`contract` / `bag` / `header`).
4) Resample each stream at the contract rate and assemble frames.
5) Coerce/resize images with the shared helpers and write to LeRobot.

Dependencies
------------
Only two shared modules (keep it unified with live inference):

- `rosetta.common.contract_utils`:
    `load_contract`, `iter_specs`, `feature_from_spec`
- `rosetta.common.processing_utils`:
    `decode_value`, `resample`, `stamp_from_header_ns`,
    `_nearest_resize_rgb`, `_coerce_to_uint8_rgb`, `zero_pad`

Command-line usage
------------------
Convert a single bag:

    $ python bag_to_lerobot.py \\
        --bag /path/to/bag_dir \\
        --contract /path/to/contract.yaml \\
        --out /path/to/out_root

Convert multiple bags:

    $ python bag_to_lerobot.py \\
        --bags /bag/epi1 /bag/epi2 \\
        --contract /path/to/contract.yaml \\
        --out /path/to/out_root

Options of note:

- `--timestamp {contract,bag,header}`
    How to pick per-message timestamps before resampling:
    * contract: per-spec `stamp_src` (default)
    * bag:      use the bag receive time
    * header:   prefer `msg.header.stamp` with bag time as fallback

- `--no-videos`
    Store PNG images instead of H.264/MP4 videos.

Outputs
-------
A LeRobot v3 dataset with:

- `videos/<image_key>/chunk-*/file-*.mp4`  (or `images/*/*.png` if `--no-videos`)
- `data/chunk-*/file-*.parquet`
- `meta/info.json`, `meta/tasks.parquet`, `meta/stats.json`
- `meta/episodes/*/*.parquet`

Notes
-----
- Image coercion uses shared helpers to consistently handle grayscale/alpha,
  float ranges, and nearest-neighbor resize.
- Feature dicts are built directly from `feature_from_spec()` so train-time and
  serve-time shapes match exactly.

"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

import numpy as np
import yaml

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# ---- LeRobot
from lerobot.datasets.lerobot_dataset import LeRobotDataset

# ---- Shared core (ONLY these two)
from rosetta.common.contract_utils import (
    load_contract,
    iter_specs,
    feature_from_spec,
    contract_fingerprint,
)
from rosetta.common.contract_utils import (
    decode_value,
    resample,
    stamp_from_header_ns,
    zero_pad as make_zero_pad,  # alias to avoid name clash with dict var
)

# Import decoders to register them
import rosetta.common.decoders  # noqa: F401

# ---------------------------------------------------------------------------


@dataclass
class _Stream:
    """Decoded per-topic stream buffers accumulated from a bag scan.

    Attributes
    ----------
    spec : Any
        The `SpecView` for this stream (observation or action).
    ros_type : str
        Fully-qualified ROS message type string for deserialization.
    ts : list[int]
        Per-message timestamps in nanoseconds (selected by policy).
    val : list[Any]
        Decoded values in contract-native form (e.g., HWC arrays for images).
    """

    spec: Any
    ros_type: str
    ts: List[int]
    val: List[Any]


# ---------------------------------------------------------------------------


def _read_yaml(p: Path) -> Dict[str, Any]:
    """Read a YAML file if it exists; return {} on absence/parse failures."""
    if not p.exists():
        print(f"[WARN] {p} does not exist")
        return {}
    with p.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _topic_type_map(reader: rosbag2_py.SequentialReader) -> Dict[str, str]:
    """Build a `{topic: type}` map from a rosbag2 reader."""
    return {t.name: t.type for t in reader.get_all_topics_and_types()}


def _plan_streams(
    specs: Iterable[Any],
    tmap: Dict[str, str],
) -> Tuple[Dict[str, _Stream], Dict[str, List[str]]]:
    """Plan `_Stream` buffers for contract specs and build a topic dispatch index.

    Parameters
    ----------
    specs : Iterable[Any]
        Iterable of `SpecView` objects derived from the contract.
    tmap : dict[str, str]
        Map from topic name to ROS type in the bag.

    Returns
    -------
    streams : dict[str, _Stream]
        Mapping from contract key to `_Stream` state.
    by_topic : dict[str, list[str]]
        Mapping from topic name to a list of contract keys using it.

    Raises
    ------
    RuntimeError
        If none of the contract topics exist in the bag.
    """
    streams: Dict[str, _Stream] = {}
    by_topic: Dict[str, List[str]] = {}
    for sv in specs:
        if sv.topic not in tmap:
            # Derive a human-readable kind for logging without assuming SpecView internals.
            if hasattr(sv, "is_action") and sv.is_action:
                kind = "action"
            elif str(getattr(sv, "key", "")).startswith("task."):
                kind = "task"
            else:
                kind = "observation"
            print(
                f"[WARN] Missing {kind} '{getattr(sv, 'key', '?')}' topic in bag: {sv.topic}"
            )
            continue
        rt = sv.ros_type or tmap[sv.topic]
        
        # Create unique key for multiple observation.state specs and action specs
        if sv.key == "observation.state":
            unique_key = f"{sv.key}_{sv.topic.replace('/', '_')}"
        elif sv.is_action:
            # For action specs, we need to check if there are multiple specs with the same key
            # This will be handled later in the consolidation logic
            unique_key = f"{sv.key}_{sv.topic.replace('/', '_')}"
        else:
            unique_key = sv.key
            
        streams[unique_key] = _Stream(spec=sv, ros_type=rt, ts=[], val=[])
        by_topic.setdefault(sv.topic, []).append(unique_key)
    if not streams:
        raise RuntimeError("No contract topics found in bag.")
    return streams, by_topic


# ---------------------------------------------------------------------------


def export_bags_to_lerobot(
    bag_dirs: List[Path],
    contract_path: Path,
    out_root: Path,
    repo_id: str = "rosbag_v30",
    use_videos: bool = True,
    image_writer_threads: int = 4,
    image_writer_processes: int = 0,
    chunk_size: int = 1000,
    data_mb: int = 100,
    video_mb: int = 500,
    timestamp_source: str = "contract",  # 'contract' | 'receive' | 'header'
) -> None:
    """Convert bag directories into a LeRobot v3 dataset under `out_root`.

    Parameters
    ----------
    bag_dirs : list[pathlib.Path]
        One or more bag directories (episodes) to convert.
    contract_path : pathlib.Path
        Path to the YAML/JSON contract. Must specify `rate_hz` and specs.
    out_root : pathlib.Path
        Root directory where the LeRobot dataset will be created/updated.
    repo_id : str, default "rosbag_v30"
        Dataset repo_id metadata stored by LeRobot.
    use_videos : bool, default True
        If True, store videos; otherwise store per-frame PNG images.
    image_writer_threads : int, default 4
        Worker threads per process for image writing.
            The optimal number of processes and threads depends on your computer capabilities.
            Lerobot advises to use 4 threads per camera with 0 processes. If the fps is not stable, try to increase or lower
            the number of threads. If it is still not stable, try to use 1 subprocess, or more.
    image_writer_processes : int, default 0
    chunk_size : int, default 1000
        Max number of frames per Parquet/video chunk.
    data_mb : int, default 100
        Target data file size in MB per chunk.
    video_mb : int, default 500
        Target video file size in MB per chunk.
    timestamp_source : {"contract","receive","header"}, default "contract"
        Timestamp selection policy per decoded message:
        - "contract": Use bag time, unless spec.stamp_src == "header"
                      and a valid header stamp exists.
        - "receive":  Always use bag receive time.
        - "header":   Prefer header stamp; fall back to bag receive time.

    Raises
    ------
    ValueError
        If contract `rate_hz` is invalid (<= 0).
    RuntimeError
        If a bag contains no usable/decodable messages.

    Notes
    -----
    - Feature shapes/dtypes are built from `feature_from_spec(..., use_videos)`,
      so your exported dataset matches online inputs exactly.
    - Image coercion uses shared utilities for consistent preprocessing.
    """
    # Contract + specs
    contract = load_contract(contract_path)
    fps = int(contract.rate_hz)
    if fps <= 0:
        raise ValueError("Contract rate_hz must be > 0")
    step_ns = int(round(1e9 / fps))
    specs = list(iter_specs(contract))

    # Features (also detect first image key as anchor)
    features: Dict[str, Dict[str, Any]] = {}
    primary_image_key: Optional[str] = None
    state_specs = []  # Track multiple observation.state specs
    action_specs_by_key: Dict[str, List[Any]] = {}  # Track multiple action specs by key
    
    for sv in specs:
        # Handle multiple observation.state specs
        if sv.key == "observation.state":
            state_specs.append(sv)
            # Don't add to features yet - we'll consolidate them
            continue
            
        # Handle action specs
        if sv.is_action:
            if sv.key not in action_specs_by_key:
                action_specs_by_key[sv.key] = []
            action_specs_by_key[sv.key].append(sv)
            # Don't add to features yet - we'll consolidate them
            continue
            
        # Process other specs normally
        k, ft, is_img = feature_from_spec(sv, use_videos)
            
        # Ensure task.* specs are treated as per-frame strings even if the
        # underlying helper doesn't special-case them yet.
        if str(k).startswith(
            "task."
        ):  # TODO: why is this special-cased? Shouldn't this be handled in constract_utils?
            # Normalize to a simple scalar string field.
            features[k] = {"dtype": "string", "shape": [1]}
        else:
            # Special handling for depth images - they now have 3 channels
            if k.endswith(".depth") and ft["shape"][-1] == 1:
                # Update the shape to reflect 3 channels
                ft["shape"] = list(ft["shape"])
                ft["shape"][-1] = 3
            features[k] = ft
        if is_img and primary_image_key is None:
            primary_image_key = sv.key
    
    # Consolidate multiple observation.state specs into a single feature
    if state_specs:
        all_names = []
        total_shape = 0
        for sv in state_specs:
            all_names.extend(sv.names)
            total_shape += len(sv.names)
        
        features["observation.state"] = {
            "dtype": "float32",
            "shape": (total_shape,),
            "names": all_names
        }
    
    # Consolidate multiple action specs with the same key into a single feature
    for action_key, action_specs in action_specs_by_key.items():
        if len(action_specs) > 1:
            # Multiple specs with same key - consolidate them
            all_names = []
            total_shape = 0
            for sv in action_specs:
                all_names.extend(sv.names)
                total_shape += len(sv.names)
            
            features[action_key] = {
                "dtype": "float32",
                "shape": (total_shape,),
                "names": all_names
            }
        else:
            # Single spec - use it as-is
            sv = action_specs[0]
            k, ft, _ = feature_from_spec(sv, use_videos)
            features[k] = ft
    
    # Mark depth videos in features metadata before dataset creation
    for key, feature in features.items():
        if key.endswith(".depth") and feature.get("dtype") == "video":
            if "info" not in feature:
                feature["info"] = {}
            feature["info"]["video.is_depth_map"] = True

    # Dataset
    ds = LeRobotDataset.create(
        repo_id=repo_id,
        fps=fps,
        features=features,
        root=out_root,
        robot_type=contract.robot_type,
        use_videos=use_videos,
        image_writer_processes=image_writer_processes,  # keep simple & predictable
        image_writer_threads=image_writer_threads,
        batch_encoding_size=1,
    )
    
    # Persist the contract fingerprint into info.json so training can validate & propagate it
    try:
        fp = contract_fingerprint(contract)
        ds.meta.info["rosetta_fingerprint"] = fp
    except Exception:
        pass  # non-fatal; downstream will just skip the check
    ds.meta.update_chunk_settings(
        chunks_size=chunk_size,
        data_files_size_in_mb=data_mb,
        video_files_size_in_mb=video_mb,
    )
    

    # Precompute zero pads + shapes for fast frame assembly.
    zero_pad_map = {k: make_zero_pad(ft) for k, ft in features.items()}
    write_keys = [
        k
        for k, ft in features.items()
        if ft["dtype"] in ("video", "image", "float32", "float64", "string")
    ]
    shapes = {k: tuple(features[k]["shape"]) for k in write_keys}

    # Episodes
    for epi_idx, bag_dir in enumerate(bag_dirs):
        print(f"[Episode {epi_idx}] {bag_dir}")

        try:
            meta = _read_yaml(bag_dir / "metadata.yaml")
            info = meta.get("rosbag2_bagfile_information") or {}
            storage = info.get("storage_identifier") or "mcap"
            meta_dur_ns = int((info.get("duration") or {}).get("nanoseconds") or 0)

            # Operator prompt (if present). Accept either old/new keys gracefully.
            prompt = ""
            cd = info.get("custom_data")
            if isinstance(cd, dict):
                prompt = cd.get("lerobot.operator_prompt", prompt) or prompt

            # Reader
            reader = rosbag2_py.SequentialReader()
            reader.open(
                rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage),
                rosbag2_py.ConverterOptions(
                    input_serialization_format="cdr",
                    output_serialization_format="cdr",
                ),
            )
        except Exception as e:
            print(f"⚠️  Skipping bag {bag_dir} due to error: {e}")
            continue

        tmap = _topic_type_map(reader)
        print(f"tmap: {tmap}")

        # Plan once - handle multiple observation.state specs and action specs
        streams, by_topic = _plan_streams(specs, tmap)
        
        # Create consolidated observation.state stream if we have multiple state specs
        if state_specs:
            # Find all observation.state streams
            state_streams = [k for k in streams.keys() if k == "observation.state"]
            if len(state_streams) > 1:
                # Create a consolidated stream that will concatenate the data
                # We'll handle this in the frame processing
                pass
        print(f"streams: {streams}")

        # Counters for light diagnostics
        decoded_msgs = 0

        # Decode single pass
        while reader.has_next():
            topic, data, bag_ns = reader.read_next()
            if topic not in by_topic:
                continue
            for key in by_topic[topic]:
                st = streams[key]
                msg = deserialize_message(data, get_message(st.ros_type))
                sv = st.spec

                # Timestamp selection policy
                if timestamp_source == "receive":
                    ts_sel = int(bag_ns)
                elif timestamp_source == "header":
                    ts_sel = stamp_from_header_ns(msg) or int(bag_ns)
                else:  # 'contract' (per-spec stamp_src)
                    ts_sel = int(bag_ns)
                    if sv.stamp_src == "header":
                        hdr = stamp_from_header_ns(msg)
                        if hdr is not None:
                            ts_sel = int(hdr)

                val = decode_value(st.ros_type, msg, sv)

                if val is not None:
                    st.ts.append(ts_sel)
                    st.val.append(val)
                    decoded_msgs += 1

        if decoded_msgs == 0:
            raise RuntimeError(f"No usable messages in {bag_dir} (none decoded).")

        # Choose anchor + duration
        valid_ts = [
            np.asarray(st.ts, dtype=np.int64) for st in streams.values() if st.ts
        ]
        if not valid_ts:
            raise RuntimeError(f"No usable messages in {bag_dir} (no timestamps).")
        if (
            primary_image_key
            and streams.get(primary_image_key)
            and streams[primary_image_key].ts
        ):
            start_ns = int(
                np.asarray(streams[primary_image_key].ts, dtype=np.int64).min()
            )
        else:
            start_ns = int(min(ts.min() for ts in valid_ts))

        ts_max = int(max(ts.max() for ts in valid_ts))
        observed_dur_ns = max(0, ts_max - start_ns)

        # Prefer observed duration unless bag metadata matches within ~2 ticks.
        if meta_dur_ns > 0 and abs(meta_dur_ns - observed_dur_ns) <= 2 * step_ns:
            dur_ns = meta_dur_ns
            print("Using duration from metadata")
        else:
            dur_ns = observed_dur_ns
            print(
                "Metadata duration disagrees with observed duration. Using observed duration"
            )

        # Ticks
        n_ticks = int(dur_ns // step_ns) + 1
        ticks_ns = start_ns + np.arange(n_ticks, dtype=np.int64) * step_ns


        # Resample onto ticks
        resampled: Dict[str, List[Any]] = {}
        for key, st in streams.items():
            if not st.ts:
                resampled[key] = [None] * n_ticks
                continue
            ts = np.asarray(st.ts, dtype=np.int64)
            pol = st.spec.resample_policy
            resampled[key] = resample(
                pol, ts, st.val, ticks_ns, step_ns, st.spec.asof_tol_ms
            )

        # Write frames
        for i in range(n_ticks):
            frame: Dict[str, Any] = {}
            
            # Handle consolidated observation.state by concatenating multiple state streams first
            if "observation.state" in features and state_specs:
                # Concatenate all observation.state values from different topics
                state_values = []
                for sv in state_specs:
                    unique_key = f"{sv.key}_{sv.topic.replace('/', '_')}"
                    stream_val = resampled.get(unique_key, [None] * n_ticks)[i]
                    if stream_val is not None:
                        val_array = np.asarray(stream_val, dtype=np.float32).reshape(-1)
                        state_values.append(val_array)
                
                if state_values:
                    # Concatenate all state values
                    concatenated_state = np.concatenate(state_values)
                    frame["observation.state"] = concatenated_state
                else:
                    # Use zero padding if no state values available
                    frame["observation.state"] = zero_pad_map["observation.state"]
            
            # Handle consolidated action specs by concatenating multiple action streams
            for action_key, action_specs in action_specs_by_key.items():
                if len(action_specs) > 1 and action_key in features:
                    # Concatenate all action values from different topics
                    action_values = []
                    for sv in action_specs:
                        unique_key = f"{sv.key}_{sv.topic.replace('/', '_')}"
                        stream_val = resampled.get(unique_key, [None] * n_ticks)[i]
                        if stream_val is not None:
                            val_array = np.asarray(stream_val, dtype=np.float32).reshape(-1)
                            action_values.append(val_array)
                    
                    if action_values:
                        # Concatenate all action values
                        concatenated_action = np.concatenate(action_values)
                        frame[action_key] = concatenated_action
                    else:
                        # Use zero padding if no action values available
                        frame[action_key] = zero_pad_map[action_key]
            
            # Process all other features
            for name in write_keys:
                # Skip observation.state as it's handled above
                if name == "observation.state":
                    continue
                
                # Skip consolidated actions as they're handled above
                if name in action_specs_by_key and len(action_specs_by_key[name]) > 1: #TODO: I think this can just be if name == "action"
                    continue
                    
                ft = features[name]
                dtype = ft["dtype"]
                val = resampled.get(name, [None] * n_ticks)[i]

                if val is None:
                    frame[name] = zero_pad_map[name]
                    continue

                if dtype in ("video", "image"):
                    arr = np.asarray(val)
                    # Ensure deterministic storage; lerobot loaders will map back to float [0,1]
                    if arr.dtype != np.uint8:
                        arr = np.clip(arr * 255.0, 0, 255).astype(np.uint8)
                    frame[name] = arr

                elif dtype in ("float32", "float64"):
                    tgt_dt = np.float32 if dtype == "float32" else np.float64
                    arr = np.asarray(val, dtype=tgt_dt).reshape(-1)
                    exp = int(ft["shape"][0])
                    if arr.shape[0] != exp:
                        fixed = np.zeros((exp,), dtype=tgt_dt)
                        fixed[: min(exp, arr.shape[0])] = arr[: min(exp, arr.shape[0])]
                        arr = fixed
                    frame[name] = arr

                elif dtype == "string":
                    frame[name] = str(val)

                else:
                    # Fallback – should not happen with current features
                    frame[name] = val

            # Episode-level operator prompt from bag metadata (kept for policy compatibility).
            # This is`` distinct from any per-frame task.* fields coming from ROS topics.
            # LeRobot requires 'task' field in every frame, so always set it (empty string if no prompt).
            frame["task"] = prompt if prompt else ""
            ds.add_frame(frame)

        ds.save_episode()
        print(
            f"  → saved {n_ticks} frames @ {int(round(fps))} FPS  | decoded_msgs={decoded_msgs}"
        )

    
    print(f"\n[OK] Dataset root: {ds.root.resolve()}")
    if use_videos:
        print("  - videos/<image_key>/chunk-*/file-*.mp4")
    else:
        print("  - images/*/*.png")
    print("  - data/chunk-*/file-*.parquet")
    print(
        "  - meta/info.json, meta/tasks.parquet, meta/stats.json, meta/episodes/*/*.parquet"
    )


# ---------------------------------------------------------------------------


def parse_args() -> argparse.Namespace:
    """Parse command-line args for bag → LeRobot conversion."""
    ap = argparse.ArgumentParser("ROS2 bag → LeRobot v3 (using rosetta.common.*)")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--bag", help="Path to a single bag directory (episode)")
    g.add_argument("--bags", nargs="+", help="Paths to multiple bag directories")
    ap.add_argument("--contract", required=True, help="Path to YAML contract")
    ap.add_argument("--out", required=True, help="Output dataset root")
    ap.add_argument("--repo-id", default="rosbag_v30", help="repo_id metadata")
    ap.add_argument(
        "--no-videos", action="store_true", help="Store images instead of videos"
    )
    ap.add_argument("--image-threads", type=int, default=4, help="Image writer threads")
    ap.add_argument(
        "--image-processes", type=int, default=0, help="Image writer processes"
    )
    ap.add_argument("--chunk-size", type=int, default=1000)
    ap.add_argument("--data-mb", type=int, default=100)
    ap.add_argument("--video-mb", type=int, default=500)
    ap.add_argument(
        "--timestamp",
        choices=("contract", "bag", "header"),
        default="contract",
        help=(
            "Which time base to use when resampling: "
            "'contract' (per-spec), 'bag' (receive), or 'header' (message header)."
        ),
    )
    return ap.parse_args()


def main() -> None:
    """CLI entry point for batch conversion of ROS 2 bags to LeRobot."""
    args = parse_args()
    bag_dirs = [Path(args.bag)] if args.bag else [Path(p) for p in args.bags]
    export_bags_to_lerobot(
        bag_dirs=bag_dirs,
        contract_path=Path(args.contract),
        out_root=Path(args.out),
        repo_id=args.repo_id,
        use_videos=not args.no_videos,
        image_writer_threads=args.image_threads,
        image_writer_processes=args.image_processes,
        chunk_size=args.chunk_size,
        data_mb=args.data_mb,
        video_mb=args.video_mb,
        timestamp_source=args.timestamp,
    )


if __name__ == "__main__":
    main()
