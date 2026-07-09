#!/usr/bin/env python3
"""
npz → LeRobot 标准格式 (parquet + mp4)
=======================================
把 recorder.py 生成的 npz+jpg 转为 LeRobot 框架直接能加载的数据集。

LeRobot 数据集结构:
  dataset/
  ├── data/
  │   └── chunk-000/
  │       └── episode_000000.parquet    ← 所有帧的关节数据
  ├── videos/
  │   ├── observation.images.cam_left/
  │   │   └── episode_000000.mp4        ← 视频编码 (压缩率极高)
  │   ├── observation.images.cam_right/
  │   │   └── episode_000000.mp4
  │   └── observation.images.cam_head/
  │       └── episode_000000.mp4
  └── meta/
      └── info.json                     ← 数据集元信息

用法:
  conda activate lerobot
  pip install pandas pyarrow av
  python convert_to_lerobot.py                              # 转所有 episode
  python convert_to_lerobot.py --episodes 1,2,3             # 只转指定
  python convert_to_lerobot.py --output ~/lerobot_dataset   # 指定输出
"""

import numpy as np
import pandas as pd
import os
import sys
import glob
import json
import argparse
import subprocess
from typing import List, Dict


def jpg_to_mp4(image_dir: str, output_mp4: str, fps: int = 15):
    """用 ffmpeg 把一序列 jpg 转成 mp4"""
    # ffmpeg -framerate 15 -pattern_type glob -i "frame_*.jpg" -c:v libx264 -pix_fmt yuv420p output.mp4
    cmd = [
        "ffmpeg", "-y", "-loglevel", "error",
        "-framerate", str(fps),
        "-pattern_type", "glob",
        "-i", os.path.join(image_dir, "frame_*.jpg"),
        "-c:v", "libx264",
        "-pix_fmt", "yuv420p",
        output_mp4
    ]
    try:
        subprocess.run(cmd, check=True)
        return True
    except (subprocess.CalledProcessError, FileNotFoundError):
        # ffmpeg 可能没装，fallback 到 cv2
        return _jpg_to_mp4_cv2(image_dir, output_mp4, fps)


def _jpg_to_mp4_cv2(image_dir: str, output_mp4: str, fps: int = 15):
    """用 OpenCV 把 jpg 转 mp4 (fallback, 更慢但不需要 ffmpeg)"""
    import cv2
    files = sorted(glob.glob(os.path.join(image_dir, "frame_*.jpg")))
    if not files:
        return False
    h, w = cv2.imread(files[0]).shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(output_mp4, fourcc, fps, (w, h))
    for f in files:
        writer.write(cv2.imread(f))
    writer.release()
    return True


def build_parquet(npz_paths: List[str],
                  img_dirs: List[str],
                  output_dir: str,
                  fps: int = 50):
    """构建 LeRobot 标准 parquet 数据集"""
    import pyarrow as pa
    import pyarrow.parquet as pq

    os.makedirs(output_dir, exist_ok=True)
    data_dir = os.path.join(output_dir, "data", "chunk-000")
    os.makedirs(data_dir, exist_ok=True)

    all_rows = []
    episode_ranges = {}
    total_frames = 0
    state_dim = None
    action_dim = None

    for ep_idx, (npz_path, img_dir) in enumerate(zip(npz_paths, img_dirs)):
        data = np.load(npz_path)
        states = data["states"]      # (N, state_dim)
        actions = data["actions"]    # (N, action_dim)
        n = len(states)

        if state_dim is None:
            state_dim = states.shape[1]
            action_dim = actions.shape[1]

        # 为每一帧生成一行
        for i in range(n):
            row = {
                "episode_index": ep_idx,
                "frame_index": i,
                "timestamp": i / fps,
                "observation.state": states[i].tolist(),
                "action": actions[i].tolist(),
            }
            all_rows.append(row)

        episode_ranges[f"episode_{ep_idx}"] = {
            "start": total_frames,
            "end": total_frames + n - 1,
            "length": n,
        }
        total_frames += n
        print(f"  ep_{ep_idx}: {n} 帧")

    # 转成 pandas DataFrame → parquet
    print(f"  写入 parquet ({total_frames} 行)...")
    df = pd.DataFrame(all_rows)

    # 用 pyarrow 写 parquet
    table = pa.Table.from_pandas(df)
    pq.write_table(
        table,
        os.path.join(data_dir, "episode_000000.parquet"),
        compression="snappy"
    )

    return {
        "total_frames": total_frames,
        "num_episodes": len(npz_paths),
        "state_dim": state_dim,
        "action_dim": action_dim,
        "episode_ranges": episode_ranges,
        "fps": fps,
    }


def build_videos(img_dirs: List[str], output_dir: str,
                 fps: int = 15):
    """把 jpg 转为 mp4 视频"""
    videos_dir = os.path.join(output_dir, "videos")
    cam_names = ["cam_left", "cam_right", "cam_head"]
    video_paths = {cam: [] for cam in cam_names}

    for ep_idx, img_dir in enumerate(img_dirs):
        if not os.path.isdir(img_dir):
            continue
        for cam in cam_names:
            cam_img_dir = os.path.join(img_dir, cam)
            if not os.path.isdir(cam_img_dir):
                continue
            cam_video_dir = os.path.join(
                videos_dir, f"observation.images.{cam}")
            os.makedirs(cam_video_dir, exist_ok=True)
            mp4_path = os.path.join(
                cam_video_dir, f"episode_{ep_idx:06d}.mp4")

            n_imgs = len(glob.glob(os.path.join(cam_img_dir, "*.jpg")))
            if n_imgs == 0:
                continue

            print(f"  编码视频: {cam}/ep_{ep_idx} ({n_imgs}帧)...")
            ok = jpg_to_mp4(cam_img_dir, mp4_path, fps)
            if ok:
                video_paths[cam].append(mp4_path)
                print(f"    ✓ {os.path.getsize(mp4_path)//1024}KB")

    return video_paths


def build_meta(output_dir: str, info: dict, video_paths: dict):
    """生成 LeRobot 标准的 meta/info.json"""
    meta_dir = os.path.join(output_dir, "meta")
    os.makedirs(meta_dir, exist_ok=True)

    # 构建 LeRobot 标准的 info.json
    meta = {
        "codebase_version": "v2.0",
        "robot_type": "dual_rm65",
        "total_episodes": info["num_episodes"],
        "total_frames": info["total_frames"],
        "total_tasks": 1,
        "total_chunks": 1,
        "chunks_size": info["total_frames"],
        "fps": info["fps"],
        "splits": {"train": f"0:{info['num_episodes']}"},
        "data_path": "data/chunk-000/episode_000000.parquet",
        "video_path": "videos/{video_key}/episode_{episode_index:06d}.mp4",
        "video_keys": [],
        "features": {
            "observation.state": {
                "dtype": "float32",
                "shape": [info["state_dim"]],
            },
            "action": {
                "dtype": "float32",
                "shape": [info["action_dim"]],
            },
        },
        "stats": {},  # 训练时自动计算
    }

    # 加视频路径
    for cam in ["cam_left", "cam_right", "cam_head"]:
        if video_paths.get(cam):
            key = f"observation.images.{cam}"
            meta["features"][key] = {
                "dtype": "video",
                "shape": [480, 640, 3],
            }
            meta["video_keys"].append(key)

    with open(os.path.join(meta_dir, "info.json"), "w") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    # stats.json (空, 训练时填充)
    with open(os.path.join(meta_dir, "stats.json"), "w") as f:
        json.dump({}, f)

    # episodes.jsonl — 每条 episode 的起止帧
    with open(os.path.join(meta_dir, "episodes.jsonl"), "w") as f:
        for ep_name, rng in info["episode_ranges"].items():
            ep_info = {
                "episode_index": int(ep_name.split("_")[1]),
                "tasks": ["task_0"],
                "length": rng["length"],
            }
            f.write(json.dumps(ep_info) + "\n")

    # tasks.jsonl
    with open(os.path.join(meta_dir, "tasks.jsonl"), "w") as f:
        f.write(json.dumps({
            "task_index": 0,
            "task": "teleoperated_task",
            "description": "Recorded via rospy teleop recorder"
        }) + "\n")

    print(f"\n✓ meta 文件已生成")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=str, default="~/lerobot_data")
    parser.add_argument("--output", type=str, default="~/lerobot_dataset")
    parser.add_argument("--episodes", type=str, default="")
    args = parser.parse_args()

    input_dir = os.path.expanduser(args.input)
    output_dir = os.path.expanduser(args.output)

    # 找所有左臂 npz (以此为基准配对)
    left_files = sorted(glob.glob(os.path.join(input_dir, "ep_*_left.npz")))
    if not left_files:
        print(f"未找到 ep_*_left.npz 在 {input_dir}/")
        sys.exit(1)

    # 过滤
    if args.episodes:
        selected = set(int(e.strip()) for e in args.episodes.split(",") if e.strip())
        left_files = [f for f in left_files if
                      int(os.path.basename(f).split("_")[1].split("_")[0]) in selected]

    print(f"找到 {len(left_files)} 条 episode\n")

    # 配对: 每个 episode 有 left + right
    npz_list, img_list = [], []
    for lf in left_files:
        fn = os.path.basename(lf)
        ep_str = fn.split("_")[1]  # "001" from "ep_001_left.npz"
        rf = lf.replace("_left", "_right")
        if not os.path.exists(rf):
            print(f"  ⚠ {fn} 缺配对右臂, 跳过")
            continue
        img_dir = os.path.join(input_dir, f"ep_{ep_str}_images")
        # 合并左右: state = left_states(12D) + right_states(12D) = 24D
        ld = np.load(lf)
        rd = np.load(rf)
        n = len(ld["states"])
        s = np.hstack([ld["states"], rd["states"]])   # (N, 24)
        a = np.hstack([ld["actions"], rd["actions"]]) # (N, 24)
        # 暂存为临时合并 npz
        tmp = f"{input_dir}/_tmp_{ep_str}.npz"
        np.savez_compressed(tmp, states=s, actions=a)
        npz_list.append(tmp)
        img_list.append(img_dir)
        print(f"  ep_{ep_str}: L+R 合并 {n}帧 → {s.shape[1]}D")

    # 1. 构建 parquet
    print("\n── 1. 构建 parquet ──")
    info = build_parquet(npz_list, img_list, output_dir)
    # 清理临时文件
    for f in npz_list: os.remove(f)
    print(f"  ✓ {info['total_frames']} 帧, state={info['state_dim']}D, "
          f"action={info['action_dim']}D")

    # 2. 构建视频
    print("\n── 2. 编码视频 ──")
    video_paths = build_videos(img_list, output_dir)

    # 3. 生成元数据
    print("\n── 3. 生成元数据 ──")
    build_meta(output_dir, info, video_paths)

    # 4. 打印结果
    print(f"\n{'='*60}")
    print(f"✓ 转换完成: {output_dir}")
    print(f"  {info['num_episodes']} episodes, {info['total_frames']} 帧")
    print(f"  state: ({info['state_dim']},) | action: ({info['action_dim']},)")
    for cam, paths in video_paths.items():
        if paths:
            print(f"  {cam}: {len(paths)} 个 mp4")
    print(f"{'='*60}")
    print(f"\n用 LeRobot 加载:")
    print(f"  from lerobot.common.datasets.lerobot_dataset import LeRobotDataset")
    print(f"  ds = LeRobotDataset(repo_id='{output_dir}')")
    print(f"  print(ds[0])  # 第一帧")


if __name__ == "__main__":
    main()
