#!/usr/bin/env python3
"""
Episode 回放查看器
===================
加载 npz + jpg，逐帧回放关节动作 + 相机图像。
就像看录像一样验证数据质量。

用法:
  conda activate lerobot
  python replay_episode.py                           # 回放最新 episode
  python replay_episode.py --episode 1               # 回放第 1 条
  python replay_episode.py --episode 1 --speed 0.5   # 半速回放
  python replay_episode.py --episode 1 --speed 2     # 2倍速

控制:  空格=暂停/继续  ←→=快退快进  q=退出
"""

import numpy as np
import cv2
import os
import sys
import glob
import argparse
import time


def load_episode(data_dir: str, ep_id: int) -> dict:
    """加载一条 episode: 左臂 + 右臂 分别加载后合并"""
    lp = os.path.join(data_dir, f"ep_{ep_id:03d}_left.npz")
    rp = os.path.join(data_dir, f"ep_{ep_id:03d}_right.npz")

    # 兼容旧格式
    if not os.path.exists(lp):
        lp = os.path.join(data_dir, f"ep_{ep_id:03d}.npz")
        if not os.path.exists(lp):
            raise FileNotFoundError(f"找不到 ep_{ep_id:03d}")
        ld = np.load(lp)
        result = {
            "states": ld["states"], "actions": ld["actions"],
            "fps": int(ld.get("fps", 50)), "n_frames": len(ld["states"]),
            "left_dof": 6, "right_dof": 6,
        }
    else:
        ld = np.load(lp); rd = np.load(rp)
        # 合并: [左臂+手(12)] + [右臂+手(12)] = 24D
        s = np.hstack([ld["states"], rd["states"]])
        a = np.hstack([ld["actions"], rd["actions"]])
        result = {
            "states": s, "actions": a,
            "fps": int(ld.get("fps", 50)), "n_frames": len(s),
            "left_dof": 6, "left_hand": 6, "right_dof": 6, "right_hand": 6,
        }

    # 加载图像路径映射
    img_dir = os.path.join(data_dir, f"ep_{ep_id:03d}_images")
    result["img_dir"] = img_dir
    result["cams"] = []
    if os.path.isdir(img_dir):
        for cam in sorted(os.listdir(img_dir)):
            cam_path = os.path.join(img_dir, cam)
            if os.path.isdir(cam_path):
                # 构建帧号→图像路径映射
                frames = {}
                for f in sorted(os.listdir(cam_path)):
                    if f.endswith(".jpg"):
                        # frame_000000.jpg → 0
                        frame_num = int(f.split("_")[1].split(".")[0])
                        frames[frame_num] = os.path.join(cam_path, f)
                result["cams"].append({"name": cam, "frames": frames})

    return result


def replay(ep: dict, speed: float = 1.0):
    """主回放循环"""
    states = ep["states"]
    actions = ep["actions"]
    fps = ep["fps"]
    n = ep["n_frames"]
    period = 1.0 / (fps * speed)
    has_imgs = len(ep["cams"]) > 0

    print(f"Episode: {n} 帧 | {n/fps:.1f}s @ {fps}Hz | "
          f"播放速度: {speed:.1f}x | {has_imgs}路相机")
    print("控制: 空格=暂停 ←→=±1s q=退出")

    cv2.namedWindow("Replay", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Replay", 960, 320)

    i = 0
    paused = False
    t_last = time.time()

    while 0 <= i < n:
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break
        elif key == ord(' '):
            paused = not paused
            print(f"\r{'⏸ 暂停' if paused else '▶ 播放'} 帧 {i}/{n}", end="")
        elif key == 81:  # ← 左箭头
            i = max(0, i - int(fps * speed))
            print(f"\r⏪ 帧 {i}/{n}", end="")
        elif key == 83:  # → 右箭头
            i = min(n - 1, i + int(fps * speed))
            print(f"\r⏩ 帧 {i}/{n}", end="")

        if paused:
            time.sleep(0.05)
            continue

        # 显示图像
        tiles = []
        for cam_info in ep["cams"]:
            # 找最近的图像帧
            frame_nums = sorted(cam_info["frames"].keys())
            if not frame_nums:
                tiles.append(np.zeros((240, 320, 3), dtype=np.uint8))
                continue
            # 找 ≤i 的最近帧
            closest = max(fn for fn in frame_nums if fn <= i) if any(fn <= i for fn in frame_nums) else frame_nums[0]
            img_path = cam_info["frames"].get(closest)
            if img_path and os.path.exists(img_path):
                img = cv2.imread(img_path)
                small = cv2.resize(img, (320, 240))
                cv2.putText(small, f"{cam_info['name']} f{closest}", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                tiles.append(small)
            else:
                tiles.append(np.zeros((240, 320, 3), dtype=np.uint8))

        if tiles:
            combined = np.hstack(tiles)
        else:
            combined = np.zeros((240, 960, 3), dtype=np.uint8)

        # 关节信息叠加在图像上: [左臂6, 左手6, 右臂6, 右手6]
        s = states[i]
        a = actions[i][:len(s)]
        la = s[0:6] * 57.3      # 左臂
        lh = s[6:12]            # 左手
        ra = s[12:18] * 57.3    # 右臂
        rh = s[18:24]           # 右手

        y = 260
        cv2.putText(combined, f"Frame {i:05d}/{n}  t={i/fps:.1f}s",
                    (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        y += 22
        cv2.putText(combined, f"L arm: {np.array2string(la, precision=1, suppress_small=True)}",
                    (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        y += 18
        cv2.putText(combined, f"L hand:{np.array2string(lh.astype(int), suppress_small=True)}",
                    (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        y += 22
        cv2.putText(combined, f"R arm: {np.array2string(ra, precision=1, suppress_small=True)}",
                    (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        y += 18
        cv2.putText(combined, f"R hand:{np.array2string(rh.astype(int), suppress_small=True)}",
                    (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        cv2.imshow("Replay", combined)

        # 帧率控制
        elapsed = time.time() - t_last
        sleep_time = period - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)
        t_last = time.time()
        i += 1

    cv2.destroyAllWindows()
    print(f"\n回放结束: {i}/{n} 帧")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--episode", type=int, default=0,
                        help="Episode 编号, 0=最新")
    parser.add_argument("--speed", type=float, default=1.0,
                        help="播放速度倍数, 0.5=半速, 2=两倍速")
    parser.add_argument("--dir", type=str, default="~/lerobot_data",
                        help="数据目录")
    args = parser.parse_args()

    data_dir = os.path.expanduser(args.dir)

    if args.episode == 0:
        files = sorted(glob.glob(os.path.join(data_dir, "ep_*.npz")))
        if not files:
            print(f"未找到 npz 文件在 {data_dir}/")
            sys.exit(1)
        fn = os.path.basename(files[-1])
        ep_id = int(fn.split("_")[1].split(".")[0])
    else:
        ep_id = args.episode

    ep = load_episode(data_dir, ep_id)
    replay(ep, args.speed)


if __name__ == "__main__":
    main()
