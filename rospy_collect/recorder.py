#!/usr/bin/env python3
"""
纯 ROS 数采录制器 (rospy 直连, 无 rosbridge)
==============================================
直接在 ROS 系统 Python 中运行, 用 rospy + cv_bridge,
不经过任何 WebSocket 桥接, 延迟最低, 最稳定。

同时录制:
  - 双臂关节角度 (/joint_states → npz)
  - 三路 D435 图像 (/camera_d435_X/color/image_raw → jpg)

启动:  conda deactivate
        source /opt/ros/noetic/setup.bash
        python3 recorder.py

控制:  按 Enter → 开始录制
        按 Enter → 结束录制并保存
        Ctrl+C → 退出
"""

import rospy
import numpy as np
import cv2
import os
import sys
import time
import threading
from collections import deque
from typing import Optional, Dict

from sensor_msgs.msg import JointState, Image
from cv_bridge import CvBridge
try:
    from embodied_arm_msgs.msg import Hand_Status
    HAS_HAND = True
except ImportError:
    HAS_HAND = False


class RosRecorder:
    """纯 ROS 数采录制器"""

    TARGET_FPS = 50

    def __init__(self, save_dir: str = "~/lerobot_data"):
        self.bridge = CvBridge()
        self.save_dir = os.path.expanduser(save_dir)

        # 关节数据缓存 (线程安全)
        self._joint_lock = threading.Lock()
        self._left_positions = None
        self._right_positions = None
        self._left_hand = None    # hand_angle[6]
        self._right_hand = None
        self._joint_seq = 0

        # 图像缓存 (线程安全)
        self._img_lock = threading.Lock()
        self._latest_images: Dict[str, np.ndarray] = {
            "cam_left": None, "cam_right": None, "cam_head": None
        }

        # 录制状态
        self._collecting = False
        self._running = True
        self._prev_state: Optional[np.ndarray] = None

        # Episode 缓存
        self._frames_states = []
        self._frames_actions = []
        self._frames_images = []
        # 自动找下一个可用 episode 编号, 防止重启后覆盖
        import glob as _g
        existing = _g.glob(os.path.join(self.save_dir, "ep_*_left.npz"))
        max_id = 0
        for f in existing:
            try:
                n = int(os.path.basename(f).split("_")[1])
                if n > max_id: max_id = n
            except: pass
        self._episode_id = max_id
        print(f"[Recorder] 已有 {max_id} 条, 从 ep_{max_id+1:03d} 开始")
        self._total_frames = 0

        # 图像帧间隔 (相机 15fps, 关节 50fps, 每 3 帧采一次 ≈ 16Hz)
        self._img_counter = 0
        self._img_step = 3

        # 打印控制
        self._last_print = 0.0
        self._frame_in_episode = 0

    # ==================== ROS 订阅 ====================

    def _on_left_joint(self, msg: JointState):
        with self._joint_lock:
            self._left_positions = list(msg.position)

    def _on_right_joint(self, msg: JointState):
        with self._joint_lock:
            self._right_positions = list(msg.position)

    def _on_left_hand(self, msg):
        with self._joint_lock:
            self._left_hand = list(msg.hand_pos)

    def _on_right_hand(self, msg):
        with self._joint_lock:
            self._right_hand = list(msg.hand_pos)

    def _make_img_callback(self, cam_name: str):
        def callback(msg: Image):
            try:
                img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                with self._img_lock:
                    self._latest_images[cam_name] = img
            except Exception:
                pass
        return callback

    def _init_subscribers(self):
        rospy.init_node("lerobot_recorder", anonymous=True)
        rospy.Subscriber("/l_arm/joint_states", JointState,
                         self._on_left_joint, queue_size=5)
        rospy.Subscriber("/r_arm/joint_states", JointState,
                         self._on_right_joint, queue_size=5)
        rospy.Subscriber("/camera_d435_0/color/image_raw", Image,
                         self._make_img_callback("cam_left"), queue_size=1)
        rospy.Subscriber("/camera_d435_1/color/image_raw", Image,
                         self._make_img_callback("cam_right"), queue_size=1)
        rospy.Subscriber("/camera_d435_2/color/image_raw", Image,
                         self._make_img_callback("cam_head"), queue_size=1)
        if HAS_HAND:
            rospy.Subscriber("/l_arm/rm_driver/Udp_Hand_Status", Hand_Status,
                             self._on_left_hand, queue_size=5)
            rospy.Subscriber("/r_arm/rm_driver/Udp_Hand_Status", Hand_Status,
                             self._on_right_hand, queue_size=5)
        rospy.loginfo("订阅就绪: /l_arm/joint_states, /r_arm/joint_states, "
                      "3×D435 color" +
                      (", L+R Hand_Status" if HAS_HAND else " (无手数据)"))

    # ==================== 主循环 ====================

    def run(self):
        print("=" * 60)
        print("纯 ROS 数采录制器 (rospy 直连)")
        print("=" * 60)
        print("控制: Enter=开始/停止录制 | Ctrl+C=退出")
        print("=" * 60)
        print()

        self._init_subscribers()

        # 等第一帧数据
        rospy.sleep(2)

        # 录制循环在后台线程, 主线程处理键盘
        record_thread = threading.Thread(target=self._record_loop)
        record_thread.daemon = True
        record_thread.start()

        # 键盘控制
        try:
            while self._running:
                input()  # 等 Enter
                if self._collecting:
                    self._stop_episode()
                else:
                    self._start_episode()
        except (KeyboardInterrupt, EOFError):
            print("\n退出...")
        finally:
            if self._collecting:
                self._stop_episode()
            self._running = False
            record_thread.join(timeout=2.0)
            print(f"\n结束: {self._episode_id} episodes, "
                  f"{self._total_frames} 总帧")

    # ==================== 录制循环 ====================

    def _record_loop(self):
        rate = rospy.Rate(self.TARGET_FPS)

        while self._running:
            # 1. 读取关节 + 手
            state = self._get_full_state()  # arm_joints + hand_angles
            if state is None:
                rate.sleep()
                continue

            # 2. 读取图像
            images = self._get_images()

            # 3. 录制
            if self._collecting and self._prev_state is not None:
                obs = self._prev_state.astype(np.float32)
                act = np.append(state, 0.0).astype(np.float32)

                self._frames_states.append(obs)
                self._frames_actions.append(act)
                self._frames_images.append(images)
                self._frame_in_episode += 1

                now = time.time()
                if now - self._last_print > 0.5:
                    self._print_frame(obs, act)
                    self._last_print = now

            # 4. 更新上一帧
            self._prev_state = state.copy()

            rate.sleep()

    def _get_full_state(self) -> Optional[np.ndarray]:
        """读取完整状态: 左臂 + 右臂 + 左手 + 右手"""
        with self._joint_lock:
            if self._left_positions is None or self._right_positions is None:
                return None
            l = list(self._left_positions)
            r = list(self._right_positions)
            lh = list(self._left_hand) if self._left_hand else [0] * 6
            rh = list(self._right_hand) if self._right_hand else [0] * 6
            return np.array(l + r + lh + rh, dtype=np.float32)

    def _get_images(self) -> Dict[str, np.ndarray]:
        self._img_counter += 1
        if self._img_counter % self._img_step != 0:
            return {}
        with self._img_lock:
            return {k: v.copy() if v is not None else None
                    for k, v in self._latest_images.items()}

    # ==================== 打印 ====================

    def _print_frame(self, obs: np.ndarray, act: np.ndarray):
        # 自动识别维度: 前 N 维 = 臂, 后 12 维 = 手 (左右各6)
        if not hasattr(self, '_left_dof'):
            with self._joint_lock:
                l_len = len(self._left_positions) if self._left_positions else 6
                r_len = len(self._right_positions) if self._right_positions else 6
            self._left_dof = l_len
            self._right_dof = r_len
            self._arm_dim = l_len + r_len
            print(f"[Recorder] 左臂{l_len}D 右臂{r_len}D + 手12D")

        ad = self._arm_dim
        # 臂部分
        l_obs, r_obs = obs[:self._left_dof], obs[self._left_dof:ad]
        l_act, r_act = act[:self._left_dof], act[self._left_dof:ad]
        d_l, d_r = l_act - l_obs, r_act - r_obs
        # 手部分 (臂数据之后)
        lh_obs = obs[ad:ad+6] if len(obs) > ad else []
        rh_obs = obs[ad+6:ad+12] if len(obs) > ad+6 else []
        lh_act = act[ad:ad+6] if len(act) > ad else []
        rh_act = act[ad+6:ad+12] if len(act) > ad+6 else []

        fmt = lambda a: " ".join(f"{j*57.3:7.1f}" for j in a)
        fmt_d = lambda a: " ".join(f"{j*57.3:+7.1f}" for j in a)

        with self._img_lock:
            present = [k for k,v in self._latest_images.items() if v is not None]
        img_str = f"📷 {len(present)}/3" if present else "📷 等待"

        print(f"\n─── 帧 {self._frame_in_episode:5d} {img_str} ───")
        # 左: 臂 + 手
        lh_str = f" 手:{' '.join(f'{int(x):>5d}' for x in lh_obs)}" if HAS_HAND and len(lh_obs) else ""
        print(f"  左 obs° [{fmt(l_obs)}]{lh_str}")
        print(f"  左 act° [{fmt(l_act)}]")
        print(f"  左 Δ°  [{fmt_d(d_l)}]")
        # 右: 臂 + 手
        rh_str = f" 手:{' '.join(f'{int(x):>5d}' for x in rh_obs)}" if HAS_HAND and len(rh_obs) else ""
        print(f"  右 obs° [{fmt(r_obs)}]{rh_str}")
        print(f"  右 act° [{fmt(r_act)}]")
        print(f"  右 Δ°  [{fmt_d(d_r)}]")

    # ==================== Episode 管理 ====================

    def _start_episode(self):
        self._frames_states = []
        self._frames_actions = []
        self._frames_images = []
        self._frame_in_episode = 0
        self._prev_state = self._get_full_state()
        if self._prev_state is None:
            print("⚠ 无关节数据, 无法开始录制")
            return
        self._prev_state = self._prev_state.copy()
        self._collecting = True
        self._episode_id += 1
        ep = self._episode_id
        print(f"\n▶ Episode {ep} 开始 (操作遥操架, Enter 结束)")

    def _stop_episode(self):
        self._collecting = False
        ep = self._episode_id
        n = len(self._frames_states)

        if n == 0:
            print(f"◼ Episode {ep}: 0 帧")
            return

        self._total_frames += n

        # 保存关节数据: 分左右两条手臂
        states = np.array(self._frames_states, dtype=np.float32)   # (N, armL+R+handL+R)
        actions = np.array(self._frames_actions, dtype=np.float32) # (N, armL+R+handL+R+1)
        os.makedirs(self.save_dir, exist_ok=True)

        # 左: arm[0:6] + hand[12:18]
        left_states = np.hstack([states[:, 0:6], states[:, 12:18]])
        left_actions = np.hstack([actions[:, 0:6], actions[:, 12:18]])
        np.savez_compressed(f"{self.save_dir}/ep_{ep:03d}_left.npz",
                            states=left_states, actions=left_actions, fps=self.TARGET_FPS)

        # 右: arm[6:12] + hand[18:24]
        right_states = np.hstack([states[:, 6:12], states[:, 18:24]])
        right_actions = np.hstack([actions[:, 6:12], actions[:, 18:24]])
        np.savez_compressed(f"{self.save_dir}/ep_{ep:03d}_right.npz",
                            states=right_states, actions=right_actions, fps=self.TARGET_FPS)

        print(f"   数据: ep_{ep:03d}_left.npz ({left_states.shape[1]}D) "
              f"ep_{ep:03d}_right.npz ({right_states.shape[1]}D)")

        # 保存图像
        n_imgs = sum(1 for f in self._frames_images if f)
        if n_imgs > 0:
            img_root = f"{self.save_dir}/ep_{ep:03d}_images"
            saved = 0
            for i, img_dict in enumerate(self._frames_images):
                for name in ["cam_left", "cam_right", "cam_head"]:
                    img = img_dict.get(name)
                    if img is not None and img.size > 0:
                        d = f"{img_root}/{name}"
                        os.makedirs(d, exist_ok=True)
                        cv2.imwrite(f"{d}/frame_{i:06d}.jpg", img,
                                    [cv2.IMWRITE_JPEG_QUALITY, 85])
                        saved += 1
            print(f"  图像: {saved} 张")

        dur = n / self.TARGET_FPS
        print(f"◼ Episode {ep}: {n}帧 {dur:.1f}s @~{self.TARGET_FPS}Hz")
        if n > 1:
            # 只看臂关节的帧间位移 (手是 raw 值不参与)
            arm_dim = len(self._left_positions) + len(self._right_positions) \
                      if self._left_positions and self._right_positions else 12
            d = np.abs(np.diff(states[:, :arm_dim], axis=0)).max()
            print(f"  臂帧间最大位移: {np.rad2deg(d):.2f}°")


if __name__ == "__main__":
    recorder = RosRecorder()
    recorder.run()
