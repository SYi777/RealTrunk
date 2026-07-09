#!/usr/bin/env python3
"""
纯 ROS 调试面板 (rospy 直连)
=============================
打印双臂关节 + 实时显示三路 D435 图像。

用法:
  conda deactivate
  source /opt/ros/noetic/setup.bash
  python3 debug_monitor.py            # 只打印关节
  python3 debug_monitor.py --images   # 关节 + imshow 图像
"""

import rospy
import numpy as np
import cv2
import sys
import time
import threading
import argparse

from sensor_msgs.msg import JointState, Image
from cv_bridge import CvBridge
try:
    from embodied_arm_msgs.msg import Hand_Status
    HAS_HAND = True
except ImportError:
    HAS_HAND = False


class DebugMonitor:
    def __init__(self, show_images=False):
        self.bridge = CvBridge()
        self.show_images = show_images

        # 关节
        self._lock = threading.Lock()
        self._left = None
        self._right = None
        self._start_time = time.time()
        self._l_count = 0
        self._r_count = 0

        # 手
        self._l_hand = None
        self._r_hand = None

        # 图像
        self._img_lock = threading.Lock()
        self._images = {"cam_left": None, "cam_right": None, "cam_head": None}
        self._img_count = {"cam_left": 0, "cam_right": 0, "cam_head": 0}

    def start(self):
        rospy.init_node("debug_monitor", anonymous=True)

        rospy.Subscriber("/l_arm/joint_states", JointState,
                         self._on_left, queue_size=5)
        rospy.Subscriber("/r_arm/joint_states", JointState,
                         self._on_right, queue_size=5)

        if HAS_HAND:
            rospy.Subscriber("/l_arm/rm_driver/Udp_Hand_Status", Hand_Status,
                             lambda m: self._cb_hand(m, "l"), queue_size=5)
            rospy.Subscriber("/r_arm/rm_driver/Udp_Hand_Status", Hand_Status,
                             lambda m: self._cb_hand(m, "r"), queue_size=5)

        if self.show_images:
            rospy.Subscriber("/camera_d435_0/color/image_raw", Image,
                             self._make_img_cb("cam_left"), queue_size=1)
            rospy.Subscriber("/camera_d435_1/color/image_raw", Image,
                             self._make_img_cb("cam_right"), queue_size=1)
            rospy.Subscriber("/camera_d435_2/color/image_raw", Image,
                             self._make_img_cb("cam_head"), queue_size=1)
            cv2.namedWindow("D435 [L|R|H]", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("D435 [L|R|H]", 1920, 540)
            print("图像窗口已打开 (按 q 退出)")

        print("\n─── 双臂关节 ───")
        print("等待数据...")

    def _on_left(self, msg: JointState):
        with self._lock:
            self._left = list(msg.position)
            self._l_count += 1

    def _on_right(self, msg: JointState):
        with self._lock:
            self._right = list(msg.position)
            self._r_count += 1

    def _cb_hand(self, msg, side):
        with self._lock:
            if side == "l": self._l_hand = list(msg.hand_pos)
            else:           self._r_hand = list(msg.hand_pos)

    def _make_img_cb(self, name):
        def cb(msg: Image):
            try:
                img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                with self._img_lock:
                    self._images[name] = img
                    self._img_count[name] += 1
            except Exception:
                pass
        return cb

    def spin(self):
        rate = rospy.Rate(10)  # 10Hz 刷新
        t0 = time.time()

        while not rospy.is_shutdown():
            with self._lock:
                l, r = self._left, self._right
            elapsed = time.time() - t0
            l_hz = self._l_count / elapsed if elapsed > 0 else 0
            r_hz = self._r_count / elapsed if elapsed > 0 else 0

            # 打印关节 + 手
            if l and r:
                l_deg = [round(j * 57.3, 1) for j in l]
                r_deg = [round(j * 57.3, 1) for j in r]
                lh_str = f" 手:{' '.join(f'{int(x):>5d}' for x in self._l_hand)}" if self._l_hand else ""
                rh_str = f" 手:{' '.join(f'{int(x):>5d}' for x in self._r_hand)}" if self._r_hand else ""
                print(f"\r[{elapsed:5.1f}s]")
                print(f"  左: [{', '.join(f'{j:6.1f}' for j in l_deg)}]{lh_str}")
                print(f"  右: [{', '.join(f'{j:6.1f}' for j in r_deg)}]{rh_str}", end="")
                sys.stdout.flush()
            elif l or r:
                print(f"\r[{elapsed:5.1f}s] 等待另一臂数据...", end="")
            else:
                print(f"\r[{elapsed:5.1f}s] 等待关节数据...", end="")

            sys.stdout.flush()

            # 显示图像
            if self.show_images:
                self._show_images()

            rate.sleep()

    def _show_images(self):
        with self._img_lock:
            imgs = {k: v.copy() if v is not None else None
                    for k, v in self._images.items()}
        tiles = []
        for name in ["cam_left", "cam_head", "cam_right"]:
            img = imgs.get(name)
            if img is not None:
                small = cv2.resize(img, (640, 480))
                cv2.putText(small, f"{name} {self._img_count[name]}f", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                tiles.append(small)
            else:
                tiles.append(np.zeros((480, 640, 3), dtype=np.uint8))

        combined = np.hstack(tiles)
        cv2.imshow("D435 [L|R|H]", combined)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            rospy.signal_shutdown("user quit")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--images", action="store_true",
                        help="显示图像窗口 (imshow)")
    args = parser.parse_args()

    m = DebugMonitor(show_images=args.images)
    m.start()
    m.spin()
