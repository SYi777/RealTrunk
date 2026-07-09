#!/usr/bin/env python3
"""
时间戳对齐检查
==============
验证 arm 关节、手、D435 图像的实时性和时间戳。

用法:
  conda deactivate
  source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash
  python3 timestamp_check.py

数据来源:
  臂: /joint_states           ← UDP(200Hz) → arm_driver → ROS
  手: /Udp_Hand_Status        ← RS485 → 控制器 → UDP → arm_driver → ROS
  图: /camera_d435_0/color/image_raw  ← USB3 → ROS
"""

import rospy, time, sys
from sensor_msgs.msg import JointState, Image
from cv_bridge import CvBridge

try:
    from embodied_arm_msgs.msg import Hand_Status; HAS_HAND = True
except: HAS_HAND = False


class Stamp:
    def __init__(self, label):
        self.label = label
        self.last_seq = 0; self.last_stamp = 0.0
        self.last_arrive = 0.0; self.count = 0
        self.first_stamp = None; self.first_arrive = None

    def hit(self, stamp_sec, seq=0):
        now = time.time()
        gap = seq - self.last_seq if seq else -1
        self.last_seq = seq
        self.last_stamp = stamp_sec
        self.last_arrive = now
        if self.first_stamp is None:
            self.first_stamp = stamp_sec
            self.first_arrive = now
        self.count += 1

    def stats(self):
        if self.count < 2:
            return "等待数据..."
        elapsed_arrive = self.last_arrive - self.first_arrive
        elapsed_stamp = self.last_stamp - self.first_stamp
        hz = self.count / elapsed_arrive if elapsed_arrive > 0 else 0
        delay = self.last_arrive - self.last_stamp  # 传输延迟
        return (f"{hz:5.1f}Hz  stamp:{self.last_stamp:.3f}  "
                f"delay:{delay*1000:5.1f}ms  ({self.count}帧)")


class Checker:
    def __init__(self):
        self.bridge = CvBridge()
        self.arm = {"l": Stamp("左臂"), "r": Stamp("右臂")}
        self.hand = {"l": Stamp("左手"), "r": Stamp("右手")}
        self.cam = {"cam_left": Stamp("左相"), "cam_right": Stamp("右相"),
                     "cam_head": Stamp("头相")}
        self.lh_val = [0]*6; self.rh_val = [0]*6

    def start(self):
        rospy.init_node("stamp_check", anonymous=True)
        rospy.Subscriber("/l_arm/joint_states", JointState,
                         lambda m: self._on_arm(m, "l"))
        rospy.Subscriber("/r_arm/joint_states", JointState,
                         lambda m: self._on_arm(m, "r"))
        if HAS_HAND:
            rospy.Subscriber("/l_arm/rm_driver/Udp_Hand_Status", Hand_Status,
                             lambda m: self._on_hand(m, "l"))
            rospy.Subscriber("/r_arm/rm_driver/Udp_Hand_Status", Hand_Status,
                             lambda m: self._on_hand(m, "r"))
        rospy.Subscriber("/camera_d435_0/color/image_raw", Image,
                         lambda m: self._on_img(m, "cam_left"))
        rospy.Subscriber("/camera_d435_1/color/image_raw", Image,
                         lambda m: self._on_img(m, "cam_right"))
        rospy.Subscriber("/camera_d435_2/color/image_raw", Image,
                         lambda m: self._on_img(m, "cam_head"))

    def _on_arm(self, m, s):
        self.arm[s].hit(m.header.stamp.to_sec(), m.header.seq)

    def _on_hand(self, m, s):
        self.hand[s].hit(rospy.Time.now().to_sec())
        if s == "l": self.lh_val = list(m.hand_pos[:6])
        else:        self.rh_val = list(m.hand_pos[:6])

    def _on_img(self, m, s):
        self.cam[s].hit(m.header.stamp.to_sec(), m.header.seq)

    def spin(self):
        print("\n时间戳对齐检查 — 跑 30 秒，动遥操架 + 手 + 挥手")
        print(f"{'─'*65}")
        print(f"{'来源':6s} {'频率':>8s}  {'最新stamp':>12s}  {'延迟':>8s}  {'帧数'}")
        print(f"{'─'*65}")

        t0 = time.time()
        rate = rospy.Rate(2)
        while not rospy.is_shutdown() and time.time() - t0 < 30:
            for name, s in self.arm.items():
                print(f"  {name:4s}  {s.stats()}")
            for name, s in self.hand.items():
                val = self.lh_val if name == "l" else self.rh_val
                print(f"  {name:4s}  {s.stats()}  [{', '.join(f'{v:>5d}' for v in val)}]")
            for name, s in self.cam.items():
                print(f"  {name:4s}  {s.stats()}")
            print(f"{'─'*65}")
            rate.sleep()

        print("\n完成。检查 delay 是否 < 50ms, Hz 是否接近 50(臂)/15(相)")


if __name__ == "__main__":
    c = Checker(); c.start(); c.spin()
