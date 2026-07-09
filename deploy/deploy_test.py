#!/usr/bin/env python3
"""部署测试: 臂(ROS JointPos) + 手(TCP JSON, 完全参照 hand_teleop)"""

import rospy, sys, socket
from sensor_msgs.msg import JointState
from embodied_arm_msgs.msg import JointPos
try:
    from embodied_arm_msgs.msg import Hand_Status; HAS_HAND = True
except: HAS_HAND = False

LEFT_IP="169.254.128.18"; RIGHT_IP="169.254.128.19"; ARM_PORT=8080
CMD_MODBUS=b'{"command":"close_modbus_mode","port":1}\r\n'
CMD_RM_PLUS=b'{"command":"set_rm_plus_mode","mode":0}\r\n'


class HandSide:
    """参照 hand_teleop.HandSide: 维护全部6指状态, 每次send全量"""
    def __init__(self, ip, side):
        self.side = side
        self.pos = [0]*6  # 内部状态: 0=张 65535=闭
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((ip, ARM_PORT))
        self.sock.send(CMD_MODBUS); self.sock.send(CMD_RM_PLUS)
        self.send()  # 初始全张开
        print(f"[{side}手] TCP {ip}:{ARM_PORT} 全张开")

    def send(self):
        cmd = '{{"command":"hand_follow_pos","hand_pos":{}}}\r\n'.format(self.pos)
        self.sock.send(cmd.encode())

    def tweak(self, j, delta):
        """j: 1-6, delta: ±raw"""
        self.pos[j-1] = max(0, min(65535, self.pos[j-1] + delta))
        self.send()

    def all_set(self, val):
        self.pos = [max(0, min(65535, val))]*6
        self.send()


class Deploy:
    def __init__(self):
        self.l = [0.0]*6; self.r = [0.0]*6
        self.lh_disp = [0]*6; self.rh_disp = [0]*6

        rospy.init_node("dt", anonymous=True)
        rospy.Subscriber("/l_arm/joint_states", JointState, lambda m: self._cb(m,"l"))
        rospy.Subscriber("/r_arm/joint_states", JointState, lambda m: self._cb(m,"r"))
        if HAS_HAND:
            rospy.Subscriber("/l_arm/rm_driver/Udp_Hand_Status", Hand_Status,
                             lambda m: self._cb_h(m,"l"))
            rospy.Subscriber("/r_arm/rm_driver/Udp_Hand_Status", Hand_Status,
                             lambda m: self._cb_h(m,"r"))

        self.lp = rospy.Publisher("/l_arm/rm_driver/JointPos", JointPos, queue_size=1)
        self.rp = rospy.Publisher("/r_arm/rm_driver/JointPos", JointPos, queue_size=1)

        self.lh = HandSide(LEFT_IP, "左")
        self.rh = HandSide(RIGHT_IP, "右")
        rospy.sleep(0.5)

    def _cb(self, m, s): (self.l if s=="l" else self.r)[:] = list(m.position[:6])
    def _cb_h(self, m, s): (self.lh_disp if s=="l" else self.rh_disp)[:] = list(m.hand_angle[:6])

    def pub_arm(self, side, joints_rad):
        m = JointPos(); m.joint = list(joints_rad); m.expand = 0.0
        (self.lp if side=="l" else self.rp).publish(m)

    def show(self):
        la = [f"{j*57.3:+6.1f}" for j in self.l]
        ra = [f"{j*57.3:+6.1f}" for j in self.r]
        print(f"  左臂: [{', '.join(la)}]°")
        print(f"  右臂: [{', '.join(ra)}]°")
        print(f"  左手控: {self.lh.pos}  (0=张 65535=闭)")
        print(f"  右手控: {self.rh.pos}  (0=张 65535=闭)")

    def tweak_arm(self, side, j, delta_deg):
        cur = (self.l if side=="l" else self.r)[:]
        cur[j-1] += delta_deg/57.3
        cur[j-1] = max(-3.14, min(3.14, cur[j-1]))
        self.pub_arm(side, cur)


if __name__ == "__main__":
    d = Deploy()
    print("\n命令: l/r <关节> <±°> | lh/rh <手指1-6> <±raw> | lh/rh all <0-65535> | s | q")
    d.show()
    while not rospy.is_shutdown():
        try: inp = input("> ").strip().lower()
        except: break
        if inp == 'q': break
        if not inp: continue
        if inp == 's': d.show(); continue
        p = inp.split()
        if len(p) == 3:
            if p[0] in ('l','r') and 1<=int(p[1])<=6:
                d.tweak_arm(p[0], int(p[1]), float(p[2]))
                rospy.sleep(0.3)
                print(f"  >>> {p[0]}臂 J{p[1]} {'+'if float(p[2])>0 else ''}{p[2]}°")
            elif p[0] in ('lh','rh') and p[1]=='all':
                (d.lh if p[0]=='lh' else d.rh).all_set(int(p[2]))
                print(f"  >>> {'左'if p[0]=='lh'else'右'}手 all→{int(p[2])}")
            elif p[0] in ('lh','rh') and 1<=int(p[1])<=6:
                (d.lh if p[0]=='lh' else d.rh).tweak(int(p[1]), int(p[2]))
                print(f"  >>> {'左'if p[0]=='lh'else'右'}手 指{int(p[1])} {'+'if int(p[2])>0 else ''}{int(p[2])}")
            else: print("  ?")
        else: print("  ?  l/r <关节> <±°> | lh/rh <手指> <±raw> | lh/rh all <0-65535>")

    print("退出")
