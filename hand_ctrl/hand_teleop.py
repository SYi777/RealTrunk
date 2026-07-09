#!/usr/bin/env python3
"""
傲意手遥操控制
==============
  BTN 3/4 → 左手拇指旋转 +/-
  BTN 8/9 → 右手拇指旋转 +/-
  捏夹爪 → 四指跟 gripper 走
  gripper > 800 启动

用法: python3 hand_teleop.py
"""

import socket, json, threading, time

LEFT_IP   = "169.254.128.18"
RIGHT_IP  = "169.254.128.19"
ARM_PORT  = 8080

# 左手大拇指弯: 1张 2闭 | 右手大拇指弯: 6张 7闭
BTN_L_BEND_OPEN  = 1; BTN_L_BEND_CLOSE = 2
BTN_R_BEND_OPEN  = 6; BTN_R_BEND_CLOSE = 7
# 左手拇指旋: 3转多 4转少 | 右手拇指旋: 8转多 9转少
BTN_L_ROT_UP = 3;  BTN_L_ROT_DOWN = 4
BTN_R_ROT_UP = 8;  BTN_R_ROT_DOWN = 9

GRIP_THRESHOLD = 800
STEP = 5000

CMD_MODBUS  = b'{"command":"close_modbus_mode","port":1}\r\n'
CMD_RM_PLUS = b'{"command":"set_rm_plus_mode","mode":0}\r\n'


class HandSide:
    def __init__(self, ip, port, side):
        self.side = side
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((ip, port))
        self.sock.send(CMD_MODBUS); self.sock.send(CMD_RM_PLUS)
        self.bend = 0;  self.rot = 0
        self.grip_max = 880          # 固定上限
        self._started = False
        self.send([0,0,0,0,0,0])
        print(f"[{side}手] {ip}:{port} 就绪")

    def send(self, pos):
        cmd = '{{"command":"hand_follow_pos","hand_pos":[{},{},{},{},{},{}]}}\r\n'.format(*pos)
        self.sock.send(cmd.encode())

    def init_calibrate(self, get_gripper):
        """5秒内反复读夹爪, 取最大值作为上限"""
        print(f"[{self.side}手] 校准夹爪上限 (5秒)...")
        t0 = time.time()
        peak = 0
        while time.time() - t0 < 5:
            g = get_gripper()
            if g > peak:
                peak = g
            time.sleep(0.05)
        self.grip_max = max(peak, 100)  # 至少 100
        print(f"[{self.side}手] 上限 = {self.grip_max}")

    def update(self, gripper, bend_up, bend_down, rot_up, rot_down):
        # 初始化后等夹爪 > 80% max 才启动, 启动后不再检查
        if not self._started:
            if gripper >= self.grip_max * 0.8:
                self._started = True
                print(f"\n[{self.side}手] 夹爪>{self.grip_max*0.8:.0f}, 启动!")
            else:
                return

        mx = self.grip_max
        finger = int((mx - gripper) / mx * 65535) if mx > 0 else 0
        if bend_up:   self.bend = min(65535, self.bend + STEP)
        if bend_down: self.bend = max(0, self.bend - STEP)
        if rot_up:    self.rot  = min(65535, self.rot + STEP)
        if rot_down:  self.rot  = max(0, self.rot - STEP)

        self.send([self.bend, finger, finger, finger, finger, self.rot])

    def close(self): self.sock.close()


def main():
    lh = HandSide(LEFT_IP, ARM_PORT, "左")
    rh = HandSide(RIGHT_IP, ARM_PORT, "右")
    grippers = {"左": 0, "右": 0}

    def grip(port, side):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("127.0.0.1", port))
        while True:
            try:
                d = json.loads(s.recvfrom(4096)[0].decode())
                grippers[side] = d.get("gripper", 0)
            except: pass

    threading.Thread(target=lambda: grip(19997, "左"), daemon=True).start()
    threading.Thread(target=lambda: grip(19998, "右"), daemon=True).start()

    def io():
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("0.0.0.0", 9999))
        while True:
            d, _ = s.recvfrom(128)
            if len(d) != 37: continue
            lbu = d[2 + BTN_L_BEND_OPEN  * 2]; lbc = d[2 + BTN_L_BEND_CLOSE * 2]
            lru = d[2 + BTN_L_ROT_UP    * 2]; lrd = d[2 + BTN_L_ROT_DOWN  * 2]
            rbu = d[2 + BTN_R_BEND_OPEN  * 2]; rbc = d[2 + BTN_R_BEND_CLOSE * 2]
            rru = d[2 + BTN_R_ROT_UP    * 2]; rrd = d[2 + BTN_R_ROT_DOWN  * 2]

            lh.update(grippers["左"], lbu, lbc, lru, lrd)
            rh.update(grippers["右"], rbu, rbc, rru, rrd)

            lf = int((990 - grippers["左"]) / 990 * 65535)
            rf = int((990 - grippers["右"]) / 990 * 65535)
            print(f"\r  "
                  f"左: 夹爪{grippers['左']:>4d}  四指{lf:>5d}  弯{lh.bend:>5d}  旋{lh.rot:>5d}  "
                  f"│  右: 夹爪{grippers['右']:>4d}  四指{rf:>5d}  弯{rh.bend:>5d}  旋{rh.rot:>5d}  "
                  f"│  L:弯{lbu}{lbc}旋{lru}{lrd} R:弯{rbu}{rbc}旋{rru}{rrd}", end="")

    threading.Thread(target=io, daemon=True).start()

    print("BTN1/2=左拇弯 BTN3/4=左拇旋 BTN6/7=右拇弯 BTN8/9=右拇旋 | 上限880 | >80%启动\n")

    try:
        while True: time.sleep(1)
    except KeyboardInterrupt:
        print("\n退出"); lh.close(); rh.close()


if __name__ == "__main__":
    main()