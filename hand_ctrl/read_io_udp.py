#!/usr/bin/env python3
"""IO 按键 + 夹爪值 (UDP :9999 + :19997/:19998)"""

import socket, json, threading, time

BTN = {1:"AGV前", 2:"AGV后", 3:"左开", 4:"左关",
       5:"停1", 6:"升", 7:"降", 8:"右开", 9:"右关", 10:"停2"}

gripper = {"左": 0, "右": 0}

def arm_listen(port, side):
    global gripper
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", port))
    while True:
        try:
            d = json.loads(s.recvfrom(4096)[0].decode())
            gripper[side] = d.get("gripper", 0)
        except: pass

def io_listen():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", 9999))
    while True:
        d, _ = s.recvfrom(128)
        if len(d) != 37: continue
        btns = [d[2+i*2] for i in range(1,13)]
        pressed = [BTN.get(i, f"BTN{i}") for i in range(1,13) if btns[i-1]]
        print(f"\r  btns={btns}  gripper: L{gripper['左']:>5d} R{gripper['右']:>5d}"
              + (f"  ▶ {pressed}" if pressed else " "*20), end="")

threading.Thread(target=lambda: arm_listen(19997, "左"), daemon=True).start()
threading.Thread(target=lambda: arm_listen(19998, "右"), daemon=True).start()
threading.Thread(target=io_listen, daemon=True).start()

print("IO 按键 + 夹爪值")
try:
    while True: time.sleep(1)
except KeyboardInterrupt:
    print("\n退出")
