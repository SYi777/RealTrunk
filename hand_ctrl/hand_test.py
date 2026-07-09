#!/usr/bin/env python3
"""
傲意手关节测试 — 逐个位置设值
用法: python3 hand_test.py
  输入格式:  <位置> <值>    如: 1 65535  (位置1设65535)
             x               退出
             当前状态         显示当前6个位置值
"""

import socket, sys

LEFT_IP   = "169.254.128.18"
RIGHT_IP  = "169.254.128.19"
ARM_PORT  = 8080

def connect(ip, port, label):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((ip, port))
    s.send(b'{"command":"close_modbus_mode","port":1}\r\n')
    s.send(b'{"command":"set_rm_plus_mode","mode":0}\r\n')
    print(f"[{label}] 已连接 {ip}:{port}")
    return s

def set_pos(sock, side, positions):
    """positions: list of 6 ints, 0=张开, 65535=闭合, -1=保持不动"""
    cmd = '{{"command":"hand_follow_pos","hand_pos":[{},{},{},{},{},{}]}}\r\n'.format(*positions)
    sock.send(cmd.encode())
    print(f"  [{side}] → {positions}")

# 连接双臂
l_sock = connect(LEFT_IP, ARM_PORT, "左")
r_sock = connect(RIGHT_IP, ARM_PORT, "右")

# 当前各位置的状态
state = {"左": [-1]*6, "右": [-1]*6}

print("""
╔══════════════════════════════════════════╗
║  傲意手关节测试                          ║
║  命令格式: <左右> <位置0-5> <值>          ║
║    l 1 65535   左手指1闭合               ║
║    r 2 0       右手指2张开               ║
║    l all 0     左手全张开                ║
║    s           显示当前状态              ║
║    x           退出                     ║
║  位置: 0-5, 值: 0=张 65535=闭 -1=不控   ║
╚══════════════════════════════════════════╝
""")

try:
    while True:
        try:
            inp = input("> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            break

        if inp == 'x':
            break
        elif inp == 's':
            print(f"  左: {state['左']}")
            print(f"  右: {state['右']}")
            continue
        elif not inp:
            continue

        parts = inp.split()

        if len(parts) >= 2 and parts[0] in ('l', 'r'):
            side = "左" if parts[0] == 'l' else "右"
            sock = l_sock if parts[0] == 'l' else r_sock

            if parts[1] == 'all' and len(parts) == 3:
                # l all 0 或 r all 65535
                v = int(parts[2])
                pos = [v]*6
                state[side] = pos[:]
                set_pos(sock, side, pos)
            elif len(parts) == 3:
                idx = int(parts[1])
                val = int(parts[2])
                if 0 <= idx <= 5:
                    pos = state[side][:]
                    pos[idx] = val
                    state[side] = pos
                    set_pos(sock, side, pos)
                else:
                    print("  位置范围: 0-5")
            else:
                print("  格式: l/r <位置0-5> <值>  或  l/r all <值>")
        else:
            print("  格式: l/r <位置0-5> <值>  (例: l 1 65535)")

except KeyboardInterrupt:
    pass
finally:
    l_sock.close()
    r_sock.close()
    print("\n退出")
