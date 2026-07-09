#!/usr/bin/env python3
"""
读取遥操架主臂实时数据 + IO 按键状态并打印。

数据来源:
- 手臂关节: SlaveArmController UDP JSON 广播
  - 左臂: UDP 127.0.0.1:19997  tag="left"
  - 右臂: UDP 127.0.0.1:19998  tag="right"
- IO 按键: IoStateListener UDP 原始 Modbus RTU 帧
  - UDP 127.0.0.1:9999

运行方式:
    python read_master_arm.py              # 同时显示双臂 + IO按键
    python read_master_arm.py --left       # 只显示左臂 + IO按键
    python read_master_arm.py --right      # 只显示右臂 + IO按键
    python read_master_arm.py --no-io      # 不显示IO按键
    python read_master_arm.py --io-raw     # 每收到IO帧打印原始hex (不刷新)
    python read_master_arm.py --raw-once   # 单次抓取原始数据
"""

import socket
import json
import argparse
import threading
import time
import os
import sys
import struct


# ==================== 配置 ====================

LEFT_UDP_IP   = "127.0.0.1"
LEFT_UDP_PORT = 19997

RIGHT_UDP_IP   = "127.0.0.1"
RIGHT_UDP_PORT = 19998

IO_UDP_IP   = "127.0.0.1"
IO_UDP_PORT = 9999

JOINT_NAMES = ["J1(底座旋转)", "J2(大臂)", "J3(小臂)", "J4(腕部)", "J5(腕部2)", "J6(末端旋转)"]

IO_BUTTONS = {
    1:  ("左手● 中键(AGV左转)",  "左手十字键"),   # 待用户确认hex
    2:  ("左手▼ AGV后退",        "左手十字键"),   # hex确认
    3:  ("左手▲ AGV前进",        "左手十字键"),   # hex确认
    4:  ("左手► AGV右转",        "左手十字键"),   # hex确认
    5:  ("遥操键1",              "遥操"),
    6:  ("右手 下降",            "右手十字键"),
    7:  ("右手 上升",            "右手十字键"),
    8:  ("待确认",              "待确认"),
    9:  ("右中键",              "右手十字键"),
    10: ("遥操键2",              "遥操"),
}


# ==================== 颜色 ====================

class Colors:
    LEFT   = "\033[94m"
    RIGHT  = "\033[91m"
    RESET  = "\033[0m"
    BOLD   = "\033[1m"
    GREEN  = "\033[92m"
    YELLOW = "\033[93m"
    DIM    = "\033[2m"
    RED    = "\033[91m"

if os.name == "nt":
    for attr in dir(Colors):
        if not attr.startswith("__"):
            setattr(Colors, attr, "")


# ==================== 数据接收 ====================

class MasterArmReader:
    """UDP 监听器, 接收单臂 JSON 数据"""

    def __init__(self, udp_ip, udp_port, tag):
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.tag = tag
        self.latest_data = None
        self.lock = threading.Lock()
        self.running = False
        self.thread = None

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False

    def _recv_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(1.0)
        try:
            sock.bind((self.udp_ip, self.udp_port))
        except OSError as e:
            return

        while self.running:
            try:
                data, addr = sock.recvfrom(65535)
                if data:
                    try:
                        parsed = json.loads(data.decode("utf-8"))
                        if parsed.get("tag") == self.tag:
                            with self.lock:
                                self.latest_data = parsed
                    except json.JSONDecodeError:
                        pass
            except socket.timeout:
                continue
            except Exception:
                pass
        sock.close()

    def get_data(self):
        with self.lock:
            return self.latest_data


class IoButtonReader:
    """UDP 监听器, 接收 IoStateListener 发出的 Modbus RTU 原始帧"""

    def __init__(self, udp_ip, udp_port, print_raw=False):
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.io_states = {}
        self.lock = threading.Lock()
        self.running = False
        self.thread = None
        self.frame_count = 0
        self.error_count = 0
        self.print_raw = print_raw
        self.last_raw_bytes = None

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._recv_loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False

    def _validate_crc16(self, data):
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def _recv_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(1.0)
        try:
            sock.bind((self.udp_ip, self.udp_port))
        except OSError as e:
            return

        while self.running:
            try:
                data, addr = sock.recvfrom(65535)
                if data and len(data) >= 7:
                    self._parse_modbus_frame(data)
            except socket.timeout:
                continue
            except Exception:
                pass
        sock.close()

    def _parse_modbus_frame(self, data):
        if len(data) < 7:
            return

        if data[0] != 0x01 or data[1] != 0x04:
            self.error_count += 1
            return

        length_field = data[2]
        if length_field != 0x20:
            self.error_count += 1
            return

        expected_len = 3 + length_field + 2
        if len(data) < expected_len:
            self.error_count += 1
            return

        frame_for_crc = data[:expected_len - 2]
        crc_received = (data[expected_len - 1] << 8) | data[expected_len - 2]
        crc_calc = self._validate_crc16(frame_for_crc)
        if crc_calc != crc_received:
            self.error_count += 1
            return

        self.frame_count += 1

        raw_bytes = data[:expected_len]
        with self.lock:
            self.last_raw_bytes = raw_bytes

        if self.print_raw:
            hex_str = " ".join(f"{b:02x}" for b in raw_bytes)
            sys.stdout.write(f"\n[IO #{self.frame_count}] {len(raw_bytes)}B  {hex_str}\n")
            sys.stdout.flush()

        # 32字节IO数据区, C++代码访问: full_frame[2 + io_num * 2]
        # full_frame[3] 是数据区的起始, 所以在数据区内的偏移 = (2 + io_num*2) - 3 = io_num*2 - 1
        io_data = data[3: 3 + length_field]
        states = {}
        for io_num in IO_BUTTONS:
            offset = io_num * 2 - 1    # 修正: 之前多算了 2 字节
            if 0 <= offset < len(io_data):
                states[io_num] = io_data[offset]

        with self.lock:
            self.io_states = states

    def get_states(self):
        with self.lock:
            return dict(self.io_states)

    def get_stats(self):
        return self.frame_count, self.error_count


# ==================== 显示 ====================

def clear_screen():
    os.system("cls" if os.name == "nt" else "clear")


def print_bar(label, value, min_val, max_val, width=30, color=""):
    if max_val == min_val:
        ratio = 0.5
    else:
        ratio = (value - min_val) / (max_val - min_val)
    ratio = max(0.0, min(1.0, ratio))
    filled = int(ratio * width)
    bar = "█" * filled + "░" * (width - filled)
    print(f"  {label:20s} {color}{value:+8.3f}°{Colors.RESET}  │{color}{bar}{Colors.RESET}│")


def display_io_panel(io_reader):
    states = io_reader.get_states()
    frames, errors = io_reader.get_stats()

    groups = {
        "左手十字键": [1, 2, 3, 4],   # ●中键 ▼后退 ▲前进 ►右转
        "右手十字键": [6, 7, 9],      # 下降 上升 右中键
        "遥操键":     [5, 10],
        "待确认":     [8],
    }

    print(f"{Colors.BOLD}  ┌── IO 按键状态 ───────────────────────────────────────────────┐{Colors.RESET}")

    for group_name, io_list in groups.items():
        parts = []
        for io_num in io_list:
            name = IO_BUTTONS.get(io_num, (f"IO{io_num}", ""))[0]
            val = states.get(io_num, 0)
            if val:
                parts.append(f"{Colors.GREEN}{Colors.BOLD}{name}=按下{Colors.RESET}")
            else:
                parts.append(f"{Colors.DIM}{name}=松开{Colors.RESET}")
        line = "  │ " + "  │  ".join(parts)
        print(line)

    teleop_key1 = states.get(5, 0)
    teleop_key2 = states.get(10, 0)
    if teleop_key1 and teleop_key2:
        hint = f"{Colors.YELLOW}{Colors.BOLD}  同时按下 → 松开后切换遥操作{Colors.RESET}"
    elif not teleop_key1 and not teleop_key2:
        hint = f"{Colors.DIM}  等待启停按键 (IO5+IO10)...{Colors.RESET}"
    else:
        hint = f"{Colors.DIM}  需同时按遥操键1+2{Colors.RESET}"
    print(f"  │ {hint}")

    status_color = Colors.GREEN if frames > 0 else Colors.RED
    print(f"  │ IO帧:{status_color}{frames}{Colors.RESET}  "
          f"CRC错:{Colors.RED if errors > 0 else Colors.DIM}{errors}{Colors.RESET}")
    print(f"{Colors.BOLD}  └──────────────────────────────────────────────────────────────┘{Colors.RESET}")


def display_data(left_reader, right_reader, io_reader, show_left, show_right, show_io):
    clear_screen()
    now = time.strftime("%Y-%m-%d %H:%M:%S")

    print(f"{Colors.BOLD}{'='*70}{Colors.RESET}")
    print(f"{Colors.BOLD}  遥操架主臂实时数据  ─  {now}{Colors.RESET}")
    print(f"{Colors.BOLD}{'='*70}{Colors.RESET}")
    print()

    if show_left:
        data = left_reader.get_data() if left_reader else None
        if data:
            joints = data.get("joint", [])
            gripper = data.get("gripper", 0)
            print(f"{Colors.LEFT}{Colors.BOLD}  ◀  左臂 (left)  ─  {len(joints)} 关节{Colors.RESET}")
            print(f"  {'─'*40}")
            for i, v in enumerate(joints):
                name = JOINT_NAMES[i] if i < len(JOINT_NAMES) else f"Joint{i}"
                print_bar(name, v, -180.0, 180.0, color=Colors.LEFT)
            print(f"  {'─'*40}")
            print(f"  {'夹爪gripper':20s} {Colors.LEFT}{Colors.BOLD}{gripper:>8d}{Colors.RESET}")
        else:
            print(f"{Colors.LEFT}  ◀  左臂 (left)  ─  等待数据...{Colors.RESET}")
        print()

    if show_right:
        data = right_reader.get_data() if right_reader else None
        if data:
            joints = data.get("joint", [])
            gripper = data.get("gripper", 0)
            print(f"{Colors.RIGHT}{Colors.BOLD}  ▸  右臂 (right)  ─  {len(joints)} 关节{Colors.RESET}")
            print(f"  {'─'*40}")
            for i, v in enumerate(joints):
                name = JOINT_NAMES[i] if i < len(JOINT_NAMES) else f"Joint{i}"
                print_bar(name, v, -180.0, 180.0, color=Colors.RIGHT)
            print(f"  {'─'*40}")
            print(f"  {'夹爪gripper':20s} {Colors.RIGHT}{Colors.BOLD}{gripper:>8d}{Colors.RESET}")
        else:
            print(f"{Colors.RIGHT}  ▸  右臂 (right)  ─  等待数据...{Colors.RESET}")
        print()

    if show_io and io_reader:
        display_io_panel(io_reader)
        print()

    print(f"{Colors.BOLD}{'='*70}{Colors.RESET}")
    print(f"  Ctrl+C 退出")
    print(f"  UDP: {LEFT_UDP_IP}:{LEFT_UDP_PORT}/{RIGHT_UDP_IP}:{RIGHT_UDP_PORT} (JSON)  |  IO: {IO_UDP_IP}:{IO_UDP_PORT} (Modbus)")
    if show_io:
        print(f"  提示: IO5+IO10 同时按然后松开来切换遥操作启停")


def print_raw_once(data, side):
    color = Colors.LEFT if side == "left" else Colors.RIGHT
    print(f"\n{color}{Colors.BOLD}═══ {side} arm raw data ═══{Colors.RESET}")
    print(json.dumps(data, indent=2, ensure_ascii=False))
    joints = data.get("joint", [])
    gripper = data.get("gripper", 0)
    print(f"\n{color}关节解析:{Colors.RESET}")
    for i, v in enumerate(joints):
        name = JOINT_NAMES[i] if i < len(JOINT_NAMES) else f"Joint{i}"
        print(f"  {name:20s} = {v:+10.6f} °")
    print(f"  {'夹爪gripper':20s} = {gripper} (raw)")
    print()


# ==================== 主入口 ====================

def main():
    parser = argparse.ArgumentParser(description="读取遥操架主臂实时数据 + IO 按键状态")
    parser.add_argument("--left", action="store_true", help="只显示左臂")
    parser.add_argument("--right", action="store_true", help="只显示右臂")
    parser.add_argument("--no-io", action="store_true", help="不显示 IO 按键")
    parser.add_argument("--io-raw", action="store_true", help="原始 hex 模式: 每收到 IO 帧就打印")
    parser.add_argument("--raw-once", action="store_true", help="单次抓取并打印原始 JSON")
    parser.add_argument("--io-ip", default=None, help=f"IO UDP IP (默认 {IO_UDP_IP})")
    parser.add_argument("--io-port", type=int, default=None, help=f"IO UDP 端口 (默认 {IO_UDP_PORT})")

    args = parser.parse_args()

    if not args.left and not args.right:
        args.left = True
        args.right = True

    io_ip = args.io_ip if args.io_ip is not None else IO_UDP_IP
    io_port = args.io_port if args.io_port is not None else IO_UDP_PORT
    show_io = not args.no_io

    if args.raw_once:
        for side_tag, ip, port in [
            ("left", LEFT_UDP_IP, LEFT_UDP_PORT),
            ("right", RIGHT_UDP_IP, RIGHT_UDP_PORT),
        ]:
            reader = MasterArmReader(ip, port, side_tag)
            reader.start()
            time.sleep(2)
            data = reader.get_data()
            if data:
                print_raw_once(data, side_tag)
            else:
                print(f"\n  [WARN] {side_tag} arm: 2秒内未收到数据")
            reader.stop()
        return

    left_reader = None
    right_reader = None
    io_reader = None

    if args.left:
        left_reader = MasterArmReader(LEFT_UDP_IP, LEFT_UDP_PORT, "left")
        left_reader.start()
        print(f"[OK] 左臂: {LEFT_UDP_IP}:{LEFT_UDP_PORT}")

    if args.right:
        right_reader = MasterArmReader(RIGHT_UDP_IP, RIGHT_UDP_PORT, "right")
        right_reader.start()
        print(f"[OK] 右臂: {RIGHT_UDP_IP}:{RIGHT_UDP_PORT}")

    if show_io:
        io_reader = IoButtonReader(io_ip, io_port, print_raw=args.io_raw)
        io_reader.start()
        print(f"[OK] IO按键: {io_ip}:{io_port}")

    time.sleep(1)

    # --io-raw 模式: 不清屏, IO 帧即时打印, 其他数据只偶尔输出一行
    if args.io_raw:
        print(f"{'='*60}")
        print(f"  IO raw 模式 — IO 帧即时打印, 按 Ctrl+C 退出")
        print(f"{'='*60}")
        try:
            last_print = 0
            while True:
                now = time.time()
                if now - last_print > 2.0:
                    left_str = ""
                    right_str = ""
                    if left_reader:
                        d = left_reader.get_data()
                        if d:
                            j = d.get("joint", [])
                            g = d.get("gripper", 0)
                            left_str = f"左: joint=[{j[0]:+.2f}...{j[-1]:+.2f}] gripper={g}"
                    if right_reader:
                        d = right_reader.get_data()
                        if d:
                            j = d.get("joint", [])
                            g = d.get("gripper", 0)
                            right_str = f"  右: joint=[{j[0]:+.2f}...{j[-1]:+.2f}] gripper={g}"
                    if left_str or right_str:
                        print(f"[{time.strftime('%H:%M:%S')}] {left_str}{right_str}")
                    last_print = now
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            if left_reader: left_reader.stop()
            if right_reader: right_reader.stop()
            if io_reader: io_reader.stop()
        return

    # 正常模式: 刷新屏幕显示
    try:
        while True:
            display_data(left_reader, right_reader, io_reader,
                         args.left, args.right, show_io)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\n退出。")
    finally:
        if left_reader:   left_reader.stop()
        if right_reader:  right_reader.stop()
        if io_reader:     io_reader.stop()


if __name__ == "__main__":
    main()
