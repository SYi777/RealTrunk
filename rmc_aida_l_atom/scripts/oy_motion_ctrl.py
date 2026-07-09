import socket
import time
import ini_parameter
import os
import threading
from queue import Queue

# 检查是否停止控制
def stop_command_is(data, stop_btns: list):
    for btn in stop_btns:
        if data[2 + btn * 2] == 1:
            return True
    return False

class ArmController:
    def __init__(self, arm_socket, open_cmd, close_cmd, close_step1_cmd, open_io_num, close_io_num, arm_side):
        self.arm_socket = arm_socket
        self.open_cmd = open_cmd
        self.close_cmd = close_cmd
        self.close_step1_cmd = close_step1_cmd
        self.open_io_num = open_io_num
        self.close_io_num = close_io_num
        self.arm_side = arm_side  # 添加手臂侧别标识
        self.last_open_state = 0
        self.last_close_state = 0
        self.command_queue = Queue()
        
    def update_state(self, data, stop_btns):
        current_open = data[2 + self.open_io_num * 2]
        current_close = data[2 + self.close_io_num * 2]
        
        # 检测开夹爪按钮按下释放
        if self.last_open_state == 0 and current_open == 1:
            self.last_open_state = 1
        elif self.last_open_state == 1 and current_open == 0:
            if not stop_command_is(data, stop_btns):
                self.command_queue.put("open")
            self.last_open_state = 0
            
        # 检测关夹爪按钮按下释放
        if self.last_close_state == 0 and current_close == 1:
            self.last_close_state = 1
        elif self.last_close_state == 1 and current_close == 0:
            if not stop_command_is(data, stop_btns):
                self.command_queue.put("close_step1")
                self.command_queue.put("close")
            self.last_close_state = 0
    
    def process_commands(self):
        while True:
            cmd = self.command_queue.get()
            if cmd == "open":
                print(f"[{self.arm_side}手] 夹爪张开 (IO:{self.open_io_num})")
                self.arm_socket.send(self.open_cmd.encode('utf-8'))
            elif cmd == "close_step1":
                print(f"[{self.arm_side}手] 夹爪第一步闭合 (IO:{self.close_io_num})")
                self.arm_socket.send(self.close_step1_cmd.encode('utf-8'))
                time.sleep(0.5)
            elif cmd == "close":
                print(f"[{self.arm_side}手] 夹爪第二步闭合 (IO:{self.close_io_num})")
                self.arm_socket.send(self.close_cmd.encode('utf-8'))
            time.sleep(0.1)

def setup_socket(ip, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    address = (ip, port)
    sock.connect(address)
    sock.send(colse_modbus_mode.encode('utf-8'))
    sock.send(colse_rm_plus_mode.encode('utf-8'))
    return sock

script_dir = os.path.dirname(os.path.abspath(__file__))
config = ini_parameter.RobotConfig(f'{script_dir}/config.ini')

# 初始化modbus协议
colse_modbus_mode = f'{{"command":"close_modbus_mode","port":1}}\r\n'
colse_rm_plus_mode = f'{{"command":"set_rm_plus_mode","mode":0}}\r\n'
aoyi_open = f'{{"command":"hand_follow_pos","hand_pos":[0,0,0,0,0,0]}}\r\n'
aoyi_close_step1 = f'{{"command":"hand_follow_pos","hand_pos":[0,0,0,0,0,65535]}}\r\n'
aoyi_close = f'{{"command":"hand_follow_pos","hand_pos":[0,65535,65535,65535,65535,65535]}}\r\n'

if __name__ == "__main__":
    # 按钮配置
    lhandOpenIoNum = 3  # 左夹爪张开
    lhandCloseIoNum = 4  # 左夹爪闭合
    rhandOpenIoNum = 8  # 右夹爪张开
    rhandCloseIoNum = 9  # 右夹爪闭合
    stop_btns = [5, 10]  # 停止按钮

    # 初始化机械臂连接
    larm_socket = setup_socket(config.Arms.left_ip, config.Arms.port)
    rarm_socket = setup_socket(config.Arms.right_ip, config.Arms.port)

    # 创建控制器实例，添加手臂侧别参数
    larm_controller = ArmController(larm_socket, aoyi_open, aoyi_close, aoyi_close_step1, 
                                  lhandOpenIoNum, lhandCloseIoNum, "左")
    rarm_controller = ArmController(rarm_socket, aoyi_open, aoyi_close, aoyi_close_step1, 
                                  rhandOpenIoNum, rhandCloseIoNum, "右")

    # 启动处理线程
    threading.Thread(target=larm_controller.process_commands, daemon=True).start()
    threading.Thread(target=rarm_controller.process_commands, daemon=True).start()

    # 创建UDP接收套接字
    recv_udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    recv_udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    local_addr = (config.Zhi_Xing.listen_ip, config.Zhi_Xing.listen_port)
    recv_udp_socket.bind(local_addr)

    print("等待接收数据...")
    time.sleep(1)

    try:
        while True:
            data, addr = recv_udp_socket.recvfrom(1024)
            larm_controller.update_state(data, stop_btns)
            rarm_controller.update_state(data, stop_btns)
    except KeyboardInterrupt:
        print("程序终止")
    finally:
        larm_socket.close()
        rarm_socket.close()
        recv_udp_socket.close()
