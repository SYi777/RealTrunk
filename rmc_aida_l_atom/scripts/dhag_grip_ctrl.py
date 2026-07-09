import socket
import time
import threading
import ini_parameter
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
# 读取配置文件
config = ini_parameter.RobotConfig(f'{script_dir}/config.ini')


# 夹爪夹取下标
LEFT_GRIP_OPEN_IONUM = 2
LEFT_GRIP_CLOSE_IONUM = 3
RIGHT_GRIP_OPEN_IONUM = 7
RIGHT_GRIP_CLOSE_IONUM = 8

# 中间按键按下时不进行控制
LEFT_CENTER_IONUM = 4
RIGHT_CENTER_IONUM = 9


# AG系列夹爪
class AG_Grip:

    def __init__(self, ip: str, port: int):
        # 创建连接机械臂的Socket套接字
        self.arm_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # 尝试连接
        arm_address = (ip, port)
        self.arm_socket.connect(arm_address)

    # 初始化modbus设备
    def init_device(self, modbus_id: int, baudrate: int, timeout: int, voltage_type: int):
        # 配置modbus设备
        self.arm_socket.send(f'{{"command":"set_modbus_mode","port":{modbus_id},"baudrate":{baudrate},"timeout":{timeout}}}\r\n'.encode('utf-8'))
        time.sleep(0.1)
        # 设置24v
        self.arm_socket.send(f'{{"command":"set_tool_voltage","voltage_type":{voltage_type}}}\r\n'.encode('utf-8'))
        time.sleep(0.1)
        # 初始化夹爪
        self.arm_socket.send(f'{{"command":"write_single_register","port":1,"address":256,"data":1,"device":1}}\r\n'.encode('utf-8'))
        time.sleep(0.1)

    # 设置夹爪力控大小（百分比）[20~100]
    def set_holding_force(self, force: int):
        self.arm_socket.send(f'{{"command":"write_single_register","port":1,"address":257,"data":{force},"device":1}}\r\n'.encode('utf-8'))
        time.sleep(0.1)

    # 设置位置（千分比）[0~1000]
    def set_gripper_position(self, position: int):
        self.arm_socket.send(f'{{"command":"write_single_register","port":1,"address":259,"data":{position},"device":1}}\r\n'.encode('utf-8'))


# UDP数据监听者
class Udp_Listener:

    def __init__(self, local_ip: str, local_port: int):
        # 配置套接字
        self._listener_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        # 配置端口允许复用
        self._listener_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        # 绑定本地端口
        local_addr = (local_ip, local_port)
        self._listener_socket.bind(local_addr)
        self._listener_socket.setblocking(False)  # 设置为非阻塞模式

        # 初始化数据
        self.recv_data = [0] * 10
        # 设置线程锁
        self.recv_lock = threading.Lock()
        self.recv_thread = threading.Thread(target=self._recv_from_udp, args=(1024,))
        self.running = True  # 控制接收线程的运行状态

    # 读取数据
    def _recv_from_udp(self, bufsize: int = 1024):
        while self.running:
            try:
                with self.recv_lock:
                    data, addr = self._listener_socket.recvfrom(bufsize)
                    self.recv_data = data
            except BlockingIOError:
                continue
            except Exception as e:
                print(f"Error receiving data: {e}")
                continue

    # 监听数据，并将其结果转化为：[左上、左下、左左、左右、左中、右上、右下、右左、右右、右中]
    def parse_datas(self):
        with self.recv_lock:
            if not self.recv_data or len(self.recv_data) < 21:
                return [0] * 10

            # 初始化数据
            button_list = [0] * 10

            # 检查数据头
            if self.recv_data[0] != 1 or self.recv_data[1] != 4:
                return [0] * 10

            button_byte = self.recv_data[3: 2 * 10 + 3]
            for index in range(10):
                if index * 2 + 1 >= len(button_byte):
                    button_list[index] = 0
                else:
                    button_list[index] = button_byte[index * 2 + 1]

        return button_list

    def stop(self):
        self.running = False


if __name__ == "__main__":
    # 左夹爪
    left_grip = AG_Grip(config.Arms.left_ip, config.Arms.port)
    left_grip.init_device(modbus_id=1, baudrate=config.Dhag_Gripper.baudrate, \
                          timeout=2, voltage_type=3)
    left_grip.set_holding_force(config.Dhag_Gripper.force_percent)
    
    # 右夹爪
    right_grip = AG_Grip(config.Arms.right_ip, config.Arms.port)
    right_grip.init_device(modbus_id=1, baudrate=config.Dhag_Gripper.baudrate, \
                           timeout=2, voltage_type=3)
    right_grip.set_holding_force(config.Dhag_Gripper.force_percent)

    # UDP监听者
    udp_listener = Udp_Listener(config.Dhag_Gripper.listen_ip, \
                                config.Dhag_Gripper.listen_port)
    udp_listener.recv_thread.start()

    try:
        interval = 0.1
        next_loop_time = time.time()

        while True:
            current_time = time.time()
            elapsed_time = current_time - next_loop_time
            if elapsed_time < 0:
                elapsed_time = 0
            next_loop_time += interval
            remaining_time = interval - elapsed_time

            # 接受数据
            buttons = udp_listener.parse_datas()
            
            # 屏蔽控制
            if buttons[RIGHT_CENTER_IONUM] or buttons[LEFT_CENTER_IONUM]:
                # 如果剩余时间大于0，则睡眠；否则直接继续
                if remaining_time > 0:
                    time.sleep(remaining_time)
                continue

            # 按钮控制
            if buttons[LEFT_GRIP_OPEN_IONUM]:
                left_grip.set_gripper_position(int(config.Dhag_Gripper.position_percent * 10))
                print("LEFT_GRIP_OPEN")
            elif buttons[LEFT_GRIP_CLOSE_IONUM]:
                left_grip.set_gripper_position(0)
                print("LEFT_GRIP_CLOSE")

            if buttons[RIGHT_GRIP_OPEN_IONUM]:
                right_grip.set_gripper_position(int(config.Dhag_Gripper.position_percent * 10))
                print("RIGHT_GRIP_OPEN")
            elif buttons[RIGHT_GRIP_CLOSE_IONUM]:
                right_grip.set_gripper_position(0)
                print("RIGHT_GRIP_CLOSE")

            # 如果剩余时间大于0，则睡眠；否则直接继续
            if remaining_time > 0:
                time.sleep(remaining_time)

    except KeyboardInterrupt:
        udp_listener.stop()
        print("Exiting...")