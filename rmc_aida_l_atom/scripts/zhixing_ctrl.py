import socket
import time
import os
import threading
import select
import json
import queue
import numpy as np
import ini_parameter
from threading import Thread

def monitor_performance(func):
    def wrapper(*args, **kwargs):
        start_time = time.perf_counter()
        result = func(*args, **kwargs)
        elapsed = (time.perf_counter() - start_time) * 1000
        if elapsed > 5:
            pass  # 移除日志输出
        return result
    return wrapper


class HighFrequencyArmController:
    def __init__(self, arm_socket, side):
        self.socket = arm_socket
        self.side = side

        
        self._cmd_queue = queue.Queue(maxsize=1000)  # 限制队列深度防内存爆炸
        self._sender_thread = threading.Thread(
            target=self._send_worker, 
            daemon=True,
            name=f"TCP_Sender_{side}"
        )
        self._sender_thread.start()
        
        # gripper 10HZ
        self.gripper_frequency = 10  # 10Hz
        self.min_interval = 1.0 / self.gripper_frequency  # 20ms
        self.last_gripper_time = 0
        
        self.connected = True
        
        self.sent_count = 0
        self.last_stat_time = time.time()
        
        self.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
        
        self.joint_lock = threading.Lock()
        self.last_left_joint = None
        self.last_right_joint = None
        self.last_left_joint_prev = None
        self.last_right_joint_prev = None
        
        #self.lift_height = None
        
        self.error_pub_interval = 1.0
        self.stable_count_threshold = 3
        self.left_stable_count = 0
        self.right_stable_count = 0
        


    @monitor_performance
    def map_value_right(self, udp_value):
        return int(np.interp(np.clip(udp_value, 0, 830), [0, 830], [0, 1000]))
        
    @monitor_performance
    def map_value_left(self, udp_value):
        return int(np.interp(np.clip(udp_value, 0, 892), [0, 892], [0, 1000]))

    @monitor_performance
    def update_gripper(self, udp_value):
        current_time = time.time()
        # 频率限制检查（新增）
        if current_time - self.last_gripper_time < self.min_interval:
            return
        
        try:
            if not isinstance(udp_value, (int, float)):
                return
            if self.side == "左":
            	tcp_value = self.map_value_left(udp_value)
            else:
            	tcp_value = self.map_value_right(udp_value)          
            self._send_command(tcp_value)
            self.last_gripper_time = current_time  # 更新时间戳
            
        except Exception as e:
            print(f"[ERROR] Gripper update failed: {e}")
            self._reconnect()

    def _send_command(self, tcp_value):
            
        try:
            cmd = f'{{"command":"hand_follow_pos","hand_pos":[{tcp_value}]}}\r\n'.encode()
            self._cmd_queue.put_nowait(cmd)  # 非阻塞入队
        except queue.Full:
            print("WARNING: Command queue full, dropping packet")
                
        except Exception:
            self._reconnect()


    def _send_worker(self):
        """专用发送线程"""
        while True:
            try:
                cmd = self._cmd_queue.get(timeout=1)  # 阻塞获取
                self.socket.sendall(cmd)
            except queue.Empty:
                continue
            except Exception as e:
                print(f"Send error: {e}")
                self._reconnect()

    def _reconnect(self):
        retry_delay = 1
        while retry_delay < 30:
            try:
                self.socket.close()
                self.socket = self._create_socket()
                self.connected = True
                return
            except Exception:
                time.sleep(retry_delay)
                retry_delay *= 2
        self.connected = False

    def _create_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
        ip = config.Left_Arm.slave_ip if self.side == "左" else config.Right_Arm.slave_ip
        port = config.Left_Arm.slave_port if self.side == "左" else config.Right_Arm.slave_port
        sock.connect((ip, port))
        # sock.send(colse_modbus_mode.encode('utf-8'))
        rec=sock.recv(1024).decode('utf-8').strip()
        print('recv',rec)
        time.sleep(0.01)
        # sock.send(set_plus_mode.encode('utf-8'))
        return sock

    def update_joint_data(self, tag, joint_data):
        with self.joint_lock:
            if tag == "left":
                self.last_left_joint = joint_data.copy()
                if self.last_left_joint_prev is None:
                    self.last_left_joint_prev = joint_data.copy()
            elif tag == "right":
                self.last_right_joint = joint_data.copy()
                if self.last_right_joint_prev is None:
                    self.last_right_joint_prev = joint_data.copy()

    def send_lift_state_command(self):
        if self.side != "右" or not self.connected:
            return
            
        try:
            cmd = '{"command":"get_lift_state"}\r\n'
            self.socket.sendall(cmd.encode('utf-8'))
            print(f"[{self.side}] Sent lift state command:{cmd.strip()}")
        except Exception:
            self._reconnect()

    def receive_lift_response(self):
        if not self.connected or not self.socket:
            return None
            
        try:
            ready, _, _ = select.select([self.socket], [], [], 0.1)
            if ready:
                data = self.socket.recv(1024)
                if data:
                    return data.decode('utf-8').strip()
        except BlockingIOError:
            pass
        except Exception:
            self._reconnect()
        return None

 
def main():
    
    try:
        left_socket = socket.create_connection(
            (config.Left_Arm.slave_ip, config.Left_Arm.slave_port),
            timeout=2
        )

        right_socket = socket.create_connection(
            (config.Right_Arm.slave_ip, config.Right_Arm.slave_port),
            timeout=2
        )
    except Exception:
        return

    left_socket.send(set_plus_mode.encode('utf-8'))
    rec=left_socket.recv(1024).decode('utf-8')
    print('recv',rec)

    right_socket.send(set_plus_mode.encode('utf-8'))
    rec=right_socket.recv(1024).decode('utf-8')
    print('recv',rec)
     
    controllers = {
        "left": HighFrequencyArmController(left_socket, "左"),
        "right": HighFrequencyArmController(right_socket, "右")
    }
    
    udp_threads = [
        threading.Thread(
            target=UDPReceiver(config.Left_Arm.udp_port, controllers["left"]).run,
            daemon=True
        ),
        threading.Thread(
            target=UDPReceiver(config.Right_Arm.udp_port, controllers["right"]).run,
            daemon=True
        )
    ]
    for t in udp_threads:
        t.start()
    
    try:
        while True:
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass

class UDPReceiver:
    def __init__(self, port, controller):
        self.port = port
        self.controller = controller
        self.running = True
        self.sock = self._init_socket()

    def _init_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 131072)
        sock.bind(('0.0.0.0', self.port))
        sock.setblocking(False)
        return sock
    
    def run(self):
        last_print = time.time()
        
        while self.running:
            ready = select.select([self.sock], [], [], 0.01)
            if ready[0]:
                try:
                    data, addr = self.sock.recvfrom(2048)
                    json_data = json.loads(data.decode('utf-8'))
                    
                    gripper = json_data.get('gripper')

                    print(f"Received gripper data: {gripper}")
                    
                
                    self.controller.update_gripper(gripper)
    
                except BlockingIOError:
                    continue
                except json.JSONDecodeError:
                    pass
                except KeyError:
                    pass
                except Exception:
                    pass
            
     
    
    def stop(self):
        self.running = False

script_dir = os.path.dirname(os.path.abspath(__file__))
config = ini_parameter.RobotConfig(f'{script_dir}/../build/config.ini')
colse_modbus_mode = f'{{"command":"close_modbus_mode","port":1}}\r\n'
set_plus_mode = f'{{"command":"set_rm_plus_mode","mode":115200}}\r\n'
if __name__ == "__main__":
    main()
