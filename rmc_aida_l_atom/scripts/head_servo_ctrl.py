import socket
import serial
import threading
import time
import json
import ini_parameter
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
# 读取配置文件
config = ini_parameter.RobotConfig(f'{ script_dir }/config.ini')

# 全局变量
angle1 = 0
angle2 = 0
last_time = time.time()

def get_servo_angle(ser, lock, broadcast_socket):
    global angle1, angle2, last_time
    data_get_angle = (0x55, 0x55, 0x05, 0x15, 0x02, 0x01, 0x02)
    broadcast_addr = (config.Head_Servo.publisher_ip, config.Head_Servo.publisher_port)

    while True:
        with lock:
            ser.write(data_get_angle)
            rev_data = ser.read(11)
        if len(rev_data) != 11 or rev_data[0] != 0x55 or rev_data[1] != 0x55:
            continue

        with lock:
            angle1 = (rev_data[7] << 8) | rev_data[6]
            angle2 = (rev_data[10] << 8) | rev_data[9]

        # print("舵机1角度：", angle1)
        # print("舵机2角度：", angle2)

        current_time = time.time()
        time_interval = (current_time - last_time) * 1000
        last_time = current_time
        # print("时间间隔：", time_interval, "毫秒")

        # message = f"angle1:{angle1}, angle2:{angle2}"
        # 获取当前时间戳
        timestamp = time.time()
        message = {"angle1":angle1, "angle2":angle2, "timestamp":timestamp}
        json_message = json.dumps(message)

        broadcast_socket.sendto(json_message.encode('utf-8'), broadcast_addr)
        
        # time.sleep(0.05)  # 每50毫秒获取一次角度
        time.sleep(1.0 / (config.Head_Servo.frequency))

# 头部回零位
def init_head_pose(ser: serial.Serial, joints: list=[500, 500]):
    # 数据模板
    data_templet = [0x55, 0x55, 0x08, 0x03, 0x01, 0x32, 0x00, 0x00, 0x00, 0x00]
    # 循环复制发布
    for id in range(len(joints)):
        data_templet[7] = id
        data_templet[8] = joints[id] & 0xFF
        data_templet[9] = (joints[id] >> 8) & 0xFF
        # 发布
        ser.write(data_templet)
        print(f"Head joint-[{id}] inited! pose is: {joints[id]}")
        time.sleep(.5)



if __name__ == "__main__":
    headCtrIoNum = 5    # 联合控制头部的按键
    headUpIoNum = 6     # 抬头
    headDownIoNum = 7   # 低头
    headLeftIoNum = 8   # 头左转
    headRightIoNum = 9  # 头右转

    # 创建一个UDP套接字
    recv_udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    recv_udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 允许地址复用
    # 绑定本地地址和端口
    local_addr = (config.Head_Servo.listen_ip, config.Head_Servo.listen_port)
    recv_udp_socket.bind(local_addr)

    #创建一个UDP广播套接字
    broadcast_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    broadcast_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    ser = serial.Serial(config.Head_Servo.port, config.Head_Servo.baudrate, timeout=2)

    data_set_angle = [0x55, 0x55, 0x08, 0x03, 0x01, 0x32, 0x00, 0x00, 0x00, 0x00]

    # 初始化头部位置
    init_head_pose(ser=ser, joints=[500, 500])
    
    print("等待接收数据...")

    servo_ot = 0
    servo_step = 50

    lock = threading.Lock()
    angle_thread = threading.Thread(target=get_servo_angle, args=(ser, lock, broadcast_socket))
    angle_thread.start()

    time.sleep(1)
    
    while True:
        # 接收数据
        data, addr = recv_udp_socket.recvfrom(1024)  # 1024表示本次接收的最大字节数
        hex_data = ' '.join(f'{byte:02x}' for byte in data)
        headBtnPressedCount = data[2 + headUpIoNum * 2] + data[2 + headDownIoNum * 2] + data[2 + headLeftIoNum * 2] + data[2 + headRightIoNum * 2]
        # 主循环中的按键处理部分（添加舵机专属角度限制）
        if data[2 + headCtrIoNum * 2] == 1 and headBtnPressedCount == 1:
            servo_ot += 1
            if servo_ot > 3:
                servo_ot = 0
                target_angle = 0
                servo_id = 0  # 目标舵机ID
                min_angle = 0
                max_angle = 0

                # 判断控制对象并设置参数
                if data[2 + headUpIoNum * 2] == 1:  # 抬头（舵机1）
                    servo_id = 1
                    min_angle = 400  # 舵机1最小角度
                    max_angle = 600  # 舵机1最大角度
                    target_angle = angle1 + servo_step
                elif data[2 + headDownIoNum * 2] == 1:  # 低头（舵机1）
                    servo_id = 1
                    min_angle = 400
                    max_angle = 600
                    target_angle = angle1 - servo_step
                elif data[2 + headLeftIoNum * 2] == 1:  # 左转（舵机2）
                    servo_id = 2
                    min_angle = 200  # 舵机2最小角度
                    max_angle = 800  # 舵机2最大角度
                    target_angle = angle2 + servo_step
                else:  # 右转（舵机2）
                    servo_id = 2
                    min_angle = 200
                    max_angle = 800
                    target_angle = angle2 - servo_step

                # 动态限制角度到硬件范围（关键修改）
                target_angle = max(min(target_angle, max_angle), min_angle)

                # 生成控制指令（确保角度拆分正确）
                data_set_angle[7] = servo_id          # 舵机ID
                data_set_angle[8] = target_angle & 0xFF  # 低8位
                data_set_angle[9] = (target_angle >> 8) & 0xFF  # 高8位

                # 发送指令（添加串口状态检查）
                with lock:
                    try:
                        sent_bytes = ser.write(bytes(data_set_angle))
                        print(f"发送成功，字节数：{sent_bytes}，指令：{[f'0x{b:02x}' for b in data_set_angle]}")
                    except serial.SerialException as e:
                        print(f"串口发送失败：{e}")

