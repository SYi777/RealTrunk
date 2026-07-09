import socket
import serial
import threading
import time
import json

# 全局变量
angle1 = 0
angle2 = 0
last_time = time.time()

def get_servo_angle(ser, lock, broadcast_socket):
    global angle1, angle2, last_time
    data_get_angle = (0x55, 0x55, 0x05, 0x15, 0x02, 0x01, 0x02)
    broadcast_addr = ('<broadcast>', 9996)

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
        time.sleep(0.07)

if __name__ == "__main__":
    headCtrIoNum = 5    # 联合控制头部的按键
    headUpIoNum = 6     # 抬头
    headDownIoNum = 7   # 低头
    headLeftIoNum = 8   # 头左转
    headRightIoNum = 9  # 头右转

    # 创建一个UDP套接字
    recv_udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # 绑定本地地址和端口
    local_addr = ('', 9999)
    recv_udp_socket.bind(local_addr)

    #创建一个UDP广播套接字
    broadcast_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    broadcast_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=2)

    data_set_angle = [0x55, 0x55, 0x08, 0x03, 0x01, 0x32, 0x00, 0x00, 0x00, 0x00]
    
    print("等待接收数据...")

    servo_ot = 0
    servo_step = 15

    lock = threading.Lock()
    angle_thread = threading.Thread(target=get_servo_angle, args=(ser, lock, broadcast_socket))
    angle_thread.start()

    time.sleep(1)
    
    while True:
        # 接收数据
        data, addr = recv_udp_socket.recvfrom(1024)  # 1024表示本次接收的最大字节数
        hex_data = ' '.join(f'{byte:02x}' for byte in data)
        headBtnPressedCount = data[2 + headUpIoNum * 2] + data[2 + headDownIoNum * 2] + data[2 + headLeftIoNum * 2] + data[2 + headRightIoNum * 2]
        if data[2 + headCtrIoNum * 2] == 1 and headBtnPressedCount == 1:
            servo_ot += 1
            if servo_ot > 3:
                servo_ot = 0
                target_angle = 0
                if data[2 + headUpIoNum * 2] == 1: # 抬头按键被按下
                    print("抬头")
                    data_set_angle[7] = 0x01
                    target_angle = angle1 + servo_step
                elif data[2 + headDownIoNum * 2] == 1: # 低头按键被按下
                    print("低头")
                    data_set_angle[7] = 0x01
                    target_angle = angle1 - servo_step
                elif data[2 + headLeftIoNum * 2] == 1: # 左转按键被按下
                    print("左转")
                    data_set_angle[7] = 0x02
                    target_angle = angle2 + servo_step
                else: # 右转按键被按下
                    print("右转")
                    data_set_angle[7] = 0x02
                    target_angle = angle2 - servo_step
                if target_angle < 0:
                    target_angle = 0
                elif target_angle > 1000:
                    target_angle = 1000
                data_set_angle[8] = target_angle & 0xFF
                data_set_angle[9] = (target_angle >> 8) & 0xFF
                print("发送到舵机数据：", data_set_angle)
                with lock:
                    ser.write(data_set_angle)
