import socket
import serial
# import time

def main():
    # 创建一个UDP套接字
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # 绑定本地地址和端口
    local_addr = ('', 9997)
    udp_socket.bind(local_addr)
    
    print("等待接收数据...")
    
    while True:
        # 接收数据
        data, addr = udp_socket.recvfrom(1024)  # 1024表示本次接收的最大字节数
        # print(f"从{addr}接收到的数据: {data.decode('utf-8')}")
        hex_data = ' '.join(f'{byte:02x}' for byte in data)
        print(f"从{addr}接收到的数据: {hex_data}")

# def servo_move_target(ser, angle):


if __name__ == "__main__":
    # main()
    headUpIoNum = 1 # 抬头
    headDownIoNum = 2   # 低头
    headLeftIoNum = 3   # 头左转
    headRightIoNum = 4  # 头右转

    # 创建一个UDP套接字
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # 绑定本地地址和端口
    local_addr = ('', 9997)
    udp_socket.bind(local_addr)

    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=2)

    data_get_angle = (0x55, 0x55, 0x05, 0x15, 0x02, 0x01, 0x02)
    # data_set_angle = [0x55, 0x55, 0x08, 0x03, 0x01, 0xe8, 0x03, 0x00, 0x00, 0x00]
    data_set_angle = [0x55, 0x55, 0x08, 0x03, 0x01, 0x32, 0x00, 0x00, 0x00, 0x00]
    
    print("等待接收数据...")

    servo_ot = 0
    servo_step = 15
    
    while True:

        ser.write(data_get_angle)
        rev_data = ser.read(11)
        if len(rev_data) != 11 or rev_data[0] != 0x55 or rev_data[1] != 0x55:
            continue

        angle1 = (rev_data[7] << 8) | rev_data[6]
        angle2 = (rev_data[10] << 8) | rev_data[9]

        print("舵机1角度：", angle1)
        print("舵机2角度：", angle2)
        
        # 接收数据
        data, addr = udp_socket.recvfrom(1024)  # 1024表示本次接收的最大字节数
        # print(f"从{addr}接收到的数据: {data.decode('utf-8')}")
        hex_data = ' '.join(f'{byte:02x}' for byte in data)
        # print(f"从{addr}接收到的数据: {hex_data}")
        headBtnPressedCount = data[2 + headUpIoNum * 2] + data[2 + headDownIoNum * 2] + data[2 + headLeftIoNum * 2] + data[2 + headRightIoNum * 2]
        if headBtnPressedCount == 1:
            
            servo_ot += 1
            if servo_ot > 3:

                servo_ot = 0

                # ser.write(data_get_angle)
                # rev_data = ser.read(11)
                # print("接收到舵机数据：", rev_data.hex())
            
                # if len(rev_data) != 11 or rev_data[0] != 0x55 or rev_data[1] != 0x55:
                #     continue

                # angle1 = (rev_data[7] << 8) | rev_data[6]
                # angle2 = (rev_data[10] << 8) | rev_data[9]

                # print("舵机1角度：", angle1)
                # print("舵机2角度：", angle2)
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
                    target_angle = angle2 - servo_step
                else: # 右转按键被按下
                    print("右转")
                    data_set_angle[7] = 0x02
                    target_angle = angle2 + servo_step
                # time.sleep(1)
                if target_angle < 0:
                    target_angle = 0
                elif target_angle > 1000:
                    target_angle = 1000
                data_set_angle[8] = target_angle & 0xFF
                data_set_angle[9] = (target_angle >> 8) & 0xFF
                print("发送到舵机数据：", data_set_angle)
                ser.write(data_set_angle)
            