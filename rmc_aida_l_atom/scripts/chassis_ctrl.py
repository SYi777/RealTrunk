import socket
import time
import serial
import ini_parameter
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
# 读取配置文件
config = ini_parameter.RobotConfig(f'{ script_dir }/config.ini')

CONTROL_X = 0
CONTROL_RZ = 0
# POWER_STATE
POWER_STATE = False

def enable_motor(serial_port):
    """
    启用电机
    :param serial_port: 已打开的串口对象
    """
    data = bytes([0x01, 0x44, 0x21, 0x00, 0x31, 0x00, 0x00, 0x01, 0x00, 0x01, 0x75, 0x34])
    serial_port.write(data)
    print("Motor Enable Success!")

def disable_motor(serial_port):
    """
    禁用电机
    :param serial_port: 已打开的串口对象
    """
    data = bytes([0x01, 0x44, 0x21, 0x00, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE5, 0x34])
    serial_port.write(data)
    print("Motor Disable Success!")

def stop_motor(serial_port):
    """
    停止电机
    :param serial_port: 已打开的串口对象
    """
    data = bytes([0x01, 0x44, 0x23, 0x18, 0x33, 0x18, 0x00, 0x00, 0x00, 0x00, 0xDD, 0x0C])
    serial_port.write(data)
    print("Motor Stop Success!")

def send_speed_command(serial_port,chassis_speed_x,chassis_speed_rz):
    """
    发送左右电机的速度指令
    :param serial_port: 已打开的串口对象
    :param chassis_speed_x: 线速度(m/s)
    :param chassis_speed_rz: 角速度(rad/s)
    """
    global CONTROL_X, CONTROL_RZ

    if chassis_speed_x == CONTROL_X and chassis_speed_rz == CONTROL_RZ or not POWER_STATE:
        return
    
    left_speed = int(50*(-chassis_speed_x - chassis_speed_rz))
    right_speed = int(50*(chassis_speed_x - chassis_speed_rz))
    # 构造基础指令
    data = bytearray([0x01, 0x44, 0x23, 0x18, 0x33, 0x18])
    
    # 添加速度数据 (高字节和低字节)
    data.append((left_speed >> 8) & 0xFF)
    data.append(left_speed & 0xFF)
    data.append((right_speed >> 8) & 0xFF)
    data.append(right_speed & 0xFF)
    
    # 计算CRC校验
    crc = calculate_crc(data)
    data.append(crc & 0xFF)
    data.append((crc >> 8) & 0xFF)
    try:
        serial_port.write(data)
        print(f"Speed Command Sent: Left={left_speed}, Right={right_speed}")
        CONTROL_X = chassis_speed_x
        CONTROL_RZ = chassis_speed_rz
    except Exception as err:
        pass

def request_encoder_data(serial_port):
    """
    请求编码器数据
    :param serial_port: 已打开的串口对象
    """
    data = bytes([0x01, 0x43, 0x50, 0x04, 0x51, 0x04, 0x28, 0x97])
    serial_port.write(data)
    print("Encoder Data Request Sent")

def run_motor(serial_port):
    """
    运行电机
    :param serial_port: 已打开的串口对象
    """
    data = bytes([0x01, 0x44, 0x23, 0x18, 0x33, 0x18, 0x00, 0x00, 0x00, 0x00, 0xDD, 0x0C])
    serial_port.write(data)
    print("Motor Run Success!")

def calculate_crc(data):
    """
    计算CRC校验值
    :param data: 需要计算CRC的字节数组
    :return: CRC校验值 (16位)
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc


if __name__ == "__main__":

    chassisCtrIoNum = 10    # 联合控制底盘的按键
    chassisForwardIoNum = 1     # 前进
    chassisBackIoNum = 2   # 后退
    chassisLeftIoNum = 3   # 左转
    chassisRightIoNum = 4  # 右转

    chassis_power = [1, 2]

    # 配置串口参数
    port = config.Chassis_Self.port  # 根据你的设备修改串口号
    baudrate = config.Chassis_Self.baudrate  # 根据你的设备修改波特率

    

    # 打开串口
    try:
        # 尝试打开串口
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=1,  # 超时时间（秒）
        )
        if ser.is_open:
            print(f"串口 {port} 打开成功！")
    except serial.SerialException as e:
        # 捕获串口打开失败的异常
        print(f"无法打开串口 {port}: {e}")
    except Exception as e:
        # 捕获其他可能的异常
        print(f"发生错误: {e}")

    # 创建一个UDP套接字
    recv_udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    recv_udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 允许地址复用
    
    # 绑定本地地址和端口
    local_addr = (config.Chassis_Self.listen_ip, config.Chassis_Self.listen_port)
    recv_udp_socket.bind(local_addr)

    broadcast_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)

    #使能电机
    enable_motor(ser)
    print("Motor Enable Success")
    
    time.sleep(1)

    run_motor(ser)
    print("Motor Run Success!")
    print("等待接收数据...")

    time.sleep(1)
        
    try:
        # 获取初始时间
        last_output_time = time.time()
        # controled
        controled = False

        while True:
            # 接收数据
            data, addr = recv_udp_socket.recvfrom(1024)  # 1024表示本次接收的最大字节数
            hex_data = ' '.join(f'{byte:02x}' for byte in data)
            headBtnPressedCount = data[2 + chassisForwardIoNum * 2] + data[2 + chassisBackIoNum * 2] + data[2 + chassisLeftIoNum * 2] + data[2 + chassisRightIoNum * 2]
            if data[2 + chassisCtrIoNum * 2] == 1 and headBtnPressedCount == 1:
                if data[2 + chassisForwardIoNum * 2] == 1: # 前进按键被按下
                    send_speed_command(ser,float(config.Chassis_Self.abs_linear_speed),0)
                    print("前进")
                elif data[2 + chassisBackIoNum * 2] == 1: # 后退按键被按下
                    send_speed_command(ser,-float(config.Chassis_Self.abs_linear_speed),0)
                    print("倒车")
                elif data[2 + chassisLeftIoNum * 2] == 1: # 左转按键被按下
                    send_speed_command(ser,0,float(config.Chassis_Self.abs_angular_speed))
                    print("原地左转")
                elif data[2 + chassisRightIoNum * 2] == 1: # 右转按键被按下
                    send_speed_command(ser,0,-float(config.Chassis_Self.abs_angular_speed))
                    print("原地右转")
            elif data[2 + chassisCtrIoNum * 2] == 1 and headBtnPressedCount == 2:
                if data[2 + chassisForwardIoNum * 2] == 1 and data[2 + chassisLeftIoNum * 2] == 1: # 前进按键，且左转按键被按下
                    send_speed_command(ser,float(config.Chassis_Self.abs_linear_speed),float(config.Chassis_Self.abs_angular_speed))
                    print("向前左转")
                elif data[2 + chassisBackIoNum * 2]  == 1 and data[2 + chassisLeftIoNum * 2] == 1: # 后退按键，且左转按键被按下
                    send_speed_command(ser,-float(config.Chassis_Self.abs_linear_speed),float(config.Chassis_Self.abs_angular_speed))
                    print("倒车左转")
                elif data[2 + chassisForwardIoNum * 2] == 1 and data[2 + chassisRightIoNum * 2] == 1: # 前进按键，且右转按键被按下
                    send_speed_command(ser,float(config.Chassis_Self.abs_linear_speed),-float(config.Chassis_Self.abs_angular_speed))
                    print("向前右转")
                elif data[2 + chassisBackIoNum * 2]  == 1 and data[2 + chassisRightIoNum * 2] == 1: # 后退按键，右转按键被按下
                    send_speed_command(ser,-float(config.Chassis_Self.abs_linear_speed),-float(config.Chassis_Self.abs_angular_speed))
                    print("倒车右转")
            else:
                send_speed_command(ser,0,0)
                print("停止")
            time.sleep(0.001)

            # power_control
            # 获取当前时间
            current_time = time.time()
            print(current_time - last_output_time)
            print(data[2 + chassis_power[0] * 2] == 1 , '\t', data[2 + chassis_power[1] * 2] == 1)
            if current_time - last_output_time > 3 and not controled:
                last_output_time = current_time
                # do
                POWER_STATE = not POWER_STATE

                # chassis_power_control(ser, power=POWER_STATE)
                print("*" * 30, POWER_STATE)
                controled = True

            elif data[2 + chassis_power[0] * 2] == 1 and data[2 + chassis_power[1] * 2] == 1:
                pass
            else:
                last_output_time = current_time
                controled = False


    except KeyboardInterrupt:
        # 捕获Ctrl+C中断
        pass
    finally:
        # chassis_power_control(config.Chassis_Self.power_port, config.Chassis_Self.power_baudrate, power=False)
        # 确保最后关闭串口和套接字
        if ser.is_open:
            stop_motor(ser)
            ser.close()
        recv_udp_socket.close()
        print("程序已安全退出。")