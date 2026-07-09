import socket
import json
import time
import threading


def calculate_robotiq_gripper_position(p):
    """将主臂夹爪开合度位置(0:闭合~1000:完全张开)转换为Robotiq夹爪对应的开合度位置(255:闭合~0:完全张开)

    Args:
        p (整形): 主臂夹爪的开合度位置(0:闭合~1000:完全张开)

    Returns:
        int: Robotiq夹爪对应的开合度位置(255:闭合~0:完全张开)
    """
    position = 255 - (p * 255 / 1000)
    return int(position)

def receive_data(sock):
    while True:
        data = sock.recv(1024)
        # print("接收到的数据：", data.decode('utf-8'))
        try:
            json_data = json.loads(data.decode('utf-8'))
            # print("command：", json_data["command"])
            if json_data.get("command") is not None and json_data["command"] == "read_multiple_holding_registers" and json_data.get("data") is not None and len(json_data["data"]) == 6:
                gripper_position = json_data["data"][4]
                print("夹爪当前位置：", gripper_position)
        except json.JSONDecodeError as e:
            print("JSON解析失败: ", e)


if __name__ == "__main__":
    # 创建连接机械臂的Socket套接字
    arm_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    arm_address = ('169.254.128.18', 8080)  # 具身双臂机器人默认左机械臂的IP为169.254.128.18，右机械臂的IP为169.254.128.19
    arm_socket.connect(arm_address)
    
    receive_thread = threading.Thread(target=receive_data, args=(arm_socket,))
    receive_thread.start()

    # 设置机械臂末端接口24V输出（给夹爪供电）
    set_24V = '{"command":"set_tool_voltage","voltage_type":3}\r\n'
    arm_socket.send(set_24V.encode('utf-8'))

    time.sleep(0.5)
    # 设置末端接口板RS485接口为Modbus RTU主站（才能与连接到末端的第三方ModBus协议的执行器如夹爪通讯）
    set_modbus_mode = '{"command":"set_modbus_mode","port":1,"baudrate":115200,"timeout":2}\r\n'
    arm_socket.send(set_modbus_mode.encode('utf-8'))

    time.sleep(0.5)
    # 夹爪激活请求-清除rACT(对应modbus协议数据：09 10 03 E8 00 03 06 00 00 00 00 00 00 73 30)
    clear_rACT = '{"command":"write_registers","port":1,"address":1000,"num":3,"data":[0,0,0,0,0,0],"device":9}\r\n'
    arm_socket.send(clear_rACT.encode('utf-8'))

    time.sleep(0.5)
    # 夹爪激活请求-设置rACT(对应modbus协议数据：09 10 03 E8 00  03 06 01 00 00 00 00 00 72 E1)
    set_rACT = '{"command":"write_registers","port":1,"address":1000,"num":3,"data":[1,0,0,0,0,0],"device":9}\r\n'
    arm_socket.send(set_rACT.encode('utf-8'))
    
    # rev_data = arm_socket.recv(1024)
    # print("接收到的数据：", rev_data.decode('utf-8'))

    time.sleep(5)   # 等待夹爪初始化激活完成


    # 创建一个UDP套接字
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # 绑定本地地址和端口
    local_addr = ('', 9997) # 背负式遥操作的主控会将舵机主臂的数据通过UDP发出来，发送的端口相关配置在背负式主控程序build目录中的config.ini配置，默认左主臂为9997，右主臂为9998
    udp_socket.bind(local_addr)
    # udp_socket.setblocking(False)   # 设置为非阻塞模式
    
    print("等待接收主臂数据...")

    read_gripper_position = '{"command":"read_multiple_holding_registers","port":1,"address":2000,"num":3,"device":9}\r\n'
    
    gripper_ot = 0
    
    while True:
        # arm_socket.send(read_gripper_position.encode('utf-8'))
        # time.sleep(0.008)
        try:
            # 接收数据
            data, addr = udp_socket.recvfrom(1024)  # 1024表示本次接收的最大字节数
            # print(f"从{addr}接收到的数据: {data.decode('utf-8')}")
            # hex_data = ' '.join(f'{byte:02x}' for byte in data)
            # print(f"从{addr}接收到的数据: {hex_data}")
            gripper_ot = gripper_ot + 1
            if (gripper_ot > 3):
                gripper_ot = 0
                try:
                    json_data = json.loads(data.decode('utf-8'))
                    # print(f"从{addr}接收到的数据: {json_data}")
                    # 将主臂夹爪开合度位置(0:闭合~1000:完全张开)转换为Robotiq夹爪对应的开合度位置(255:闭合~0:完全张开)
                    gripper_position = calculate_robotiq_gripper_position(json_data['gripper'])
                    # print(f"收到的开合度: {json_data['gripper']}")
                    # print(f"控制夹爪位置: {gripper_position}")
                    registers_data = [9,0,0,gripper_position,255,255]   # 写入寄存器数据，前3个固定为9,0,0,后3个代表夹爪位置、速度、力度（0～255之间）
                    write_registers = {"command":"write_registers", "port":1, "address":1000, "num":3, "data":registers_data,"device":9}
                    json_wrte_registers = json.dumps(write_registers) + '\r\n'
                    arm_socket.send(json_wrte_registers.encode('utf-8'))
                except json.JSONDecodeError:
                    print("接收到的数据不是有效的JSON格式")
            else:
                arm_socket.send(read_gripper_position.encode('utf-8'))
        except BlockingIOError:
            continue
