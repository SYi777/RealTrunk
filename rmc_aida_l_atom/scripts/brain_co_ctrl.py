import socket
import time
import ini_parameter
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
# 读取配置文件
config = ini_parameter.RobotConfig(f'{ script_dir }/config.ini')

# 检查是否停止控制
def stop_command_is(data, stop_btns: list):
    for btn in stop_btns:
        if data[2 + btn * 2] == 1:
            return True
    
    return False

if __name__ == "__main__":

    leftOpenIoNum = 3   # 手张开
    leftCloseIoNum = 4  # 手闭合

    rightOpenIoNum = 8   # 手张开
    rightCloseIoNum = 9  # 手闭合

    # 按钮按下，则不能进行控制的系列
    stop_btns = [5, 10]

    # 创建连接机械臂的Socket套接字
    left_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    larm_address = (config.Arms.left_ip, config.Arms.port)
    left_socket.connect(larm_address)

    # 创建连接机械臂的Socket套接字
    right_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    rarm_address = (config.Arms.right_ip, config.Arms.port)
    right_socket.connect(rarm_address)

    # # 读取 YAML 文件
    # with open('./scripts/brain_co_config.yaml', 'r', encoding='utf-8') as file:
    #     data = yaml.safe_load(file)
    # left = data['left']
    l_thumb_Flex = config.Brain_Co.thumb_flex
    l_four_finger = config.Brain_Co.four_finger
    l_thumb_Aux = config.Brain_Co.thumb_aux

    # right = data['right']
    r_thumb_Flex = config.Brain_Co.thumb_flex
    r_four_finger = config.Brain_Co.four_finger
    r_thumb_Aux = config.Brain_Co.thumb_aux


    # 设置set_modbus_mode
    set_modbus_mode = f'{{"command":"set_modbus_mode","port":1,"baudrate":{ config.Brain_Co.baudrate },"timeout":2}}\r\n'
    left_socket.send(set_modbus_mode.encode('utf-8'))
    right_socket.send(set_modbus_mode.encode('utf-8'))


    # 设置24伏输出
    set_24 = '{"command":"set_tool_voltage","voltage_type":3}\r\n'
    left_socket.send(set_24.encode('utf-8'))
    right_socket.send(set_24.encode('utf-8'))

    # 强脑手开关指令
    l_close1 = f'{{"command":"write_registers","port":1,"address":1010,"num":6,"data":[0,0,0,{l_thumb_Flex},0,{l_four_finger},0,{l_four_finger},0,{l_four_finger},0,{l_four_finger}],"device":1}}\r\n'
    l_close2 = f'{{"command":"write_registers","port":1,"address":1010,"num":6,"data":[0,{l_thumb_Aux},0,{l_thumb_Flex},0,{l_four_finger},0,{l_four_finger},0,{l_four_finger},0,{l_four_finger}],"device":1}}\r\n'

    l_open1 = f'{{"command":"write_registers","port":1,"address":1010,"num":6,"data":[0,0,0,{l_thumb_Flex},0,{l_four_finger},0,{l_four_finger},0,{l_four_finger},0,{l_four_finger}],"device":1}}\r\n'
    l_open2 = f'{{"command":"write_registers","port":1,"address":1010,"num":6,"data":[0,0,0,{l_thumb_Flex},0,0,0,0,0,0,0,0],"device":1}}\r\n'
    
    r_close1 = f'{{"command":"write_registers","port":1,"address":1010,"num":6,"data":[0,0,0,{r_thumb_Flex},0,{r_four_finger},0,{r_four_finger},0,{r_four_finger},0,{r_four_finger}],"device":1}}\r\n'
    r_close2 = f'{{"command":"write_registers","port":1,"address":1010,"num":6,"data":[0,{r_thumb_Aux},0,{r_thumb_Flex},0,{r_four_finger},0,{r_four_finger},0,{r_four_finger},0,{r_four_finger}],"device":1}}\r\n'

    r_open1 = f'{{"command":"write_registers","port":1,"address":1010,"num":6,"data":[0,0,0,{r_thumb_Flex},0,{r_four_finger},0,{r_four_finger},0,{r_four_finger},0,{r_four_finger}],"device":1}}\r\n'
    r_open2 = f'{{"command":"write_registers","port":1,"address":1010,"num":6,"data":[0,0,0,{r_thumb_Flex},0,0,0,0,0,0,0,0],"device":1}}\r\n'

    print(l_close1, "\n", l_close2)

    time.sleep(0.5)
    left_socket.send(l_open2.encode('utf-8'))
    right_socket.send(r_open2.encode('utf-8'))

    lhand_open = True
    rhand_open = True


    # 创建一个UDP套接字
    recv_udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    recv_udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 允许地址复用
    
    # 绑定本地地址和端口
    local_addr = (config.Brain_Co.listen_ip, config.Brain_Co.listen_port)
    recv_udp_socket.bind(local_addr)

    #创建一个UDP广播套接字
    broadcast_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # broadcast_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    
    print("等待接收数据...")

    time.sleep(1)
    
    while True:
        # 接收数据
        data, addr = recv_udp_socket.recvfrom(1024)  # 1024表示本次接收的最大字节数

        if stop_command_is(data, stop_btns):
            pass
        elif data[2 + leftCloseIoNum * 2] == 1: # 手张开按键被按下
            if lhand_open == True:
                print("手闭合")
                left_socket.send(l_close1.encode('utf-8'))
                time.sleep(0.5)
                left_socket.send(l_close2.encode('utf-8'))
                lhand_open = False

        elif data[2 + leftOpenIoNum * 2] == 1: # 手闭合按键被按下
            if lhand_open == False:
                print("手张开")
                left_socket.send(l_open1.encode('utf-8'))
                time.sleep(0.5)
                left_socket.send(l_open2.encode('utf-8'))
                lhand_open = True

        if stop_command_is(data, stop_btns):
            pass
        elif data[2 + rightCloseIoNum * 2] == 1: # 手张开按键被按下
            if rhand_open == True:
                print("手闭合")
                right_socket.send(r_close1.encode('utf-8'))
                time.sleep(0.5)
                right_socket.send(r_close2.encode('utf-8'))
                rhand_open = False

        elif data[2 + rightOpenIoNum * 2] == 1: # 手闭合按键被按下
            if rhand_open == False:
                print("手张开")
                right_socket.send(r_open1.encode('utf-8'))
                time.sleep(0.5)
                right_socket.send(r_open2.encode('utf-8'))
                rhand_open = True