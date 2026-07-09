import socket

PORT=9999
IP=""

def main():
    # 创建一个UDP套接字
    # 背负式udp监听套接字声明
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 允许地址复用
    
    # 绑定本地地址和端口
    local_addr = (IP, PORT)
    udp_socket.bind(local_addr)
    
    print("等待接收数据...")
    
    while True:
        # 接收数据
        data, addr = udp_socket.recvfrom(1024)  # 1024表示本次接收的最大字节数
        if PORT == 9999:
            print(f"从{addr}接收到的数据: {data}")
        else:
            print(f"从{addr}接收到的数据: {data.decode('utf-8')}")
        # hex_data = ' '.join(f'{byte:02x}' for byte in data)
        # print(f"从{addr}接收到的数据: {hex_data}")

if __name__ == "__main__":
    main()
