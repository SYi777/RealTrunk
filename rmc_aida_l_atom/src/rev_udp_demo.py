import socket

def main():
    # 创建一个UDP套接字
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    
    # 绑定本地地址和端口
    local_addr = ('', 9999)
    udp_socket.bind(local_addr)
    
    print("等待接收数据...")
    
    while True:
        # 接收数据
        data, addr = udp_socket.recvfrom(1024)  # 1024表示本次接收的最大字节数
        print(f"从{addr}接收到的数据: {data.decode('utf-8')}")
        # hex_data = ' '.join(f'{byte:02x}' for byte in data)
        # print(f"从{addr}接收到的数据: {hex_data}")

if __name__ == "__main__":
    main()
