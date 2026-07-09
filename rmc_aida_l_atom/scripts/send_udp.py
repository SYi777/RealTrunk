import socket
import time

# 创建UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)  # 启用广播

# 广播地址和端口
broadcast_address = "<broadcast>"
port = 5005

while True:
    data = b'This is a broadcast message'
    sock.sendto(data, (broadcast_address, port))
    print(f"Sent: {data}")
    time.sleep(1)