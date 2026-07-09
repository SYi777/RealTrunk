#include <iostream>
#include <iomanip>
#include <thread>
#include <mutex>
#include <chrono>
#include <ctime>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <jsoncpp/json/json.h>

// 全局变量
int angle1 = 0;
int angle2 = 0;
std::mutex mtx;
std::chrono::time_point<std::chrono::steady_clock> last_time;

// 接收数据从串口
std::vector<uint8_t> receiveData(int fd)
{
    std::vector<uint8_t> buffer(256);
    int n = read(fd, buffer.data(), buffer.size());
    if (n > 0)
    {
        buffer.resize(n);
    }
    else
    {
        buffer.clear();
    }
    return buffer;
}

void get_servo_angle(int serial_fd, int broadcast_socket)
{
    unsigned char data_get_angle[] = {0x55, 0x55, 0x05, 0x15, 0x02, 0x01, 0x02};
    struct sockaddr_in broadcast_addr;
    broadcast_addr.sin_family = AF_INET;
    broadcast_addr.sin_port = htons(9996);
    broadcast_addr.sin_addr.s_addr = inet_addr("255.255.255.255");

    std::vector<uint8_t> receivedDataBuffer; // 存储接收到的数据缓冲区

    while (true)
    {
        // {
        //     std::lock_guard<std::mutex> lock(mtx);
        //     write(serial_fd, data_get_angle, sizeof(data_get_angle));
        //     unsigned char rev_data[11];
        //     int len = read(serial_fd, rev_data, sizeof(rev_data));
        //     std::cout << "接收到舵机返回的数据：" << std::endl;
        //     for (int i = 0; i < len; ++i)
        //     {
        //         std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)rev_data[i] << " ";
        //     }
        //     std::cout << std::endl;

        //     if (len != 11 || rev_data[0] != 0x55 || rev_data[1] != 0x55)
        //     {
        //         std::cout << "接收到舵机返回的数据不满足协议!" << std::endl;
        //         continue;
        //     }
        //     angle1 = (rev_data[7] << 8) | rev_data[6];
        //     angle2 = (rev_data[10] << 8) | rev_data[9];
        // }

        {
            std::lock_guard<std::mutex> lock(mtx);
            write(serial_fd, data_get_angle, sizeof(data_get_angle));
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2)); // 等待一下再接收

        bool isDataComplete = false;

        while (!isDataComplete)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1)); // 等待一下再接收

            // 接收数据
            auto received_data = receiveData(serial_fd);
            if (received_data.empty())
            {
                continue; // 如果没接收到数据就继续接收
            }

            // std::cout << "Received data: ";
            // for (auto byte : received_data)
            // {
            //     std::cout << std::hex << static_cast<int>(byte) << " ";
            // }
            // std::cout << std::endl;

            // 将新接收到的数据添加到缓冲区
            receivedDataBuffer.insert(receivedDataBuffer.end(), received_data.begin(), received_data.end());

            // 尝试解析缓冲区中的数据
            while (receivedDataBuffer.size() >= 5) // 至少5字节才能构成一个完整的数据帧
            {
                // 查找完整的协议帧
                size_t packetSize = receivedDataBuffer.size();
                // 计算数据帧的总长度
                uint8_t lengthField = receivedDataBuffer[2];
                size_t expectedPacketSize = 2 + lengthField;

                if (packetSize < expectedPacketSize)
                {
                    // std::cout << "The Received data incomplete, keep waiting! " << std::endl;
                    break; // 数据还不完整，继续等待更多数据
                }

                // 提取完整的协议帧
                std::vector<uint8_t> completePacket(receivedDataBuffer.begin(), receivedDataBuffer.begin() + expectedPacketSize);
                receivedDataBuffer.erase(receivedDataBuffer.begin(), receivedDataBuffer.begin() + expectedPacketSize); // 删除已处理的数据

                // 校验协议
                if (completePacket.size() == 11 && completePacket[0] == 0x55 && completePacket[1] == 0x55 && completePacket[3] == 0x15)
                {
                    // std::cout << "completePacket data: ";
                    // for (auto byte : completePacket)
                    // {
                    //     // std::cout << std::hex << static_cast<int>(byte) << " ";
                    //     std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)byte << " ";
                    // }
                    // std::cout << std::endl;

                    angle1 = (completePacket[7] << 8) | completePacket[6];
                    angle2 = (completePacket[10] << 8) | completePacket[9];

                    std::cout << "舵机1角度：" << angle1 << std::endl;
                    std::cout << "舵机2角度：" << angle2 << std::endl;

                    auto current_time = std::chrono::steady_clock::now();
                    std::chrono::duration<double, std::milli> time_span = current_time - last_time;
                    last_time = current_time;
                    std::cout << "时间间隔：" << time_span.count() << " 毫秒" << std::endl;

                    // 获取当前时间戳
                    // std::time_t timestamp = std::time(nullptr);
                    auto now = std::chrono::high_resolution_clock::now();
                    auto duration = now.time_since_epoch();
                    auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();

                    // 发送广播消息
                    Json::Value message;
                    message["angle1"] = angle1;
                    message["angle2"] = angle2;
                    // message["timestamp"] = timestamp;
                    message["timestamp"] = static_cast<Json::Int64>(timestamp);
                    // Json::StreamWriterBuilder writer;
                    // std::string json_message = Json::writeString(writer, message);
                    Json::FastWriter writer;
                    std::string json_message = writer.write(message).substr(0, writer.write(message).length() - 1);
                    sendto(broadcast_socket, json_message.c_str(), json_message.size(), 0, (struct sockaddr *)&broadcast_addr, sizeof(broadcast_addr));

                    isDataComplete = true; // 完整数据已处理，退出接收循环

                    // auto sendArmCanDataTime = steady_clock::now();
                    // int64_t usedTime = duration_cast<milliseconds>(sendArmCanDataTime - requestDataTime).count();
                    // int64_t duration = duration_cast<milliseconds>(sendArmCanDataTime - lastSendArmCanDataTime).count();
                    // lastSendArmCanDataTime = sendArmCanDataTime;
                    // // printf("usedTime: %ld ms\r\n", usedTime);
                    // // printf("Time since last data:  :%ld ms\r\n", duration);

                    // isDataComplete = true; // 完整数据已处理，退出接收循环

                    // if (sendDataWaitTime > usedTime)
                    // {
                    //     std::this_thread::sleep_for(std::chrono::milliseconds(sendDataWaitTime - usedTime - 1)); // 100Hz
                    // }
                    // return;
                }
                else
                {
                    std::cout << "Error: Invalid protocol format" << std::endl;
                    std::cout << "completePacket data: ";
                    for (auto byte : completePacket)
                    {
                        // std::cout << std::hex << static_cast<int>(byte) << " ";
                        std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)byte << " ";
                    }
                    std::cout << std::endl;
                    receivedDataBuffer.clear(); // 清空缓冲区，重新开始接收
                    isDataComplete = true;      // 防止死循环
                }
            }
        }

        // std::cout << "舵机1角度：" << angle1 << std::endl;
        // std::cout << "舵机2角度：" << angle2 << std::endl;

        // auto current_time = std::chrono::steady_clock::now();
        // std::chrono::duration<double, std::milli> time_span = current_time - last_time;
        // last_time = current_time;
        // std::cout << "时间间隔：" << time_span.count() << " 毫秒" << std::endl;

        // // 获取当前时间戳
        // std::time_t timestamp = std::time(nullptr);

        // // 发送广播消息
        // Json::Value message;
        // message["angle1"] = angle1;
        // message["angle2"] = angle2;
        // message["timestamp"] = static_cast<Json::Int64>(timestamp);
        // // Json::StreamWriterBuilder writer;
        // // std::string json_message = Json::writeString(writer, message);
        // Json::FastWriter writer;
        // std::string json_message = writer.write(message).substr(0, writer.write(message).length() - 1);
        // sendto(broadcast_socket, json_message.c_str(), json_message.size(), 0, (struct sockaddr *)&broadcast_addr, sizeof(broadcast_addr));

        std::this_thread::sleep_for(std::chrono::milliseconds(30)); // 每30毫秒获取一次角度
    }
}

int main()
{
    int headCtrIoNum = 5;   // 需要按下该按键再按头部方向的按键才能触发控制
    int headUpIoNum = 6;    // 抬头
    int headDownIoNum = 7;  // 低头
    int headLeftIoNum = 8;  // 头左转
    int headRightIoNum = 9; // 头右转

    // 创建一个UDP接收套接字
    int recv_socket = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in local_addr;
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(9997);
    local_addr.sin_addr.s_addr = INADDR_ANY;
    bind(recv_socket, (struct sockaddr *)&local_addr, sizeof(local_addr));

    // 创建一个UDP广播套接字
    int broadcast_socket = socket(AF_INET, SOCK_DGRAM, 0);
    int broadcast_enable = 1;
    setsockopt(broadcast_socket, SOL_SOCKET, SO_BROADCAST, &broadcast_enable, sizeof(broadcast_enable));

    // 打开串口
    int serial_fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd == -1)
    {
        std::cerr << "无法打开串口" << std::endl;
        return -1;
    }
    struct termios options;
    tcgetattr(serial_fd, &options);
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    // options.c_cflag |= (CLOCAL | CREAD);
    // options.c_cflag &= ~PARENB;
    // options.c_cflag &= ~CSTOPB;
    // options.c_cflag &= ~CSIZE;
    // options.c_cflag |= CS8;
    // tcsetattr(serial_fd, TCSANOW, &options);
    // 设置数据位, 停止位和校验位
    options.c_cflag &= ~PARENB; // 无校验位
    options.c_cflag &= ~CSTOPB; // 1个停止位
    options.c_cflag &= ~CSIZE;  // 清除数据位
    options.c_cflag |= CS8;     // 8个数据位

    // 设置串口模式
    options.c_cflag |= CLOCAL; // 本地连接, 不需要控制线
    options.c_cflag |= CREAD;  // 启用接收功能

    // 设置输入输出模式
    options.c_iflag &= ~(IXON | IXOFF | IXANY);         // 禁用软件流控
    options.c_iflag &= ~ICRNL;                          // 禁用换行符转换
    options.c_oflag &= ~OPOST;                          // 禁用输出处理
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 禁用行模式

    // 清除等待队列
    tcflush(serial_fd, TCIFLUSH);

    // 设置新的串口参数
    tcsetattr(serial_fd, TCSANOW, &options);

    std::cout << "Serial port configured successfully." << std::endl;

    unsigned char data_set_angle[] = {0x55, 0x55, 0x08, 0x03, 0x01, 0x32, 0x00, 0x00, 0x00, 0x00};

    std::cout << "等待接收数据..." << std::endl;

    int servo_ot = 0;
    int servo_step = 15;

    last_time = std::chrono::steady_clock::now();

    std::thread angle_thread(get_servo_angle, serial_fd, broadcast_socket);
    angle_thread.detach();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    while (true)
    {
        // 接收数据
        unsigned char data[1024];
        struct sockaddr_in addr;
        socklen_t addr_len = sizeof(addr);
        int len = recvfrom(recv_socket, data, sizeof(data), 0, (struct sockaddr *)&addr, &addr_len);
        if (len < 0)
        {
            continue;
        }
        int headBtnPressedCount = data[2 + headUpIoNum * 2] + data[2 + headDownIoNum * 2] + data[2 + headLeftIoNum * 2] + data[2 + headRightIoNum * 2];
        if (data[2 + headCtrIoNum * 2] == 1 && headBtnPressedCount == 1)
        {
            servo_ot += 1;
            if (servo_ot > 3)
            {
                servo_ot = 0;
                int target_angle = 0;
                if (data[2 + headUpIoNum * 2] == 1)
                { // 抬头按键被按下
                    std::cout << "抬头" << std::endl;
                    data_set_angle[7] = 0x01;
                    target_angle = angle1 + servo_step;
                }
                else if (data[2 + headDownIoNum * 2] == 1)
                { // 低头按键被按下
                    std::cout << "低头" << std::endl;
                    data_set_angle[7] = 0x01;
                    target_angle = angle1 - servo_step;
                }
                else if (data[2 + headLeftIoNum * 2] == 1)
                { // 左转按键被按下
                    std::cout << "左转" << std::endl;
                    data_set_angle[7] = 0x02;
                    target_angle = angle2 + servo_step;
                }
                else
                { // 右转按键被按下
                    std::cout << "右转" << std::endl;
                    data_set_angle[7] = 0x02;
                    target_angle = angle2 - servo_step;
                }
                if (target_angle < 0)
                {
                    target_angle = 0;
                }
                else if (target_angle > 1000)
                {
                    target_angle = 1000;
                }
                data_set_angle[8] = target_angle & 0xFF;
                data_set_angle[9] = (target_angle >> 8) & 0xFF;
                std::cout << "发送到舵机数据：" << std::endl;
                for (int i = 0; i < sizeof(data_set_angle); ++i)
                {
                    std::cout << std::hex << std::setw(2) << std::setfill('0') << (int)data_set_angle[i] << " ";
                }
                std::cout << std::endl;
                {
                    std::lock_guard<std::mutex> lock(mtx);
                    write(serial_fd, data_set_angle, sizeof(data_set_angle));
                }
            }
        }
    }

    close(serial_fd);
    close(recv_socket);
    close(broadcast_socket);

    return 0;
}
