#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <thread>
#include <chrono>
#include <vector>
#include <fstream>
#include <cstring>
#include <arpa/inet.h>
// #include <sys/types.h>
// #include <sys/socket.h>
// #include <netinet/in.h>
#include <jsoncpp/json/json.h>
#include "rm_service.h"

RM_Service robotic_arm;

// 滤波函数相关
#define HUADONGLVBOSHU 64
float FilterData_Buf[8][HUADONGLVBOSHU] = {0};
float FilterData_Bufsum[8] = {0};
int FilterData_Bufi[8] = {0};
int FilterData_Bufk[8] = {0};

// 上一次串口获取到的主臂6个关节的角度
float last_joint[6] = {0};

// 休眠(毫秒)
void sleep_cp(int milliseconds)
{
#ifdef _WIN32
    Sleep(milliseconds);
#endif

#ifdef __linux
    usleep(milliseconds * 1000);
#endif
}

// 滑动平均滤波函数
float Refilter(int joint_id, float InData, int InCount)
{
    int j = 0;
    float ReAverageData = 0;

    FilterData_Bufk[joint_id]++;

    if (FilterData_Bufk[joint_id] > InCount)
    {
        FilterData_Bufk[joint_id] = InCount;

        FilterData_Bufsum[joint_id] = FilterData_Bufsum[joint_id] - FilterData_Buf[joint_id][0];

        for (j = 1; j < InCount; j++)
        {
            FilterData_Buf[joint_id][j - 1] = FilterData_Buf[joint_id][j];
        }
        FilterData_Buf[joint_id][InCount - 1] = InData;

        FilterData_Bufsum[joint_id] += InData;

        ReAverageData = FilterData_Bufsum[joint_id] / InCount;
    }
    else
    {
        FilterData_Buf[joint_id][FilterData_Bufi[joint_id]] = InData;

        FilterData_Bufi[joint_id]++;

        FilterData_Bufsum[joint_id] += InData;

        ReAverageData = FilterData_Bufsum[joint_id] / FilterData_Bufi[joint_id];
    }

    return ReAverageData;
}

void custom_api_log(const char *message, va_list args)
{
    if (!message)
    {
        fprintf(stderr, "Error: message is a null pointer\n");
        return;
    }

    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), message, args);
    printf(" %s\n", buffer);
}

/******************************************************************
 *  CRC8 码表
 ******************************************************************/
// 生成多项式：X8+X5+X4+1=0x31 ，CRC8TAB[1]=生成多项式
const uint8_t CRC8TAB[256] = {
    0x00, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97,
    0xB9, 0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E,
    0x43, 0x72, 0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4,
    0xFA, 0xCB, 0x98, 0xA9, 0x3E, 0x0F, 0x5C, 0x6D,
    0x86, 0xB7, 0xE4, 0xD5, 0x42, 0x73, 0x20, 0x11,
    0x3F, 0x0E, 0x5D, 0x6C, 0xFB, 0xCA, 0x99, 0xA8,
    0xC5, 0xF4, 0xA7, 0x96, 0x01, 0x30, 0x63, 0x52,
    0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA, 0xEB,
    0x3D, 0x0C, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA,
    0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13,
    0x7E, 0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9,
    0xC7, 0xF6, 0xA5, 0x94, 0x03, 0x32, 0x61, 0x50,
    0xBB, 0x8A, 0xD9, 0xE8, 0x7F, 0x4E, 0x1D, 0x2C,
    0x02, 0x33, 0x60, 0x51, 0xC6, 0xF7, 0xA4, 0x95,
    0xF8, 0xC9, 0x9A, 0xAB, 0x3C, 0x0D, 0x5E, 0x6F,
    0x41, 0x70, 0x23, 0x12, 0x85, 0xB4, 0xE7, 0xD6,
    0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC, 0xED,
    0xC3, 0xF2, 0xA1, 0x90, 0x07, 0x36, 0x65, 0x54,
    0x39, 0x08, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE,
    0x80, 0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17,
    0xFC, 0xCD, 0x9E, 0xAF, 0x38, 0x09, 0x5A, 0x6B,
    0x45, 0x74, 0x27, 0x16, 0x81, 0xB0, 0xE3, 0xD2,
    0xBF, 0x8E, 0xDD, 0xEC, 0x7B, 0x4A, 0x19, 0x28,
    0x06, 0x37, 0x64, 0x55, 0xC2, 0xF3, 0xA0, 0x91,
    0x47, 0x76, 0x25, 0x14, 0x83, 0xB2, 0xE1, 0xD0,
    0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0x0B, 0x58, 0x69,
    0x04, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93,
    0xBD, 0x8C, 0xDF, 0xFE, 0x79, 0x48, 0x1B, 0x2A,
    0xC1, 0xF0, 0xA3, 0x92, 0x05, 0x34, 0x67, 0x56,
    0x78, 0x49, 0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF,
    0x82, 0xB3, 0xE0, 0xD1, 0x46, 0x77, 0x24, 0x15,
    0x3B, 0x0A, 0x59, 0x68, 0xFF, 0xCE, 0x9D, 0xAC};
/******************************************************
 *函数名称:CRC8
 *输   入:buf 要校验的数据; len 校验数据的长
 *输   出:校验值
 *功   能:循环冗余校验-8（国际标准-0x31）
 *******************************************************/
uint8_t CRC8(const uint8_t *buf, size_t len)
{
    uint8_t crc = 0;

    while (len--)
    {
        crc = CRC8TAB[crc ^ *buf];
        buf++;
    }

    return crc;
}

// CRC8计算函数
// uint8_t CRC8(const uint8_t* data, size_t length) {
//     uint8_t crc = 0x00; // 初始值
//     while (length--) {
//         crc ^= *data++;
//         for (uint8_t i = 0; i < 8; ++i) {
//             if (crc & 0x80) {
//                 crc = (crc << 1) ^ 0x07; // 多项式0x07
//             } else {
//                 crc <<= 1;
//             }
//         }
//     }
//     return crc;
// }

// 关闭串口
void CloseSerialPort(int fd)
{
    close(fd);
    std::cout << "Serial port closed." << std::endl;
}

// 配置串口
int ConfigureSerialPort(const char *portName)
{
    int fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        std::cerr << "Error: Unable to open serial port." << std::endl;
        exit(1);
    }
    else
    {
        fcntl(fd, F_SETFL, 0);
        std::cout << "Serial port opened successfully." << std::endl;
    }
    struct termios options;

    if (tcgetattr(fd, &options) != 0)
    {
        std::cerr << "Error: Unable to get serial port attributes." << std::endl;
        CloseSerialPort(fd);
        exit(1);
    }

    cfsetispeed(&options, B460800);
    cfsetospeed(&options, B460800);

    options.c_cflag |= (CLOCAL | CREAD); // Enable the receiver and set local mode
    options.c_cflag &= ~PARENB;          // No parity bit
    options.c_cflag &= ~CSTOPB;          // 1 stop bit
    options.c_cflag &= ~CSIZE;           // Mask the character size bits
    options.c_cflag |= CS8;              // 8 data bits

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input mode
    options.c_iflag &= ~(IXON | IXOFF | IXANY);         // Disable software flow control
    options.c_oflag &= ~OPOST;                          // Raw output mode
    options.c_iflag &= ~ICRNL;                          // 禁用换行符转换

    tcflush(fd, TCIFLUSH);

    if (tcsetattr(fd, TCSANOW, &options) != 0)
    {
        std::cerr << "Error: Unable to set serial port attributes." << std::endl;
        CloseSerialPort(fd);
        exit(1);
    }

    std::cout << "Serial port configured successfully." << std::endl;
    return fd;
}

// 配置串口
// int configureSerialPort(const char* portName) {
//     int fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
//     if (fd == -1) {
//         std::cerr << "Failed to open port" << std::endl;
//         return -1;
//     }

//     struct termios options;
//     tcgetattr(fd, &options);
//     cfsetispeed(&options, B460800);
//     cfsetospeed(&options, B460800);
//     options.c_cflag |= (CLOCAL | CREAD);
//     options.c_cflag &= ~PARENB;
//     options.c_cflag &= ~CSTOPB;
//     options.c_cflag &= ~CSIZE;
//     options.c_cflag |= CS8;
//     tcsetattr(fd, TCSANOW, &options);

//     return fd;
// }

// 配置串口参数
int configureSerialPort(int fd, speed_t baudRate)
{
    struct termios options;

    // 获取当前串口设置
    if (tcgetattr(fd, &options) < 0)
    {
        perror("tcgetattr failed");
        return -1;
    }

    // 设置波特率
    cfsetispeed(&options, baudRate);
    cfsetospeed(&options, baudRate);

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
    tcflush(fd, TCIFLUSH);

    // 设置新的串口参数
    if (tcsetattr(fd, TCSANOW, &options) < 0)
    {
        perror("tcsetattr failed");
        return -1;
    }

    fcntl(fd, F_SETFL, 0);

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(fd, &read_fds);

    // 设置超时时间
    struct timeval tv;
    tv.tv_sec = timeout_ms / 1000;          // 秒
    tv.tv_usec = (timeout_ms % 1000) * 1000; // 微秒

    // 使用select等待文件描述符可读
    int select_ret = select(fd + 1, &read_fds, nullptr, nullptr, &tv);
    if (select_ret == -1) {
        // select错误
        std::cerr << "select error: " << strerror(errno) << std::endl;
        return -1;
    } else if (select_ret == 0) {
        // 超时
        return 0;
    }

    return 0;
}

// 打开串口设备
int openSerialPort(const char *device)
{
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        perror("Unable to open serial port");
        return -1;
    }

    // 设置为正常模式，不是非阻塞模式
    fcntl(fd, F_SETFL, 0);

    return fd;
}

// 发送数据到串口
int sendData(int fd, const std::vector<uint8_t> &data)
{
    ssize_t written = write(fd, data.data(), data.size());
    if (written < 0)
    {
        perror("Write failed");
        return -1;
    }
    return 0;
}

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

// 校验协议格式
bool isValidProtocol(const std::vector<uint8_t> &data)
{
    // 数据长度必须大于等于 7 字节
    if (data.size() < 7)
    {
        std::cout << "Error: Data too short" << std::endl;
        return false;
    }

    // 验证前5个字节
    if (data[0] != 0xAA || data[1] != 0x55 || data[2] != 0x02 || data[3] != 0x00 || data[4] != 0x00)
    {
        std::cout << "Error: Invalid header" << std::endl;
        return false;
    }

    // 获取第6个字节，表示长度字段
    uint8_t lengthField = data[5];

    // 获取实际数据部分的长度
    size_t actualLength = data.size() - 7; // 7 = header (5) + length field (1) + CRC (1)

    // 检查长度字段是否与实际长度匹配
    if (lengthField != actualLength)
    {
        std::cout << "Error: Length mismatch" << std::endl;
        return false;
    }

    // // 校验 CRC8 校验
    // std::vector<uint8_t> crcData(data.begin(), data.end() - 1); // 去掉 CRC 字节
    // // uint8_t calculatedCRC = calculateCRC8(crcData);
    // uint8_t calculatedCRC = CRC8(crcData.data(), crcData.size() - 1);
    // if (calculatedCRC != data.back()) {
    //     std::cout << "Error: CRC mismatch" << std::endl;
    //     return false;
    // }

    return true;
}

// 通过网络发送数据
void sendNetworkData(int sockfd, const sockaddr_in &broadcastAddr, const std::string &data)
{
    sendto(sockfd, data.c_str(), data.size(), 0, (const struct sockaddr *)&broadcastAddr, sizeof(broadcastAddr));
}

// 设置夹爪开合度
void set_gripper_position(rm_robot_handle *arm_handle, int position)
{
    robotic_arm.rm_set_gripper_position(arm_handle, position, false, 0);
}

// 发送和接收数据的线程函数
void communicationThread(int fd, int sockfd, bool follow_mode, sockaddr_in &broadcastAddr, rm_robot_handle *arm_handle, int arm_dof, std::string tag)
{
    using namespace std::chrono;
    int gripper_ot = 0; // 夹爪控制计数，30毫秒控制一次
    int sendDataWaitTime = 10;

    std::vector<uint8_t> receivedDataBuffer; // 存储接收到的数据缓冲区

    auto lastSendArmCanDataTime = steady_clock::now();

    while (true)
    {
        auto requestDataTime = steady_clock::now();
        sendData(fd, {0x55, 0xAA, 0x02, 0x00, 0x00, 0x67});
        std::this_thread::sleep_for(milliseconds(4)); // 等待一段时间确保数据可以接收
        bool isDataComplete = false;

        while (!isDataComplete)
        {
            // 接收数据
            auto receivedData = receiveData(fd);
            if (!receivedData.empty())
            {
                // std::cout << "Received data: ";
                // for (auto byte : receivedData)
                // {
                //     std::cout << std::hex << static_cast<int>(byte) << " ";
                // }
                // // std::cout << receivedData.size();
                // std::cout << std::endl;

                // 将新接收到的数据添加到缓冲区
                receivedDataBuffer.insert(receivedDataBuffer.end(), receivedData.begin(), receivedData.end());

                // 尝试解析缓冲区中的数据
                while (receivedDataBuffer.size() >= 7) // 至少8字节才能构成一个完整的数据帧
                {
                    // 查找完整的协议帧
                    size_t packetSize = receivedDataBuffer.size();
                    // 计算数据帧的总长度
                    uint8_t lengthField = receivedDataBuffer[5];
                    size_t expectedPacketSize = 6 + lengthField + 1;

                    if (packetSize >= expectedPacketSize)
                    {
                        // 提取完整的协议帧
                        std::vector<uint8_t> completePacket(receivedDataBuffer.begin(), receivedDataBuffer.begin() + expectedPacketSize);
                        receivedDataBuffer.erase(receivedDataBuffer.begin(), receivedDataBuffer.begin() + expectedPacketSize); // 删除已处理的数据

                        // 校验协议
                        if (isValidProtocol(completePacket))
                        {
                            // std::cout << "completePacket data: ";
                            // for (auto byte : completePacket)
                            // {
                            //     std::cout << std::hex << static_cast<int>(byte) << " ";
                            // }
                            // std::cout << std::endl;

                            float joint[arm_dof] = {0};
                            float joint_filter[arm_dof] = {0};

                            Json::Value root;
                            Json::Value jointArray(Json::arrayValue);

                            // int32_t value = 0;
                            // int i = 6;  //从joint1在线状态位开始
                            size_t i = 0;
                            for (; i < arm_dof; i++)
                            {
                                if (completePacket[i * 5 + 6] == 1) // joint i在线
                                {
                                    joint[i] = ((completePacket[i * 5 + 10] << 24) | (completePacket[i * 5 + 9] << 16) | (completePacket[i * 5 + 8] << 8) | completePacket[i * 5 + 7]) / 10000.0;

                                    // if (last_joint[i] != joint[i])
                                    // {
                                    //     std::cout << "***************************************************joint change************************************************" << std::endl;
                                    // }
                                    last_joint[i] = joint[i];
                                    // joint_filter[i] = Refilter(i, joint[i], 64);
                                    joint_filter[i] = Refilter(i, joint[i], 10);

                                    // Populate the joint array
                                    jointArray.append(joint_filter[i]);
                                }
                            }
                            int gripper_deg = (completePacket[i * 5 + 10] << 24) | (completePacket[i * 5 + 9] << 16) | (completePacket[i * 5 + 8] << 8) | completePacket[i * 5 + 7];

                            root["tag"] = tag;
                            root["joint"] = jointArray;
                            root["gripper"] = gripper_deg;

                            // Convert to JSON string
                            // Json::StreamWriterBuilder writer;
                            Json::FastWriter writer;
                            // std::string jsonString = Json::writeString(writer, root);
                            std::string jsonString = writer.write(root).substr(0, writer.write(root).length() - 1);

                            // std::cout << jsonString << std::endl;
                            sendNetworkData(sockfd, broadcastAddr, jsonString);

                            gripper_ot++;
                            if (gripper_ot > 3)
                            {
                                gripper_ot = 0;
                                // Set_Gripper_Release(m_sockhand,gripper_deg,0,1);
                                // Set_Gripper_Position(m_sockhand, gripper_deg, false, 0);
                                // robotic_arm.rm_set_gripper_position(arm_handle, gripper_deg, false, 0);
                                // 创建一个新的线程控制夹爪避免阻塞等待
                                std::thread(set_gripper_position, arm_handle, gripper_deg).detach();
                                // printf("gripper_deg :%d\r\n", gripper_deg);
                            }

                            // std::cout << "Servo joint: ";
                            // for (size_t i = 0; i < 6; i++)
                            // {
                            //     std::cout << "joint[" << i << "]: " << joint[i] << " " << "filter[" << i << "]: " << joint_filter[i] << " ";
                            // }
                            // std::cout << std::endl;
                            // int ret = Movej_CANFD(m_sockhand, joint_filter, follow_mode, 0);
                            // std::cout << "joint[5]: " << joint[4] << " " << "filter[5]: " << joint_filter[4] << " " << std::endl;
                            // std::cout << "joint5: " << joint[4] << " " << std::endl;
                            // robotic_arm.rm_movej_follow(arm_handle, joint_filter);
                            // robotic_arm.rm_movej_follow(arm_handle, joint);
                            robotic_arm.rm_movej_canfd(arm_handle, joint_filter, follow_mode, 0);

                            auto sendArmCanDataTime = steady_clock::now();
                            int64_t usedTime = duration_cast<milliseconds>(sendArmCanDataTime - requestDataTime).count();
                            int64_t duration = duration_cast<milliseconds>(sendArmCanDataTime - lastSendArmCanDataTime).count();
                            lastSendArmCanDataTime = sendArmCanDataTime;
                            // printf("usedTime: %ld ms\r\n", usedTime);
                            printf("Time since last data:  :%ld ms\r\n", duration);

                            isDataComplete = true; // 完整数据已处理，退出接收循环

                            if (sendDataWaitTime > usedTime)
                            {
                                std::this_thread::sleep_for(std::chrono::milliseconds(sendDataWaitTime - usedTime - 1)); // 100Hz
                            }
                        }
                        else
                        {
                            std::cout << "Error: Invalid protocol format" << std::endl;
                            receivedDataBuffer.clear(); // 清空缓冲区，重新开始接收
                            isDataComplete = true;      // 防止死循环
                        }
                    }
                    else
                    {
                        // std::cout << "The Received data incomplete, keep waiting! " << std::endl;
                        break; // 数据还不完整，继续等待更多数据
                    }
                }
            }
            // else
            // {
            //     std::cout << "#######receive data empty!#######" << std::endl;
            // }
            std::this_thread::sleep_for(milliseconds(1)); // 1000Hz，稍微等待一下以减少CPU占用
        }

        // auto sendArmCanDataTime = steady_clock::now();
        // int64_t duration = duration_cast<milliseconds>(sendArmCanDataTime - requestDataTime).count();
        // requestDataTime = sendArmCanDataTime;
        // // std::cout << "Time since last data: " << duration << " ms" << std::endl;
        // printf("Time since last data:  :%ld ms\r\n", duration);

        // std::cout << "sendDataWaitTime: " << sendDataWaitTime << std::endl;
        // if (sendDataWaitTime > 2)
        // {
        //     std::this_thread::sleep_for(std::chrono::milliseconds(sendDataWaitTime-2)); // 100Hz
        // }
    }
}

int main()
{

    std::string arm_ip;
    int arm_port = 0;
    std::string serial_name;
    int udp_port = 0;
    int follow_mode = 1; // 1为高跟随，0为低跟随
    bool follow = true;
    std::string tag = "arm";
    int arm_dof = 6;

    std::ifstream configFile("config.txt");
    if (configFile.is_open())
    {
        std::string line;
        while (std::getline(configFile, line))
        {
            size_t pos = line.find('=');
            if (pos != std::string::npos)
            {
                std::string key = line.substr(0, pos);
                std::string value = line.substr(pos + 1);

                if (key == "arm_ip")
                {
                    arm_ip = value;
                }
                else if (key == "arm_port")
                {
                    arm_port = std::stoi(value);
                }
                else if (key == "arm_dof")
                {
                    arm_dof = std::stoi(value);
                }
                else if (key == "serial_name")
                {
                    serial_name = value;
                }
                else if (key == "udp_port")
                {
                    udp_port = std::stoi(value);
                }
                else if (key == "follow_mode")
                {
                    follow_mode = std::stoi(value);
                }
                else if (key == "tag")
                {
                    tag = value;
                }
            }
        }

        configFile.close();
    }
    else
    {
        std::cerr << "Error opening config file." << std::endl;
        return -1;
    }

    int result = -1;

    robotic_arm.rm_set_log_call_back(custom_api_log, 3);
    result = robotic_arm.rm_init(RM_DUAL_MODE_E);
    if (result != 0)
    {
        printf("Initialization failed with error code: %d.\n", result);
        return -1;
    }

    char *api_version = robotic_arm.rm_api_version();
    printf("API Version: %s.\n", api_version);

    rm_robot_handle *robot_handle = robotic_arm.rm_create_robot_arm((char *)arm_ip.c_str(), arm_port);
    if (robot_handle == NULL)
    {
        printf("Failed to create robot handle.\n");
        return -1;
    }
    else
    {
        printf("Robot handle created successfully: %d\n", robot_handle->id);
    }
    rm_arm_software_version_t software_info;
    result = robotic_arm.rm_get_arm_software_info(robot_handle, &software_info);
    if (result == 0)
    {
        printf("================== Arm Software Information ==================\n");
        printf("Arm Model:  %s\n", software_info.product_version);
        printf("Algorithm Library Version:  %s\n", software_info.algorithm_info.version);
        printf("Control Layer Software Version:  %s\n", software_info.ctrl_info.version);
        printf("Dynamics Version:  %s\n", software_info.dynamic_info.model_version);
        printf("Planning Layer Software Version:  %s\n", software_info.plan_info.version);
        printf("==============================================================\n");
    }
    // 设置工具端电源输出24V
    robotic_arm.rm_set_tool_voltage(robot_handle, 3);

    // const char* portName = serial_name.c_str(); // 根据你的串口设备名称调整

    // 配置串口
    int fd = ConfigureSerialPort(serial_name.c_str());
    // int fd = openSerialPort(serial_name.c_str());
    // if (fd == -1) {
    //     return 1;
    // }

    // // 配置串口波特率为460800
    // if (configureSerialPort(fd, B460800) < 0) {
    //     close(fd);
    //     return 1;
    // }

    // 设置网络通信
    int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0)
    {
        std::cerr << "Failed to create socket" << std::endl;
        return -1;
    }

    // 启用广播选项
    int broadcastEnable = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable)) < 0)
    {
        std::cerr << "Failed to enable broadcast option" << std::endl;
        close(sockfd);
        return -1;
    }

    sockaddr_in broadcastAddr;
    memset(&broadcastAddr, 0, sizeof(broadcastAddr));
    broadcastAddr.sin_family = AF_INET;
    broadcastAddr.sin_port = htons(udp_port);                     // 修改为目标端口
    broadcastAddr.sin_addr.s_addr = inet_addr("255.255.255.255"); // 广播地址
    // inet_pton(AF_INET, "192.168.10.33", &broadcastAddr.sin_addr); // 修改为目标IP地址

    usleep(2000000); // 100 ms
    // 第2步
    sendData(fd, {0x02, 0x18, 0x00, 0x00});
    // 等待一段时间确保数据可以接收
    usleep(100000); // 100 ms
    sendData(fd, {0x02, 0x18, 0x00, 0x00});
    usleep(100000); // 100 ms
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    sendData(fd, {0x55, 0xAA, 0x01, 0x00, 0x00, 0xAD});
    // 等待一段时间确保数据可以接收
    usleep(100000); // 100 ms
    std::cout << "sendData ok!" << std::endl;

    while (true)
    {
        auto receivedData = receiveData(fd);
        if (!receivedData.empty())
        {
            std::cout << "Received data: ";
            for (auto byte : receivedData)
            {
                std::cout << std::hex << static_cast<int>(byte) << " ";
            }
            std::cout << std::endl;
            // 继续往下执行
            break;
        }
        // usleep(100000); // 100 ms
        // std::this_thread::sleep_for(std::chrono::seconds(1));
        sendData(fd, {0x02, 0x18, 0x00, 0x00});
        // std::this_thread::sleep_for(std::chrono::seconds(1));
        usleep(100000); // 100 ms
        sendData(fd, {0x55, 0xAA, 0x01, 0x00, 0x00, 0xAD});
        std::cout << "sendData ok!" << std::endl;
        // 等待一段时间确保数据可以接收
        usleep(100000); // 100 ms
    }
    usleep(100000); // 100 ms
    // 机械臂与主臂状态同步
    sendData(fd, {0x55, 0xAA, 0x02, 0x00, 0x00, 0x67});
    usleep(100000); // 100 ms
    auto receivedData = receiveData(fd);
    if (!receivedData.empty())
    {
        std::cout << "Received data: ";
        for (auto byte : receivedData)
        {
            std::cout << std::hex << static_cast<int>(byte) << " ";
        }
        // std::cout << receivedData.size();
        std::cout << std::endl;
        if (receivedData.size() >= 42 && receivedData[0] == 0xaa && receivedData[1] == 0x55 && receivedData[2] == 0x02 && receivedData[4] == 0x00) // 关节信息数据
        {

            float joint[ARM_DOF] = {0};
            int i = 0;
            for (; i < ARM_DOF; i++)
            {
                if (receivedData[i * 5 + 6] == 1) // joint i在线
                {
                    joint[i] = ((receivedData[i * 5 + 10] << 24) | (receivedData[i * 5 + 9] << 16) | (receivedData[i * 5 + 8] << 8) | receivedData[i * 5 + 7]) / 10000.0;
                }
            }
            int gripper_deg = (receivedData[i * 5 + 10] << 24) | (receivedData[i * 5 + 9] << 16) | (receivedData[i * 5 + 8] << 8) | receivedData[i * 5 + 7];
            printf("gripper_deg :%d\r\n", gripper_deg);

            robotic_arm.rm_set_gripper_position(robot_handle, gripper_deg, false, 5);

            int ret = robotic_arm.rm_movej(robot_handle, joint, 10, 0, 0, 1);
            if (ret != 0)
            {
                printf("Movej_Cmd :%d\r\n", ret);
                // return;
            }
        }
    }

    if (follow_mode == 0)
    {
        follow = false;
    }

    // 启动100Hz的发送和接收线程
    std::thread commThread(communicationThread, fd, sockfd, follow, std::ref(broadcastAddr), robot_handle, arm_dof, tag);
    commThread.join();
    std::cout << "Thread start!" << std::endl;
    close(fd);

    return 0;
}
