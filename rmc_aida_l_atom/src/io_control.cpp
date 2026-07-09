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
#include <jsoncpp/json/json.h>
#include "rm_service.h"
#include "woosh_robot_control.h"

RM_Service robotic_arm;

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

    return 0;
}

// 打开串口设备
int openSerialPort(const char *device)
{
    int timeout_ms = 2;
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)
    {
        perror("Unable to open serial port");
        return -1;
    }

    // 设置为正常模式，不是非阻塞模式
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

// 计算Modbus CRC值
uint16_t calculateCRC(const std::vector<uint8_t> &data)
{
    uint16_t crc = 0xFFFF; // 初始化CRC寄存器
    for (size_t i = 0; i < data.size(); ++i)
    {
        crc ^= data[i];                // 将当前字节与CRC寄存器异或
        for (size_t j = 0; j < 8; ++j) // 对每个字节的每一位进行处理
        {
            if (crc & 0x0001)
            {
                crc >>= 1;     // 右移一位
                crc ^= 0xA001; // 与多项式0xA001异或
            }
            else
            {
                crc >>= 1; // 右移一位
            }
        }
    }
    // 高低字节转换
    crc = ((crc & 0x00ff) << 8) | ((crc & 0xff00) >> 8);
    return crc; // 返回计算得到的CRC值
}

// 校验Modbus协议数据的CRC是否正确
bool validateCRC(const std::vector<uint8_t> &data)
{
    if (data.size() < 2) // 至少需要两个字节来存储CRC
    {
        return false;
    }
    uint16_t receivedCRC = (data[data.size() - 2] << 8) | data[data.size() - 1]; // 获取接收到的CRC
    std::vector<uint8_t> dataWithoutCRC(data.begin(), data.end() - 2);           // 去掉CRC字节
    uint16_t calculatedCRC = calculateCRC(dataWithoutCRC);
    // std::cout << "receivedCRC" << std::hex << receivedCRC << " " << std::endl;
    // std::cout << "calculatedCRC" << std::hex << calculatedCRC << " " << std::endl;
    return receivedCRC == calculatedCRC; // 比较接收到的CRC与计算得到的CRC
}

// 校验协议格式
bool isValidProtocol(const std::vector<uint8_t> &data)
{
    // 数据长度必须大于等于 6 字节
    if (data.size() < 6)
    {
        std::cout << "Error: Data too short" << std::endl;
        return false;
    }

    // 验证前3个字节
    if (data[0] != 0x01 || data[1] != 0x04 || data[2] != 0x20)
    {
        std::cout << "Error: Invalid header" << std::endl;
        return false;
    }

    // 获取第3个字节，表示长度字段
    uint8_t lengthField = data[2];

    // 获取实际数据部分的长度
    size_t actualLength = data.size() - 5; // 5 = header (3) + CRC (2)

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

    if (!validateCRC(data))
    {
        std::cout << "Error: CRC mismatch" << std::endl;
        return false;
    }

    return true;
}

// 发送和接收数据的线程函数
void communicationThread(int fd, rm_robot_handle *arm_handle, int arm_dof)
// void communicationThread(int fd, int arm_dof)
{
    int sendDataWaitTime = 50;

    using namespace std::chrono;
    auto lastReceivedTime = steady_clock::now();
    int gripper_ot = 0; // 夹爪控制计数，30毫秒控制一次

    std::vector<uint8_t> receivedDataBuffer; // 存储接收到的数据缓冲区

    int currentLiftSpeed = 0; // 导轨当前运动速度
    int liftSpeed = 0;        // 导轨预期速度
    int lifUpIoNum = 6;       // 导轨上升按键对应的IO口序号
    int lifDownIoNum = 7;     // 导轨下降按键对应的IO口序号
    int agvForwardIoNum = 1;     // AGV前进按键对应的IO口序号
    int agvBackIoNum = 2;     // AGV后退按键对应的IO口序号
    int agvLeftIoNum = 3;     // AGV左转按键对应的IO口序号
    int agvRightIoNum = 4;     // AGV右转按键对应的IO口序号

    std::string addr = "169.254.128.2";
    uint32_t port = 5410;
    WooshRobotControl robot_control(addr, port);
    // 初始化
    robot_control.initialize();

    auto lastSendArmCanDataTime = steady_clock::now();

    while (true)
    {
        auto requestDataTime = steady_clock::now();
        sendData(fd, {0x01, 0x04, 0x00, 0x00, 0x00, 0x10, 0xF1, 0xC6});
        std::this_thread::sleep_for(milliseconds(13)); // 等待一段时间确保数据可以接收
        // sendDataWaitTime -= 13;
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
                while (receivedDataBuffer.size() >= 6) // ModBus RTU至少6字节才能构成一个完整的数据帧
                {
                    // 查找完整的协议帧
                    size_t packetSize = receivedDataBuffer.size();
                    // 计算数据帧的总长度
                    uint8_t lengthField = receivedDataBuffer[2];
                    size_t expectedPacketSize = 3 + lengthField + 2;

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

                            // 升降控制判断处理
                            if (completePacket[2 + lifDownIoNum * 2] == 0)
                            {
                                if (completePacket[2 + lifUpIoNum * 2] == 1)
                                {
                                    liftSpeed = 50;
                                }
                                else
                                {
                                    liftSpeed = 0;
                                }
                            }
                            else
                            {
                                if (completePacket[2 + lifUpIoNum * 2] == 0)
                                {
                                    liftSpeed = -50;
                                }
                                else
                                {
                                    liftSpeed = 0;
                                }
                            }

                            if (currentLiftSpeed != liftSpeed)
                            {
                                currentLiftSpeed = liftSpeed;
                                robotic_arm.rm_set_lift_speed(arm_handle, currentLiftSpeed);
                                // 控制导轨升降
                                if (currentLiftSpeed > 0)
                                {
                                    std::cout << "up" << std::endl;
                                }
                                else if (currentLiftSpeed < 0)
                                {
                                    std::cout << "down" << std::endl;
                                }
                                else
                                {
                                    std::cout << "stop" << std::endl;
                                }
                            }

                            // AGV按键控制判断处理
                            double agvLinear = 0.0;
                            double agvAngular = 0.0;
                            int agvBtnPressedCount = completePacket[2 + agvForwardIoNum * 2] + completePacket[2 + agvBackIoNum * 2] + completePacket[2 + agvLeftIoNum * 2] + completePacket[2 + agvRightIoNum * 2]; // AGV控制按键按下数量
                            if (agvBtnPressedCount == 1)    // 前后左右只有一个按键被按下
                            {
                                if (completePacket[2 + agvForwardIoNum * 2] == 1)   // 前进按键被按下
                                {
                                    std::cout << "AGV go forward" << std::endl;
                                    agvLinear = 0.2;
                                    agvAngular = 0.0;
                                }
                                else if (completePacket[2 + agvBackIoNum * 2] == 1) // 后退按键被按下
                                {
                                    std::cout << "AGV go back" << std::endl;
                                    agvLinear = -0.2;
                                    agvAngular = 0.0;
                                }
                                else if (completePacket[2 + agvLeftIoNum * 2] == 1) // 左转按键被按下
                                {
                                    std::cout << "AGV turn left" << std::endl;
                                    agvLinear = 0.0;
                                    agvAngular = 0.785;
                                }
                                else    // 右转按键被按下
                                {
                                    std::cout << "AGV turn right" << std::endl;
                                    agvLinear = 0.0;
                                    agvAngular = -0.785;
                                }
                            }
                            else if (agvBtnPressedCount == 2)    // 前后左右同时有两个按键被按下
                            {
                                if (completePacket[2 + agvForwardIoNum * 2] == 1 && completePacket[2 + agvLeftIoNum * 2] == 1)
                                {
                                    std::cout << "AGV left forward" << std::endl;
                                    agvLinear = 0.2;
                                    agvAngular = 0.785;
                                }
                                else if (completePacket[2 + agvForwardIoNum * 2] == 1 && completePacket[2 + agvRightIoNum * 2] == 1)
                                {
                                    std::cout << "AGV right forward" << std::endl;
                                    agvLinear = 0.2;
                                    agvAngular = -0.785;
                                }
                                else if (completePacket[2 + agvBackIoNum * 2] == 1 && completePacket[2 + agvRightIoNum * 2] == 1)
                                {
                                    std::cout << "AGV right back" << std::endl;
                                    agvLinear = -0.2;
                                    agvAngular = -0.785;
                                }
                                else if (completePacket[2 + agvBackIoNum * 2] == 1 && completePacket[2 + agvLeftIoNum * 2] == 1)
                                {
                                    std::cout << "AGV left back" << std::endl;
                                    agvLinear = -0.2;
                                    agvAngular = 0.785;
                                }
                                else
                                {
                                    std::cout << "AGV stop" << std::endl;
                                    agvLinear = 0.0;
                                    agvAngular = 0.0;
                                }
                            }
                            else    // 前后左右同时有超过两个按键被按下或一个都没被按下
                            {
                                std::cout << "AGV stop" << std::endl;
                                agvLinear = 0.0;
                                agvAngular = 0.0;
                            }

                            // 设置速度
                            // if (agvLinear != 0 || agvAngular != 0)
                            // {
                            //     robot_control.setLinearSpeed(agvLinear);
                            //     robot_control.setAngularSpeed(agvAngular);
                            // }
                            robot_control.setLinearSpeed(agvLinear);
                            robot_control.setAngularSpeed(agvAngular);
                            
                            // auto currentReceivedTime = steady_clock::now();
                            // int64_t duration = duration_cast<milliseconds>(currentReceivedTime - lastReceivedTime).count();
                            // lastReceivedTime = currentReceivedTime;
                            // // std::cout << "Time since last data: " << duration << " ms" << std::endl;
                            // printf("Time since last data:  :%ld ms\r\n", duration);

                            isDataComplete = true; // 完整数据已处理，退出接收循环

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
                            // receivedDataBuffer.clear(); // 清空缓冲区，重新开始接收
                            isDataComplete = true; // 防止死循环
                        }
                        receivedDataBuffer.clear(); // 清空缓冲区，重新开始接收
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
            std::this_thread::sleep_for(milliseconds(2)); // 1000Hz，稍微等待一下以减少CPU占用
            // sendDataWaitTime -= 2;
        }

        // auto currentReceivedTime = steady_clock::now();
        // int64_t duration = duration_cast<milliseconds>(currentReceivedTime - lastReceivedTime).count();
        // lastReceivedTime = currentReceivedTime;
        // // std::cout << "Time since last data: " << duration << " ms" << std::endl;
        // printf("Time since last data:  :%ld ms\r\n", duration);
        // if (sendDataWaitTime > 0)
        // {
        //     std::this_thread::sleep_for(std::chrono::milliseconds(sendDataWaitTime)); // 20Hz
        // }
    }
}

int main()
{

    std::string arm_ip;
    int arm_port = 0;
    std::string serial_name;
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

    // 配置串口
    // int fd = ConfigureSerialPort(serial_name.c_str());
    int fd = openSerialPort(serial_name.c_str());
    if (fd == -1)
    {
        return 1;
    }

    // 配置串口波特率为460800
    if (configureSerialPort(fd, B115200) < 0)
    {
        close(fd);
        return 1;
    }


    // AGV
    // std::string addr = "169.254.128.2";
    // uint32_t port = 5410;
    // WooshRobotControl robot_control(addr, port);
    // // 初始化
    // robot_control.initialize();

    // 启动100Hz的发送和接收线程
    // std::thread commThread(communicationThread, fd, arm_dof);
    std::thread commThread(communicationThread, fd, robot_handle, arm_dof);
    commThread.join();
    std::cout << "Thread start!" << std::endl;
    close(fd);

    return 0;
}
