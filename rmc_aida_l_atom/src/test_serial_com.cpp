#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
// #include <arpa/inet.h>
#include <jsoncpp/json/json.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <fstream>
#include <cstring>
#include "rm_service.h"
#include "serial_port.h"
#include "data_broadcast.h"
#include "data_filter.h"

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

struct MasterSlaveArm
{
    std::string slave_arm_ip;
    int slave_arm_port;
    std::string master_arm_port_name;
    int udp_broadcast_port;
    int slave_arm_follow_mode; // 0为透传低跟随，1为透传高跟随，2为跟随运动1
    std::string tag;
    MasterSlaveArm(std::string slave_arm_ip, int slave_arm_port, std::string master_arm_port_name, int slave_arm_follow_mode,
                   int udp_broadcast_port, std::string tag)
        : slave_arm_ip(slave_arm_ip),
          slave_arm_port(slave_arm_port),
          master_arm_port_name(master_arm_port_name),
          slave_arm_follow_mode(slave_arm_follow_mode),
          udp_broadcast_port(udp_broadcast_port),
          tag(tag)
    {
    }
};

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

// 设置夹爪开合度
// void set_gripper_position(rm_robot_handle *arm_handle, int position)
// {
//     robotic_arm.rm_set_gripper_position(arm_handle, position, false, 0);
// }

// 不断获取主臂数据然后透传控制机械臂
void handleMasterSlaveArmControl(SerialPort &serialPort, DataBroadcast &dataBroadcast, int arm_follow_mode)
{
    using namespace std::chrono;
    int gripper_ot = 0; // 夹爪控制计数，30毫秒控制一次
    int sendDataWaitTime = 10;

    std::vector<uint8_t> receivedDataBuffer; // 存储接收到的数据缓冲区

    auto lastSendArmCanDataTime = steady_clock::now();

    int arm_dof = 6;

    DataFilter DataFilter;

    while (true)
    {
        auto requestDataTime = steady_clock::now();
        serialPort.SendData({0x55, 0xAA, 0x02, 0x00, 0x00, 0x67});
        std::this_thread::sleep_for(milliseconds(3)); // 等待一段时间确保数据可以接收
        bool isDataComplete = false;

        while (!isDataComplete)
        {
            std::this_thread::sleep_for(milliseconds(1)); // 1000Hz，稍微等待一下以减少CPU占用

            // 接收数据
            auto received_data = serialPort.ReceiveData();
            if (received_data.empty())
            {
                continue; // 如果没接收到数据就继续接收
            }

            // std::cout << "Received data: ";
            // for (auto byte : received_data)
            // {
            //     std::cout << std::hex << static_cast<int>(byte) << " ";
            // }
            // // std::cout << received_data.size();
            // std::cout << std::endl;

            // 将新接收到的数据添加到缓冲区
            receivedDataBuffer.insert(receivedDataBuffer.end(), received_data.begin(), received_data.end());

            // 尝试解析缓冲区中的数据
            while (receivedDataBuffer.size() >= 7) // 至少8字节才能构成一个完整的数据帧
            {
                // 查找完整的协议帧
                size_t packetSize = receivedDataBuffer.size();
                // 计算数据帧的总长度
                uint8_t lengthField = receivedDataBuffer[5];
                size_t expectedPacketSize = 6 + lengthField + 1;

                if (packetSize < expectedPacketSize)
                {
                    // std::cout << "The Received data incomplete, keep waiting! " << std::endl;
                    break; // 数据还不完整，继续等待更多数据
                }

                // 提取完整的协议帧
                std::vector<uint8_t> completePacket(receivedDataBuffer.begin(), receivedDataBuffer.begin() + expectedPacketSize);
                receivedDataBuffer.erase(receivedDataBuffer.begin(), receivedDataBuffer.begin() + expectedPacketSize); // 删除已处理的数据

                // 校验协议
                if (isValidProtocol(completePacket))
                {
                    std::cout << "Thread *** " << arm_follow_mode << " *** completePacket data: ";
                    for (auto byte : completePacket)
                    {
                        std::cout << std::hex << static_cast<int>(byte) << " ";
                    }
                    std::cout << std::endl;

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

                            // joint_filter[i] = Refilter(i, joint[i], 64);
                            joint_filter[i] = DataFilter.Refilter(i, joint[i], 10);

                            // Populate the joint array
                            jointArray.append(joint_filter[i]);
                        }
                    }
                    int gripper_deg = (completePacket[i * 5 + 10] << 24) | (completePacket[i * 5 + 9] << 16) | (completePacket[i * 5 + 8] << 8) | completePacket[i * 5 + 7];

                    // root["tag"] = tag;
                    root["joint"] = jointArray;
                    root["gripper"] = gripper_deg;

                    // Convert to JSON string
                    // Json::StreamWriterBuilder writer;
                    Json::FastWriter writer;
                    // std::string jsonString = Json::writeString(writer, root);
                    std::string jsonString = writer.write(root).substr(0, writer.write(root).length() - 1);

                    // std::cout << jsonString << std::endl;
                    dataBroadcast.SendNetworkData(jsonString);

                    gripper_ot++;
                    if (gripper_ot > 3)
                    {
                        gripper_ot = 0;
                        // Set_Gripper_Position(m_sockhand, gripper_deg, false, 0);
                        // robotic_arm.rm_set_gripper_position(arm_handle, gripper_deg, false, 0);
                        // 创建一个新的线程控制夹爪避免阻塞等待
                        // std::thread(set_gripper_position, arm_handle, gripper_deg).detach();
                        // printf("gripper_deg :%d\r\n", gripper_deg);
                    }

                    // std::cout << "Servo joint: ";
                    // for (size_t i = 0; i < 6; i++)
                    // {
                    //     std::cout << "joint[" << i << "]: " << joint[i] << " " << "filter[" << i << "]: " << joint_filter[i] << " ";
                    // }
                    // std::cout << std::endl;
                    // int ret = Movej_CANFD(m_sockhand, joint_filter, l_arm_follow_mode, 0);
                    // std::cout << "joint[5]: " << joint[4] << " " << "filter[5]: " << joint_filter[4] << " " << std::endl;
                    // std::cout << "joint5: " << joint[4] << " " << std::endl;
                    // robotic_arm.rm_movej_follow(arm_handle, joint_filter);
                    // robotic_arm.rm_movej_follow(arm_handle, joint);
                    // if (l_arm_follow_mode == 1)
                    // {
                    //     robotic_arm.rm_movej_canfd(arm_handle, joint_filter, true, 0);
                    // }
                    // else if (l_arm_follow_mode == 0)
                    // {
                    //     robotic_arm.rm_movej_canfd(arm_handle, joint_filter, false, 0);
                    // }
                    // else
                    // {
                    //     robotic_arm.rm_movej_follow(arm_handle, joint_filter);
                    //     // robotic_arm.rm_movej_follow(arm_handle, joint);
                    // }

                    auto sendArmCanDataTime = steady_clock::now();
                    int64_t usedTime = duration_cast<milliseconds>(sendArmCanDataTime - requestDataTime).count();
                    int64_t duration = duration_cast<milliseconds>(sendArmCanDataTime - lastSendArmCanDataTime).count();
                    lastSendArmCanDataTime = sendArmCanDataTime;
                    // printf("usedTime: %ld ms\r\n", usedTime);
                    printf("Thread *** %d *** Time since last data:  :%ld ms\r\n", arm_follow_mode, duration);

                    isDataComplete = true; // 完整数据已处理，退出接收循环

                    if (sendDataWaitTime > usedTime)
                    {
                        std::this_thread::sleep_for(std::chrono::milliseconds(sendDataWaitTime - usedTime - 1)); // 100Hz
                    }
                    // return;
                }
                else
                {
                    std::cout << "Error: Invalid protocol format" << std::endl;
                    receivedDataBuffer.clear(); // 清空缓冲区，重新开始接收
                    isDataComplete = true;      // 防止死循环
                }
            }
        }
    }
}

int main(int argc, char *argv[])
{
    std::vector<MasterSlaveArm> master_slave_arms = {MasterSlaveArm("169.254.128.18", 8080, "/dev/ttyUSB0", 0, 8888, "left"),
                                                     MasterSlaveArm("169.254.128.19", 8080, "/dev/ttyUSB1", 1, 9999, "right")};

    // std::vector<MasterSlaveArm> master_slave_arms = {MasterSlaveArm("169.254.128.19", 8080, "/dev/ttyUSB0", 1, 9999, "left")};

    // std::vector<SerialPort> master_arm_ports = {SerialPort(master_slave_arms[0].master_arm_port_name, 460800), SerialPort(master_slave_arms[1].master_arm_port_name, 460800)};
    std::vector<SerialPort> master_arm_ports;
    std::vector<DataBroadcast> master_arm_data_broadcasts = {DataBroadcast(master_slave_arms[0].udp_broadcast_port), DataBroadcast(master_slave_arms[1].udp_broadcast_port)};

    std::vector<std::thread> atom_threads;
    // MasterSlaveArm l_master_slave_arm = MasterSlaveArm("192.168.1.18", 8080, "/dev/ttyUSB0", 1, 8888, "left");
    // MasterSlaveArm r_master_slave_arm = MasterSlaveArm("192.168.1.19", 8080, "/dev/ttyUSB1", 1, 9999, "right");
    // std::string l_arm_ip = "169.254.128.18";
    // std::string l_arm_ip = "192.168.1.18";
    // std::string r_arm_ip = "192.168.1.19";
    // int l_arm_port = 8080;
    // int r_arm_port = 8080;
    // std::string l_m_arm_port_name = "/dev/ttyUSB0";
    // std::string r_m_arm_port_name = "/dev/ttyUSB1";
    // int l_udp_port = 8888;
    // int r_udp_port = 9999;
    // int l_arm_follow_mode = 1; // 0为透传低跟随，1为透传高跟随，2为跟随运动1
    // int r_arm_follow_mode = 1; // 0为透传低跟随，1为透传高跟随，2为跟随运动1
    // std::string tag = "arm";

    for (int i = 0; i < master_slave_arms.size(); i++)
    // for (auto &arm : master_slave_arms)
    {
        // RM_Service robotic_arm;

        // int result = -1;

        // robotic_arm.rm_set_log_call_back(custom_api_log, 3);
        // result = robotic_arm.rm_init(RM_DUAL_MODE_E);
        // if (result != 0)
        // {
        //     printf("Initialization failed with error code: %d.\n", result);
        //     return -1;
        // }

        // char *api_version = robotic_arm.rm_api_version();
        // printf("API Version: %s.\n", api_version);

        
        // printf("slave_arm_ip: %s.\n", arm.slave_arm_ip.c_str());
        // rm_robot_handle *salve_arm_handle = robotic_arm.rm_create_robot_arm((char *)arm.slave_arm_ip.c_str(), arm.slave_arm_port);
        // if (salve_arm_handle == NULL)
        // {
        //     printf("Failed to create robot handle.\n");
        //     continue;
        // }
        // else
        // {
        //     printf("Robot handle created successfully: %d\n", salve_arm_handle->id);
        // }
        // rm_arm_software_version_t software_info;
        // result = robotic_arm.rm_get_arm_software_info(salve_arm_handle, &software_info);
        // if (result == 0)
        // {
        //     printf("================== Arm Software Information ==================\n");
        //     printf("Arm Model:  %s\n", software_info.product_version);
        //     printf("Algorithm Library Version:  %s\n", software_info.algorithm_info.version);
        //     printf("Control Layer Software Version:  %s\n", software_info.ctrl_info.version);
        //     printf("Dynamics Version:  %s\n", software_info.dynamic_info.model_version);
        //     printf("Planning Layer Software Version:  %s\n", software_info.plan_info.version);
        //     printf("==============================================================\n");
        // }
        // // 设置工具端电源输出24V
        // robotic_arm.rm_set_tool_voltage(salve_arm_handle, 3);

        // 配置串口
        SerialPort master_arm_port(master_slave_arms[i].master_arm_port_name, B460800);

        if (!master_arm_port.OpenPort())
        {
            std::cerr << "Failed to Open serial!" << std::endl;
            continue;
        }

        master_arm_ports.push_back(master_arm_port);
        // std::cout << "Port name: " << master_arm_ports[i].port_name_ << std::endl;

        // 设置网络通信
        // DataBroadcast master_arm_data_broadcast(arm.udp_broadcast_port);
        if (!master_arm_data_broadcasts[i].Init())
        {
            std::cerr << "Failed to create socket" << std::endl;
            continue;
        }
        // master_arm_data_broadcasts.push_back(master_arm_data_broadcast);

        // usleep(2000000); // 100 ms
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // 第2步
        master_arm_port.SendData({0x02, 0x18, 0x00, 0x00});
        // 等待一段时间确保数据可以接收
        // usleep(100000); // 100 ms
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        master_arm_port.SendData({0x02, 0x18, 0x00, 0x00});
        // usleep(100000); // 100 ms
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        master_arm_port.SendData({0x55, 0xAA, 0x01, 0x00, 0x00, 0xAD});
        // 等待一段时间确保数据可以接收
        // usleep(100000); // 100 ms
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << "SendData ok!" << std::endl;

        while (true)
        {
            auto received_data = master_arm_port.ReceiveData();
            if (!received_data.empty())
            {
                std::cout << "Received data: ";
                for (auto byte : received_data)
                {
                    std::cout << std::hex << static_cast<int>(byte) << " ";
                }
                std::cout << std::endl;
                // 继续往下执行
                break;
            }
            // usleep(100000); // 100 ms
            // std::this_thread::sleep_for(std::chrono::seconds(1));
            master_arm_port.SendData({0x02, 0x18, 0x00, 0x00});
            // std::this_thread::sleep_for(std::chrono::seconds(1));
            // usleep(100000); // 100 ms
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            master_arm_port.SendData({0x55, 0xAA, 0x01, 0x00, 0x00, 0xAD});
            std::cout << "SendData ok!" << std::endl;
            // 等待一段时间确保数据可以接收
            // usleep(100000); // 100 ms
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        // usleep(100000); // 100 ms
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        // 机械臂与主臂状态同步
        master_arm_port.SendData({0x55, 0xAA, 0x02, 0x00, 0x00, 0x67});
        // usleep(100000); // 100 ms
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        auto received_data = master_arm_port.ReceiveData();
        if (received_data.empty())
        {
            std::cerr << "No data is received from serial port!" << std::endl;
            continue;
        }

        std::cout << "Received data: ";
        for (auto byte : received_data)
        {
            std::cout << std::hex << static_cast<int>(byte) << " ";
        }
        // std::cout << received_data.size();
        std::cout << std::endl;
        if (received_data.size() >= 42 && received_data[0] == 0xaa && received_data[1] == 0x55 && received_data[2] == 0x02 && received_data[4] == 0x00) // 关节信息数据
        {

            float joint[ARM_DOF] = {0};
            int i = 0;
            for (; i < ARM_DOF; i++)
            {
                if (received_data[i * 5 + 6] == 1) // joint i在线
                {
                    joint[i] = ((received_data[i * 5 + 10] << 24) | (received_data[i * 5 + 9] << 16) | (received_data[i * 5 + 8] << 8) | received_data[i * 5 + 7]) / 10000.0;
                }
            }
            int gripper_deg = (received_data[i * 5 + 10] << 24) | (received_data[i * 5 + 9] << 16) | (received_data[i * 5 + 8] << 8) | received_data[i * 5 + 7];
            printf("gripper_deg :%d\r\n", gripper_deg);

            // robotic_arm.rm_set_gripper_position(salve_arm_handle, gripper_deg, false, 5);

            // int ret = robotic_arm.rm_movej(salve_arm_handle, joint, 10, 0, 0, 1);
            // if (ret != 0)
            // {
            //     printf("Movej_Cmd :%d\r\n", ret);
            //     // return;
            // }
            // atom_threads.push_back(std::thread(handleMasterSlaveArmControl, std::ref(master_arm_ports[i]), std::ref(master_arm_data_broadcasts[i]), master_slave_arms[i].slave_arm_follow_mode));
        }
    }

    atom_threads.push_back(std::thread(handleMasterSlaveArmControl, std::ref(master_arm_ports[0]), std::ref(master_arm_data_broadcasts[0]), master_slave_arms[0].slave_arm_follow_mode));
    atom_threads.push_back(std::thread(handleMasterSlaveArmControl, std::ref(master_arm_ports[1]), std::ref(master_arm_data_broadcasts[1]), master_slave_arms[1].slave_arm_follow_mode));


    for (auto &t : atom_threads)
    {
        t.join();
        // t.detach();
    }

    // master_arm_port.ClosePort();
    return 0;
}
