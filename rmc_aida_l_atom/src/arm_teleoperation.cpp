#include "arm_teleoperation.h"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
// #include <arpa/inet.h>
#include <jsoncpp/json/json.h>
#include <iostream>
// #include <thread>
#include <chrono>
#include <fstream>
#include <cstring>
// #include "rm_service.h"
#include "serial_port.h"
#include "data_broadcast.h"
#include "data_filter.h"

ArmTeleoperation::ArmTeleoperation(std::string slave_arm_ip,
                                   int slave_arm_port,
                                   std::string master_arm_port_name,
                                   unsigned int master_arm_baud_rate,
                                   int slave_arm_follow_mode,
                                   int udp_broadcast_port,
                                   std::string udp_broadcast_ip,
                                   std::string tag)
    : slave_arm_ip(slave_arm_ip),
      slave_arm_port(slave_arm_port),
      master_arm_port_name(master_arm_port_name),
      master_arm_baud_rate(master_arm_baud_rate),
      slave_arm_follow_mode(slave_arm_follow_mode),
      udp_broadcast_port(udp_broadcast_port),
      udp_broadcast_ip(udp_broadcast_ip),
      tag(tag),
      running(false)
{
    int result = -1;
    // robotic_arm.rm_set_log_call_back(custom_arm_api_log, 3);
    robotic_arm.rm_set_log_call_back(NULL, 3);
    result = robotic_arm.rm_init(RM_DUAL_MODE_E);
    if (result != 0)
    {
        printf("Initialization failed with error code: %d.\n", result);
        return;
    }
    // thread_ = std::thread(&ArmTeleoperation::Run, this);
    // thread_.detach();
}

uint8_t ArmTeleoperation::CRC8(const uint8_t *buf, size_t len)
{
    uint8_t crc = 0;

    while (len--)
    {
        crc = CRC8TAB[crc ^ *buf];
        buf++;
    }

    return crc;
}

// 校验协议格式
bool ArmTeleoperation::isValidProtocol(const std::vector<uint8_t> &data)
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

void ArmTeleoperation::custom_arm_api_log(const char *message, va_list args)
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

void ArmTeleoperation::Run()
{
    using namespace std::chrono;

    // running = true;
    // RM_Service robotic_arm;
    // int result = -1;
    // // robotic_arm.rm_set_log_call_back(custom_arm_api_log, 3);
    // robotic_arm.rm_set_log_call_back(NULL, 3);
    // result = robotic_arm.rm_init(RM_DUAL_MODE_E);
    // if (result != 0)
    // {
    //     printf("Initialization failed with error code: %d.\n", result);
    //     return;
    // }
    rm_robot_handle *arm_handle = robotic_arm.rm_create_robot_arm((char *)slave_arm_ip.c_str(), slave_arm_port);
    if (arm_handle == NULL)
    {
        printf("Failed to create robot handle.\n");
        return;
    }
    else
    {
        printf("Robot handle created successfully: %d\n", arm_handle->id);
    }
    char *api_version = robotic_arm.rm_api_version();
    printf("API Version: %s\n", api_version);

    printf("slave_arm_ip: %s\n", slave_arm_ip.c_str());

    // int result = -1;
    // rm_arm_software_version_t software_info;
    // result = robotic_arm.rm_get_arm_software_info(arm_handle, &software_info);
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

    rm_robot_info_t arm_info;
    robotic_arm.rm_get_robot_info(arm_handle, &arm_info);
    int arm_dof = arm_info.arm_dof;

    // 设置工具端电源输出24V
    robotic_arm.rm_set_tool_voltage(arm_handle, 3);

    // 配置串口
    SerialPort master_arm_port(master_arm_port_name, master_arm_baud_rate);
    if (!master_arm_port.OpenPort())
    {
        std::cerr << "Failed to Open serial!" << std::endl;
        return;
    }

    // 设置网络通信
    DataBroadcast master_arm_data_broadcast(udp_broadcast_ip, udp_broadcast_port);
    if (!master_arm_data_broadcast.Init())
    {
        std::cerr << "Failed to create socket" << std::endl;
        return;
    }

    usleep(2000000); // 100 ms
    // 第2步
    master_arm_port.SendData({0x02, 0x18, 0x00, 0x00});
    // 等待一段时间确保数据可以接收
    usleep(100000); // 100 ms
    master_arm_port.SendData({0x02, 0x18, 0x00, 0x00});
    usleep(100000); // 100 ms
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    master_arm_port.SendData({0x55, 0xAA, 0x01, 0x00, 0x00, 0xAD});
    // 等待一段时间确保数据可以接收
    usleep(100000); // 100 ms
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
        usleep(100000); // 100 ms
        master_arm_port.SendData({0x55, 0xAA, 0x01, 0x00, 0x00, 0xAD});
        std::cout << "SendData ok!" << std::endl;
        // 等待一段时间确保数据可以接收
        usleep(100000); // 100 ms
    }
    usleep(100000); // 100 ms

    DataFilter DataFilter;

    while (true)
    {
        if (!running)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 100Hz
            continue;
        }

        // 机械臂与主臂状态同步
        master_arm_port.SendData({0x55, 0xAA, 0x02, 0x00, 0x00, 0x67});
        usleep(100000); // 100 ms
        auto received_data = master_arm_port.ReceiveData();
        if (received_data.empty())
        {
            std::cerr << "No data is received from serial port!" << std::endl;
            return;
        }

        std::cout << "Received data: ";
        for (auto byte : received_data)
        {
            std::cout << std::hex << static_cast<int>(byte) << " ";
        }
        // std::cout << received_data.size();
        std::cout << std::endl;
        if (received_data.size() < 42 || received_data[0] != 0xaa || received_data[1] != 0x55 || received_data[2] != 0x02 || received_data[4] != 0x00) // 关节信息数据
        {
            std::cerr << "Error: Invalid protocol format!" << std::endl;
            return;
        }

        uint8_t master_dof = received_data[5] / 5 - 1; // 主臂的自由度(减去1个夹爪的)

        float joints[arm_dof] = {0};

        joints[0] = ((received_data[10] << 24) | (received_data[9] << 16) | (received_data[8] << 8) | received_data[7]) / 10000.0;
        joints[1] = ((received_data[5 + 10] << 24) | (received_data[5 + 9] << 16) | (received_data[5 + 8] << 8) | received_data[5 + 7]) / 10000.0;

        int index = 0;
        size_t p = 2;
        for (; p < master_dof; p++)
        {
            if (received_data[p * 5 + 6] != 1) // joint i不在线
            {
                continue;
            }

            if (master_dof == arm_dof) // 主从臂自由度相同
            {
                joints[p] = ((received_data[p * 5 + 10] << 24) | (received_data[p * 5 + 9] << 16) | (received_data[p * 5 + 8] << 8) | received_data[p * 5 + 7]) / 10000.0;
            }
            else if (master_dof < arm_dof) // 6轴主臂控制7轴从臂
            {
                joints[p + 1] = ((received_data[p * 5 + 10] << 24) | (received_data[p * 5 + 9] << 16) | (received_data[p * 5 + 8] << 8) | received_data[p * 5 + 7]) / 10000.0;
            }
            else // 7轴主臂控制6轴从臂
            {
                index = p + 1;
                joints[p] = ((received_data[index * 5 + 10] << 24) | (received_data[index * 5 + 9] << 16) | (received_data[index * 5 + 8] << 8) | received_data[index * 5 + 7]) / 10000.0;
            }
        }
        int gripper_deg = (received_data[p * 5 + 10] << 24) | (received_data[p * 5 + 9] << 16) | (received_data[p * 5 + 8] << 8) | received_data[p * 5 + 7];

        printf("gripper_deg :%d\r\n", gripper_deg);

        std::cout << "Servo joint: ";
        for (size_t i = 0; i < arm_dof; i++)
        {
            std::cout << "joint[" << i << "]: " << joints[i] << " ";
        }
        std::cout << std::endl;

        // robotic_arms.push_back(robotic_arm);
        // salve_arm_handles.push_back(salve_arm_handle);

        printf("salve_arm_handle->id :%d\r\n", arm_handle->id);

        robotic_arm.rm_set_gripper_position(arm_handle, gripper_deg, false, 5);

        int ret = robotic_arm.rm_movej(arm_handle, joints, 10, 0, 0, 0);
        if (ret != 0)
        {
            printf("Movej_Cmd :%d\r\n", ret);
            continue;
        }

        float arm_degree[arm_dof];
        float arm_degree_offset = 0.0;
        bool set_arm_stop = false;
        do
        {
            arm_degree_offset = 0.0;
            std::this_thread::sleep_for(milliseconds(100)); // 等待一段时间确保数据可以接收
            if (robotic_arm.rm_get_joint_degree(arm_handle, arm_degree) != 0)
            {
                return;
            }
            for (size_t i = 0; i < arm_dof; i++)
            {
                arm_degree_offset = arm_degree_offset + (std::abs(arm_degree[i] - joints[i]));
            }
            if (!running)
            {
                robotic_arm.rm_set_arm_stop(arm_handle);
                set_arm_stop = true;
                std::this_thread::sleep_for(milliseconds(1000));
                break;
            }
            // std::cout << "arm_degree_off: " << arm_degree_offset << std::endl;
            // printf("arm_degree_offset :%f\r\n", arm_degree_offset);
        } while (arm_degree_offset > 5);

        std::this_thread::sleep_for(milliseconds(500)); // 等待一段时间确保数据可以接收
        if (set_arm_stop)
        {
            continue;
        }
        

        int gripper_ot = 0; // 夹爪控制计数，30毫秒控制一次
        int sendDataWaitTime = 10;

        std::vector<uint8_t> receivedDataBuffer; // 存储接收到的数据缓冲区

        auto lastSendArmCanDataTime = steady_clock::now();

        printf("****handleMasterSlaveArmControl****arm_handle->id: %d    arm_dof: %d\r\n", arm_handle->id, arm_dof);

        DataFilter.ResetFilterData();

        while (running)
        {
            auto requestDataTime = steady_clock::now();
            master_arm_port.SendData({0x55, 0xAA, 0x02, 0x00, 0x00, 0x67});
            std::this_thread::sleep_for(milliseconds(3)); // 等待一段时间确保数据可以接收
            bool isDataComplete = false;

            while (!isDataComplete)
            {
                std::this_thread::sleep_for(milliseconds(1)); // 1000Hz，稍微等待一下以减少CPU占用

                // 接收数据
                auto received_data = master_arm_port.ReceiveData();
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
                        // std::cout << "completePacket data: ";
                        // for (auto byte : completePacket)
                        // {
                        //     std::cout << std::hex << static_cast<int>(byte) << " ";
                        // }
                        // std::cout << std::endl;

                        // uint8_t master_dof = lengthField / 5; // 主臂的自由度

                        float joint[arm_dof] = {0};
                        float joint_filter[arm_dof] = {0};

                        Json::Value root;
                        Json::Value jointArray(Json::arrayValue);

                        joint[0] = ((completePacket[10] << 24) | (completePacket[9] << 16) | (completePacket[8] << 8) | completePacket[7]) / 10000.0;
                        joint_filter[0] = DataFilter.Refilter(0, joint[0], 10);
                        jointArray.append(joint_filter[0]);

                        joint[1] = ((completePacket[5 + 10] << 24) | (completePacket[5 + 9] << 16) | (completePacket[5 + 8] << 8) | completePacket[5 + 7]) / 10000.0;
                        joint_filter[1] = DataFilter.Refilter(1, joint[1], 10);
                        jointArray.append(joint_filter[1]);

                        int index = 0;
                        size_t i = 2;
                        for (; i < master_dof; i++)
                        {
                            if (completePacket[i * 5 + 6] != 1) // joint i不在线
                            {
                                jointArray.append(0);
                                continue;
                            }

                            if (master_dof == arm_dof) // 主从臂自由度相同
                            {
                                joint[i] = ((completePacket[i * 5 + 10] << 24) | (completePacket[i * 5 + 9] << 16) | (completePacket[i * 5 + 8] << 8) | completePacket[i * 5 + 7]) / 10000.0;
                                joint_filter[i] = DataFilter.Refilter(i, joint[i], 10);
                            }
                            else if (master_dof < arm_dof) // 6轴主臂控制7轴从臂
                            {
                                joint[i + 1] = ((completePacket[i * 5 + 10] << 24) | (completePacket[i * 5 + 9] << 16) | (completePacket[i * 5 + 8] << 8) | completePacket[i * 5 + 7]) / 10000.0;
                                joint_filter[i + 1] = DataFilter.Refilter(i + 1, joint[i + 1], 10);
                            }
                            else // 7轴主臂控制6轴从臂
                            {
                                index = i + 1;
                                joint[i] = ((completePacket[index * 5 + 10] << 24) | (completePacket[index * 5 + 9] << 16) | (completePacket[index * 5 + 8] << 8) | completePacket[index * 5 + 7]) / 10000.0;
                                joint_filter[i] = DataFilter.Refilter(i, joint[i], 10);
                            }

                            // Populate the joint array
                            jointArray.append(joint_filter[i]);
                        }
                        int gripper_deg = (completePacket[i * 5 + 10] << 24) | (completePacket[i * 5 + 9] << 16) | (completePacket[i * 5 + 8] << 8) | completePacket[i * 5 + 7];

                        auto now = std::chrono::high_resolution_clock::now();
                        auto duration_microseconds = now.time_since_epoch();
                        auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(duration_microseconds).count();

                        root["tag"] = tag;
                        root["joint"] = jointArray;
                        root["gripper"] = gripper_deg;
                        root["timestamp"] = static_cast<Json::Int64>(timestamp);

                        // Convert to JSON string
                        // Json::StreamWriterBuilder writer;
                        Json::FastWriter writer;
                        // std::string jsonString = Json::writeString(writer, root);
                        std::string jsonString = writer.write(root).substr(0, writer.write(root).length() - 1);

                        // std::cout << jsonString << std::endl;
                        master_arm_data_broadcast.SendNetworkData(jsonString);

                        gripper_ot++;
                        if (gripper_ot > 3)
                        {
                            gripper_ot = 0;
                            // Set_Gripper_Position(m_sockhand, gripper_deg, false, 0);
                            // robotic_arm.rm_set_gripper_position(arm_handle, gripper_deg, false, 0);
                            // 创建一个新的线程控制夹爪避免阻塞等待
                            // std::thread(set_gripper_position, arm_handle, gripper_deg).detach();
                            std::thread(&ArmTeleoperation::SetGripperPosition, this, std::ref(robotic_arm), arm_handle, gripper_deg).detach();
                            // gripper_position_callback(gripper_deg);
                            // printf("gripper_deg :%d\r\n", gripper_deg);
                        }

                        // std::cout << "Servo joint: ";
                        // for (size_t i = 0; i < 6; i++)
                        // {
                        //     std::cout << "joint[" << i << "]: " << joint[i] << " " << "filter[" << i << "]: " << joint_filter[i] << " ";
                        // }
                        // std::cout << std::endl;
                        // int ret = Movej_CANFD(m_sockhand, joint_filter, slave_arm_follow_mode, 0);
                        // std::cout << "joint[5]: " << joint[4] << " " << "filter[5]: " << joint_filter[4] << " " << std::endl;
                        // std::cout << "joint5: " << joint[4] << " " << std::endl;
                        // robotic_arm.rm_movej_follow(arm_handle, joint_filter);
                        // robotic_arm.rm_movej_follow(arm_handle, joint);
                        if (slave_arm_follow_mode == 1)
                        {
                            robotic_arm.rm_movej_canfd(arm_handle, joint_filter, true, 0);
                        }
                        else if (slave_arm_follow_mode == 0)
                        {
                            robotic_arm.rm_movej_canfd(arm_handle, joint_filter, false, 0);
                        }
                        else
                        {
                            robotic_arm.rm_movej_follow(arm_handle, joint_filter);
                            // robotic_arm.rm_movej_follow(arm_handle, joint);
                        }

                        auto sendArmCanDataTime = steady_clock::now();
                        int64_t usedTime = duration_cast<milliseconds>(sendArmCanDataTime - requestDataTime).count();
                        int64_t duration = duration_cast<milliseconds>(sendArmCanDataTime - lastSendArmCanDataTime).count();
                        lastSendArmCanDataTime = sendArmCanDataTime;
                        // printf("usedTime: %ld ms\r\n", usedTime);
                        // printf("Time since last data:  :%ld ms\r\n", duration);

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

    //
    robotic_arm.rm_delete_robot_arm(arm_handle);
    master_arm_port.ClosePort();
    // robotic_arm.rm_destory();
}

void ArmTeleoperation::Start()
{
    running = true;
}

void ArmTeleoperation::Stop()
{
    running = false;
}

void ArmTeleoperation::ThreadInit()
{
    thread_ = std::thread(&ArmTeleoperation::Run, this);
    // thread_.join();
}

// 设置夹爪开合度
void ArmTeleoperation::SetGripperPosition(RM_Service &rm_service, rm_robot_handle *arm_handle, int position)
{
    rm_service.rm_set_gripper_position(arm_handle, position, false, 0);
}

void ArmTeleoperation::SetGripperPositionCallback(GripperPositionCallback callback)
{
    gripper_position_callback = callback;
}