/**
 * 按键IO监听器
 * 1.通过串口实时获取IO状态数据
 * 2.UDP发送IO状态数据
 */
#include "io_state_listener.h"
#include <iostream>
#include <serial_port.h>
#include "data_broadcast.h"


IoStateListener::IoStateListener(std::string port_name, unsigned int baud_rate, int udp_broadcast_port, std::string udp_broadcast_ip)
    : port_name_(port_name),
      baud_rate_(baud_rate),
      udp_broadcast_port_(udp_broadcast_port),
      udp_broadcast_ip_(udp_broadcast_ip),
      stop_thread_(false)
{
}

IoStateListener::~IoStateListener()
{
}

void IoStateListener::Start()
{
    thread_ = std::thread(&IoStateListener::Run, this);
}

void IoStateListener::Stop()
{
    stop_thread_ = true;
    if (thread_.joinable())
    {
        thread_.join();
    }
}

std::vector<uint8_t> IoStateListener::GetData()
{
    // std::lock_guard<std::mutex> lock(mutex_);
    return io_state_data_;
}

std::mutex& IoStateListener::GetMutex()
{
    return mutex_;
}

// 计算Modbus CRC值
uint16_t IoStateListener::calculateCRC(const std::vector<uint8_t> &data)
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
bool IoStateListener::validateCRC(const std::vector<uint8_t> &data)
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
bool IoStateListener::isValidProtocol(const std::vector<uint8_t> &data)
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

void IoStateListener::Run()
{
    // 配置串口
    SerialPort io_port(port_name_, baud_rate_);
    if (!io_port.OpenPort())
    {
        std::cerr << "Failed to Open serial!" << std::endl;
        return;
    }

    // 设置网络通信
    DataBroadcast master_arm_data_broadcast(udp_broadcast_ip_, udp_broadcast_port_);
    if (!master_arm_data_broadcast.Init())
    {
        std::cerr << "Failed to create socket" << std::endl;
        return;
    }

    int sendDataWaitTime = 50;  // 获取IO数据的频率

    using namespace std::chrono;
    auto lastReceivedTime = steady_clock::now();
    int gripper_ot = 0; // 夹爪控制计数，30毫秒控制一次

    std::vector<uint8_t> receivedDataBuffer; // 存储接收到的数据缓冲区

    auto lastSendArmCanDataTime = steady_clock::now();

    while (!stop_thread_)
    {
        auto requestDataTime = steady_clock::now();
        io_port.SendData({0x01, 0x04, 0x00, 0x00, 0x00, 0x10, 0xF1, 0xC6});
        std::this_thread::sleep_for(milliseconds(5)); // 等待一段时间确保数据可以接收
        // sendDataWaitTime -= 13;
        bool isDataComplete = false;

        while (!isDataComplete)
        {
            // 接收数据
            auto receivedData = io_port.ReceiveData();
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
                            {
                                std::lock_guard<std::mutex> lock(mutex_);
                                io_state_data_ = completePacket;
                            }
                            auto now = std::chrono::high_resolution_clock::now();
                            auto duration_microseconds = now.time_since_epoch();
                            auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(duration_microseconds).count();
                            // std::cout << "listener get io data timestamp: " << timestamp << std::endl;
                            cv.notify_all();    // 通知所有等待线程
                            
                            master_arm_data_broadcast.SendNetworkHexData(completePacket);   // UDP广播发送IO数据
                            
                            // std::cout << "completePacket data: ";
                            // for (auto byte : completePacket)
                            // {
                            //     std::cout << std::hex << static_cast<int>(byte) << " ";
                            // }
                            // std::cout << std::endl;

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
                            // printf("Time since last data:  :%ld ms\r\n", duration);

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
            std::this_thread::sleep_for(milliseconds(2)); // 1000Hz，稍微等待一下以减少CPU占用
        }
    }
}