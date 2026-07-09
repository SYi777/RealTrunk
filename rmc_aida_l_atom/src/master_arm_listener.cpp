/**
 * 主臂数据Listener：通过串口以100HZ的频率获取主臂数据
 */
#include "master_arm_listener.h"
#include <iostream>
#include <unistd.h>
#include <serial_port.h>
#include "data_broadcast.h"

#include <iostream>
#include <chrono>
#include <thread>
#include <time.h>
#include <sys/timerfd.h>


MasterArmListener::MasterArmListener(std::string port_name, unsigned int baud_rate, int udp_broadcast_port, std::string udp_broadcast_ip)
    : port_name_(port_name),
      baud_rate_(baud_rate),
      udp_broadcast_port_(udp_broadcast_port),
      udp_broadcast_ip_(udp_broadcast_ip),
      stop_thread_(false)
{
}

MasterArmListener::~MasterArmListener()
{
}

void MasterArmListener::Start()
{
    thread_ = std::thread(&MasterArmListener::Run, this);
}

void MasterArmListener::Stop()
{
    stop_thread_ = true;
    if (thread_.joinable())
    {
        thread_.join();
    }
}

std::vector<uint8_t> MasterArmListener::GetData()
{
    // std::lock_guard<std::mutex> lock(mutex_);
    return master_arm_data_;
}

std::mutex &MasterArmListener::GetMutex()
{
    return mutex_;
}

uint8_t MasterArmListener::CRC8(const uint8_t *buf, size_t len)
{
    uint8_t crc = 0;

    while (len--)
    {
        crc = CRC8_TAB[crc ^ *buf];
        buf++;
    }

    return crc;
}

// 校验协议格式
bool MasterArmListener::isValidProtocol(const std::vector<uint8_t> &data)
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

void MasterArmListener::Run()
{
    using namespace std::chrono;

    // 配置串口
    SerialPort master_arm_port(port_name_, baud_rate_);
    if (!master_arm_port.OpenPort())
    {
        std::cerr << "Failed to Open serial!" << std::endl;
        return;
    }

    // // 设置网络通信
    // DataBroadcast master_arm_data_broadcast(udp_broadcast_port_);
    // if (!master_arm_data_broadcast.Init())
    // {
    //     std::cerr << "Failed to create socket" << std::endl;
    //     return;
    // }

    master_arm_port.SendData({0x02, 0x18, 0x00, 0x00});
    // 等待一段时间确保数据可以接收
    usleep(100000); // 100 ms
    master_arm_port.SendData({0x02, 0x18, 0x00, 0x00});
    usleep(100000); // 100 ms
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    master_arm_port.SendData({0x55, 0xAA, 0x01, 0x00, 0x00, 0xAD});
    // 等待一段时间确保数据可以接收
    usleep(100000); // 100 ms
    std::cout << "Request get master version!" << std::endl;

    while (true)
    {
        auto received_data = master_arm_port.ReceiveData();
        if (!received_data.empty())
        {
            std::cout << "Get master version data: ";
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

    int sendDataWaitTime = 10;

    std::vector<uint8_t> receivedDataBuffer; // 存储接收到的数据缓冲区

    auto lastSendArmCanDataTime = steady_clock::now();
    auto start = std::chrono::high_resolution_clock::now();
    
    // 模拟耗时操作（这里使用休眠）
    std::this_thread::sleep_for(std::chrono::milliseconds(123));
    
    // 获取结束时间点
    auto end = std::chrono::high_resolution_clock::now();
    
    // 计算时间差（毫秒）
    auto duration_new = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    int fd = timerfd_create(CLOCK_MONOTONIC, 0);
    if (fd == -1) {
        perror("timerfd_create");
        return ;
    }
    struct itimerspec new_value;
        // 首次超时时间为 1ms
        new_value.it_value.tv_sec = 0;
        new_value.it_value.tv_nsec = 10000000;  // 1ms = 1000000ns
        // 后续超时周期为 1ms
        new_value.it_interval.tv_sec = 0;
        new_value.it_interval.tv_nsec = 10000000;

        // 设置定时器
        if (timerfd_settime(fd, 0, &new_value, NULL) == -1) {
            perror("timerfd_settime");
            close(fd);
        }
        uint64_t expirations;
        // int over_times = 0;
        // while (1) {  // 这里设置循环次数，避免无限循环
        //     // 读取定时器事件
        //     ssize_t s = read(fd, &expirations, sizeof(uint64_t));
        //     if (s != sizeof(uint64_t)) {
        //         perror("read");
        //         break;
        //     }
        //     end = std::chrono::high_resolution_clock::now();
    
        //             // 计算时间差（毫秒）
        //     duration_new = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        //     printf("Time since last data:  :%ld ms xxxx\r\n", duration_new.count());

        //     start = end;
        // }
    while (!stop_thread_)
    {
        // 读取定时器事件
        ssize_t s = read(fd, &expirations, sizeof(uint64_t));
        if (s != sizeof(uint64_t)) {
            perror("read");
            break;
        }

        end = std::chrono::high_resolution_clock::now();
    
        // 计算时间差（毫秒）
        duration_new = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        // printf("Time since last data:  :%ld ms xxxx\r\n", duration_new.count());

        start = end;

        auto requestDataTime = steady_clock::now();
        master_arm_port.SendData({0x55, 0xAA, 0x02, 0x00, 0x00, 0x67});
        std::this_thread::sleep_for(milliseconds(5)); // 等待一段时间确保数据可以接收
        bool isDataComplete = false;

        // while (!isDataComplete)
        {
            // 接收数据
            auto received_data = master_arm_port.ReceiveData();
            // printf("recived_data size %d\n",receivedDataBuffer.size());
            if (received_data.empty())
            {
                continue; // 如果没接收到数据就继续接收
            }
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
                    std::cout << "The Received data incomplete, keep waiting! " << std::endl;
                    break; // 数据还不完整，继续等待更多数据
                }

                // 提取完整的协议帧
                std::vector<uint8_t> completePacket(receivedDataBuffer.begin(), receivedDataBuffer.begin() + expectedPacketSize);
                receivedDataBuffer.erase(receivedDataBuffer.begin(), receivedDataBuffer.begin() + expectedPacketSize); // 删除已处理的数据

                // 校验协议
                if (isValidProtocol(completePacket))
                {
                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        master_arm_data_ = completePacket;
                    }
                    auto now = std::chrono::high_resolution_clock::now();
                    auto duration_microseconds = now.time_since_epoch();
                    auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(duration_microseconds).count();
                    // printf("success:\n");
                    // std::cout << "listener get io data timestamp: " << timestamp << std::endl;
                    cv.notify_all(); // 通知所有等待线程

                    isDataComplete = true; // 完整数据已处理，退出接收循环
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