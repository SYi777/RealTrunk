#ifndef IOSTATELISTENER_H
#define IOSTATELISTENER_H

#include <thread>
#include <vector>
#include <mutex>
#include <condition_variable>

class IoStateListener
{
private:
    std::thread thread_;
    std::vector<uint8_t> io_state_data_; // IO状态数据
    std::mutex mutex_;                   // 线程安全mutex
    bool stop_thread_;                   // 关闭线程
    std::string port_name_;              // 串口名称
    unsigned int baud_rate_;             // 串口波特率
    int udp_broadcast_port_;             // UDP广播数据端口号
    std::string udp_broadcast_ip_;       // UDP广播数据特定网段的内网发送
    // 计算CRC值
    uint16_t calculateCRC(const std::vector<uint8_t> &data);
    // CRC校验
    bool validateCRC(const std::vector<uint8_t> &data);
    // 数据协议校验
    bool isValidProtocol(const std::vector<uint8_t> &data);

public:
    IoStateListener(std::string port_name, unsigned int baud_rate, int udp_broadcast_port, std::string udp_broadcast_ip);
    ~IoStateListener();
    std::condition_variable cv;
    /// @brief 启动线程
    void Start();
    /// @brief 停止线程
    void Stop();
    /// @brief 实时获取UDP数据并通过广播发送
    void Run();
    /// @brief 获取一帧完整的IO数据
    /// @return 一帧完整的IO数据
    std::vector<uint8_t> GetData();
    /// @brief 获取线程同步Mutex
    /// @return 线程同步Mutex
    std::mutex &GetMutex();
};

#endif IOSTATELISTENER_H