#ifndef LIFTAGVTELEOPERATION_H
#define LIFTAGVTELEOPERATION_H

#include <string>
#include <vector>
#include <thread>
#include <functional>
#include "rm_service.h"



class LiftAgvTeleoperation
{
public:
    using IoDataCallback = std::function<void(const std::vector<uint8_t>&)>;

    LiftAgvTeleoperation(std::string arm_ip,
                     int arm_port,
                     std::string io_port_name,
                     unsigned int io_baud_rate,
                     std::string agv_ip,
                     int agv_port,
                     int udp_broadcast_port,
                     std::string udp_broadcast_ip);

    // 计算Modbus CRC值
    uint16_t calculateCRC(const std::vector<uint8_t> &data);
    // 校验Modbus协议数据的CRC是否正确
    bool validateCRC(const std::vector<uint8_t> &data);
    // 校验协议格式
    bool isValidProtocol(const std::vector<uint8_t> &data);

    void Start();
    void Run();
    void SetIoDataCallback(IoDataCallback callback);

private:
    std::thread thread_;
    std::string arm_ip;
    int arm_port;
    std::string io_port_name;
    unsigned int io_baud_rate;
    std::string agv_ip;
    int agv_port;
    IoDataCallback io_data_callback_;
    RM_Service robotic_arm;
    int udp_broadcast_port;
    std::string udp_broadcast_ip;
};

#endif LIFTAGVTELEOPERATION_H