#ifndef AGVCONTROLLER_H
#define AGVCONTROLLER_H

#include <thread>
#include "io_state_listener.h"

class AgvController
{
private:
    std::thread thread_;
    bool stop_thread_;                   // 停止线程
    std::string ip_;                     // AGV IP
    int port_;                           // AGV端口
    float linear_speed_;                 // AGV线速度
    float angular_speed_;                // AGV角速度
    IoStateListener &io_state_listener_; // 按键IO Listener

public:
    /// @brief AGVController构造函数
    /// @param ip AGV IP
    /// @param port AGV 端口
    /// @param linear_speed AGV线速度
    /// @param angular_speed AGV角速度
    /// @param io_state_listener 按键IO Listener
    AgvController(std::string ip, int port, float linear_speed, float angular_speed, IoStateListener &io_state_listener);
    ~AgvController();
    /// @brief 开启线程监听按键控制AGV
    void Start();
    /// @brief 停止线程
    void Stop();
    /// @brief 实时获取IO状态对AGV进行控制
    void Run();
};

#endif AGVCONTROLLER_H