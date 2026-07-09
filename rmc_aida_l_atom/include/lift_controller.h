#ifndef LIFTCONTROLLER_H
#define LIFTCONTROLLER_H

#include <thread>
#include "io_state_listener.h"
#include "rm_service.h"

class LiftController
{
private:
    std::thread thread_;
    bool stop_thread_;
    std::string ip_;                     // 升降控制器IP
    int port_;                           // 升降控制器端口号
    int lift_speed_;                     // 升降控制速度
    RM_Service robotic_arm;              // 机械臂RM_Service
    IoStateListener &io_state_listener_; // 按键IO状态监听器

public:
    /// @brief 升降Controller构造函数
    /// @param robotic_arm 机械臂RM_Service
    /// @param ip 升降控制器IP
    /// @param port 升降控制器端口号
    /// @param lift_speed 升降控制速度
    /// @param io_state_listener 按键IO状态监听器
    LiftController(RM_Service &robotic_arm, std::string ip, int port, int lift_speed, IoStateListener &io_state_listener);
    ~LiftController();
    /// @brief 启动线程
    void Start();
    /// @brief 关闭线程
    void Stop();
    /// @brief 根据相应按键IO状态控制升降
    void Run();
};

#endif LIFTCONTROLLER_H