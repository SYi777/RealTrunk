#ifndef SLAVEARMCONTROLLER_H
#define SLAVEARMCONTROLLER_H

#include <thread>
#include <atomic>
#include "master_arm_listener.h"
#include "rm_service.h"

class SlaveArmController
{
private:
    std::thread thread_;
    // bool stop_thread_;
    std::atomic<bool> running;               // 是否开始遥操作
    std::string ip_;                         // 机械臂IP地址
    int port_;                               // 机械臂端口号
    int follow_mode_;                        // 0为透传低跟随，1为透传高跟随，2为跟随运动1
    int udp_broadcast_port_;                 // UDP广播发送透传控制数据的端口号
    std::string udp_broadcast_ip_;           // UDP广播数据特定网段的内网发送
    std::string tag_;                        // 广播数据中的标签
    RM_Service robotic_arm;                  // 机械臂服务接口
    int gripper_enable;                      // 是否启用因时夹爪控制，1为启用，0为不启用
    MasterArmListener &master_arm_listener_; // 主臂数据Listener
    int filter_depth_;                       // 主臂数据滤波深度
    int calibrate_speed_;

public:
    /// @brief 从臂Controller构造函数
    /// @param robotic_arm 机械臂RM_Service
    /// @param ip 机械臂IP
    /// @param port 机械臂端口号
    /// @param follow_mode 跟随模式
    /// @param udp_broadcast_port UDP广播端口号
    /// @param tag 广播数据标签
    /// @param filter_depth 滤波深度
    /// @param master_arm_listener 主臂监听器
    SlaveArmController(RM_Service &robotic_arm, std::string ip, int port, int follow_mode, int udp_broadcast_port, std::string udp_broadcast_ip,\
        int calibrate_speed, std::string tag, int filter_depth, int gripper_enable, MasterArmListener &master_arm_listener);
    ~SlaveArmController();
    /// @brief 初始化线程
    void ThreadInit();
    /// @brief 开始遥操作
    void Start();
    /// @brief 停止遥操作
    void Stop();
    /// @brief 线程函数，获取主臂数据处理后透传控制从臂
    void Run();
    /// @brief 设置夹爪位置
    /// @param robotic_arm 机械臂API Service
    /// @param arm_handle 机械臂handle
    /// @param position 夹爪位置
    void SetGripperPosition(RM_Service &robotic_arm, rm_robot_handle *arm_handle, int position);
};

#endif SLAVEARMCONTROLLER_H