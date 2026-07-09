/**
 * 升降Controller，速度可调
 */
#include "lift_controller.h"
#include <iostream>

LiftController::LiftController(RM_Service &robotic_arm, std::string ip, int port, int lift_speed, IoStateListener &io_state_listener)
    : robotic_arm(robotic_arm),
      ip_(ip),
      port_(port),
      lift_speed_(lift_speed),
      io_state_listener_(io_state_listener),
      stop_thread_(false)
{
}

LiftController::~LiftController()
{
}

void LiftController::Start()
{
    thread_ = std::thread(&LiftController::Run, this);
}

void LiftController::Stop()
{
    stop_thread_ = true;
    if (thread_.joinable())
    {
        thread_.join();
    }
}

void LiftController::Run()
{
    int currentLiftSpeed = 0; // 导轨当前运动速度
    int liftSpeed = 0;        // 导轨预期速度
    int lifUpIoNum = 6;       // 导轨上升按键对应的IO口序号
    int lifDownIoNum = 7;     // 导轨下降按键对应的IO口序号
    int headCtrIoNum = 5;     // Head按键对应的IO口序号

    int result = -1;

    rm_robot_handle *robot_handle = robotic_arm.rm_create_robot_arm((char *)ip_.c_str(), port_);
    if (robot_handle == NULL)
    {
        printf("Failed to create robot handle.\n");
        return;
    }
    else
    {
        printf("****LiftAgvTeleoperation****Robot handle created successfully: %d\n", robot_handle->id);
    }

    while (!stop_thread_)
    {
        std::unique_lock<std::mutex> lock(io_state_listener_.GetMutex());
        io_state_listener_.cv.wait(lock);
        std::vector<uint8_t> io_data = io_state_listener_.GetData();
        lock.unlock();

        // 升降控制: 必须按住中键(IO5)才允许上下, 松开即停
        if (io_data[2 + headCtrIoNum * 2] == 1)
        {
            // 中键按下 → 允许升降
            if (io_data[2 + lifUpIoNum * 2] == 1 && io_data[2 + lifDownIoNum * 2] == 0)
            {
                liftSpeed = lift_speed_;           // 仅按上升 → 上升
            }
            else if (io_data[2 + lifDownIoNum * 2] == 1 && io_data[2 + lifUpIoNum * 2] == 0)
            {
                liftSpeed = lift_speed_ * (-1);    // 仅按下降 → 下降
            }
            else
            {
                liftSpeed = 0;                     // 同时按/都没按 → 停
            }
        }
        else
        {
            liftSpeed = 0;                         // 中键松开 → 停
        }

        if (currentLiftSpeed != liftSpeed)
        {
            currentLiftSpeed = liftSpeed;
            robotic_arm.rm_set_lift_speed(robot_handle, currentLiftSpeed);
        }
    }
}