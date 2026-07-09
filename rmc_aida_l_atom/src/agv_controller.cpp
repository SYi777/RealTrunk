/**
 * AGV Controller：
 * 1.支持平滑加减速
 * 2.线速度和角速度可调
 */
#include "agv_controller.h"
#include <iostream>
#include "woosh_robot_control.h"

AgvController::AgvController(std::string ip, int port, float linear_speed, float angular_speed, IoStateListener &io_state_listener)
    : ip_(ip),
      port_(port),
      linear_speed_(linear_speed),
      angular_speed_(angular_speed),
      io_state_listener_(io_state_listener),
      stop_thread_(false)
{
}

AgvController::~AgvController()
{
}

void AgvController::Start()
{
    thread_ = std::thread(&AgvController::Run, this);
}

void AgvController::Stop()
{
    stop_thread_ = true;
    if (thread_.joinable())
    {
        thread_.join();
    }
}

void AgvController::Run()
{
    int agvForwardIoNum = 1; // AGV前进按键对应的IO口序号
    int agvBackIoNum = 2;    // AGV后退按键对应的IO口序号
    int agvLeftIoNum = 3;    // AGV左转按键对应的IO口序号
    int agvRightIoNum = 4;   // AGV右转按键对应的IO口序号
    int agvCentralControllerIoNum = 10;    // AGV中控器控制对应的IO口序号

    WooshRobotControl robot_control(ip_, port_);
    // 初始化
    robot_control.initialize();

    double agvLinear = 0.0;
    double agvAngular = 0.0;

    int hertz = 20;                                   // 控制周期 HZ
    float zero_time = 1.0;                            // 减速归零时间
    int zero_num = zero_time * hertz;                 // 平滑减速指令次数
    float linear_reduce = linear_speed_ / zero_num;   // 线速度次减量
    float angular_reduce = angular_speed_ / zero_num; // 线速度次减量
    int count = 0;                                    // 第几次减速

    float full_speed_time = 2.0;    // 提速的时间
    int full_speed_num = full_speed_time * hertz;
    float linear_increase = linear_speed_ / full_speed_num; // 线速度次增量
    float angular_increase = angular_speed_ / full_speed_num; // 角速度次增量

    while (!stop_thread_)
    {
        std::unique_lock<std::mutex> lock(io_state_listener_.GetMutex());
        io_state_listener_.cv.wait(lock);
        std::vector<uint8_t> io_data = io_state_listener_.GetData();
        lock.unlock();

        // 判断中控器是否被触发
        if (io_data[2 + agvCentralControllerIoNum * 2] != 1) {
            continue;
        }

        // AGV按键控制判断处理
        int agvBtnPressedCount = io_data[2 + agvForwardIoNum * 2] + io_data[2 + agvBackIoNum * 2] + io_data[2 + agvLeftIoNum * 2] + io_data[2 + agvRightIoNum * 2]; // AGV控制按键按下数量
        if (agvBtnPressedCount == 1)                                                                                                                                // 前后左右只有一个按键被按下
        {
            count = 0;

            if (io_data[2 + agvForwardIoNum * 2] == 1) // 前进按键被按下
            {
                std::cout << "AGV go forward" << std::endl;
                // agvLinear = linear_speed_;
                agvAngular = 0.0;
                // 线速度计算
                if (agvLinear < linear_speed_)
                {
                    if (agvLinear < 0)
                    {
                        agvLinear = 0;
                    }
                    
                    agvLinear += linear_increase;
                    if (agvLinear > linear_speed_)
                    {
                        agvLinear = linear_speed_;
                    }
                }
            }
            else if (io_data[2 + agvBackIoNum * 2] == 1) // 后退按键被按下
            {
                std::cout << "AGV go back" << std::endl;
                // agvLinear = (linear_speed_ * (-1));
                agvAngular = 0.0;
                // 线速度计算
                if (agvLinear > (linear_speed_ * (-1)))
                {
                    if (agvLinear > 0)
                    {
                        agvLinear = 0;
                    }
                    agvLinear -= linear_increase;
                    if (agvLinear < (linear_speed_ * (-1)))
                    {
                        agvLinear = linear_speed_ * (-1);
                    }
                }
            }
            else if (io_data[2 + agvLeftIoNum * 2] == 1) // 左转按键被按下
            {
                std::cout << "AGV turn left" << std::endl;
                agvLinear = 0.0;
                // agvAngular = angular_speed_;
                // 角速度计算
                if (agvAngular < angular_speed_)
                {
                    if (agvAngular < 0)
                    {
                        agvAngular = 0;
                    }
                    
                    agvAngular += angular_increase;
                    if (agvAngular > angular_speed_)
                    {
                        agvAngular = angular_speed_;
                    }
                }
            }
            else // 右转按键被按下
            {
                std::cout << "AGV turn right" << std::endl;
                agvLinear = 0.0;
                // agvAngular = (angular_speed_ * (-1));
                // 角速度计算
                if (agvAngular > (angular_speed_ * (-1)))
                {
                    if (agvAngular > 0)
                    {
                        agvAngular = 0;
                    }
                    agvAngular -= angular_increase;
                    if (agvAngular < (angular_speed_ * (-1)))
                    {
                        agvAngular = angular_speed_ * (-1);
                    }
                }
            }
        }
        else if (agvBtnPressedCount == 2) // 前后左右同时有两个按键被按下
        {
            count = 0;

            if (io_data[2 + agvForwardIoNum * 2] == 1 && io_data[2 + agvLeftIoNum * 2] == 1)
            {
                std::cout << "AGV left forward" << std::endl;
                // agvLinear = linear_speed_;
                // agvAngular = angular_speed_;
                // 线速度计算
                if (agvLinear < linear_speed_)
                {
                    if (agvLinear < 0)
                    {
                        agvLinear = 0;
                    }
                    
                    agvLinear += linear_increase;
                    if (agvLinear > linear_speed_)
                    {
                        agvLinear = linear_speed_;
                    }
                }
                // 角速度计算
                if (agvAngular < angular_speed_)
                {
                    if (agvAngular < 0)
                    {
                        agvAngular = 0;
                    }
                    
                    agvAngular += angular_increase;
                    if (agvAngular > angular_speed_)
                    {
                        agvAngular = angular_speed_;
                    }
                }
            }
            else if (io_data[2 + agvForwardIoNum * 2] == 1 && io_data[2 + agvRightIoNum * 2] == 1)
            {
                std::cout << "AGV right forward" << std::endl;
                // agvLinear = linear_speed_;
                // agvAngular = (angular_speed_ * (-1));
                // 线速度计算
                if (agvLinear < linear_speed_)
                {
                    if (agvLinear < 0)
                    {
                        agvLinear = 0;
                    }
                    
                    agvLinear += linear_increase;
                    if (agvLinear > linear_speed_)
                    {
                        agvLinear = linear_speed_;
                    }
                }
                // 角速度计算
                if (agvAngular > (angular_speed_ * (-1)))
                {
                    if (agvAngular > 0)
                    {
                        agvAngular = 0;
                    }
                    agvAngular -= angular_increase;
                    if (agvAngular < (angular_speed_ * (-1)))
                    {
                        agvAngular = angular_speed_ * (-1);
                    }
                }
            }
            else if (io_data[2 + agvBackIoNum * 2] == 1 && io_data[2 + agvRightIoNum * 2] == 1)
            {
                std::cout << "AGV right back" << std::endl;
                // agvLinear = (linear_speed_ * (-1));
                // agvAngular = (angular_speed_ * (-1));
                // 线速度计算
                if (agvLinear > (linear_speed_ * (-1)))
                {
                    if (agvLinear > 0)
                    {
                        agvLinear = 0;
                    }
                    agvLinear -= linear_increase;
                    if (agvLinear < (linear_speed_ * (-1)))
                    {
                        agvLinear = linear_speed_ * (-1);
                    }
                }
                // 角速度计算
                if (agvAngular > (angular_speed_ * (-1)))
                {
                    if (agvAngular > 0)
                    {
                        agvAngular = 0;
                    }
                    agvAngular -= angular_increase;
                    if (agvAngular < (angular_speed_ * (-1)))
                    {
                        agvAngular = angular_speed_ * (-1);
                    }
                }
            }
            else if (io_data[2 + agvBackIoNum * 2] == 1 && io_data[2 + agvLeftIoNum * 2] == 1)
            {
                std::cout << "AGV left back" << std::endl;
                // agvLinear = (linear_speed_ * (-1));
                // agvAngular = angular_speed_;
                // 线速度计算
                if (agvLinear > (linear_speed_ * (-1)))
                {
                    if (agvLinear > 0)
                    {
                        agvLinear = 0;
                    }
                    agvLinear -= linear_increase;
                    if (agvLinear < (linear_speed_ * (-1)))
                    {
                        agvLinear = linear_speed_ * (-1);
                    }
                }
                // 角速度计算
                if (agvAngular < angular_speed_)
                {
                    if (agvAngular < 0)
                    {
                        agvAngular = 0;
                    }
                    
                    agvAngular += angular_increase;
                    if (agvAngular > angular_speed_)
                    {
                        agvAngular = angular_speed_;
                    }
                }
            }
            else
            {
                count = zero_num;
                // std::cout << "AGV stop" << std::endl;
                agvLinear = 0.0;
                agvAngular = 0.0;
            }
        }
        else // 前后左右同时有超过两个按键被按下或一个都没被按下
        {
            // 平滑减速
            if (count < zero_num)
            {
                count++;
                // 线速度计算
                if (agvLinear > 0)
                {
                    agvLinear -= linear_reduce;
                    if (agvLinear < 0)
                    {
                        agvLinear = 0;
                    }
                }
                else
                {
                    agvLinear += linear_reduce;
                    if (agvLinear > 0)
                    {
                        agvLinear = 0;
                    }
                }
                // 角速度计算
                if (agvAngular > 0)
                {
                    agvAngular -= angular_reduce;
                    if (agvAngular < 0)
                    {
                        agvAngular = 0;
                    }
                }
                else
                {
                    agvAngular += angular_reduce;
                    if (agvAngular > 0)
                    {
                        agvAngular = 0;
                    }
                }
            }
            else
            {
                // std::cout << "AGV stop" << std::endl;
                agvLinear = 0.0;
                agvAngular = 0.0;
            }
        }

        // 设置速度
        // if (agvLinear != 0 || agvAngular != 0)
        // {
        //     robot_control.setLinearSpeed(agvLinear);
        //     robot_control.setAngularSpeed(agvAngular);
        // }
        robot_control.setLinearSpeed(agvLinear);
        robot_control.setAngularSpeed(agvAngular);
    }
}