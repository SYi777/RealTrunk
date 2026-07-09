/**
 * 从臂遥操作Controller：
 * 1.支持随时开启/关闭遥操作
 * 2.支持6/7轴主从臂自动适配控制
 * 3.机械臂跟随模式可调，默认高跟随
 * 4.滤波深度可调
 * 5.UDP广播发送透传数据
 */

#include "slave_arm_controller.h"
#include <iostream>
#include <jsoncpp/json/json.h>
#include "data_broadcast.h"
#include "data_filter.h"


SlaveArmController::SlaveArmController(RM_Service &robotic_arm,
                                       std::string ip, int port,
                                       int follow_mode,
                                       int udp_broadcast_port,
                                       std::string udp_broadcast_ip,
                                       int calibrate_speed,
                                       std::string tag,
                                       int filter_depth, 
                                       int gripper_enable,
                                       MasterArmListener &master_arm_listener)
    : robotic_arm(robotic_arm),
      ip_(ip),
      port_(port),
      follow_mode_(follow_mode),
      udp_broadcast_port_(udp_broadcast_port),
      udp_broadcast_ip_(udp_broadcast_ip),
      calibrate_speed_(calibrate_speed),
      tag_(tag),
      filter_depth_(filter_depth),
      master_arm_listener_(master_arm_listener),
      running(false),
      gripper_enable(gripper_enable)
{
}

SlaveArmController::~SlaveArmController()
{
}

void SlaveArmController::Start()
{
    running = true;
}

void SlaveArmController::Stop()
{
    running = false;
}

void SlaveArmController::ThreadInit()
{
    thread_ = std::thread(&SlaveArmController::Run, this);
    // thread_.join();
}

void SlaveArmController::Run()
{
    using namespace std::chrono;

    /////////////////////
    /// 机械臂连接初始化
    /// udp广播初始化
    /////////////////////
    int result = -1;
    // 机械臂连接
    rm_robot_handle *arm_handle = robotic_arm.rm_create_robot_arm((char *)ip_.c_str(), port_);
    if (arm_handle == NULL)
    {
        printf("Failed to create robot handle.\n");
        return;
    }
    else
    {
        printf("****SlaveArmController****Robot handle created successfully: %d\n", arm_handle->id);
    }

    // 获取机械臂自由度
    rm_robot_info_t arm_info;
    robotic_arm.rm_get_robot_info(arm_handle, &arm_info);
    int arm_dof = arm_info.arm_dof;

    // 设置工具端电源输出24V
    robotic_arm.rm_set_tool_voltage(arm_handle, 3);

    // 设置网络通信
    DataBroadcast master_arm_data_broadcast(udp_broadcast_ip_, udp_broadcast_port_);
    if (!master_arm_data_broadcast.Init())
    {
        std::cerr << "Failed to create socket" << std::endl;
        return;
    }

    // 平滑滤波器
    DataFilter DataFilter;

    while (true)
    {
        
        if (!running)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 100Hz
            continue;
        }

        ////////////////////////
        /// 遥操作考试前主从臂同构
        ////////////////////////

        // 等待主臂数据更新通知获取数据
        std::unique_lock<std::mutex> lock(master_arm_listener_.GetMutex());
        master_arm_listener_.cv.wait(lock);
        std::vector<uint8_t> master_arm_data = master_arm_listener_.GetData();
        lock.unlock();

        uint8_t master_dof = master_arm_data[5] / 5 - 1; // 主臂的自由度(减去1个夹爪的)
        float joints[arm_dof] = {0};

        joints[0] = ((master_arm_data[10] << 24) | (master_arm_data[9] << 16) | (master_arm_data[8] << 8) | master_arm_data[7]) / 10000.0;
        joints[1] = ((master_arm_data[5 + 10] << 24) | (master_arm_data[5 + 9] << 16) | (master_arm_data[5 + 8] << 8) | master_arm_data[5 + 7]) / 10000.0;

        int index = 0;
        size_t p = 2;
        for (; p < master_dof; p++)
        {
            if (master_arm_data[p * 5 + 6] != 1) // joint i不在线
            {
                continue;
            }

            if (master_dof == arm_dof) // 主从臂自由度相同
            {
                joints[p] = ((master_arm_data[p * 5 + 10] << 24) | (master_arm_data[p * 5 + 9] << 16) | (master_arm_data[p * 5 + 8] << 8) | master_arm_data[p * 5 + 7]) / 10000.0;
            }
            else if (master_dof < arm_dof) // 6轴主臂控制7轴从臂
            {
                joints[p + 1] = ((master_arm_data[p * 5 + 10] << 24) | (master_arm_data[p * 5 + 9] << 16) | (master_arm_data[p * 5 + 8] << 8) | master_arm_data[p * 5 + 7]) / 10000.0;
            }
            else // 7轴主臂控制6轴从臂
            {
                index = p + 1;
                joints[p] = ((master_arm_data[index * 5 + 10] << 24) | (master_arm_data[index * 5 + 9] << 16) | (master_arm_data[index * 5 + 8] << 8) | master_arm_data[index * 5 + 7]) / 10000.0;
            }
        }
        int gripper_deg = (master_arm_data[p * 5 + 10] << 24) | (master_arm_data[p * 5 + 9] << 16) | (master_arm_data[p * 5 + 8] << 8) | master_arm_data[p * 5 + 7];
        if (bool(gripper_enable)) {
            robotic_arm.rm_set_gripper_position(arm_handle, gripper_deg, false, 5);
        }

        int ret = robotic_arm.rm_movej(arm_handle, joints, calibrate_speed_, 0, 0, 0);
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

        //////////////////////////////
        /// 获取主臂数据处理后透传控制从臂
        //////////////////////////////

        int gripper_ot = 0; // 夹爪控制计数，30毫秒控制一次
        int sendDataWaitTime = 10;

        std::vector<uint8_t> receivedDataBuffer; // 存储接收到的数据缓冲区

        auto lastSendArmCanDataTime = steady_clock::now();

        // printf("****handleMasterSlaveArmControl****arm_handle->id: %d    arm_dof: %d\r\n", arm_handle->id, arm_dof);

        DataFilter.ResetFilterData();

        while (running)
        {
            std::unique_lock<std::mutex> lock(master_arm_listener_.GetMutex());
            master_arm_listener_.cv.wait(lock);
            master_arm_data = master_arm_listener_.GetData();
            lock.unlock();

            float joint[arm_dof] = {0};
            float joint_filter[arm_dof] = {0};

            Json::Value root;
            Json::Value jointArray(Json::arrayValue);

            joint[0] = ((master_arm_data[10] << 24) | (master_arm_data[9] << 16) | (master_arm_data[8] << 8) | master_arm_data[7]) / 10000.0;
            joint_filter[0] = DataFilter.Refilter(0, joint[0], filter_depth_);
            jointArray.append(joint_filter[0]);

            joint[1] = ((master_arm_data[5 + 10] << 24) | (master_arm_data[5 + 9] << 16) | (master_arm_data[5 + 8] << 8) | master_arm_data[5 + 7]) / 10000.0;
            joint_filter[1] = DataFilter.Refilter(1, joint[1], filter_depth_);
            jointArray.append(joint_filter[1]);

            int index = 0;
            size_t i = 2;
            for (; i < master_dof; i++)
            {
                if (master_arm_data[i * 5 + 6] != 1) // joint i不在线
                {
                    jointArray.append(0);
                    continue;
                }

                if (master_dof == arm_dof) // 主从臂自由度相同
                {
                    joint[i] = ((master_arm_data[i * 5 + 10] << 24) | (master_arm_data[i * 5 + 9] << 16) | (master_arm_data[i * 5 + 8] << 8) | master_arm_data[i * 5 + 7]) / 10000.0;
                    joint_filter[i] = DataFilter.Refilter(i, joint[i], 10);
                }
                else if (master_dof < arm_dof) // 6轴主臂控制7轴从臂
                {
                    joint[i + 1] = ((master_arm_data[i * 5 + 10] << 24) | (master_arm_data[i * 5 + 9] << 16) | (master_arm_data[i * 5 + 8] << 8) | master_arm_data[i * 5 + 7]) / 10000.0;
                    joint_filter[i + 1] = DataFilter.Refilter(i + 1, joint[i + 1], filter_depth_);
                }
                else // 7轴主臂控制6轴从臂
                {
                    index = i + 1;
                    joint[i] = ((master_arm_data[index * 5 + 10] << 24) | (master_arm_data[index * 5 + 9] << 16) | (master_arm_data[index * 5 + 8] << 8) | master_arm_data[index * 5 + 7]) / 10000.0;
                    joint_filter[i] = DataFilter.Refilter(i, joint[i], filter_depth_);
                }

                // Populate the joint array
                jointArray.append(joint_filter[i]);
            }
            int gripper_deg = (master_arm_data[i * 5 + 10] << 24) | (master_arm_data[i * 5 + 9] << 16) | (master_arm_data[i * 5 + 8] << 8) | master_arm_data[i * 5 + 7];

            auto now = std::chrono::high_resolution_clock::now();
            auto duration_microseconds = now.time_since_epoch();
            auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(duration_microseconds).count();

            root["tag"] = tag_;
            root["joint"] = jointArray;
            root["gripper"] = gripper_deg;
            root["timestamp"] = static_cast<Json::Int64>(timestamp);

            // Convert to JSON string
            // Json::StreamWriterBuilder writer;
            Json::FastWriter writer;
            // std::string jsonString = Json::writeString(writer, root);
            std::string jsonString = writer.write(root).substr(0, writer.write(root).length() - 1);

            // std::cout << jsonString << std::endl;
            master_arm_data_broadcast.SendNetworkData(jsonString);  // UDP广播发送封装好的透传控制机械臂的数据

            // 获取主臂的频率大概100HZ，夹爪控制支持不了这么快的频率所以处理适应夹爪控制频率
            gripper_ot++;
            if ( bool(gripper_enable) && gripper_ot > 3)
            {
                gripper_ot = 0;
                // 创建一个新的线程控制夹爪避免阻塞等待
                // std::thread(set_gripper_position, arm_handle, gripper_deg).detach();
                std::thread(&SlaveArmController::SetGripperPosition, this, std::ref(robotic_arm), arm_handle, gripper_deg).detach();
                // gripper_position_callback(gripper_deg);
                // printf("gripper_deg :%d\r\n", gripper_deg);
            }
            
            // 根据设置的跟随模式控制机械臂
            if (follow_mode_ == 1)
            {
                robotic_arm.rm_movej_canfd(arm_handle, joint_filter, true, 0);
            }
            else if (follow_mode_ == 0)
            {
                robotic_arm.rm_movej_canfd(arm_handle, joint_filter, false, 0);
            }
            else
            {
                robotic_arm.rm_movej_follow(arm_handle, joint_filter);
                // robotic_arm.rm_movej_follow(arm_handle, joint);
            }
        }
    }
}

// 设置夹爪开合度
void SlaveArmController::SetGripperPosition(RM_Service &rm_service, rm_robot_handle *arm_handle, int position)
{
    rm_service.rm_set_gripper_position(arm_handle, position, false, 0);
}