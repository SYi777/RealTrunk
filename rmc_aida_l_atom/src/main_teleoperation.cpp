#include <termios.h>
#include <iostream>
#include <thread>
#include <vector>
#include <memory>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>  // 包含 inet_addr 所需的头文件

#include "arm_teleoperation.h"
#include "lift_agv_teleoperation.h"
#include "inih/INIReader.h"
#include "io_state_listener.h"
#include "master_arm_listener.h"
#include "agv_controller.h"
#include "lift_controller.h"
#include "slave_arm_controller.h"
#include "tcp_service.h"


// RM_Service robotic_arm;
bool start_teleoperation = false;
bool start_btn_press = false;

uint8_t teleop_btn_index[2] = {5, 10};  // 控制遥操作开始/停止按钮的编号


void custom_api_log(const char *message, va_list args)
{
    if (!message)
    {
        fprintf(stderr, "Error: message is a null pointer\n");
        return;
    }

    char buffer[1024];
    vsnprintf(buffer, sizeof(buffer), message, args);
    printf(" %s\n", buffer);
}


int main(int argc, char *argv[])
{
    // 读取配置文件
    INIReader ini_reader("config.ini");
    if (ini_reader.ParseError() < 0)
    {
        std::cerr << "Can't load 'config.ini'" << std::endl;
        return -1;
    }
    unsigned int tcp_service_port = ini_reader.GetUnsigned("Tcp_Service", "service_port", 8000);
    std::string tcp_service_ip = ini_reader.GetString("Tcp_Service", "service_ip", "127.0.0.1");

    std::string l_m_arm_port_name = ini_reader.GetString("Left_Arm", "master_port", "UNKNOWN");
    unsigned int l_m_arm_port_baud = ini_reader.GetUnsigned("Left_Arm", "master_baud", -1);
    std::string l_arm_ip = ini_reader.GetString("Left_Arm", "slave_ip", "UNKNOWN");
    unsigned int l_arm_port = ini_reader.GetUnsigned("Left_Arm", "slave_port", -1);
    unsigned int l_arm_follow_mode = ini_reader.GetUnsigned("Left_Arm", "follow_mode", -1);
    unsigned int l_udp_port = ini_reader.GetUnsigned("Left_Arm", "udp_port", -1);
    std::string l_udp_ip = ini_reader.GetString("Left_Arm", "udp_ip", "UNKNOWN");
    std::string l_tag = ini_reader.GetString("Left_Arm", "tag", "UNKNOWN");
    unsigned int l_filter_depth = ini_reader.GetUnsigned("Left_Arm", "filter_depth", 10);
    unsigned int l_calibrate_speed = ini_reader.GetUnsigned("Left_Arm", "calibrate_speed", 10);
    unsigned int l_gripper_enable = ini_reader.GetUnsigned("Left_Arm", "gripper_enable", 1);

    std::string r_m_arm_port_name = ini_reader.GetString("Right_Arm", "master_port", "UNKNOWN");
    unsigned int r_m_arm_port_baud = ini_reader.GetUnsigned("Right_Arm", "master_baud", -1);
    std::string r_arm_ip = ini_reader.GetString("Right_Arm", "slave_ip", "UNKNOWN");
    unsigned int r_arm_port = ini_reader.GetUnsigned("Right_Arm", "slave_port", -1);
    unsigned int r_arm_follow_mode = ini_reader.GetUnsigned("Right_Arm", "follow_mode", -1);
    unsigned int r_udp_port = ini_reader.GetUnsigned("Right_Arm", "udp_port", -1);
    std::string r_udp_ip = ini_reader.GetString("Right_Arm", "udp_ip", "UNKNOWN");
    std::string r_tag = ini_reader.GetString("Right_Arm", "tag", "UNKNOWN");
    unsigned int r_filter_depth = ini_reader.GetUnsigned("Right_Arm", "filter_depth", 10);
    unsigned int r_calibrate_speed = ini_reader.GetUnsigned("Right_Arm", "calibrate_speed", 10);
    unsigned int r_gripper_enable = ini_reader.GetUnsigned("Right_Arm", "gripper_enable", 1);

    std::string io_port_name = ini_reader.GetString("IO", "port", "UNKNOWN");
    unsigned int io_port_baud = ini_reader.GetUnsigned("IO", "baud", -1);
    unsigned int io_data_udp_port = ini_reader.GetUnsigned("IO", "udp_port", 9999);
    std::string io_data_udp_ip = ini_reader.GetString("IO", "udp_ip", "UNKNOWN");

    std::string lift_ip = ini_reader.GetString("Lift", "ip", "UNKNOWN");
    unsigned int lift_port = ini_reader.GetUnsigned("Lift", "port", -1);
    unsigned int lift_speed = ini_reader.GetUnsigned("Lift", "speed", 50);
    unsigned int lift_enable = ini_reader.GetUnsigned("Lift", "enable", 0);

    std::string agv_ip = ini_reader.GetString("AGV", "ip", "UNKNOWN");
    unsigned int agv_port = ini_reader.GetUnsigned("AGV", "port", -1);
    float agv_linear_speed = ini_reader.GetReal("AGV", "linear_speed", 0.2);
    float agv_angular_speed = ini_reader.GetReal("AGV", "angular_speed", 0.4);
    unsigned int agv_enable = ini_reader.GetUnsigned("AGV", "enable", 0);

    std::cout << "service_port: " << tcp_service_port << std::endl;
    std::cout << "service_ip: " << tcp_service_ip << std::endl;

    std::cout << "l_m_arm_port_name: " << l_m_arm_port_name << std::endl;
    std::cout << "l_m_arm_port_baud: " << l_m_arm_port_baud << std::endl;
    std::cout << "l_arm_ip: " << l_arm_ip << std::endl;
    std::cout << "l_arm_port: " << l_arm_port << std::endl;
    std::cout << "l_arm_follow_mode: " << l_arm_follow_mode << std::endl;
    std::cout << "l_udp_port: " << l_udp_port << std::endl;
    std::cout << "l_udp_ip: " << l_udp_ip << std::endl;
    std::cout << "l_tag: " << l_tag << std::endl;
    std::cout << "l_filter_depth: " << l_filter_depth << std::endl;
    std::cout << "l_calibrate_speed: " << l_calibrate_speed << std::endl;

    std::cout << "r_m_arm_port_name: " << r_m_arm_port_name << std::endl;
    std::cout << "r_m_arm_port_baud: " << r_m_arm_port_baud << std::endl;
    std::cout << "r_arm_ip: " << r_arm_ip << std::endl;
    std::cout << "r_arm_port: " << r_arm_port << std::endl;
    std::cout << "r_arm_follow_mode: " << r_arm_follow_mode << std::endl;
    std::cout << "r_udp_port: " << r_udp_port << std::endl;
    std::cout << "r_udp_ip: " << r_udp_ip << std::endl;
    std::cout << "r_tag: " << r_tag << std::endl;
    std::cout << "r_filter_depth: " << r_filter_depth << std::endl;
    std::cout << "r_calibrate_speed: " << r_calibrate_speed << std::endl;

    std::cout << "io_port_name: " << io_port_name << std::endl;
    std::cout << "io_port_baud: " << io_port_baud << std::endl;
    std::cout << "io_data_udp_port: " << io_data_udp_port << std::endl;
    std::cout << "io_data_udp_ip: " << io_data_udp_ip << std::endl;

    std::cout << "lift_ip: " << lift_ip << std::endl;
    std::cout << "lift_port: " << lift_port << std::endl;
    std::cout << "lift_speed: " << lift_speed << std::endl;
    std::cout << "lift_enable: " << lift_enable << std::endl;

    std::cout << "agv_ip: " << agv_ip << std::endl;
    std::cout << "agv_port: " << agv_port << std::endl;
    std::cout << "agv_linear_speed: " << agv_linear_speed << std::endl;
    std::cout << "agv_angular_speed: " << agv_angular_speed << std::endl;
    std::cout << "agv_enable: " << agv_enable << std::endl;

    std::cout << "l_gripper_enable: " << l_gripper_enable << std::endl;
    std::cout << "r_gripper_enable: " << r_gripper_enable << std::endl;

    /*创建ros控制的服务端******************************************************************************************************/
    Tcp_Service tcp_service(tcp_service_port, tcp_service_ip);
    tcp_service.start_thread();

    int result = -1;
    RM_Service robotic_arm;
    robotic_arm.rm_set_log_call_back(custom_api_log, 3);
    result = robotic_arm.rm_init(RM_DUAL_MODE_E); // 在main中初始化一次，通过参数传递给其他类使用，如果在多个类中多次初始化会存在问题
    if (result != 0)
    {
        printf("Initialization failed with error code: %d.\n", result);
        return 0;
    }

    // IO状态的Listener
    IoStateListener io_state_listener(io_port_name, io_port_baud, io_data_udp_port, io_data_udp_ip);
    io_state_listener.Start();  // 单独线程不断获取
    // AgvController agv_controller(agv_ip, agv_port, 0.08, 0.2, io_state_listener);
    // AGV控制实例
    AgvController agv_controller(agv_ip, agv_port, agv_linear_speed, agv_angular_speed, io_state_listener);
    if (agv_enable == 1) {
        agv_controller.Start();
    } else {}

    // 升降控制实例
    LiftController lift_controller(robotic_arm, lift_ip, lift_port, lift_speed, io_state_listener);
    if (lift_enable == 1) {
        lift_controller.Start();
    } else {}
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // 主臂实时数据Listener
    MasterArmListener l_master_arm_listener(l_m_arm_port_name, l_m_arm_port_baud, l_udp_port, l_udp_ip);
    l_master_arm_listener.Start();
    MasterArmListener r_master_arm_listener(r_m_arm_port_name, r_m_arm_port_baud, r_udp_port, r_udp_ip);
    r_master_arm_listener.Start();
    // 从臂控制实例

    SlaveArmController l_slave_arm_listener(robotic_arm, l_arm_ip, l_arm_port, l_arm_follow_mode, l_udp_port, l_udp_ip, l_calibrate_speed, \
        l_tag, l_filter_depth, r_gripper_enable, l_master_arm_listener);
    l_slave_arm_listener.ThreadInit();
    SlaveArmController r_slave_arm_listener(robotic_arm, r_arm_ip, r_arm_port, r_arm_follow_mode, r_udp_port, r_udp_ip, r_calibrate_speed, \
        r_tag, r_filter_depth, r_gripper_enable, r_master_arm_listener);
    r_slave_arm_listener.ThreadInit();

    // 获取IO即按键控制状态控制遥操作的起停
    while (true)
    {
        std::unique_lock<std::mutex> lock(io_state_listener.GetMutex());
        io_state_listener.cv.wait(lock);
        std::vector<uint8_t> io_data = io_state_listener.GetData();
        lock.unlock();
        auto now = std::chrono::high_resolution_clock::now();
        auto duration_microseconds = now.time_since_epoch();
        auto timestamp = std::chrono::duration_cast<std::chrono::microseconds>(duration_microseconds).count();
        // std::cout << "main get io data timestamp: " << timestamp << std::endl;
        // std::cout << "main recv data: ";
        // for (auto byte : io_data)
        // {
        //     std::cout << std::hex << static_cast<int>(byte) << " ";
        // }
        // std::cout << std::endl;
        // std::cout << " io_data_callback ..." << std::endl;
        if (*tcp_service.ros_ctrl) 
        {
            *tcp_service.ros_ctrl = false;
            std::cout << "The main arm is remotely operated through the ros message stamp!" << std::endl;
            start_teleoperation = false;
            l_slave_arm_listener.Stop();
            r_slave_arm_listener.Stop();
        }
        else if (io_data[2 + teleop_btn_index[0] * 2] == 1 && io_data[2 + teleop_btn_index[1] * 2] == 1) // 开始遥操作按钮被按下
        {
            start_btn_press = true;
        }
        else if (io_data[2 + teleop_btn_index[0] * 2] == 0 && io_data[2 + teleop_btn_index[1] * 2] == 0 && start_btn_press)
        {
            start_btn_press = false;
            std::cout << "teleop key click!" << std::endl;
            if (start_teleoperation)
            {
                start_teleoperation = false;
                l_slave_arm_listener.Stop();
                r_slave_arm_listener.Stop();
                std::cout << "stop teleop!" << std::endl;
            }
            else
            {
                std::cout << "start teleop!" << std::endl;
                start_teleoperation = true;
                l_slave_arm_listener.Start();
                r_slave_arm_listener.Start();
            }

            // if (!start_teleoperation)
            // {
            //     std::cout << "start teleop!" << std::endl;
            //     start_teleoperation = true;
            //     l_arm_teleop_ptr->Start();
            //     r_arm_teleop_ptr->Start();
            // }
        }
    }

    return 0;
}
