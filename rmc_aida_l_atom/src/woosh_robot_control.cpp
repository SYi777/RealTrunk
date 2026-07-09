#include "woosh_robot_control.h"
#include <iostream>
#include <unistd.h>

WooshRobotControl::WooshRobotControl(const std::string& addr, uint32_t port)
    : addr_(addr), port_(port) {}

WooshRobotControl::~WooshRobotControl() {
    // 清理工作可以在这里添加
}

void WooshRobotControl::initialize() {
    setupCommuSetting();
    robot_ = woosh::Factory::newRobotInterface(commu_setting_);
    if (!robot_->run()) {
        std::cerr << "Failed to initialize the robot interface." << std::endl;
        exit(-1);
    }
}

void WooshRobotControl::setupCommuSetting() {
    commu_setting_.addr = addr_;
    commu_setting_.port = port_;
    commu_setting_.identity = "woosdk-demo";
    commu_setting_.log_call_fun = [](const std::string& log) { printf("%s\n", log.c_str()); };
    commu_setting_.print_pack_call_fun = [](const std::string& log) { printf("%s\n", log.c_str()); };
    commu_setting_.connect_status_call_fun = [&](const bool& is_connect) {
        printf("woosh robot %s:%d %s.\n", commu_setting_.addr.c_str(), commu_setting_.port, 
               (is_connect ? "connected" : "disconnected"));
    };
}

void WooshRobotControl::subscribePoseSpeed() {
    robot_->robotPoseSpeedSub(
        [](const woosh::robot::PoseSpeed& info) {
            printf("位姿速度更新:\n%s\n", info.DebugString().c_str());
            printf("所在地图ID: %d\n", info.map_id());
            printf("线速度: %f\n", info.twist().linear());
            printf("角速度: %f\n", info.twist().angular());
            printf("累计里程: %u\n", info.mileage());
            printf("坐标 x: %f\n", info.pose().x());
            printf("坐标 y: %f\n", info.pose().y());
            printf("坐标 theta: %f\n", info.pose().theta());
        },
        woosh::PPLB
    );
}

// void WooshRobotControl::controlSpeed() {
//     int mchar; // 回车退出速度控制
//     while ((mchar = getchar()) != EOF && mchar != '\n') {
//         robot_->twistReq(
//             twist_,
//             [](const google::protobuf::Empty&, const bool& ok, const std::string& msg) {
//                 if (ok) {
//                     printf("速度控制请求成功\n");
//                 } else {
//                     printf("速度控制请求失败, msg: %s\n", msg.c_str());
//                 }
//             },
//             woosh::PPLB, woosh::PPLB
//         );

//         // sleep 100ms
//         usleep(100 * 1000);
//     }
// }

void WooshRobotControl::setLinearSpeed(double linear) {
    // printf("linear: %f\n", linear);
    twist_.set_linear(linear);
    robot_->twistReq(
        twist_,
        [](const google::protobuf::Empty&, const bool& ok,
           const std::string& msg) {
          if (!ok) {
            printf("线速度控制请求失败, msg: %s\n", msg.c_str());
          }
        //   if (ok) {
        //     printf("线速度控制请求成功\n");
        //   } else {
        //     printf("线速度控制请求失败, msg: %s\n", msg.c_str());
        //   }
        },
        woosh::PPLB, woosh::PPLB);
}

void WooshRobotControl::setAngularSpeed(double angular) {
    // printf("angular: %f\n", angular);
    twist_.set_angular(angular);
    robot_->twistReq(
        twist_,
        [](const google::protobuf::Empty&, const bool& ok,
           const std::string& msg) {
          if (!ok) {
            printf("角速度控制请求失败, msg: %s\n", msg.c_str());
          }
        //   if (ok) {
        //     printf("角速度控制请求成功\n");
        //   } else {
        //     printf("角速度控制请求失败, msg: %s\n", msg.c_str());
        //   }
        },
        woosh::PPLB, woosh::PPLB);
}
