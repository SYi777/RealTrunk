#ifndef WOOSH_ROBOT_CONTROL_H
#define WOOSH_ROBOT_CONTROL_H

#include "woosh_robot.h"
#include <string>

class WooshRobotControl {
public:
    WooshRobotControl(const std::string& addr, uint32_t port);
    ~WooshRobotControl();
    void initialize();
    void subscribePoseSpeed();
    //void controlSpeed();
    void setLinearSpeed(double linear);
    void setAngularSpeed(double angular);

private:
    std::string addr_;
    uint32_t port_;
    woosh::CommuSetting commu_setting_;
    std::shared_ptr<woosh::RobotInterface> robot_;
    woosh::robot::Twist twist_;

    void setupCommuSetting();
};

#endif // WOOSH_ROBOT_CONTROL_H
