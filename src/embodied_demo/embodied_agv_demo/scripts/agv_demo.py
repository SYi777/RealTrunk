#!/usr/bin/env python3
# -*- coding=UTF-8 -*-

"""
这里给一个调用悟时底盘ROS接口的示例
这里调用悟时底盘的exec_task接口执行到达点位的功能  调用robot_status接口获取底盘状态判断是否到达点位

更多的底盘功能请看 底盘说明文件里的 《移动机器人的ROS接口（对外） - WOOSH Robotics.pdf》
"""


import rospy


from woosh_msgs.msg import RobotStatus
from woosh_msgs.srv import ExecTask, ExecTaskRequest



def status_callback(msg: RobotStatus):

    global arrive_state

    if msg.task_state == 7:  # 动作完成状态码为7
        
        rospy.loginfo("Reached the first point")

        arrive_state = True


# 规划底盘导航运动
def navigation_plan(mark_no: str, task_id: int=1, task_exect: int=1, 
                        task_type: int=1, direction: int=0):
    """规划底盘导航运动

    Args:
        mark_no (str): _description_
        task_id (int, optional): _description_. Defaults to 1.
        task_exect (int, optional): _description_. Defaults to 1.
        task_type (int, 创建时间戳optional): _description_. Defaults to 1.
        direction (int, optional): _description_. Defaults to 0.

    Returns:
        _type_: bool
    """

    req = ExecTaskRequest()
    req.task_id = task_id
    req.task_exect = task_exect
    req.task_type = task_type
    req.direction = direction
    req.task_type_no = 0
    req.mark_no = mark_no
    res = navigation_point(req)

    global arrive_state

    arrive_state = False

    # 创建时间戳
    if res.success:
        
        rate_ = rospy.Rate(10)

        while not rospy.is_shutdown():
            if arrive_state:
                break
            rate_.sleep()
        
        rospy.loginfo(f"Navigation execution was successful!")
        return True

            

    else:
        rospy.logerr(f"Navigation execution command error!")
        return False



def main():
    """
    主函数
    """

    

    # 底盘到达第一个点位A
    # 发送目标位置到move_base

    navigation_plan("A")

if __name__ == "__main__":

    
    # 初始化节点
    rospy.init_node("agv_demo", anonymous=True)

    # 底盘状态订阅
    rospy.Subscriber("/robot_status", RobotStatus, status_callback, queue_size=10)
    # 底盘导航点位请求
    navigation_point = rospy.ServiceProxy("/exec_task", ExecTask)


    rospy.sleep(1)

    # 底盘达到状态
    arrive_state = False 

    main()
    rospy.spin()
