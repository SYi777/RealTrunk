#!/usr/bin/env python3
# -*- coding=UTF-8 -*-
"""
版权所有 (c) 2024 [睿尔曼智能科技有限公司]。保留所有权利。

在满足以下条件的情况下，允许重新分发和使用源代码和二进制形式的代码，无论是否修改：
1. 重新分发的源代码必须保留上述版权声明、此条件列表和以下免责声明。
2. 以二进制形式重新分发的代码必须在随分发提供的文档和/或其他材料中复制上述版权声明、此条件列表和以下免责声明。

本软件由版权持有者和贡献者“按原样”提供，不提供任何明示或暗示的保证，
包括但不限于对适销性和特定用途适用性的暗示保证。
在任何情况下，即使被告知可能发生此类损害的情况下，
版权持有者或贡献者也不对任何直接的、间接的、偶然的、特殊的、惩罚性的或后果性的损害
（包括但不限于替代商品或服务的采购；使用、数据或利润的损失；或业务中断）负责，
无论是基于合同责任、严格责任还是侵权行为（包括疏忽或其他原因）。

此模块通过话题的发布者，将双臂复合机器人模块的基本功能都进行了测试。
"""

import rospy
import math

from embodied_arm_msgs.msg import MoveJ_P, MoveJ, Lift_Height, Plan_State



def degrees_to_radians(degrees):  
    """  
    将度转换为弧度  
      
    参数:  
    degrees -- 度值  
      
    返回:  
    转换后的弧度值  
    """  
    radians = degrees * (math.pi / 180)  
    return radians  
  

def plan_state_callback(msg):
    
    """当订阅的机械臂执行状态消息到达时，会调用此回调函数

    """
    global run_state

    if msg.state:
        run_state = True
        rospy.loginfo("*******Plan State OK")
    else:
        run_state = False
        rospy.loginfo("*******Plan State Fail")

def is_arrive():
    """机械臂movej和movej_p动作执行成功检测以及左臂升降执行成功检测
    """

    global run_state


    # 等待run_state变成True
    while 1:

        rospy.sleep(0.1)
        if run_state:
            run_state = False
            break

def dual_arm_example():

    """
    具身机器人左臂和右臂分别执行movej和movej_p动作指令，给了一个具身双臂通过ros执行机械臂基本动作指令的示例
    """


    
    radians_0 = degrees_to_radians(0) #0°对应的弧度值
    radians_57 = degrees_to_radians(57) #57°对应的弧度值

    #测试moveJ 左臂回到零点，单位是弧度。
    movej_cmd_l = MoveJ()
    movej_cmd_l.speed = 0.2
    movej_cmd_l.trajectory_connect = 0
    movej_cmd_l.joint = [radians_0, radians_0,  radians_0, radians_0, radians_0, radians_57]
    pub_MoveJ_l.publish(movej_cmd_l)
    rospy.loginfo("Published MoveJ Command to /l_arm/MoveJ_Cmd")
    is_arrive()
   
    #测试moveJ 右臂回到零点，单位是弧度。
    movej_cmd_r = MoveJ()
    movej_cmd_r.speed = 0.2
    movej_cmd_l.trajectory_connect = 0
    movej_cmd_r.joint = [radians_0, radians_0,  radians_0, radians_0, radians_0, radians_57]
    pub_MoveJ_r.publish(movej_cmd_r)
    rospy.loginfo("Published MoveJ Command to /r_arm/MoveJ_Cmd")
    is_arrive()
    
    #测试机械臂moveJ-P空间运动
    message = MoveJ_P()
    message.Pose.position.x = -0.1
    message.Pose.position.y = 0.0   
    message.Pose.position.z = 0.8   
    message.Pose.orientation.x = 0.0
    message.Pose.orientation.y = -0.37
    message.Pose.orientation.z = 0.0
    message.Pose.orientation.w = 0.92
    message.speed = 0.2
    message.trajectory_connect = 0


    #左臂运动到该点
    pub_MoveJ_P_l.publish(message)
    is_arrive()

    #右臂运动到该点
    pub_MoveJ_P_r.publish(message)
    is_arrive()



def lift_example():
    """
    给了一个具身机器人通过左臂控制升降机升降的ros示例
    """
    global run_state

    #测试升降
    height_msg = Lift_Height()
    height_msg.height = 380         #范围： 10 - 390   
    height_msg.speed = 30
    pub_height.publish(height_msg)
    rospy.loginfo(f"发布目标高度: {height_msg.height}")


    rospy.sleep(1)

    run_state = False

    is_arrive()
    
    #测试升降
    height_msg = Lift_Height()
    height_msg.height = 200     #范围： 10 - 390   
    height_msg.speed = 30
    pub_height.publish(height_msg)
    rospy.loginfo(f"发布目标高度: {height_msg.height}")

    rospy.sleep(1)

    run_state = False

    
    is_arrive()


def main():

    #双臂样例
    dual_arm_example()

    #升降示例
    lift_example()

if __name__ == '__main__':
    
    rospy.init_node('embodied_arm_demo', anonymous=True)  # 初始化ROS节点
    
    #双臂相关话题
    pub_MoveJ_l = rospy.Publisher('/l_arm/rm_driver/MoveJ_Cmd', MoveJ, queue_size=10)
    pub_MoveJ_P_l = rospy.Publisher('/l_arm/rm_driver/MoveJ_P_Cmd', MoveJ_P, queue_size=10)
    pub_MoveJ_r = rospy.Publisher('/r_arm/rm_driver/MoveJ_Cmd', MoveJ, queue_size=10)
    pub_MoveJ_P_r = rospy.Publisher('/r_arm/rm_driver/MoveJ_P_Cmd', MoveJ_P, queue_size=10)

    #左臂movej和movej_p执行状态获取以及升降机执行状态获取
    planState_sub_l = rospy.Subscriber("/l_arm/rm_driver/Plan_State", Plan_State, plan_state_callback)

    #右臂movej和movej_p执行状态获取
    planState_sub_r = rospy.Subscriber("/r_arm/rm_driver/Plan_State", Plan_State, plan_state_callback)


    #升降
    pub_height = rospy.Publisher("/l_arm/rm_driver/Lift_SetHeight", Lift_Height, queue_size=10)  

    #双臂movej、movej_p轨迹规划情况和升降机升降执行情况
    run_state = False

    rospy.sleep(1)  # 等待发布器初始化

    main()
    

