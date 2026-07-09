#  具身双臂机器人embodied_arm_msgs功能包详解

## **一. **embodied_arm_msgs功能包说明

embodied_arm_msgs功能包为机械臂在 ROS 的框架下运行提供必要的消息文件

## 二.embodied_arm_msgs功能包架构说明

```
embodied_arm_msgs
    ├── CMakeLists.txt 
    ├── msg   当前的消息文件（详细请看下方介绍）
    │   ├── Arm_Analog_Output.msg
    │   ├── Arm_Current_State.msg
    │   ├── Arm_Current_Status.msg
    │   ├── Arm_Digital_Output.msg
    │   ├── Arm_IO_State.msg
    │   ├── Arm_Joint_Speed_Max.msg
    │   ├── Arm_Software_Version.msg
    │   ├── ArmState.msg
    │   ├── Cabinet.msg
    │   ├── CartePosCustom.msg
    │   ├── CartePos.msg
    │   ├── ChangeTool_Name.msg
    │   ├── ChangeTool_State.msg
    │   ├── ChangeWorkFrame_Name.msg
    │   ├── ChangeWorkFrame_State.msg
    │   ├── Force_Position_Move_Joint.msg
    │   ├── Force_Position_Move_Pose_Custom.msg
    │   ├── Force_Position_Move_Pose.msg
    │   ├── Force_Position_State.msg
    │   ├── GetArmState_Command.msg
    │   ├── Gripper_Pick.msg
    │   ├── Gripper_Set.msg
    │   ├── Hand_Angle.msg
    │   ├── Hand_Force.msg
    │   ├── Hand_Posture.msg
    │   ├── Hand_Seq.msg
    │   ├── Hand_Speed.msg
    │   ├── Hand_Status.msg
    │   ├── IO_Update.msg
    │   ├── Joint_Current.msg
    │   ├── Joint_Enable.msg
    │   ├── Joint_En_Flag.msg
    │   ├── Joint_Error_Code.msg
    │   ├── Joint_Max_Speed.msg
    │   ├── JointPosCustom.msg
    │   ├── Joint_PoseEuler.msg
    │   ├── JointPos.msg
    │   ├── Joint_Speed.msg
    │   ├── Joint_Step.msg
    │   ├── Joint_Teach.msg
    │   ├── Joint_Temperature.msg
    │   ├── Joint_Voltage.msg
    │   ├── Lift_Height.msg
    │   ├── Lift_In_Position.msg
    │   ├── Lift_Speed.msg
    │   ├── LiftState.msg
    │   ├── Manual_Set_Force_Pose.msg
    │   ├── MoveC.msg
    │   ├── MoveJ.msg
    │   ├── MoveJ_P.msg
    │   ├── MoveL.msg
    │   ├── Ort_Teach.msg
    │   ├── Plan_State.msg
    │   ├── Pos_Teach.msg
    │   ├── Servo_GetAngle.msg
    │   ├── Servo_Move.msg
    │   ├── Set_Force_Position.msg
    │   ├── Set_Realtime_Push.msg
    │   ├── Six_Force.msg
    │   ├── Socket_Command.msg
    │   ├── Start_Multi_Drag_Teach.msg
    │   ├── Stop.msg
    │   ├── Stop_Teach.msg
    │   ├── Tool_Analog_Output.msg
    │   ├── Tool_Digital_Output.msg
    │   ├── Tool_IO_State.msg
    │   └── Turtle_Driver.msg
    ├── package.xml
    └── ros1具身双臂机器人embodied_arm_msgs功能包详解.md

    1 directory, 65 files


```

## **三.**embodied_arm_msgs功能包使用

该功能包并没有可执行的使用命令，其主要作用是为其他功能包中使用机械臂提供必须的消息文件。
