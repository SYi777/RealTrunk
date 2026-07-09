#  具身双臂机器人embodied_arm_control功能包详解



## **一. **embodied_arm_control功能包说明

embodied_arm_control为实现 moveit 控制真实机械臂时所必须的一个功能包，该功能包的主要作用为将 moveit 规划好的路径点进行进一步的细分，将细分后的路径点以透传的方式给到 rm_driver，实现机械臂的规划运行。

## 二.embodied_arm_control功能包架构说明

```
embodied_arm_control
    ├── CMakeLists.txt
    ├── package.xml
    ├── ros1具身双臂机器人embodied_arm_control功能包详解.md
    └── src
        ├── arm_control.cpp #代码源文件
        └── cubicSpline.h #三次样条插值头文件

    1 directory, 6 files

```

## **三.**embodied_arm_control功能包使用

在单独启动该功能包的节点时并不发挥作用，需要结合 embodied_arm_driver 功能包和 moveit的相关节点一起使用才能发挥作用，详细请查看《ros1具身双臂机器人embodied_moveit功能包详解.md》相关内容。
