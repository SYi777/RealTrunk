# 具身双臂机器人embodied_servo_demo舵机控制演示包详解

## 一.embodied_servo_demo舵机控制演示包说明

embodied_servo_demo包提供示例，展示如何通过调用其接口实现具身机器人舵机的精确控制实现对舵机的实时监控和角度控制，适用于机器人头部运动等简单控制场景。

## 二.embodied_servo_demo舵机控制演示包架构说明



```
embodied_servo_demo
    ├── CMakeLists.txt
    ├── launch
    │   └── servo_control_demo.launch 舵机控制演示包launch文件
    ├── package.xml
    ├── ros1具身双臂机器人embodied_servo_demo舵机控制演示包详解.md
    └── scripts
        └── servo_control_demo.py 舵机控制的示例脚本

    2 directories, 5 files

```


## 三.embodied_servo_demo舵机控制演示包使用

首先环境配置和ros包编译完成后，打开一个终端进入具身ros包的工作空间，我们通过以下命令启动embodied_servo_demo舵机控制演示包：

```bash
cd ~/embodied_robot
source devel/setup.bash
roslaunch embodied_servo_demo servo_control_demo.launch 
```

启动后，演示程序会自动执行以下动作：

- 舵机1（头部俯仰舵机）旋转到500角度刻度处
- 舵机2（头部左右转动舵机）旋转到450角度刻度处

**日志输出**：

- 终端将显示接收到的舵机状态信息（来自 /servo_state）。
- 每次发布控制指令时，显示舵机 ID 和目标角度。



## 四.demo运行逻辑流程图

![image-20250514141123328](images/image-20250514141123328.png)

运行`roslaunch embodied_servo_demo servo_control_demo.launch`启动两个节点：`servo_controller`连接舵机硬件，订阅/servo_control/move控制舵机转动，并以10 Hz循环读取舵机角度发布到/servo_state；servo_control_demo设置舵机1角度500、舵机2角度450，订阅/servo_state记录状态，并以1 Hz循环发布控制指令到/servo_control/move。两节点通过话题交互，直至ROS关闭结束。

## 五.注意事项

1. 舵机1（俯仰舵机）的转动范围是400-600位置，舵机2（左右转动舵机）的转动范围是0-1000位置。

   位置相对于角度的换算为1位置=0.24°

2. 调试提示：若未看到舵机运动，检查话题通信是否正常。

   打开一个终端，输入下面命令：

   ```bash
   rostopic echo /servo_state 
   ```

   此时终端输出舵机状态信息

3. 代码修改：如需调整舵位置，可修改 servo_control_demo.py 中的 servo1_angle 和 servo2_angle

