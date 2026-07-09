# 具身双臂机器人embodied_servo功能包详解

## 一.embodied_servo功能包说明

embodied_servo功能包实现ros1控制具身机器人头部两个舵机以及显示头部两个舵机当前角度的功能。

## 二.embodied_servo功能包功能架构说明



```
embodied_servo
    ├── CMakeLists.txt
    ├── launch
    │   └── servo_start.launch 启动舵机功能包launch入口文件
    ├── msg
    │   ├── ServoAngle.msg 自定义舵机角度消息
    │   └── ServoMove.msg  自定义舵机移动消息
    ├── package.xml
    ├── ros1具身双臂机器人embodied_servo功能包详解.md
    └── src
        └── servo_controller.cpp 控制舵机和显示舵机角度逻辑文件

```


## 三.embodied_servo功能包使用

首先环境配置和ros包编译完成完成后，打开一个终端进入具身ros包的工作空间，我们通过以下命令启动embodied_servo功能包：

```bash
cd ~/embodied_robot
source devel/setup.bash
roslaunch embodied_servo servo_start.launch 
```



## 四.embodied_servo功能包话题
| 话题名称            | 消息类型                  | 描述                                                         |
| ------------------- | ------------------------- | ------------------------------------------------------------ |
| /servo_control/move | embodied_servo/ServoMove  | 订阅话题，用于接收舵机转动控制指令，控制指定舵机转动到指定角度 |
| /servo_state        | embodied_servo/ServoAngle | 发布话题，用于发布两个舵机的当前角度位置状态                 |

## 五.具身机器人头部舵机二次开发参考资料

具身机器人头部舵机硬件上是通过一个串行总线舵机控制板连接上下移动、左右移动的两个舵机。

embodied_servo功能包开发使用的资料包含下面：

[串行总线舵机控制板用户手册](../embodied_hardware_docs/head_servo/用户手册.pdf)

[总线舵机控制板二次开发串口通信协议.pdf](../embodied_hardware_docs/head_servo/总线舵机控制板二次开发串口通信协议.pdf)

如果想使用舵机更多功能，请参考这两个文件进行开发。



## 六. embodied_servo功能包关键代码分析

servo_controller.cpp是功能包的核心逻辑文件，负责串口通信、舵机控制和状态发布。以下分析其关键代码和功能。

### 1. 主要功能

- **串口通信**：通过/dev/ttyUSB0（波特率9600）与舵机控制板通信。
- **订阅话题**：监听/servo_control/move，接收ServoMove消息以控制舵机。
- **发布话题**：通过/servo_state发布ServoAngle消息，报告舵机角度。
- **协议实现**：遵循串行总线舵机控制板通信协议，发送指令和解析响应。

### 2. 核心函数分析

#### (1) callback_servoMove

- **功能**：处理/servo_control/move话题消息，生成舵机转动指令。

- **逻辑**：

  - 构造10字节指令包（头部0x55 0x55，指令类型0x03，舵机ID和角度）。
  - 通过串口发送指令。

- **代码**：

  ```cpp
  void callback_servoMove(const embodied_servo::ServoMove& msg){
      memset(s_buffer, 0, sizeof(s_buffer));
      s_buffer[0] = 0x55;
      s_buffer[1] = 0x55;
      s_buffer[2] = 0x08;
      s_buffer[3] = 0x03;
      s_buffer[4] = 0x01;
      s_buffer[5] = 0xe8;
      s_buffer[6] = 0x03;
      s_buffer[7] = msg.servo_id;  // 舵机ID
      s_buffer[8] = msg.angle;     // 角度低8位
      s_buffer[9] = msg.angle >> 8; // 角度高8位
      ros_ser.write(s_buffer, sBUFFERSIZE);
      ROS_INFO_STREAM("Control Servo Move");
  }
  ```

#### (2) servoGetAngle

- **功能**：发送查询两个舵机角度的指令。

- **逻辑**：

  - 构造7字节指令包（头部0x55 0x55，指令类型0x15，舵机ID 1和2）。
  - 发送指令，触发控制板返回角度数据。

- **代码**：

  ```cpp
  void servoGetAngle(){
      memset(s_buffer, 0, sizeof(s_buffer));
      s_buffer[0] = 0x55;
      s_buffer[1] = 0x55;
      s_buffer[2] = 0x05;
      s_buffer[3] = 0x15;
      s_buffer[4] = 0x02;
      s_buffer[5] = 0x01;  // 舵机ID 1
      s_buffer[6] = 0x02;  // 舵机ID 2
      ros_ser.write(s_buffer, 7);
  }
  ```

### 3. 二次开发建议

- **支持更多舵机**：修改servoGetAngle和数据解析，支持额外舵机。
- 参考[串行总线舵机控制板用户手册](../embodied_hardware_docs/head_servo/用户手册.pdf)和[总线舵机控制板二次开发串口通信协议](../embodied_hardware_docs/head_servo/总线舵机控制板二次开发串口通信协议.pdf)
