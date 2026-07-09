# Real_Trunk — 具身双臂机器人模仿学习全流程

基于睿尔曼人形升降平台开发，上位机 NVIDIA Jetson AGX Xavier，搭配 RM65-BI 双臂、傲意 ROH-A001 灵巧手、三路 Intel D435 深度相机。


## 目录

- [Real\_Trunk — 具身双臂机器人模仿学习全流程](#real_trunk--具身双臂机器人模仿学习全流程)
  - [目录](#目录)
  - [硬件与运行环境](#硬件与运行环境)
  - [系统架构](#系统架构)
    - [通信拓扑](#通信拓扑)
    - [软件分层](#软件分层)
    - [C++ 底层（atom）线程模型](#c-底层atom线程模型)
    - [灵巧手控制链路](#灵巧手控制链路)
    - [遥操数采架构](#遥操数采架构)
  - [项目文件结构](#项目文件结构)
  - [快速上手](#快速上手)
    - [1. 底层遥操（atom）](#1-底层遥操atom)
    - [2. 灵巧手控制](#2-灵巧手控制)
    - [3. Debug 调试](#3-debug-调试)
    - [4. 数据采集](#4-数据采集)
    - [5. 模型训练](#5-模型训练)
    - [6. 推理部署](#6-推理部署)
  - [数据格式](#数据格式)
    - [录制格式（recorder.py 输出）](#录制格式recorderpy-输出)
    - [合并后 24D 结构](#合并后-24d-结构)
    - [单位约定（全链路）](#单位约定全链路)
    - [ROS Topic 速查](#ros-topic-速查)
  - [重要注意事项](#重要注意事项)
  - [文件清单](#文件清单)

---

## 硬件与运行环境

| 组件 | 型号 | 数量 | 连接 | IP / 串口 | 说明 |
| ----- | ----- | :--: | ----- | --------- | ----- |
| 上位机 | NVIDIA Jetson AGX Xavier | 1 | — | — | Ubuntu 20.04 |
| 从端左臂 | RM65-BI | 1 | TCP | 169.254.128.18:8080 | 6DOF |
| 从端右臂 | RM65-BI | 1 | TCP | 169.254.128.19:8080 | 6DOF |
| 主端左臂 | RM65 同款 | 1 | RS485→USB | `/dev/rmUSB0`, 460800baud | 遥操架左臂 |
| 主端右臂 | RM65 同款 | 1 | RS485→USB | `/dev/rmUSB1`, 460800baud | 遥操架右臂 |
| IO 按键 | Modbus RTU | 1 | RS485→USB | `/dev/rmUSB2`, 38400baud | 12 路按键 + 夹爪 |
| 灵巧手 | 傲意 ROH-A001 | 2 | 控制器 RS485 工具端 | — | 经机械臂控制器转发 |
| D435 相机 | Intel RealSense | 3 | USB3 | S/N 见 camera_params.yaml | 左/右/头, 640×480@15Hz |
| 升降导轨 | — | 1 | TCP | 169.254.128.18:8080 | 与左臂共控制器 |
| 移动底盘 | 悟时 Woosh | 1 | TCP | 169.254.128.2:5410 | — |

AGX、双臂、底盘通过交换机组成子网，以 TCP/UDP 通信。

| 软件环境 | 版本 / 说明 |
| -------- | ---------- |
| OS | Ubuntu 20.04 |
| ROS | Noetic |
| 系统 Python | Python 3.12 |
| LeRobot | v0.3.2 |
| PyTorch | 2.7.1+cu118 |
| 训练 GPU | NVIDIA RTX 3060 6GB (Windows) |

---

## 系统架构

### 通信拓扑

```
遥操架(主端) ──RS485──→ AGX ──TCP──→ 双臂(从端)
   │                      │
   └── IO按键+夹爪         ├── UDP :9999 ──→ IO + 夹爪 广播
                          ├── UDP :19997 ──→ 左主端臂关节+夹爪 广播
                          └── UDP :19998 ──→ 右主端臂关节+夹爪 广播

AGX ──TCP──→ Woosh 底盘 (169.254.128.2:5410)
AGX ──TCP──→ 升降导轨 (经左臂控制器 169.254.128.18:8080)
AGX ──USB3──→ D435×3 相机
```

### 软件分层

```
┌─────────────────────────────────────────────────────┐
│  Python 应用层 (本项目核心)                          │
│  ├── rospy_collect/  数据采集 · 检查 · 转换 · 回放   │
│  ├── hand_ctrl/      灵巧手 TCP JSON 控制            │
│  ├── train/          LeRobot ACT 训练                │
│  └── deploy/         推理部署 + 安全测试              │
├─────────────────────────────────────────────────────┤
│  ROS 中间层 (Noetic)                                │
│  ├── /l_arm/arm_driver, /r_arm/arm_driver  双臂驱动 │
│  ├── /camera_d435_{0,1,2}                 D435 相机  │
│  └── /joint_states, /Udp_Hand_Status      状态话题   │
├─────────────────────────────────────────────────────┤
│  C++ 底层 — atom (rmc_aida_l_atom/build/atom)       │
│  ├── MasterArmListener     主端臂 RS485 串口读取     │
│  ├── SlaveArmController    从端臂 TCP 控制 + UDP 广播│
│  ├── IoStateListener       IO 按键 Modbus RTU 读取   │
│  ├── AgvController         底盘 TCP 控制             │
│  ├── LiftController        升降 TCP 控制             │
│  └── DataBroadcast         UDP 数据广播              │
└─────────────────────────────────────────────────────┘
```

### C++ 底层（atom）线程模型

`main_teleoperation.cpp` 并行启动以下线程：

| 线程 | 类 | 数据源 | 输出 |
| ---- | --- | ------ | ---- |
| IO 监听 | `IoStateListener` | `/dev/rmUSB2` Modbus RTU | UDP :9999 广播 IO(12路) + 夹爪值 |
| 左主端臂监听 | `MasterArmListener` | `/dev/rmUSB0` RS485 | 内存队列 → `SlaveArmController` |
| 右主端臂监听 | `MasterArmListener` | `/dev/rmUSB1` RS485 | 内存队列 → `SlaveArmController` |
| 左从端臂控制 | `SlaveArmController` | 内存队列 + RM SDK | TCP 控从端 + UDP :19997 广播主端数据 |
| 右从端臂控制 | `SlaveArmController` | 内存队列 + RM SDK | TCP 控从端 + UDP :19998 广播主端数据 |
| 底盘 | `AgvController` | IO 按键 | TCP 控 Woosh 底盘 |
| 升降 | `LiftController` | IO 按键 + RM SDK | TCP 控升降导轨 |

遥操启停：同时按下 IO 中键（BTN5 + BTN10）切换遥操开关，通过条件变量通知主线程。

### 灵巧手控制链路

```
IO按键 ──RS485──→ IoStateListener ──UDP :9999──→ hand_teleop.py
                                                      │
                               TCP JSON "hand_follow_pos"
                                                      │
                               机械臂控制器 :8080 ──RS485工具端──→ 傲意手
```

手控制不经过 ROS——ROS 的 `Hand_Angle` 消息用 int16 装不下 0-65535——而是通过 TCP JSON `hand_follow_pos` 直发机械臂控制器，由控制器固件翻译后通过工具端 RS485 下发给手。数据读取走 ROS `/Udp_Hand_Status` 话题的 `hand_pos` 字段。

**按键映射（hand_teleop.py）：**

| 动作 | 左手键 | 右手键 | hand_pos 索引 |
| ---- | ------ | ------ | :----------: |
| 拇指弯曲 | BTN1 张 / BTN2 闭 | BTN6 张 / BTN7 闭 | `[0]` |
| 拇指旋转 | BTN3 多 / BTN4 少 | BTN8 多 / BTN9 少 | `[5]` |
| 四指开合 | 跟夹爪映射 | 跟夹爪映射 | `[1:5]` |

四指映射公式：`finger = (max_grip - gripper) / max_grip × 65535`

安全机制：夹爪值 > 上限的 80% 才激活（默认上限 880，启动时 5 秒自动校准）。

### 遥操数采架构

```
主端臂 ──RS485──→ atom ──TCP──→ 从端臂 ──/joint_states──→ recorder.py
                                                              │
D435×3 ───────────── ROS topic ────────────────────────────────┤
                                                              │
Udp_Hand_Status ── ROS topic ──────────────────────────────────┘
                                                              ↓
                                                    npz(关节) + jpg(图像)
```

Python 数采脚本只读不写，atom（C++）负责遥操控制，互不抢占串口。图像和关节在同一 recorder.py 中通过 rospy 回调同步录制，不经过 rosbridge，延迟最低。

---

## 项目文件结构

```
real_trunk/
├── README.md                       本文档
├── start_all_nodes.sh               一键启动 roscore + arm_driver + D435
│
├── rospy_collect/                   数据采集管线
│   ├── recorder.py                  关节+图像一体化录制 (50Hz)
│   ├── debug_monitor.py             调试面板 (关节 + 手 + 三路相机)
│   ├── replay_episode.py            逐帧回放 (空格暂停, ←→快进, 倍速)
│   ├── check_data.py                数据质量检查 (NaN/范围/时间偏移)
│   ├── convert_to_lerobot.py        npz+jpg → LeRobot 标准格式
│   └── timestamp_check.py           时间戳对齐验证 (臂/手/相机)
│
├── hand_ctrl/                       灵巧手控制
│   ├── hand_teleop.py               傲意手遥操 (夹爪→四指, IO→拇指)
│   ├── hand_test.py                 交互式手关节测试
│   └── read_io_udp.py               UDP 监听 debug (IO 按键 + 夹爪)
│
├── train/                           模型训练
│   └── train_lerobot.py             LeRobot 官方 ACT (含三路图像)
│
├── deploy/                          推理部署
│   ├── deploy_test.py               安全测试 (臂 JointPos + 手 TCP)
│   └── step_inference.py            单步/连续推理 (25Hz, --side 可选)
│
├── rmc_aida_l_atom/                 C++ 底层遥操系统
│   ├── build/atom                   编译产物 (可执行文件)
│   ├── config.ini                   串口/IP/跟随模式等全部配置
│   ├── CMakeLists.txt               主 CMake + 子 CMake
│   ├── src/                         C++ 源码 (main_teleoperation 等)
│   ├── include/                     头文件
│   ├── Robotic_Arm/                 Realman SDK (libapi_cpp.so)
│   ├── scripts/                     Python 辅助脚本 (oy_motion_ctrl.py 等)
│   └── upstart_all.sh               开机自启脚本
│
├── src/                             ROS 功能包 (Realman 提供)
│   ├── embodied_arm_msgs/           消息定义
│   ├── embodied_arm_driver/         机械臂驱动 (核心)
│   ├── embodied_arm_control/        MoveIt 控制
│   ├── embodied_camera/             D435/USB 相机
│   ├── embodied_servo/              头部舵机
│   ├── embodied_moveit/             MoveIt 配置
│   ├── embodied_gazebo/             Gazebo 仿真
│   └── robot_description/           URDF 模型
│
└── lerobot_dataset/                 LeRobot 标准数据集
    ├── data/chunk-000/
    │   └── episode_000000.parquet   全部帧 (100 episodes)
    ├── videos/
    │   ├── observation.images.cam_head/
    │   ├── observation.images.cam_left/
    │   └── observation.images.cam_right/
    └── meta/
        ├── info.json                数据集元信息 (robot_type: dual_rm65)
        ├── stats.json               归一化统计量
        ├── episodes.jsonl           各 episode 长度
        └── tasks.jsonl              任务描述
```

---

## 快速上手

> **前置条件：** 机器人已开机，数采架航空线已连 AGX。

### 1. 底层遥操（atom）

atom 通常随系统自启（`upstart_all.sh`）。如未启动：

```bash
cd real_trunk/rmc_aida_l_atom
./build/atom
```

> 如同时有 `servo` / `effect` 进程在跑，先 `killall -9 servo effect`。

在遥操架同时按下 IO 中键（BTN5 + BTN10），机械臂复位后自动开始跟随遥操架运动。再次同时按下停止遥操。

### 2. 灵巧手控制

```bash
# 检查 UDP 是否收到 IO + 夹爪数据
cd real_trunk/hand_ctrl
python3 read_io_udp.py

# 傲意手功能测试
python3 hand_test.py

# 启动手遥操 (夹爪 > 80%上限后自动激活)
python3 hand_teleop.py
```

### 3. Debug 调试

```bash
# 终端 1: atom（已运行）

# 终端 2: ROS 节点
cd real_trunk
bash start_all_nodes.sh

# 终端 3: 调试面板
source /opt/ros/noetic/setup.bash
cd real_trunk/rospy_collect
python3 debug_monitor.py --images    # 含三路相机窗口
python3 debug_monitor.py             # 仅关节角度
python3 timestamp_check.py           # 时间戳对齐检查 (跑30秒)
```

### 4. 数据采集

```bash
# 终端 1: atom
# 终端 2: bash start_all_nodes.sh
# 终端 3: cd hand_ctrl && python3 hand_teleop.py

# 终端 4: 数采录制
source /opt/ros/noetic/setup.bash
cd real_trunk/rospy_collect
python3 recorder.py          # Enter 开始/停止, Ctrl+C 退出
```

录制数据保存到 `~/lerobot_data/`，自动检测已有文件编号，重启不覆盖。

**数据处理流水线（conda 环境）：**

```bash
conda activate lerobot
cd real_trunk/rospy_collect

# 回放验证
python replay_episode.py                         # 最新一条
python replay_episode.py --episode 1 --speed 0.5 # 半速回放第1条

# 质量检查
python check_data.py                             # 全部 episode
python check_data.py 001                         # 指定 episode

# 转换 LeRobot 格式
python convert_to_lerobot.py                     # 全部
python convert_to_lerobot.py --episodes 1,2,3    # 指定 episode

# 检查数据集
python -c "
from lerobot.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset(repo_id='~/lerobot_dataset')
print(f'{len(ds)} 帧, state shape={ds[0][\"observation.state\"].shape}')
"
```

### 5. 模型训练

```bash
conda activate lerobot
cd real_trunk/train

python train_lerobot.py \
  --dataset ~/lerobot_dataset \
  --batch_size 2 \
  --num_epochs 8000 \
  --lr 1e-3 \
  --kl_weight 0.1 \
  --chunk_size 50 \
  --hidden_dim 256 \
  --dim_feedforward 1024 \
  --ckpt_dir ~/lerobot_trained_act
```

输出：`policy_best.pt` + `norm_params.pt`（推理必备）。

> **重要：** 模型对臂关节(弧度)和手数据(0-65535 raw)做了 Z-score 归一化 `(x-mean)/std`。训练和推理必须配套使用同一组 `norm_params.pt`，否则反归一化映射错误导致机器人暴走。

### 6. 推理部署

**安全测试（部署前必做）：**

```bash
# 终端 1: atom
# 终端 2: bash start_all_nodes.sh

# 终端 3: 安全测试
source /opt/ros/noetic/setup.bash
cd real_trunk/deploy
python3 deploy_test.py
```

交互命令：

```
l/r <关节1-6> <±角度°>     微调臂关节   (例: r 6 +5)
lh/rh <手指1-6> <±raw>     微调手指     (例: lh 1 65535)
lh/rh all <0-65535>        手全张/全闭  (例: lh all 0)
s                          显示当前状态
q                          退出
```

**推理：**

```bash
# 单步推理 (推荐首次，Enter预测→Enter发送)
python3 step_inference.py --model policy_best.pt --norm norm_params.pt

# 连续推理 @25Hz
python3 step_inference.py --model policy_best.pt --norm norm_params.pt --run

# 仅控制单侧
python3 step_inference.py --model policy_best.pt --norm norm_params.pt --run --side left
```

---

## 数据格式

### 录制格式（recorder.py 输出）

```
~/lerobot_data/
├── ep_001_left.npz      左臂6 + 左手6 = 12D
├── ep_001_right.npz     右臂6 + 右手6 = 12D
├── ep_001_images/
│   ├── cam_left/frame_000000.jpg ...
│   ├── cam_right/...
│   └── cam_head/...
├── ep_002_left.npz
├── ep_002_right.npz
└── ...
```

### 合并后 24D 结构

| 维度 | 内容 | 单位 |
| :--: | ---- | ---- |
| `[0:6]` | 左臂 6 关节 | rad |
| `[6:12]` | 右臂 6 关节 | rad |
| `[12:18]` | 左手 6 指 | raw 0-65535 |
| `[18:24]` | 右手 6 指 | raw 0-65535 |

时间偏移与主从同步：

- `obs[t]` = 当前从端关节状态（`/joint_states` 实时读取）。
- `action[t]` 实际取自从端 **前一帧** 数据（`t-1` 时刻的从端关节状态）。
- 从端执行取 **后一帧**：主端 `t` 帧指令 → 从端 `t+1` 帧到位。

> **原因：** 主端臂数据不经过 ROS 广播（仅通过 atom 内部内存队列传递，UDP 广播的主端数据未落地为 ROS topic），recorder.py 无法直接读取主端关节值。因此 action 用从端前一帧状态近似主端指令——因为从端 `t-1` 帧的位置正是主端 `t-1` 帧指令的执行结果，两者等价。主端→从端存在一帧控制延迟，所以从端实际在 `t+1` 帧才到达主端 `t` 帧指定的位置。

### 单位约定（全链路）

```
主端臂串口 → degree/10000 → atom (内部用度)
从端 /joint_states → rad → recorder → observation (rad) ✓
action (rad) → JointPos消息 → arm_driver ×RAD_DEGREE → 控制器(度) ✓
手数据 → raw 0-65535 → hand_pos → TCP hand_follow_pos → 控制器 RS485 → 手 ✓
```

### ROS Topic 速查

| Topic | 类型 | 单位 | 读写 | 频率 |
| ----- | ---- | :--: | :--: | :--: |
| `/l_arm/joint_states` | `sensor_msgs/JointState` | rad | 读 | 200Hz |
| `/r_arm/joint_states` | `sensor_msgs/JointState` | rad | 读 | 200Hz |
| `/l_arm/rm_driver/JointPos` | `embodied_arm_msgs/JointPos` | rad | 写 | — |
| `/r_arm/rm_driver/JointPos` | `embodied_arm_msgs/JointPos` | rad | 写 | — |
| `/l_arm/rm_driver/Udp_Hand_Status` | `embodied_arm_msgs/Hand_Status` | raw | 读 | 需 `Udp_hand:=true` |
| `/r_arm/rm_driver/Udp_Hand_Status` | `embodied_arm_msgs/Hand_Status` | raw | 读 | 需 `Udp_hand:=true` |
| `/camera_d435_0/color/image_raw` | `sensor_msgs/Image` (BGR8) | — | 读 | 15Hz |
| `/camera_d435_1/color/image_raw` | `sensor_msgs/Image` (BGR8) | — | 读 | 15Hz |
| `/camera_d435_2/color/image_raw` | `sensor_msgs/Image` (BGR8) | — | 读 | 15Hz |

ROS namespace：`/l_arm/`（左臂）、`/r_arm/`（右臂）。`/joint_states` 是移动底盘关节，不是双臂。

---

## 重要注意事项

1. **手数据字段是 `hand_pos` 不是 `hand_angle`：** 范围 0-65535，对应 TCP `hand_follow_pos` 命令值。`hand_angle` 是编码器原始值（不准、不回零），不要使用。

2. **归一化参数必须配套：** `policy_best.pt` 和 `norm_params.pt` 来自同一次训练，混用会导致反归一化映射错误、机器人暴走。

3. **单位转换陷阱：** `/joint_states` 读回已经是弧度，`JointPos` 消息也要求弧度。不要在 Python 侧多做 rad↔deg 转换——arm_driver 内部自动 `×RAD_DEGREE` 把弧度转成控制器所需的角度。如果 Python 多转一次，控制器收到的值放大 57.3 倍。

4. **D435 时间戳：** `multi_d435_publisher.py` 发布的图像 Header 未设时间戳（stamp=0），需时间对齐时通过数据回放人工对照。实际使用中几乎不出问题。

5. **atom 与 rospy 脚本可共存：** atom 占用 `/dev/ttyUSB1-3`，rospy 脚本只通过 ROS topic 读数据，不抢串口。

6. **rmUSB 软链接重启丢失：** 每次重启后需重建：
   ```bash
   sudo ln -s /dev/ttyUSB1 /dev/rmUSB0
   sudo ln -s /dev/ttyUSB2 /dev/rmUSB1
   sudo ln -s /dev/ttyUSB3 /dev/rmUSB2
   ```

7. **手控制绕过 ROS：** `Hand_Angle` 消息使用 `int16` 装不下 0-65535 范围，手控制直接走 TCP JSON 到控制器，手数据读取走 ROS `hand_pos`。

8. **升降按键修改：** 按下主臂左手中键（BTN5）+ 右边上下键（BTN6/BTN7）才会升降。

---

## 文件清单

| 文件 | 功能 | Python 环境 |
| ---- | ---- | :--------: |
| `start_all_nodes.sh` | 一键启动 roscore + arm_driver + D435 | — |
| `rospy_collect/recorder.py` | 关节+图像一体化录制 (50Hz, Enter控录) | 系统 Python3 |
| `rospy_collect/debug_monitor.py` | 调试面板 (关节+手+三路图像) | 系统 Python3 |
| `rospy_collect/replay_episode.py` | 逐帧回放 (空格暂停, 倍速, ←→跳转) | conda lerobot |
| `rospy_collect/check_data.py` | 数据质量检查 (NaN/范围/时间偏移) | conda lerobot |
| `rospy_collect/convert_to_lerobot.py` | npz+jpg → parquet+mp4 | conda lerobot |
| `rospy_collect/timestamp_check.py` | 时间戳对齐验证 (30秒统计) | 系统 Python3 |
| `hand_ctrl/hand_teleop.py` | 傲意手遥操 (夹爪→四指, IO→拇指) | 系统 Python3 |
| `hand_ctrl/hand_test.py` | 傲意手交互式关节测试 | 系统 Python3 |
| `hand_ctrl/read_io_udp.py` | UDP IO/夹爪监听 debug | 系统 Python3 |
| `train/train_lerobot.py` | LeRobot 官方 ACT 训练 | conda lerobot |
| `deploy/deploy_test.py` | 部署安全测试 (臂+手全接口) | 系统 Python3 |
| `deploy/step_inference.py` | 单步/连续推理 (25Hz, --side 可选) | 系统 Python3 |
