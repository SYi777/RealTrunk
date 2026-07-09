#ifndef WOOSH_ROBOT_DEF_H
#define WOOSH_ROBOT_DEF_H

#include <google/protobuf/empty.pb.h>
#include <woosh/proto/device/device.pb.h>
#include <woosh/proto/dispatch/robot.pb.h>
#include <woosh/proto/dispatch/system.pb.h>
#include <woosh/proto/dispatch/taskexec.pb.h>
#include <woosh/proto/map/map_pack.pb.h>
#include <woosh/proto/map/storage_edit.pb.h>
#include <woosh/proto/robot/robot.pb.h>
#include <woosh/proto/robot/robot_pack.pb.h>
#include <woosh/proto/ros/ros_pack.pb.h>

#include <functional>
#include <thread>

#ifdef _WIN32

#ifdef WOOSH_ROBOT_SHARED_BUILD

#ifdef WOOSH_ROBOT_SHARED_BUILD
#define DLLEXPORT __declspec(dllexport)
#else
#define DLLEXPORT __declspec(dllimport)
#endif

#else
#define DLLEXPORT
#endif

#else
#define DLLEXPORT
#endif

/// \brief 请求超时
#define WOOSH_COMMU_REQUEST_TIMEOUT "WOOSH_COMMU_REQUEST_TIMEOUT"

namespace woosh {

/// 回调函数定义

/// \brief 日志打印
using LogCallFun = std::function<void(const std::string &)>;
/// \brief 通信包打印
using PrintPackCallFun = std::function<void(const std::string &)>;
/// \brief 通信连接状态
using ConnectStatusCallFun = std::function<void(const bool &)>;

using RspOK = const bool &;
using RspMsg = const std::string &;

using GPEmpty = const google::protobuf::Empty &;
using EmptyRspFun = std::function<void(GPEmpty, RspOK, RspMsg)>;

/// IRobotInfo

using NavPath = const nav::Path &;

/// \brief 机器人信息
using RobotInfo = const robot::RobotInfo &;
using RobotInfoRspFun = std::function<void(RobotInfo, RspOK, RspMsg)>;

/// \brief 常规信息
using RobotGeneral = const robot::General &;
using GeneralRspFun = std::function<void(RobotGeneral, RspOK, RspMsg)>;

/// \brief 配置信息
using RobotSetting = const robot::Setting &;
using SettingRspFun = std::function<void(RobotSetting, RspOK, RspMsg)>;
using SettingSubFun = std::function<void(RobotSetting)>;

/// \brief 机器人状态
using RobotState = const robot::RobotState &;
using RobotStateRspFun = std::function<void(RobotState, RspOK, RspMsg)>;
using RobotStateSubFun = std::function<void(RobotState)>;

/// \brief 控制模式信息
using RobotMode = const robot::Mode &;
using ModeRspFun = std::function<void(RobotMode, RspOK, RspMsg)>;
using ModeSubFun = std::function<void(RobotMode)>;

/// \brief 位姿速度
using RobotPoseSpeed = const robot::PoseSpeed &;
using PoseSpeedRspFun = std::function<void(RobotPoseSpeed, RspOK, RspMsg)>;
using PoseSpeedSubFun = std::function<void(RobotPoseSpeed)>;

/// \brief 电量信息
using RobotBattery = const robot::Battery &;
using BatteryRspFun = std::function<void(RobotBattery, RspOK, RspMsg)>;
using BatterySubFun = std::function<void(RobotBattery)>;

/// \brief 网络信息
using RobotNetwork = const robot::Network &;
using NetworkRspFun = std::function<void(RobotNetwork, RspOK, RspMsg)>;
using NetworkSubFun = std::function<void(RobotNetwork)>;

/// \brief 场景信息
using RobotScene = const robot::Scene &;
using SceneRspFun = std::function<void(RobotScene, RspOK, RspMsg)>;
using SceneSubFun = std::function<void(RobotScene)>;

/// \brief 任务进度信息
using RobotTaskProcess = const robot::TaskProc &;
using TaskProcessRspFun = std::function<void(RobotTaskProcess, RspOK, RspMsg)>;
using TaskProcessSubFun = std::function<void(RobotTaskProcess)>;

/// \brief 设备状态信息
using RobotDeviceState = const robot::DeviceState &;
using DeviceStateRspFun = std::function<void(RobotDeviceState, RspOK, RspMsg)>;
using DeviceStateSubFun = std::function<void(RobotDeviceState)>;

/// \brief 硬件状态信息
using RobotHardwareState = const robot::HardwareState &;
using HardwareStateRspFun =
    std::function<void(RobotHardwareState, RspOK, RspMsg)>;
using HardwareStateSubFun = std::function<void(RobotHardwareState)>;

/// \brief 运行状态信息
using RobotOperationState = const robot::OperationState &;
using OperationStateRspFun =
    std::function<void(RobotOperationState, RspOK, RspMsg)>;
using OperationStateSubFun = std::function<void(RobotOperationState)>;

/// \brief 机器人模型
using RobotModel = const robot::Model &;
using ModelRspFun = std::function<void(RobotModel, RspOK, RspMsg)>;
using ModelSubFun = std::function<void(RobotModel)>;

/// \brief 机器人导航路径
using RobotNavPath = const robot::NavPath &;
using RobotNavPathRspFun = std::function<void(RobotNavPath, RspOK, RspMsg)>;
using RobotNavPathSubFun = std::function<void(RobotNavPath)>;

/// \brief 任务历史(最近50条)
using RobotTaskHistory = const robot::TaskHistory &;
using TaskHistoryRspFun = std::function<void(RobotTaskHistory, RspOK, RspMsg)>;

/// \brief 状态码(最近50条)
using RobotStatusCode = const robot::count::StatusCode &;
using RobotStatusCodes = const robot::count::StatusCodes &;
using StatusCodesRspFun = std::function<void(RobotStatusCodes, RspOK, RspMsg)>;
using StastusCodeSubFun = std::function<void(RobotStatusCode)>;

/// \brief 未恢复的异常码
using RobotAbnormalCodes = const robot::count::AbnormalCodes &;
using AbnormalCodesRspFun =
    std::function<void(RobotAbnormalCodes, RspOK, RspMsg)>;
using AbnormalCodesSubFun = std::function<void(RobotAbnormalCodes)>;

/// IRobotSetting

/// \brief 机器人标识设置
using RsIdentity = const robot::setting::Identity &;
using RsIdentityRspFun = std::function<void(RsIdentity, RspOK, RspMsg)>;

/// \brief 连接服务器配置
using RsServer = const robot::setting::Server &;
using RsServerRspFun = std::function<void(RsServer, RspOK, RspMsg)>;

/// \brief 开关自主回充
using RsAutoCharge = const robot::setting::AutoCharge &;
using RsAutoChargeRspFun = std::function<void(RsAutoCharge, RspOK, RspMsg)>;

/// \brief 开关自主泊车
using RsAutoPark = const robot::setting::AutoPark &;
using RsAutoParkRspFun = std::function<void(RsAutoPark, RspOK, RspMsg)>;

/// \brief 开关货物检测
using RsGoodsCheck = const robot::setting::GoodsCheck &;
using RsGoodsCheckRspFun = std::function<void(RsGoodsCheck, RspOK, RspMsg)>;

/// \brief 电量配置
using RsPower = const robot::setting::Power &;
using RsPowerRspFun = std::function<void(RsPower, RspOK, RspMsg)>;

/// IMapInfo

/// \brief 场景列表
using MapSceneList = const map::SceneList &;
using SceneListRspFun = std::function<void(MapSceneList, RspOK, RspMsg)>;

/// \brief 场景数据
using MapSceneData = const map::SceneData &;
using SceneDataRspFun = std::function<void(MapSceneData, RspOK, RspMsg)>;

/// \brief 场景数据(简易版)
using MapSceneDataEasy = const map::SceneDataEasy &;
using SceneDataEasyRspFun =
    std::function<void(MapSceneDataEasy, RspOK, RspMsg)>;

/// \brief 下载地图
using MapDownload = const map::Download &;
using MapDownloadResponse = const map::DownloadResponse &;
using MapDownloadRspFun =
    std::function<void(MapDownloadResponse, RspOK, RspMsg)>;

/// \brief 上传地图
using MapUpload = const map::Upload &;

/// \brief 修改地图或场景名
using MapRename = const map::Rename &;

/// \brief 删除场景地图
using MapDelete = const map::Delete &;

/// IMapEdit

/// \brief 储位
using Storage = const map::mark::Storage &;
using StorageRspFun = std::function<void(Storage, RspOK, RspMsg)>;
/// \brief 创建储位
using StorageCreate = const map::mark::storage::Create &;
/// \brief 删除储位
using StorageDelete = const map::mark::storage::Delete &;
using StorageDeleteRspFun = std::function<void(StorageDelete, RspOK, RspMsg)>;
/// \brief 更新储位
using StorageUpdate = const map::mark::storage::Update &;
/// \brief 查找储位
using StorageFind = const map::mark::storage::Find &;

/// IDeviceInfo

/// \brief 射频遥控器
using RfRemoteController = const device::RfRemoteController &;
using RfRemoteControllerSubFun = std::function<void(RfRemoteController)>;

/// RobotInterface

/// \brief 初始化机器人
using InitRobot = const robot::InitRobot &;

/// \brief 设置机器人位姿
using SetRobotPose = const robot::SetRobotPose &;

/// \brief 设置机器人占用
using SetOccupancy = const robot::SetOccupancy &;
/// \brief 设置机器人屏蔽呼叫
using SetMuteCall = const robot::SetMuteCall &;

/// \brief 切换控制模式
using SwitchCtrlMode = const robot::SwitchControlMode &;
using SwitchCtrlModeRspFun = std::function<void(RobotMode, RspOK, RspMsg)>;

/// \brief 切换工作模式
using SwitchWorkMode = const robot::SwitchWorkMode &;
using SwitchWorkModeRspFun = std::function<void(RobotMode, RspOK, RspMsg)>;

/// \brief 切换地图
using SwitchRobotMap = const robot::SwitchMap &;

/// \brief 构图
using BuildMap = const robot::BuildMap &;
using BuildMapData = const robot::BuildMapData &;
using BuildMapDataSubFun = std::function<void(BuildMapData)>;

/// \brief 部署
using Deployment = const robot::Deployment &;
using DeploymentResponse = const robot::DeploymentResponse &;
using DeploymentRspFun = std::function<void(DeploymentResponse, RspOK, RspMsg)>;

/// \brief 执行预定义任务
using ExecPreTask = const robot::ExecPreTask &;

/// \brief 执行任务
using ExecTask = const robot::ExecTask &;

/// \brief 动作指令
using ActionOrder = const robot::ActionOrder &;

/// \brief 循环任务
using RepeatTask = const task::RepeatTask &;
using RepeatTaskRspFun = std::function<void(RepeatTask, RspOK, RspMsg)>;

/// \brief 规划导航路径
using PlanNavPath = const robot::PlanNavPath &;
using PlanNavPathRspFun = std::function<void(RobotNavPath, RspOK, RspMsg)>;

/// \brief 改变导航路径
using ChangeNavPath = const robot::ChangeNavPath &;

/// \brief 改变导航模式
using ChangeNavMode = const robot::ChangeNavMode &;

/// \brief 语音播报
using RobotSpeak = const robot::Speak &;

/// \brief 速度控制
using RobotTwist = const robot::Twist &;

/// \brief 跟随
using RobotFollow = const robot::Follow &;

/// \brief WIFi信息
using RobotWiFi = const robot::RobotWiFi &;
using RobotWiFiRspFun = std::function<void(RobotWiFi, RspOK, RspMsg)>;

/// \brief 统计运行
using RCountData = const robot::CountData &;
using RCountDataResponse = const robot::CountDataResponse &;
using RCountDataRspFun = std::function<void(RCountDataResponse, RspOK, RspMsg)>;
using RCountOperation = const robot::count::Operation &;
using RCountOperationSubFun = std::function<void(RCountOperation)>;
using RCountTask = const robot::count::Task &;
using RCountTaskSubFun = std::function<void(RCountTask)>;
using RCountStatus = const robot::count::Status &;
using RCountStatusSubFun = std::function<void(RCountStatus)>;

/// \brief 雷达点云数据
using ScannerData = const robot::ScannerData &;
using ScannerDataRspFun = std::function<void(ScannerData, RspOK, RspMsg)>;
using ScannerDataSubFun = std::function<void(ScannerData)>;

/// \brief ros call action
using CallAction = const ros::CallAction &;
using CallActionRspFun = std::function<void(CallAction, RspOK, RspMsg)>;
/// \brief ros action feedback
using Feedbacks = const ros::Feedbacks &;
using FeedbacksSubFun = std::function<void(Feedbacks)>;

/// DispatchInterface

/// \brief 机器人调度信息
using DispatchRobot = const dispatch::robot::Robot &;
using DRobotRspFun = std::function<void(DispatchRobot, RspOK, RspMsg)>;
using DRobotSubFun = std::function<void(DispatchRobot)>;

/// \brief 机器人调度列表
using DispatchRobots = const dispatch::robot::Robots &;
using DRobotsRspFun = std::function<void(DispatchRobots, RspOK, RspMsg)>;

/// \brief 切换调度场景
using DSwitchScene = const dispatch::system::SwitchScene &;

/// \brief 获取场景配置
using DSceneSettings = const dispatch::system::SceneSettings &;
using DSceneSettingsRspFun = std::function<void(DSceneSettings, RspOK, RspMsg)>;
using DSceneSettingsSubFun = std::function<void(DSceneSettings)>;

/// \brief 调度电量配置
using DChargeSettings = const dispatch::system::ChargeSettings &;
using DChargeSettingsRspFun =
    std::function<void(DChargeSettings, RspOK, RspMsg)>;
using DChargeSettingsSubFun = std::function<void(DChargeSettings)>;

/// \brief 指定机器人充电
using DGotoCharge = const dispatch::system::GotoCharge &;

/// \brief 查找任务
using DFindTask = const dispatch::task::FindTask &;
using TaskSetList = const task::TaskSetList &;
using DFindTaskRspFun = std::function<void(TaskSetList, RspOK, RspMsg)>;

/// \brief 任务指令
using DTaskOrder = const dispatch::task::TaskOrder &;

/// \brief 置顶任务
using DStickTask = const dispatch::task::StickTask &;

/// \brief 添加woosh任务
using WooshTaskSet = const task::WooshTaskSet &;
using WooshTaskSetRspFun = std::function<void(WooshTaskSet, RspOK, RspMsg)>;

/// \brief 调度预定义任务
using DExecPreTask = const dispatch::task::ExecPreTask &;
using DExecPreTaskFun = std::function<void(DExecPreTask, RspOK, RspMsg)>;

/// \brief 调度当前执行的预定义任务集
using DExecPreTasks = const dispatch::task::ExecPreTasks &;
using DExecPreTasksFun = std::function<void(DExecPreTasks, RspOK, RspMsg)>;

/// \brief 任务集状态
using DTaskSetState = const dispatch::task::TaskSetState &;
using DTaskSetStateSubFun = std::function<void(DTaskSetState)>;

/// PPL

/// \brief 通信包打印等级
enum class DLLEXPORT PrintPackLevel {
  kDoNot = 0,  // 不打印
  kHead = 1,   // 打印包头
  kBody = 3    // 打印包头和包体
};
using PPL = PrintPackLevel;
/// \brief 请求包打印等级
using ReqPPL = const PPL &;
/// \brief 应答包打印等级
using RepPPL = const PPL &;
/// \brief 订阅包打印等级
using SubPPL = const PPL &;

#define PPLD PPL::kDoNot
#define PPLH PPL::kHead
#define PPLB PPL::kBody

}  // namespace woosh

#endif  // WOOSH_ROBOT_DEF_H
