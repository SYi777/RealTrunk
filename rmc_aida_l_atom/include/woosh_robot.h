#ifndef WOOSHROBOT_H
#define WOOSHROBOT_H

#include "woosh_robot_def.h"

namespace woosh {

/// \brief 通信连接设置
struct CommuSetting {
  /// \brief 远程地址
  std::string addr;
  /// \brief 远程端口
  uint32_t port;
  /// \brief 客户端标识
  std::string identity;

  /// \brief poll超时(msg)
  int32_t poll_timeout;

  /// \brief 通信包打印最大长度
  size_t print_pack_max_byte_size = 2048;

  /// \brief 日志打印回调函数
  LogCallFun log_call_fun;
  /// \brief 通信包打印回调函数
  PrintPackCallFun print_pack_call_fun;
  /// \brief 连接状态回调函数
  ConnectStatusCallFun connect_status_call_fun;

  CommuSetting() {
    identity = "WOOSDK";
    poll_timeout = 10000;             // 10s
    print_pack_max_byte_size = 2048;  // 2kb
  }
};

/// \brief common
class DLLEXPORT ICommon {
 public:
  virtual ~ICommon() = default;

  /// \brief 运行SDK, 调用一次, SDK实例化后调用的第一个方法
  virtual bool run() = 0;

  /// \brief 取消订阅
  virtual bool unsubscribe(const std::string &topic) = 0;
};

/// \brief 机器人信息相关接口类
class DLLEXPORT IRobotInfo {
 public:
  virtual ~IRobotInfo() = default;

  /// \brief 机器人信息请求
  virtual bool robotInfoReq(RobotInfo, RobotInfoRspFun, ReqPPL = PPLD,
                            RepPPL = PPLD) = 0;

  /// \brief 常规信息请求
  virtual bool robotGeneralReq(RobotGeneral, GeneralRspFun, ReqPPL = PPLD,
                               RepPPL = PPLD) = 0;

  /// \brief 配置信息请求
  virtual bool robotSettingReq(RobotSetting, SettingRspFun, ReqPPL = PPLD,
                               RepPPL = PPLD) = 0;
  /// \brief 配置信息订阅
  virtual bool robotSettingSub(SettingSubFun, SubPPL = PPLD) = 0;

  /// \brief 机器人状态请求
  virtual bool robotStateReq(RobotState, RobotStateRspFun, ReqPPL = PPLD,
                             RepPPL = PPLD) = 0;
  /// \brief 机器人状态订阅
  virtual bool robotStateSub(RobotStateSubFun, SubPPL = PPLD) = 0;

  /// \brief 机器人模式请求
  virtual bool robotModeReq(RobotMode, ModeRspFun, ReqPPL = PPLD,
                            RepPPL = PPLD) = 0;
  /// \brief 机器人模式订阅
  virtual bool robotModeSub(ModeSubFun, SubPPL = PPLD) = 0;

  /// \brief 位姿速度请求
  virtual bool robotPoseSpeedReq(RobotPoseSpeed, PoseSpeedRspFun, ReqPPL = PPLD,
                                 RepPPL = PPLD) = 0;
  /// \brief 位姿速度订阅
  virtual bool robotPoseSpeedSub(PoseSpeedSubFun, SubPPL = PPLD) = 0;

  /// \brief 电量信息请求
  virtual bool robotBatteryReq(RobotBattery, BatteryRspFun, ReqPPL = PPLD,
                               RepPPL = PPLD) = 0;
  /// \brief 电量信息订阅
  virtual bool robotBatterySub(BatterySubFun, SubPPL = PPLD) = 0;

  /// \brief 网络信息请求
  virtual bool robotNetworkReq(RobotNetwork, NetworkRspFun, ReqPPL = PPLD,
                               RepPPL = PPLD) = 0;
  /// \brief 网络信息订阅
  virtual bool robotNetworkSub(NetworkSubFun, SubPPL = PPLD) = 0;

  /// \brief 场景信息请求
  virtual bool robotSceneReq(RobotScene, SceneRspFun, ReqPPL = PPLD,
                             RepPPL = PPLD) = 0;
  /// \brief 场景信息订阅
  virtual bool robotSceneSub(SceneSubFun, SubPPL = PPLD) = 0;

  /// \brief 任务进度信息请求
  virtual bool robotTaskProcessReq(RobotTaskProcess, TaskProcessRspFun,
                                   ReqPPL = PPLD, RepPPL = PPLD) = 0;
  /// \brief 任务进度信息订阅
  virtual bool robotTaskProcessSub(TaskProcessSubFun, SubPPL = PPLD) = 0;

  /// \brief 设备状态信息请求
  virtual bool robotDeviceStateReq(RobotDeviceState, DeviceStateRspFun,
                                   ReqPPL = PPLD, RepPPL = PPLD) = 0;
  /// \brief 设备状态信息订阅
  virtual bool robotDeviceStateSub(DeviceStateSubFun, SubPPL = PPLD) = 0;

  /// \brief 硬件状态信息请求
  virtual bool robotHardwareStateReq(RobotHardwareState, HardwareStateRspFun,
                                     ReqPPL = PPLD, RepPPL = PPLD) = 0;
  /// \brief 硬件状态信息订阅
  virtual bool robotHardwareStateSub(HardwareStateSubFun, SubPPL = PPLD) = 0;

  /// \brief 运行状态信息请求
  virtual bool robotOperationStateReq(RobotOperationState, OperationStateRspFun,
                                      ReqPPL = PPLD, RepPPL = PPLD) = 0;
  /// \brief 运行状态信息订阅
  virtual bool robotOperationStateSub(OperationStateSubFun, SubPPL = PPLD) = 0;

  /// \brief 机器人模型请求
  virtual bool robotModelReq(RobotModel, ModelRspFun, ReqPPL = PPLD,
                             RepPPL = PPLD) = 0;
  /// \brief 机器人模型订阅
  virtual bool robotModelSub(ModelSubFun, SubPPL = PPLD) = 0;

  /// \brief 机器人导航路径请求
  virtual bool robotNavPathReq(RobotNavPath, RobotNavPathRspFun, ReqPPL = PPLD,
                               RepPPL = PPLD) = 0;
  /// \brief 机器人导航路径订阅
  virtual bool robotNavPathSub(RobotNavPathSubFun, SubPPL = PPLD) = 0;

  /// \brief 历史任务请求(最近50条)
  virtual bool robotTaskHistoryReq(RobotTaskHistory, TaskHistoryRspFun,
                                   ReqPPL = PPLD, RepPPL = PPLD) = 0;

  /// \brief 状态码请求(最近50条)
  virtual bool robotStatusCodesReq(RobotStatusCodes, StatusCodesRspFun,
                                   ReqPPL = PPLD, RepPPL = PPLD) = 0;
  /// \brief 状态码订阅
  virtual bool robotStatusCodeSub(StastusCodeSubFun, SubPPL = PPLD) = 0;

  /// \brief 未恢复的异常码请求
  virtual bool robotAbnormalCodesReq(RobotAbnormalCodes, AbnormalCodesRspFun,
                                     ReqPPL = PPLD, RepPPL = PPLD) = 0;
  /// \brief 未恢复的异常码订阅
  virtual bool robotAbnormalCodesSub(AbnormalCodesSubFun, SubPPL = PPLD) = 0;
};

/// \brief 机器人配置接口类
class DLLEXPORT IRobotSetting {
 public:
  virtual ~IRobotSetting() = default;

  /// \brief 设置标识
  virtual bool setIdentity(RsIdentity, RsIdentityRspFun, ReqPPL = PPLD,
                           RepPPL = PPLD) = 0;

  /// \brief 设置连接服务器配置
  virtual bool setServer(RsServer, RsServerRspFun, ReqPPL = PPLD,
                         RepPPL = PPLD) = 0;

  /// \brief 开关自主回充
  virtual bool autoCharge(RsAutoCharge, RsAutoChargeRspFun, ReqPPL = PPLD,
                          RepPPL = PPLD) = 0;

  /// \brief 开关自主泊车
  virtual bool autoPark(RsAutoPark, RsAutoParkRspFun, ReqPPL = PPLD,
                        RepPPL = PPLD) = 0;

  /// \brief 开关货物检测
  virtual bool goodsCheck(RsGoodsCheck, RsGoodsCheckRspFun, ReqPPL = PPLD,
                          RepPPL = PPLD) = 0;

  /// \brief 电量配置
  virtual bool configPower(RsPower, RsPowerRspFun, ReqPPL = PPLD,
                           RepPPL = PPLD) = 0;
};

/// \brief 地图相关接口
class DLLEXPORT IMapInfo {
 public:
  virtual ~IMapInfo() = default;

  /// \brief 场景列表请求
  virtual bool sceneListReq(MapSceneList, SceneListRspFun, ReqPPL = PPLD,
                            RepPPL = PPLD) = 0;

  /// \brief 场景数据请求
  virtual bool sceneDataReq(MapSceneData, SceneDataRspFun, ReqPPL = PPLD,
                            RepPPL = PPLD) = 0;

  /// \brief 场景数据请求(简易版)
  virtual bool sceneDataEasyReq(MapSceneDataEasy, SceneDataEasyRspFun,
                                ReqPPL = PPLD, RepPPL = PPLD) = 0;

  /// \brief 下载地图请求
  virtual bool downloadMap(MapDownload, MapDownloadRspFun, ReqPPL = PPLD,
                           RepPPL = PPLD) = 0;

  /// \brief 上传地图请求
  virtual bool uploadMap(MapUpload, EmptyRspFun, ReqPPL = PPLD,
                         RepPPL = PPLD) = 0;

  /// \brief 修改地图或场景名请求
  virtual bool renameMap(MapRename, EmptyRspFun, ReqPPL = PPLD,
                         RepPPL = PPLD) = 0;

  /// \brief 删除场景或地图请求
  virtual bool deleteMap(MapDelete, EmptyRspFun, ReqPPL = PPLD,
                         RepPPL = PPLD) = 0;
};

/// \brief 地图编辑接口
class DLLEXPORT IMapEdit {
 public:
  virtual ~IMapEdit() = default;

  /// \brief 创建储位请求
  virtual bool createStorage(StorageCreate, StorageRspFun, ReqPPL = PPLD,
                             RepPPL = PPLD) = 0;

  /// \brief 删除储位请求
  virtual bool deleteStorage(StorageDelete, StorageDeleteRspFun, ReqPPL = PPLD,
                             RepPPL = PPLD) = 0;

  /// \brief 更新储位请求
  virtual bool updateStorage(StorageUpdate, StorageRspFun, ReqPPL = PPLD,
                             RepPPL = PPLD) = 0;

  /// \brief 查找储位请求
  virtual bool findStorage(StorageFind, StorageRspFun, ReqPPL = PPLD,
                           RepPPL = PPLD) = 0;
};

/// \brief 外围设备相关接口
class DLLEXPORT IDeviceInfo {
 public:
  virtual ~IDeviceInfo() = default;

  /// \brief 射频遥控器信号订阅
  virtual bool rfRemoteControllerSub(RfRemoteControllerSubFun,
                                     SubPPL = PPLD) = 0;
};

/// \brief 代理机器人接口(调度转发)
class DLLEXPORT IRobotProxy {
 public:
  virtual ~IRobotProxy() = default;

  /// \brief 部署请求
  virtual bool deploymentReq(Deployment, DeploymentRspFun, ReqPPL = PPLD,
                             RepPPL = PPLD) = 0;

  /// \brief 规划导航路径请求
  virtual bool planNavPathReq(PlanNavPath, PlanNavPathRspFun, ReqPPL = PPLD,
                              RepPPL = PPLD) = 0;

  /// \brief 雷达数据请求
  virtual bool scannerDataReq(ScannerData, ScannerDataRspFun, ReqPPL = PPLD,
                              RepPPL = PPLD) = 0;
};

/// \brief 机器人接口类
class DLLEXPORT RobotInterface : virtual public ICommon,
                                 virtual public IRobotInfo,
                                 virtual public IRobotSetting,
                                 virtual public IMapInfo,
                                 virtual public IMapEdit,
                                 virtual public IDeviceInfo,
                                 virtual public IRobotProxy {
 public:
  virtual ~RobotInterface() = default;

  /// \brief 初始化机器人请求
  virtual bool initRobotReq(InitRobot, EmptyRspFun, ReqPPL = PPLD,
                            RepPPL = PPLD) = 0;

  /// \brief 设置机器人位姿
  virtual bool setRobotPoseReq(SetRobotPose, EmptyRspFun, ReqPPL = PPLD,
                               RepPPL = PPLD) = 0;

  /// \brief 设置机器人占用
  virtual bool setOccupancyReq(SetOccupancy, EmptyRspFun, ReqPPL = PPLD,
                               RepPPL = PPLD) = 0;

  /// \brief 设置机器人屏蔽呼叫
  virtual bool setMuteCallReq(SetMuteCall, EmptyRspFun, ReqPPL = PPLD,
                              RepPPL = PPLD) = 0;

  /// \brief 切换控制模式请求
  virtual bool switchControlModeReq(SwitchCtrlMode, SwitchCtrlModeRspFun,
                                    ReqPPL = PPLD, RepPPL = PPLD) = 0;

  /// \brief 切换工作模式请求
  virtual bool switchWorkModeReq(SwitchWorkMode, SwitchWorkModeRspFun,
                                 ReqPPL = PPLD, RepPPL = PPLD) = 0;

  /// \brief 切换地图请求
  virtual bool switchMapReq(SwitchRobotMap, EmptyRspFun, ReqPPL = PPLD,
                            RepPPL = PPLD) = 0;

  /// \brief 构图请求
  virtual bool buildMapReq(BuildMap, EmptyRspFun, ReqPPL = PPLD,
                           RepPPL = PPLD) = 0;
  /// \brief 构图数据订阅
  virtual bool buildMapDataSub(BuildMapDataSubFun, SubPPL = PPLD) = 0;

  /// RP deploymentReq

  /// \brief 执行预定义任务请求
  virtual bool execPreTaskReq(ExecPreTask, EmptyRspFun, ReqPPL = PPLD,
                              RepPPL = PPLD) = 0;

  /// \brief 执行任务请求
  virtual bool execTaskReq(ExecTask, EmptyRspFun, ReqPPL = PPLD,
                           RepPPL = PPLD) = 0;

  /// \brief 动作指令请求
  virtual bool actionOrderReq(ActionOrder, EmptyRspFun, ReqPPL = PPLD,
                              RepPPL = PPLD) = 0;

  /// \brief 循环任务请求
  virtual bool repeatTaskReq(RepeatTask, RepeatTaskRspFun, ReqPPL = PPLD,
                             RepPPL = PPLD) = 0;

  /// RP planNavPathReq

  /// \brief 改变导航路径请求
  virtual bool changeNavPathReq(ChangeNavPath, EmptyRspFun, ReqPPL = PPLD,
                                RepPPL = PPLD) = 0;

  /// \brief 改变导航模式请求
  virtual bool changeNavModeReq(ChangeNavMode, EmptyRspFun, ReqPPL = PPLD,
                                RepPPL = PPLD) = 0;

  /// \brief 语音播报请求
  virtual bool speakReq(RobotSpeak, EmptyRspFun, ReqPPL = PPLD,
                        RepPPL = PPLD) = 0;

  /// \brief 速度控制请求
  virtual bool twistReq(RobotTwist, EmptyRspFun, ReqPPL = PPLD,
                        RepPPL = PPLD) = 0;

  /// \brief 跟随请求
  virtual bool followReq(RobotFollow, EmptyRspFun, ReqPPL = PPLD,
                         RepPPL = PPLD) = 0;

  /// \brief WIFi信息请求
  virtual bool robotWiFiReq(RobotWiFi, RobotWiFiRspFun, ReqPPL = PPLD,
                            RepPPL = PPLD) = 0;

  /// \brief 运行统计信息请求
  virtual bool countDataReq(RCountData, RCountDataRspFun, ReqPPL = PPLD,
                            RepPPL = PPLD) = 0;

  /// \brief 运行统计运作订阅
  virtual bool robotCountOperationSub(RCountOperationSubFun, SubPPL = PPLD) = 0;
  /// \brief 运行统计任务订阅
  virtual bool robotCountTaskSub(RCountTaskSubFun, SubPPL = PPLD) = 0;
  /// \brief 运行统计状态订阅
  virtual bool robotCountStatusSub(RCountStatusSubFun, SubPPL = PPLD) = 0;

  /// RP scannerDataReq

  /// \brief 雷达点云数据订阅
  virtual bool scannerDataSub(ScannerDataSubFun, SubPPL = PPLD) = 0;

  // ros

  /// \brief ros call action
  virtual bool callActionReq(CallAction, CallActionRspFun, ReqPPL = PPLD,
                             RepPPL = PPLD) = 0;
  /// \brief ros action feedback
  virtual bool feedbacksSub(FeedbacksSubFun, SubPPL = PPLD) = 0;
};

/// \brief 调度系统接口类
class DLLEXPORT DispatchInterface : virtual public ICommon,
                                    virtual public IRobotInfo,
                                    virtual public IRobotSetting,
                                    virtual public IMapInfo,
                                    virtual public IRobotProxy {
 public:
  virtual ~DispatchInterface() = default;

  /// \brief 机器人调度信息请求
  virtual bool dispatchRobotReq(DispatchRobot, DRobotRspFun, ReqPPL = PPLD,
                                RepPPL = PPLD) = 0;

  /// \brief 机器人调度信息订阅
  virtual bool dispatchRobotSub(DRobotSubFun, SubPPL = PPLD) = 0;

  /// \brief 机器人调度列表请求
  virtual bool dispatchRobotsReq(DispatchRobots, DRobotsRspFun, ReqPPL = PPLD,
                                 RepPPL = PPLD) = 0;

  /// \brief 切换场景请求
  virtual bool switchSceneReq(DSwitchScene, EmptyRspFun, ReqPPL = PPLD,
                              RepPPL = PPLD) = 0;

  /// \brief 获取场景配置请求
  virtual bool sceneSettingsReq(DSceneSettings, DSceneSettingsRspFun,
                                ReqPPL = PPLD, RepPPL = PPLD) = 0;

  /// \brief 场景配置更新订阅
  virtual bool sceneSettingsSub(DSceneSettingsSubFun, SubPPL = PPLD) = 0;

  /// \brief 调度电量配置
  virtual bool chargeSettingsReq(DChargeSettings, DChargeSettingsRspFun,
                                 ReqPPL = PPLD, RepPPL = PPLD) = 0;

  /// \brief 调度电量配置更新订阅
  virtual bool chargeSettingsSub(DChargeSettingsSubFun, SubPPL = PPLD) = 0;

  /// \brief 指定机器人充电请求
  virtual bool gotoChargeReq(DGotoCharge, EmptyRspFun, ReqPPL = PPLD,
                             RepPPL = PPLD) = 0;

  /// \brief 查找任务请求
  virtual bool findTaskReq(DFindTask, DFindTaskRspFun, ReqPPL = PPLD,
                           RepPPL = PPLD) = 0;

  /// \brief 任务指令请求
  virtual bool taskOrderReq(DTaskOrder, EmptyRspFun, ReqPPL = PPLD,
                            RepPPL = PPLD) = 0;

  /// \brief 置顶任务请求
  virtual bool stickTaskReq(DStickTask, EmptyRspFun, ReqPPL = PPLD,
                            RepPPL = PPLD) = 0;

  /// \brief 添加任务请求
  virtual bool addTaskReq(WooshTaskSet, WooshTaskSetRspFun, ReqPPL = PPLD,
                          RepPPL = PPLD) = 0;

  /// \brief 执行调度预定义任务请求
  virtual bool execPreTaskReq(DExecPreTask, DExecPreTaskFun, ReqPPL = PPLD,
                              RepPPL = PPLD) = 0;

  /// \brief 调度当前执行的预定义任务集
  virtual bool execPreTasksReq(DExecPreTasks, DExecPreTasksFun, ReqPPL = PPLD,
                               RepPPL = PPLD) = 0;

  /// \brief 任务集状态订阅
  virtual bool taskSetStateSub(DTaskSetStateSubFun, SubPPL = PPLD) = 0;
};

/// \brief 机器人SDK
using RobotPtr = std::shared_ptr<RobotInterface>;
/// \brief 调度SDK
using DispatchPtr = std::shared_ptr<DispatchInterface>;

class DLLEXPORT Factory {
 public:
  /// \brief 创建机器人连接实例
  static RobotPtr newRobotInterface(const CommuSetting &cs);
  /// \brief 创建调度连接实例
  static DispatchPtr newDispatchInterface(const CommuSetting &cs);

  /// \brief 返回SDK库版本号
  static std::string version();

  /// \brief 返回依赖通讯库版本
  static std::string commuVer();

  /// \brief 返回库所依赖的msg版本
  static std::string protoVer();
};
}  // namespace woosh

#endif  // WOOSHROBOT_H
