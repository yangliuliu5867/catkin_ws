# Jiangyin baseline：仅替换 cmd == 3 的悬停控制

来源：Jiangyin `baseline`，提交 `9e0ab72acb92bee4e81ae86ec8ac5a30c5a88669`。
按原目录引入 `src/ctrl_math` 和 `src/fsm_ctrl/fsm_ctrl`，未引入个人 IDE 配置和 .bak 文件。
保留原包名、C++ 可执行文件名、节点名、UDP 12001、定位代码和数字命令。

唯一的飞行控制替换点在 `single_offboard_fsm.cpp` 的 `case 3`：

```
动捕/雷达 → 原 px4_estimator → /mavros/vision_pose/pose → PX4 EKF
PX4 本地位置 + 世界坐标速度 + IMU
    → single_offboard_fsm 的 case 3
    → 原 px4ctrl 的 LinearControl::calculateControl
    → /mavros/setpoint_raw/attitude
```

这里不启动 `px4ctrl_node`，也不启动之前的 Python 门控/测试节点。
`single_offboard_fsm` 内嵌调用原控制器，仍是唯一控制出口。其他 case 保持 baseline。

## 原控制器如何保持不变

`src/px4ctrl` 下所有源文件、配置、launch、CMake 均未修改。
`fsm_ctrl/CMakeLists.txt` 直接编译引用原来的 `controller.cpp`、`PX4CtrlParam.cpp`、
`input.cpp`，没有拷贝控制公式或另写一套 PID。接口转换位于本包 `px4ctrl_hover.cpp`。
使用原 `estimateThrustModel` 在线推力估计：只在已解锁且 OFFBOARD 时调用。
进入 cmd=3 或 cmd=5 都会重建控制器实例，清除之前的推力样本并恢复原悬停推力映射。

参数也由原 `Parameter_t::config_from_ros_handle` 加载，不另行覆盖控制增益。
输出接口将推力限制在 MAVROS 的 [0,1] 范围。
位置、速度、IMU、飞控状态缺失或过期时，case 3 不请求解锁、不发布控制指令；
这不等于自动降落，失联动作由 PX4 自身配置决定。
原 baseline 的 OFFBOARD/解锁重试逻辑保留，没有移植 px4ctrl 的 CH5/CH9 状态机。

## 编译

在 ROS Noetic 运行机：

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
rospack find fsm_ctrl
```

最后应解析到此工作区 `src/fsm_ctrl/fsm_ctrl`，避免 source 到 Jiangyin 的同名旧包。
只编译控制相关包、避开规划的 CUDA/TensorRT 依赖：

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DCATKIN_WHITELIST_PACKAGES='cmake_utils;uav_utils;quadrotor_msgs;px4ctrl;ctrl_math;fsm_ctrl'
```

恢复全工作区编译时传 `-DCATKIN_WHITELIST_PACKAGES=''`。
移除了 baseline 清单里未使用的 bspline_race/super_msgs/traj_utils 依赖，
补上实际使用的 MAVROS 消息、IMU、px4ctrl 等依赖；命名和运行逻辑保留。

## 启动（沿用原入口）

先启动你已有的 VRPN 动捕或 LIO。下面两个终端仍沿用 Jiangyin 的启动方式：

```bash
# 终端1：MAVROS + 原 single 状态机，cmd=3/cmd=5 接入 catkin_ws 控制器
roslaunch fsm_ctrl single.launch

# MAVROS 已启动时用：
# roslaunch fsm_ctrl single.launch start_mavros:=false

# 终端2：定位 + 原输入菜单（二选一）
roslaunch fsm_ctrl swarm.launch vision_source:=0 mocap_topic:=/vrpn_client_node/catch/pose
# 或雷达：
# roslaunch fsm_ctrl swarm.launch vision_source:=1 lidar_topic:=/aft_mapped_to_init
```

若 roslaunch 的终端无法输入，可把终端2拆成：

```bash
roslaunch fsm_ctrl px4_estimator.launch vision_source:=0 mocap_topic:=/vrpn_client_node/catch/pose
# 再开终端：
rosrun fsm_ctrl swarm_user_cmd
```

不要同时运行 `swarm.launch` 与上述拆分命令，以免重复定位/菜单节点。
不要同时启动 `controller_stability_test.launch` 或 `px4ctrl/run_ctrl.launch`。

输入 `3` 执行本次接入的悬停。默认仍是 baseline 的世界/本地坐标 `(0,0,0.4)`，
不是当前位置抬升0.4米。默认 yaw=0。需要换悬停点时：

```bash
roslaunch fsm_ctrl single.launch hover_x:=0.0 hover_y:=0.0 hover_z:=1.0 hover_yaw:=0.0
```

目标参数启动时读取；修改后落地重启。数字3持续重发不会反复初始化悬停控制器。

## 调参数

`single.launch` 的 `px4ctrl_config` 默认指向原 `px4ctrl/config/tx_uav.yaml`，
加载到 `/single_offboard_fsm/px4ctrl`，未叠加其他 mass override。
本次默认质量是该文件的1.17kg；以前 controller_stability_test 的1.75kg override
不会在本入口隐式生效。参数必须与实际飞机一致。
调 Kp/Kv/悬停油门仍改原配置，或者通过 `px4ctrl_config:=/绝对路径/你的配置.yaml` 指定配置。
原 single.launch 保留的 kp_x、kv_x、NMPC 等参数不参与 cmd=3 或 cmd=5 的计算。

## 跟踪曲线

cmd=3 和 cmd=5 运行时，会以 50 Hz 发布以下 `geometry_msgs/PoseStamped` 话题：

- `/fsm_ctrl/desired_trajectory`：控制器的期望位置

实际位置直接使用 MAVROS 原话题 `/mavros/local_position/pose`。在 PlotJuggler 对比两个话题的 `pose.position.x/y/z`。
支持原 ctrl_mode 0/1 的姿态控制；不启用 bodyrate/bridge/MPC 分支。

## 原命令含义提醒

原菜单文字和 baseline 的 switch 不完全一致，本次未修改菜单或其他 case：

| cmd | baseline 实际行为 |
| --- | --- |
| 1 | 请求 OFFBOARD/解锁，发送低推力 |
| 2 | 位置目标 (0,0,1)，不是菜单上显示的 Disarm |
| 3 | 位置目标 (0,0,0.4)；本次改为 catkin_ws 姿态/推力控制 |
| 4 | 原降落分支；原有上锁条件仍保留 |
| 5/6/10 | 原圆/方形/八字位置轨迹 |

保留名称不表示这些 baseline 行为已经重新验证。

## 验证与记录

```bash
rostopic hz /mavros/local_position/pose
rostopic hz /mavros/local_position/velocity_local
rostopic hz /mavros/imu/data
rostopic info /mavros/setpoint_raw/attitude
rosbag record /mavros/local_position/pose /mavros/local_position/velocity_local \
  /mavros/imu/data /mavros/state /mavros/setpoint_raw/attitude /debugPx4ctrl

# 无需飞控的 ROS 适配测试：
catkin_make run_tests_fsm_ctrl
catkin_test_results
```

新增 rostest 检查零误差悬停推力、位置纠偏、世界坐标速度输入、数据超时和无效姿态。
开发机无 ROS/Eigen，尚不能执行这些 C++ 测试或 catkin 编译；需在运行机执行，
不能把本地静态检查当作已验证的实机稳定性。
