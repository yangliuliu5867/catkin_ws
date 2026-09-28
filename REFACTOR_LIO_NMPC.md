# 雷达定位与 NMPC 重构说明

本文中的“原项目”统一指当前的 `catkin_ws`。Jiangyin 只作为 NMPC、状态机使用方式和定位接法的参考。当前实飞定位只保留 Point-LIO，底层轨迹跟踪只保留 NMPC。

## 当前实飞链

默认数据流如下：

```text
MID360
  -> Point-LIO
     -> /aft_mapped_to_init       雷达里程计
     -> /cloud_registered         配准点云
     -> ROG-Map
        -> /rm_node/rog_map/occ   在线占据点云

/aft_mapped_to_init
  -> fsm_ctrl/px4_estimator
     -> /mavros/vision_pose/pose  送给 PX4 EKF
     -> /visual_slam/odom         给规划、状态估计和控制使用

在线占据点云
  -> intention_get_corridor       更新体素地图、A*和安全走廊
  -> MPD/轨迹生成
  -> /position_command
  -> /position_command_horizon    未来0.4秒的9点真实参考
  -> fsm_ctrl/single_offboard_fsm（cmd 5）
  -> NMPC
  -> /mavros/setpoint_raw/attitude
```

`real_flight_step1.launch`、`controller_stability_test.launch` 和
`offline_collision_reference_follow.launch` 的控制器只启动 NMPC。旧 `px4ctrl`、DFBC、
动捕融合、多机指令以及旧方形/圆形位置指令已经删除。

规划器生成的多项式轨迹仍要经过 `polytraj_poscmd_bridge_node` 转成
`/position_command`。这个节点位于 `real_flight_bringup`，规划到控制不再依赖旧控制器包。

## Launch 启动顺序

雷达实飞时，每条命令分别在一个终端中执行，并先 source 对应工作空间。

1. 启动 MAVROS：

   ```bash
   roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600
   ```

   如果 MAVROS 已由其他 launch 启动，则不要重复启动。

2. 启动 MID360 驱动：

   ```bash
   roslaunch livox_ros_driver2 msg_MID360.launch
   ```

   主要输出为 `/livox/lidar` 和 `/livox/imu`。

3. 启动 Point-LIO：

   ```bash
   roslaunch point_lio mapping_mid360.launch rviz:=false
   ```

   必须能看到 `/aft_mapped_to_init` 和 `/cloud_registered`。

4. 如果要构建在线占据地图，再启动 ROG-Map：

   ```bash
   roslaunch rog_map_example pointlio_mid360.launch rviz:=false
   ```

   它使用 `/cloud_registered` 和 `/aft_mapped_to_init`，发布占据点云和膨胀占据点云。
   规划器默认读取 `/rm_node/rog_map/occ`，再按自身 `DilateRadius` 做一次统一膨胀。

5. 以下项目 launch 只能选择一个：

   - NMPC 定点和轨迹跟踪调试：

     ```bash
     roslaunch real_flight_bringup controller_stability_test.launch
     ```

     默认使用不查地图的点到点最小加加速度轨迹，适合先调 NMPC 跟踪效果。

   - 原项目完整意图、碰撞、走廊、轨迹和 NMPC 链：

     ```bash
     roslaunch real_flight_bringup real_flight_step1.launch
     ```

     需要 TensorRT、CUDA 和 `mpd_splines` 的三个 engine 文件，默认使用 ROG-Map 在线地图。

   - CSV 参考轨迹复现：

     ```bash
     roslaunch real_flight_bringup offline_collision_reference_follow.launch
     ```

     用于把保存的 `/position_command` 参考轨迹重新送给 NMPC。

## 各 launch 的用途

- `real_flight_bringup/localization.launch`：Point-LIO 到 PX4 和控制系统的定位桥接。三个项目
  实飞 launch 已经包含它，只有单独检查定位时才单独启动。
- `real_flight_bringup/controller_stability_test.launch`：NMPC 定点、简单轨迹和跟踪稳定性测试。
- `real_flight_bringup/real_flight_step1.launch`：原项目完整规划、外力估计、碰撞逻辑和 NMPC 实飞链。
- `real_flight_bringup/offline_collision_reference_follow.launch`：离线 CSV 轨迹复现。
- `real_flight_bringup/collision_recovery_nmpc.launch`：旧碰撞恢复状态机 + NMPC 跟踪。
- `real_flight_bringup/thrust_calibrate_nmpc.launch`：记录 NMPC 油门指令与电池电压。
- `real_flight_bringup/collision_planning_preview.launch`：假里程计下的规划预览，不控制真机。
- `fsm_ctrl/px4_estimator.launch`：单独启动定位源到 PX4 的桥接。
- `fsm_ctrl/single.launch`：单独启动状态机和 NMPC 的测试入口，不启动 Point-LIO 和规划器。
- `rog_map_example/pointlio_mid360.launch`：位于独立的 `lio` 工作空间，用雷达点云在线更新 ROG-Map。

运行任一项目实飞 launch 时，不要再单独启动 `localization.launch`、
`px4_estimator.launch` 或 `single.launch`，否则会出现重复节点或重复发布控制指令。

## 状态机命令

- `cmd 1`：原低油门和解锁测试。
- `cmd 2`：使用 PX4 位置设定点起飞到 1 m。
- `cmd 3`：NMPC 悬停。单独运行时默认目标为 `(0, 0, 0.5)`；项目实飞 launch
  使用 `pre_align_altitude` 覆盖 Z，默认是 1.0 m。
- `cmd 4`：原降落和上锁流程。
- `cmd 5`：NMPC 跟踪规划器输出的 `/position_command`。
- `cmd 7`：NMPC 八字轨迹测试。

UDP 指令端口保持为 `12001`。

## 雷达定位

定位源固定为 Point-LIO：

```bash
roslaunch real_flight_bringup controller_stability_test.launch
```

雷达安装俯仰补偿默认是绕机体系 Y 轴 `-25.3°`，参数为
`lidar_to_body_pitch_deg`。只有确认实际安装角度后才修改。

`lidar_imu_state_bridge` 会把雷达里程计和 MAVROS IMU 合成为
`quadrotor_msgs/EstimatorState`，发布到 `/estimator/state`，外力估计和碰撞链继续使用该接口。

## 在线建图与在线规划

Point-LIO 实时输出定位和配准点云，ROG-Map 的现有配置已对齐：

- 点云输入：`/cloud_registered`
- 里程计输入：`/aft_mapped_to_init`
- 占据点云：`/rm_node/rog_map/occ`
- 膨胀占据点云：`/rm_node/rog_map/inf_occ`

`real_flight_step1.launch` 默认参数为：

```yaml
PointCloudUsePCD: false
OnlineMapTopic: /rm_node/rog_map/occ
OnlineMapUpdatePeriod: 0.5
OnlineReplanPeriod: 1.0
```

在线点云到达后，规划器在 `R3Bound` 范围内重建自己的占据栅格，按原参数膨胀障碍，
原子替换旧地图，并使旧 A*、局部路径和走廊缓存全部失效。有活动目标且当前不在执行碰撞
过渡时，地图更新会触发异步重规划。规划器膨胀后的可视化点云改发到
`/planner/inflated_map`，不会再反馈到在线地图输入。

当前接入的是 ROG-Map 的已占据点云，因此规划策略会把尚未观测到的空间当作可通行空间，
靠雷达不断发现新障碍并周期重规划。这适合“目标点已知、环境未知”的在线导航。若要做
严格的未知区禁入或自主探索，还需要把 ROG-Map 的未知栅格状态接入搜索代价，并增加前沿
目标选择；这不属于本次目标点导航链。

默认规划边界在 `intention_real.yaml` 中为 XY 各 ±10 m、Z 为 0～3 m。实飞场地超过该范围
时应先修改 `R3Bound`。静态 PCD 功能仍保留，可用下面的参数恢复：

```bash
roslaunch real_flight_bringup real_flight_step1.launch \
  point_cloud_use_pcd:=true
```

## /debugPx4ctrl 兼容调试话题

NMPC 状态机继续发布原话题名：

```text
/debugPx4ctrl
```

消息类型仍为 `quadrotor_msgs/Px4ctrlDebug`，当前由 NMPC 填写：

- 期望位置、速度、加速度和姿态；
- NMPC 输出的机体系角速度和油门；
- IMU 加速度、角速度反馈；
- 悬停油门、推力到加速度比例和电池电压；
- 姿态跟踪误差。

原来的 PlotJuggler 布局可以继续订阅同名话题。NMPC 预测时域的完整参考和反馈仍在
`/nmpc_state`。

## 当前仍保留的辅助功能

- `direct_gcopter_frontend.py`：用于不避障的 NMPC 点到点测试。
- 离线 CSV 回放、假碰撞触发、规划预览等台架复现实用工具。
- 静态 PCD、`generate_static_map.py` 和静态地图配置：在线主链不用，作为旧实验回退保留。
- Unity 和飞行仿真包：真机不启动，但其中 `quadrotor_msgs` 仍是实飞消息依赖，不能整目录删除。
- 编译生成目录和 Python 缓存：不属于项目功能，已加入 `.gitignore`，已跟踪的缓存会从 Git 中移除。

## 已删除的旧链路

- `imu_mocap_fusion` 整包、动捕假数据/修正节点和动捕配置；
- `px4ctrl` 整包及 PD、SO3 等底层控制代码；旧碰撞恢复已迁移到
  `real_flight_bringup/collision_recovery_nmpc_node`，输出改由 NMPC 跟踪；
- DFBC、多机指令节点和 `swarm.launch`；
- `cmd 6`、`cmd 8` 及其旧方形/圆形位置轨迹；
- 动捕、相机/VIO 在 `px4_estimator` 中的输入分支；
- `px4ctrl_override.yaml`、`px4ctrl_poscmd_override.yaml` 等旧控制参数。

`/debugPx4ctrl` 和 `quadrotor_msgs/Px4ctrlDebug` 仅作为现有 PlotJuggler 布局的兼容调试接口保留，
其中的数据由 NMPC 填写，不包含旧控制器执行逻辑。

原推力标定的数据记录能力已迁移到 `thrust_calibrate_nmpc.py`，它直接记录 NMPC 发送的
油门和电池电压，不包含旧控制算法。

## 运行依赖

- ROS Noetic、MAVROS、Eigen。
- NMPC 需要 CasADi C++ 和 IPOPT。
- 雷达定位需要独立 `lio` 工作空间中的 Point-LIO 和 `livox_ros_driver2`。
- 在线占据地图使用独立 `lio` 工作空间中的 ROG-Map。
- 完整学习型碰撞规划需要 CUDA、TensorRT 和 `mpd_splines` 的三个 engine 文件。

## 主动碰撞实验保持不变

重构只把最终轨迹跟踪控制器从原来的 PD 换成 NMPC。主动碰撞实验的规划与执行思路保持为：

```text
主动撞击 -> 按碰撞模型完成速度过渡/恢复 -> 连续跟踪碰撞后轨迹
```

`real_flight_step1.launch` 中的轨迹桥接器默认关闭
`use_external_force_collision_hold`，因此不会在检测到外力后改成定点悬停并等待重规划。
规划器产生的碰撞前轨迹、碰撞事件和碰撞后轨迹会连续转换成
`/position_command`，由 `cmd 5` 下的 NMPC 负责跟踪。摩擦、阻尼、碰撞速度上限、
碰撞点和碰撞后速度计算仍沿用原项目。

外力触发定点保持仍作为轨迹桥接器的可选模式保留；旧 PD 底层控制器已经删除，碰撞恢复中
用于生成位置、速度和加速度参考的导纳模型继续保留。

旧碰撞恢复实验另有独立入口：

```bash
roslaunch real_flight_bringup collision_recovery_nmpc.launch
```

先输入 `cmd 3` 完成悬停，再输入 `cmd 5`。恢复节点保留原来的外力触发、导纳响应、速度
衰减和定点停止状态，输出 `/position_command`；NMPC 是唯一产生姿态角速度和油门的控制器。

## NMPC真实轨迹预测窗口

轨迹桥接器在保留原 `/position_command` 的同时，新增
`/position_command_horizon`。该消息包含9个状态参考点，间隔0.05秒，总预测范围0.4秒。
每个点都直接从完整多项式轨迹采样，并经过原碰撞速度过渡逻辑，因此NMPC可以看到真实的
碰撞前、碰撞瞬间和碰撞后参考，不再只根据单个点做恒加速度外推。

NMPC优先使用该9点窗口；窗口缺失或超时后自动回退到原 `/position_command` 局部外推，
因此原测试节点和离线回放接口继续可用。

## XY位置积分修正

NMPC接入了本机 `Jiangyin/nmpc` 分支的有界XY低频位置积分。默认参数为：

```yaml
nmpc_enable_xy_integral: true
nmpc_xy_integral_gain: 0.08
nmpc_xy_integral_limit: 0.10
nmpc_xy_integral_leak: 0.01
nmpc_xy_integral_max_error: 0.30
```

积分修正统一加到整个NMPC参考窗口的X、Y位置上，不修改速度、加速度、轨迹形状和碰撞时序。
位置误差超过0.30米时停止累积，修正量每轴限制在±0.10米，状态机命令切换时清零。

## MPD模型接入

`real_flight_step1.launch` 使用 `mpd_model_dir` 统一指定MPD模型目录，并把三个engine路径同时
传给 `intention_get_corridor` 和 `traj_gen_in_corridor`。Orin NX示例：

```bash
roslaunch real_flight_bringup real_flight_step1.launch \
  mpd_model_dir:=/实际绝对路径/mpd_splines/small
```
