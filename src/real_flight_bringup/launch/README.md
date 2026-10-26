# real_flight_bringup 启动文件说明

本目录只负责 `catkin_ws` 项目的定位、规划、状态估计和控制节点。MAVROS、Livox
驱动、Point-LIO 和 ROG-Map 按需在各自工作空间中单独启动。

## 真机启动前置顺序

```bash
roslaunch mavros px4.launch fcu_url:=/dev/ttyACM0:921600
roslaunch livox_ros_driver2 msg_MID360.launch
roslaunch point_lio mapping_mid360.launch rviz:=false
```

运行默认在线模式的 `real_flight_step1.launch` 时必须启动：

```bash
roslaunch rog_map_example pointlio_mid360.launch rviz:=false
```

确认至少存在以下话题：

```text
/mavros/state
/mavros/imu/data
/aft_mapped_to_init
/cloud_registered
/rm_node/rog_map/occ
```

随后只选择一个项目 launch。

## controller_stability_test.launch

用途：先调 NMPC 稳定性和点到点跟踪，不依赖静态地图和 TensorRT 模型。

```bash
roslaunch real_flight_bringup controller_stability_test.launch \
  lidar_odom_topic:=/aft_mapped_to_init
```

默认行为：

- 雷达定位；
- 启动外力估计接口；
- 启动 `single_offboard_fsm` 和 NMPC；
- 使用简单点到点轨迹前端；
- 不启动完整走廊和学习型碰撞规划。

常用参数：

- `pre_align_altitude:=1.0`：预对齐和 cmd 3 的高度；
- `lidar_odom_topic:=/aft_mapped_to_init`：雷达里程计；
- `lidar_to_body_pitch_deg:=-25.3`：雷达安装俯仰补偿；
- `enable_direct_gcopter_frontend:=true`：启用不查地图的点到点轨迹；
- `enable_intention:=false`：是否启用原走廊规划前端；
- `enable_traj_gen:=false`：是否启用多项式轨迹生成；
- `enable_polytraj_bridge:=false`：是否把多项式轨迹转换为
  `/position_command`。

## real_flight_step1.launch

用途：启动原项目完整的意图、走廊、轨迹、外力估计、碰撞逻辑和 NMPC 控制链。

主动碰撞执行逻辑保持原项目思路：碰撞前轨迹、碰撞速度过渡和碰撞后恢复轨迹连续执行，
轨迹桥接器同时保留 `/position_command` 并发布 `/position_command_horizon`，NMPC优先使用
后者的9点真实预测窗口。默认不会在碰撞后切换成定点悬停等待重规划。

### Jiangyin交互终端与RViz目标点

完整系统启动后，另开一个终端运行原Jiangyin使用习惯的交互界面：

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch fsm_ctrl swarm.launch
```

该终端实时显示Point-LIO原始定位、送入PX4的视觉位姿、PX4 Local Position、
MAVROS连接/解锁/模式以及状态机当前命令，并通过原UDP 12001接口发送cmd。
完整系统已经启动`px4_estimator`，因此不要给`swarm.launch`传
`start_estimator:=true`。

RViz中使用“2D Nav Goal”可点击目标XY。`rviz_goal_bridge`会补上目标高度并转发到
规划器的`/goal`。交互选点启动方式：

```bash
roslaunch real_flight_bringup real_flight_step1.launch \
  enable_goal_publish:=false rviz_goal_z:=1.0
```

必须关闭`enable_goal_publish`，否则`mission.yaml`里的`SetPos`仍会自动发布。规划器可视化包括：

- `/visualizer/selected_candidate`：前端代价函数从候选中选出的轨迹（绿色粗线）；
- `/visualizer/trajectory`：后端MINCO优化后真正发送给NMPC跟踪的参考轨迹；
- `/path`：实际定位轨迹，仅用于和参考轨迹比较，不属于规划候选；
- `/visualizer/route`：规划路线；
- `/visualizer/waypoints`：轨迹关键点；
- `/visualizer/mesh`和`/visualizer/edge`：安全走廊。

```bash
roslaunch real_flight_bringup real_flight_step1.launch \
  lidar_odom_topic:=/aft_mapped_to_init
```

该 launch 默认从 `/rm_node/rog_map/occ` 接收 ROG-Map 的在线占据点云。地图有效更新后，
规划器会废弃旧的 A* 路径和走廊，并按设定频率重新生成规划；规划器自己膨胀后的地图发布到
`/planner/inflated_map`，避免和输入话题形成回环。它仍需要 CUDA、TensorRT 和
`mpd_splines` 的三个 engine 文件。

单点流程的轨迹执行使用 `fsm_ctrl` 的 `cmd 5`：进入 `cmd 5` 时，状态机向
`/traj_start_trigger` 发布 `start`，采样桥开始发布 `/position_command` 和
`/position_command_horizon`。如果轨迹已在 `cmd 3` 悬停期间生成，进入 `cmd 5`
后从该轨迹的起点计时；如果尚未生成，则保持当前位置，等待正式轨迹。
退出轨迹跟踪命令（例如切回 `cmd 3` 或输入 `cmd 4`）时发布 `stop`，采样桥停止跟踪指令。
不需要再手动发布启动触发。执行前应在 RViz 核对选中轨迹及撞击位置。
目标发布节点会等待 `cmd 3`（已切换到 `cmd 5` 也可）、解锁、OFFBOARD 和
悬停高度持续满足条件后，才开始预对齐并发布目标。等待操作和起飞的时间不计入
预对齐的 20 秒超时，避免进入 OFFBOARD 后飞机尚未起飞就取消目标发布。

常用参数：

- `enable_goal_publish:=true`：当`Mission.mode: single`时自动发送`mission.yaml`中的`SetPos`；
- `enable_waypoint_executor:=true`：当`Mission.mode: multi`时读取`Mission.segments.cmd5/cmd6`；
- `enable_rviz_goal:=true`：接收RViz的`/move_base_simple/goal`并转发给规划器；
- `rviz_goal_z:=1.0`：RViz点击目标使用的固定高度；
- `point_cloud_use_pcd:=false`：默认在线地图；改成 `true` 可回到原静态 PCD；
- `online_map_topic:=/rm_node/rog_map/occ`：在线原始占据点云；
- `online_map_update_period:=0.5`：重建规划占据栅格的最小间隔，单位秒；
- `online_replan_period:=1.0`：有活动目标时重规划的最小间隔，单位秒；
- `pre_align_altitude:=1.0`：规划前悬停高度；
- `engine_collision_bs72`、`engine_collision_bs1`、`engine_student`：
  TensorRT engine 路径；
- `mpd_model_dir`：三个 MPD TensorRT engine 所在目录，三个单独路径默认由该目录生成；
- `pointseq_topic`：规划与轨迹生成之间的点序列话题；
- `ext_force_topic`：外力估计输出话题。
- `use_external_force_collision_hold:=false`：保持原主动碰撞连续轨迹；只有在明确需要
  “碰撞后定点保持并等待新轨迹”的备用模式时才设为 `true`。

默认从当前用户的 `~/mpd_splines/small` 加载。在Orin NX上放到该目录后可直接启动：

```bash
roslaunch real_flight_bringup real_flight_step1.launch \
  lidar_odom_topic:=/aft_mapped_to_init
```

如需复现原静态 PCD 实验：

```bash
roslaunch real_flight_bringup real_flight_step1.launch \
  point_cloud_use_pcd:=true
```

目录内需要存在：

```text
best_model_multimat.engine
best_model_multimat_bs1.engine
student_distilled.engine
```

启动时 `intention_get_corridor` 和 `traj_gen_in_corridor` 会分别加载并预热这三个模型。
如果放在其他目录，启动时传入 `mpd_model_dir:=/实际绝对路径/small`。
如果 engine 与 Orin NX 上的 TensorRT/CUDA 版本不兼容，需要用同目录中的 ONNX 文件在
Orin NX 上重新生成 engine，文件名保持不变即可。

## offline_collision_reference_follow.launch

用途：读取 CSV 中保存的 `/position_command` 参考，使用 NMPC 重复跟踪，适合对照
不同参数的效果。

```bash
roslaunch real_flight_bringup offline_collision_reference_follow.launch \
  reference_csv:=/绝对路径/参考轨迹.csv
```

常用参数：

- `reference_csv`：轨迹 CSV；
- `cmd_topic:=/position_command`：NMPC 轨迹指令；
- `pre_align_altitude:=1.0`：开始回放前的高度。

## collision_recovery_nmpc.launch

用途：保留原项目的外力碰撞检测、导纳恢复、速度衰减和最终定点停止状态机，最终控制器改为 NMPC。
该入口不启动在线规划器，适合单独复现旧碰撞恢复实验。

```bash
roslaunch real_flight_bringup collision_recovery_nmpc.launch
```

启动后先输入 `cmd 3` 悬停，确认高度和定位正常后输入 `cmd 5`。恢复节点收到 cmd 5 后从
飞机当前位置开始，按照 `collision_recovery_nmpc.yaml` 中的 `LinearMotionSpeed` 直线运动；
检测到外力后依次执行：

```text
直线运动 -> 外力触发 -> 导纳恢复 -> 速度衰减 -> 定点停止
```

恢复节点只发布 `/position_command`。机体系角速度和油门仍全部由 NMPC 计算。

## localization.launch

用途：单独检查定位链，或由其他三个实飞 launch 内部包含。

定位源只保留 Point-LIO：

```bash
roslaunch real_flight_bringup localization.launch \
  lidar_odom_topic:=/aft_mapped_to_init
```

运行项目实飞 launch 时不要再单独启动它。

## thrust_calibrate_nmpc.launch

用途：保留原项目的推力/电压记录功能，记录 NMPC 实际发布到
`/mavros/setpoint_raw/attitude` 的油门和 `/mavros/battery` 电压，不启动额外控制器。

```bash
roslaunch real_flight_bringup thrust_calibrate_nmpc.launch \
  mass_kg:=1.17
```

第一次向 `/traj_start_trigger` 发布消息开始记录，第二次发布后停止并写入
`~/.ros/nmpc_thrust_calibration.csv`。

## collision_planning_preview.launch

用途：使用假里程计预览静态地图、碰撞和轨迹规划，不向真机发送 NMPC 控制。

```bash
roslaunch real_flight_bringup collision_planning_preview.launch
```

它用于检查规划结果、碰撞切换和曲线，不是实飞入口。

## 航点打点程序

`record_flight_waypoints.py`参考Jiangyin2026的实机打点流程，从定位姿态中同时记录：

- 世界坐标系XYZ；
- yaw角，文件中统一使用弧度；
- 飞行模式：`avoidance`（避碰）或`collision`（计划碰撞）；
- 与`quadrotor_msgs/TrajectoryPlan`一致的数值模式：避碰为1，碰撞为2；

程序只读取定位并写文件，不会解锁、切换OFFBOARD或向飞机发布控制指令。推荐在可交互终端中运行：

打点默认写入当前机器上`real_flight_bringup`包的`config/flight_waypoints.yaml`。
启动时日志会打印解析后的完整路径；也可用`_output_file:=/绝对路径/flight_waypoints.yaml`覆盖。
实际执行始终读取`config/mission.yaml`。打点完成后，由操作者把记录文件的`waypoints`
列表手工复制到`mission.yaml`的`Mission.segments.cmd5`或`cmd6`下。

```bash
rosrun real_flight_bringup record_flight_waypoints.py \
  _pose_topic:=/mavros/local_position/pose \
  _pose_type:=pose
```

如果希望直接读取项目规划定位，也可以使用里程计：

```bash
rosrun real_flight_bringup record_flight_waypoints.py \
  _pose_topic:=/visual_slam/odom \
  _pose_type:=odometry
```

### 在mission.yaml选择单点或多点

`config/mission.yaml`是最终执行文件。原有`SetPos`、`SetAtt`仍供单点任务使用：

```yaml
Mission:
  mode: single
  frame_id: world
SetPos: [0.0, 3.5, 1.40]
SetAtt: [0.0, 0.0, 1.0, 0.0]
```

多点任务把`mode`改为`multi`，并将`flight_waypoints.yaml`中的列表项按飞行阶段复制到
`Mission.segments.cmd5`和`Mission.segments.cmd6`。每段可以包含多个航点，每段的`id`
分别从0连续编号；保留`flight_mode`、`trajectory_mode`、XYZ和yaw。
`Mission.frame_id`也要与打点坐标系一致。例如：

```yaml
Mission:
  mode: multi
  frame_id: world
  segments:
    cmd5:
      - id: 0
        flight_mode: avoidance
        trajectory_mode: 1
        x: 0.0
        y: 1.0
        z: 1.0
        yaw: 0.0
    cmd6:
      - id: 0
        flight_mode: avoidance
        trajectory_mode: 1
        x: 1.0
        y: 2.0
        z: 1.0
        yaw: 0.0
```

示例坐标仅展示格式，实飞前应替换为实际打点结果。修改`mission.yaml`后重新启动
`real_flight_step1.launch`，因为执行器在启动时读取任务。`single`使用原`SetPos`
发布目标；`multi`由执行器按`cmd 5/6`选择对应航点段，两种模式不会同时自动发布目标。

### 依次执行多点任务

航点执行器通过新的`quadrotor_msgs/FlightWaypoint`消息发布每个点，因此XYZ、yaw和
`avoidance/collision`模式会在同一次目标更新中送到前端规划器和轨迹生成器：

```text
mission.yaml / Mission.segments.cmd5 或 cmd6
  -> /flight_waypoint_goal (XYZ + mode + yaw)
  -> intention_get_corridor (按点选择避碰或计划碰撞)
  -> traj_gen_in_corridor (生成位置轨迹和终点yaw)
  -> polytraj_poscmd_bridge -> NMPC
```

完整实飞入口默认根据`Mission.mode`选择单点或多点。建议关闭RViz手动目标，避免覆盖任务：

```bash
roslaunch real_flight_bringup real_flight_step1.launch \
  enable_rviz_goal:=false
```

`Mission.mode: multi`时，进入`cmd 5`会自动执行第一段；第一段完成后保持末点，
输入`cmd 6`才会执行第二段。切换段时清除旧轨迹并等待新轨迹；退出`cmd 5/6`
会停止当前航点序列，再次进入某段则从该段第0个航点重新执行。
`Mission.mode: single`仍由原`SetPos`单点流程发布目标。
单点模式下`cmd 6`不发布第二段目标，控制器保持进入该命令时的位置。
实际飞行前应确认两段航点的坐标系和高度正确。

执行器只有在以下条件持续满足0.5秒后才发送下一点：XYZ误差不超过0.20米、yaw误差
不超过0.20弧度、`/trajectory_remaining_time`不超过0.25秒。状态可查看：

```bash
rostopic echo /execute_flight_waypoints/status
```

控制命令包括`start`、`pause`、`resume`、`retry`、`next`、`stop`和`status`；
`start/resume`仅在`cmd 5`或`cmd 6`下有效。正常进入对应命令已自动开始，无需手工发送。
`pause/stop`只停止航点序列推进，不会替代飞控的悬停/停止命令；需要停机或悬停仍应使用
现有FSM命令。也可单独启动执行器：

```bash
roslaunch real_flight_bringup execute_flight_waypoints.launch
```

单点`/goal`保持原项目“先计划碰撞、碰撞结束后再避碰到终点”的流程；多点
`/flight_waypoint_goal`按每个航点的模式选择规划流程。飞行速度由MINCO优化及
`traj_gen_real.yaml`中的`MaxVelRate`等约束决定，航点文件不设置逐点速度。

交互命令与Jiangyin2026的打点方式一致：

```text
Enter / a    记录当前位置XYZ和当前yaw
1            后续航点使用避碰模式
2            后续航点使用计划碰撞模式
u            撤销上一个点
s            保存
q            保存并退出
```

也可使用`record_flight_waypoints.launch`非交互启动，再通过话题发送同样的命令：

```bash
roslaunch real_flight_bringup record_flight_waypoints.launch
rostopic pub -1 /waypoint_recorder/command std_msgs/String "data: '2'"
rostopic pub -1 /waypoint_recorder/command std_msgs/String "data: 'a'"
```

输出采用临时文件加原子替换，断电或进程异常时不会留下只写了一半的YAML。再次启动默认读取
已有文件并继续追加。原始打点保存在`config/flight_waypoints.yaml`，最终任务保存在
`config/mission.yaml`。打点文件不直接执行。

## 控制指令与调试话题

状态机 UDP 端口为 `12001`。主流程使用：

- `cmd 1`：低油门和解锁测试；
- `cmd 2`：PX4 位置起飞；
- `cmd 3`：NMPC 悬停；
- `cmd 4`：降落和上锁；
- `cmd 5`：NMPC 跟踪第一段航点；
- `cmd 6`：NMPC 跟踪第二段航点；
- `cmd 7`：NMPC 八字轨迹。

调试时可同时查看：

- `/debugPx4ctrl`：兼容原 PlotJuggler 配置的 NMPC 调试消息；
- `/nmpc_state`：NMPC 预测时域、参考状态和反馈；
  其中`disturbance_accel`为NDOB估计的世界系等效加速度扰动；
- `/position_command_horizon`：轨迹桥接器按0.05秒间隔发布的9个真实未来参考点；
- `/visual_slam/odom`：实际定位轨迹；
- `/position_command`：规划器当前期望状态；
- `/single_offboard_fsm/trajectory_reference`：状态机发布的参考轨迹。
- `/fsm_ctrl/command`：当前状态机指令，碰撞恢复节点用它检测 cmd 5；
- `/collision_recovery/collision`、`fix_pose`、`stop`：旧恢复状态机的三个阶段；
- `/collision_recovery/filtered_force`：恢复节点使用的滤波外力。

底层轨迹跟踪控制器只有 `single_offboard_fsm` 中的 NMPC，输出到
`/mavros/setpoint_raw/attitude`。
