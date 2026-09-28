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

必须关闭`enable_goal_publish`，否则`mission.yaml`里的`SetPos`仍会自动发布。RViz应同时显示：

- `/path`：实际定位轨迹；
- `/visualizer/route`：规划路线；
- `/visualizer/trajectory`：最终可执行轨迹；
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

常用参数：

- `enable_goal_publish:=true`：自动发送 `mission.yaml` 中的目标点；
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

## 控制指令与调试话题

状态机 UDP 端口为 `12001`。主流程使用：

- `cmd 1`：低油门和解锁测试；
- `cmd 2`：PX4 位置起飞；
- `cmd 3`：NMPC 悬停；
- `cmd 4`：降落和上锁；
- `cmd 5`：NMPC 跟踪 `/position_command`；
- `cmd 7`：NMPC 八字轨迹。

调试时可同时查看：

- `/debugPx4ctrl`：兼容原 PlotJuggler 配置的 NMPC 调试消息；
- `/nmpc_state`：NMPC 预测时域、参考状态和反馈；
- `/position_command_horizon`：轨迹桥接器按0.05秒间隔发布的9个真实未来参考点；
- `/visual_slam/odom`：实际定位轨迹；
- `/position_command`：规划器当前期望状态；
- `/single_offboard_fsm/trajectory_reference`：状态机发布的参考轨迹。
- `/fsm_ctrl/command`：当前状态机指令，碰撞恢复节点用它检测 cmd 5；
- `/collision_recovery/collision`、`fix_pose`、`stop`：旧恢复状态机的三个阶段；
- `/collision_recovery/filtered_force`：恢复节点使用的滤波外力。

底层轨迹跟踪控制器只有 `single_offboard_fsm` 中的 NMPC，输出到
`/mavros/setpoint_raw/attitude`。
