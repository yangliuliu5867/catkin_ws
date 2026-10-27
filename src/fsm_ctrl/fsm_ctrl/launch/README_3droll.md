# 3Droll 分支：可逆推力与翻滚

本分支的机体系角速度／推力指令统一通过 MAVROS 的可逆姿态插件发送。`cmd8` 只负责从正飞悬停翻转 180°；实测姿态稳定倒置后自动切回 `cmd3` 倒飞悬停。`cmd3`、`cmd5/6` 和 `cmd7` 保留悬停、轨迹跟踪、八字轨迹的原有目标；倒飞状态下自动转换姿态参考和推力符号。`cmd5` 的世界坐标位置、速度和目标点不反号，规划器要求向前飞时仍向前飞。`cmd4` 在倒飞状态下直接倒飞下降，并在飞控确认着陆后请求上锁。

## 单独测试翻滚与悬停

启动 LIO：

```bash
cd ~/lio
bash start_lio.sh
```

另开终端启动 MAVROS、定位转发和控制器：

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch fsm_ctrl hover_3d_roll.launch hover_z:=1.8 hover_thrust:=0.48
```

另开终端启动交互界面：

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch fsm_ctrl swarm.launch
```

依次输入 `1`、`3`，确认稳定悬停后输入 `8`。此专用启动文件没有启动规划器，因此 `cmd5/6` 不会生成新轨迹。

## 翻滚后继续运行原规划链

LIO 启动后，单独启动配置可逆推力的 MAVROS：

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch fsm_ctrl mavros_3d.launch
```

随后启动原完整流程与交互界面：

```bash
roslaunch real_flight_bringup real_flight_step1.launch pre_align_altitude:=1.8
roslaunch fsm_ctrl swarm.launch
```

上述两条命令在不同终端运行，每个终端都先执行 `source ~/catkin_ws/devel/setup.bash`。完整流程中输入 `8` 完成翻滚后，FSM 会自动回到倒飞 `cmd3`；此时输入 `5`，仍跟踪规划器原来的世界坐标轨迹。

不要在同一系统内再次启动普通的 `mavros px4.launch` 或另一份 `px4_estimator.launch`。MAVROS 姿态插件只在启动时读取 `reverse_thrust` 配置；如果此前已经运行普通 MAVROS，需要停止后按上述方式重启。

## 参数与验证

- `hover_3d_roll.launch`：`hover_z`、`hover_thrust`、`roll_duration_sec`、`roll_rate_limit`、`roll_min_height`、`inverted_land_speed`。翻转时间默认 1.2 s，NMPC 翻滚角速度上限默认 6.0 rad/s，最低入场高度默认 1.5 m，倒飞下降速度默认 0.20 m/s。
- 完整流程的 NMPC 参数在 `real_flight_bringup/config/nmpc.yaml`，高度还会由 `real_flight_step1.launch` 的 `pre_align_altitude` 覆盖。
- `cmd8` 只允许从 `cmd3` 进入，要求飞控处于 OFFBOARD 且已解锁、定位新鲜、姿态接近正飞、速度较低。翻转未完成时不会切换到其他指令。
- `/debugPx4ctrl`、`/nmpc_state` 可查看参考姿态、速度和带符号推力。

参考提供的 360° 翻转项目后，`cmd8` 现在同时给 NMPC 位置、速度、姿态和推力参考，并按 3D 推力上限预先积分出侧移与高度变化。它并非原地定高动作：默认参数下，理想模型预测沿机体右侧约 1 m 位移并先上升约 0.2 m；启动日志会打印实际参数对应的预测值。翻转完成进入 `cmd3` 后，控制器会逐渐返回原悬停点。应给这段侧移和高度变化留出空间。

参考项目的 360° 动作使用正推力，NMPC 的推力约束为 0～20 m/s²，结束后回到正飞。本分支要求 180° 后倒飞悬停，因此采用带符号的 ±15 m/s² 推力约束；参考项目的 360° 轨迹不能直接作为 `cmd8` 轨迹使用。

当前 Mac 环境没有 ROS/Catkin，修改尚未编译或实飞验证。先在拆桨状态确认 PX4、电调、电机均支持可逆推力，负推力能到达电机，翻滚角速度方向正确。飞控实际允许的角速度也要覆盖 `roll_rate_limit`；3D 桨叶对称不保证正反电调输出完全一致，尤其需要检查反向切换延迟。倒飞降落还依赖飞控的 `/mavros/extended_state` 着陆判断以及飞机倒置时的实际离地间隙。
