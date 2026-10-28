# 3Droll 分支：可逆推力与翻滚

本分支默认使用 MAVROS 可逆姿态插件发送机体系角速度和带符号推力。启动时读取新鲜的姿态反馈，自动识别飞机是正放还是倒放；姿态接近侧立时不接受飞行指令。倒放地面起飞可直接输入 `cmd3`，它会按倒飞姿态和负推力飞向同一个世界坐标悬停点。`cmd8` 只负责在正飞和倒飞之间切换 180°，每次完成后自动回到 `cmd3`。其余指令保持原有目标：倒飞状态下的 `cmd5/6` 仍按原世界坐标轨迹飞行，`cmd4` 倒飞下降并等待飞控确认着陆。

未解锁时会持续按实测姿态更新正飞／倒飞判断；解锁后只允许通过 `cmd8` 切换。`cmd7` 从输入该指令时的实测高度开始画八字，不再跳到固定的 0.5 m。默认 `cmd3` 悬停高度与完整流程的预对齐高度均为 1.4 m。

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
roslaunch fsm_ctrl hover_3d_roll.launch hover_z:=1.4 hover_thrust:=0.48
```

另开终端启动交互界面：

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch fsm_ctrl swarm.launch
```

正放或倒放地面起飞时，确认终端打印的启动姿态与实际一致，再依次输入 `1`、`3`。确认稳定悬停后输入 `8`，它会切换到另一种姿态；再次稳定悬停后还可输入 `8` 切回。此专用启动文件没有启动规划器，因此 `cmd5/6` 不会生成新轨迹。

## 翻滚后继续运行原规划链

LIO 启动后，单独启动配置可逆推力的 MAVROS：

```bash
cd ~/catkin_ws
source devel/setup.bash
roslaunch fsm_ctrl mavros_3d.launch
```

随后启动原完整流程与交互界面：

```bash
roslaunch real_flight_bringup real_flight_step1.launch pre_align_altitude:=1.4
roslaunch fsm_ctrl swarm.launch
```

上述两条命令在不同终端运行，每个终端都先执行 `source ~/catkin_ws/devel/setup.bash`。完整流程中的 `cmd8` 会切换当前姿态并返回 `cmd3`；随后输入 `5`，仍跟踪规划器原来的世界坐标轨迹。

不要在同一系统内再次启动普通的 `mavros px4.launch` 或另一份 `px4_estimator.launch`。MAVROS 姿态插件只在启动时读取 `reverse_thrust` 配置；如果此前已经运行普通 MAVROS，需要停止后按上述方式重启。

## 参数与验证

- `hover_3d_roll.launch`：`hover_z`、`hover_thrust`、`roll_duration_sec`、`roll_rate_limit`、`roll_min_height`、`inverted_land_speed`。翻转时间默认 1.2 s，NMPC 翻滚角速度上限默认 6.0 rad/s，最低入场高度默认 0.8 m，倒飞下降速度默认 0.20 m/s。
- `roll_min_height=0.8` 只是允许执行 `cmd8` 的最低实测入场高度；默认 `cmd3` 悬停目标仍是 1.4 m。理想轨迹的最低高度不低于入场高度，实机反向切换延迟和控制误差可能造成下降。
- 完整流程的 NMPC 参数在 `real_flight_bringup/config/nmpc.yaml`，高度还会由 `real_flight_step1.launch` 的 `pre_align_altitude` 覆盖。
- `cmd8` 只允许从 `cmd3` 进入，要求飞控处于 OFFBOARD 且已解锁、定位新鲜、姿态接近当前正飞或倒飞状态、速度较低。翻转未完成时不会切换到其他指令。
- `/debugPx4ctrl`、`/nmpc_state` 可查看参考姿态、速度和带符号推力。

参考提供的 360° 翻转项目后，`cmd8` 同时给 NMPC 位置、速度、姿态和推力参考，并按 3D 推力上限预先积分出侧移与高度变化。它并非原地定高动作：默认参数下，正飞到倒飞的理想模型预测向机体右侧约 1 m，反向切换约向左侧 1 m，均先上升约 0.2 m；启动日志会打印正飞到倒飞的预测值。翻转完成进入 `cmd3` 后，控制器会逐渐返回原悬停点。

参考项目的 360° 动作使用正推力，NMPC 的推力约束为 0～20 m/s²，结束后回到正飞。本分支要求在正飞与倒飞之间切换，因此采用带符号的 ±15 m/s² 推力约束；参考项目的 360° 轨迹不能直接作为 `cmd8` 轨迹使用。

当前 Mac 环境没有 ROS/Catkin，修改尚未编译或实飞验证。先在拆桨状态确认 PX4、电调、电机均支持可逆推力，负推力能到达电机，翻滚角速度方向正确。飞控实际允许的角速度也要覆盖 `roll_rate_limit`；3D 桨叶对称不保证正反电调输出完全一致，尤其需要检查反向切换延迟。倒飞降落还依赖飞控的 `/mavros/extended_state` 着陆判断以及飞机倒置时的实际离地间隙。

PX4 的可逆电机配置必须与电调 3D 模式一致；若使用 DShot，核对 PX4 的 `DSHOT_3D_ENABLE` 及电机 Reversible 配置。倒放地面起飞还可能被 PX4 解锁前检查拒绝；此处不会自动关闭飞控检查，应通过 QGroundControl 查看具体拒绝原因。
