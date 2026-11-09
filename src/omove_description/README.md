# OMOVE 仿真

ROS 2 Humble / Gazebo Classic 下的四轮麦克纳姆底盘实验包。

## 构建和启动

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select omove_description
source install/setup.bash
ros2 launch omove_description gazebo.launch.py gui:=false
```

在另一个已加载工作空间环境的终端检查：

```bash
ros2 control list_controllers
ros2 topic echo /odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

应有四个轮速控制器和一个 joint_state_broadcaster 处于 active 状态。
两个仿真入口 gazebo.launch.py、slam_sim.launch.py 都会启动控制器、
运动学和里程计节点。不要同时运行两个入口。

## 里程计

数据链路：`/cmd_vel -> 四轮速度控制器 -> /joint_states -> /odom + TF`。
轮速顺序按关节名称匹配，为前左、前右、后左、后右，采用 X 型麦轮公式。
里程计以关节反馈时间戳积分，发布 `odom -> base_footprint`。
重复时间戳不积分；仿真时间倒退时重置累计位姿。
默认轮半径 0.076 m、半轴距 0.18 m、半轮距 0.20 m。
修改底盘尺寸时，应同时保持 URDF、运动学和里程计参数一致。
协方差为初始估计，尚未通过滑移数据标定。

轮速里程计不是仿真真实位姿。当前使用每轮 12 个被动滚子的简化麦轮接触模型，已完成前进、横移和转向位姿对比。
轮速里程计仍有接触误差，需要按用途进一步标定。

## 测试

在工作空间根目录执行：

```bash
python3 -m unittest discover -s src/omove_description/test -v
```

覆盖直线、横移、旋转、曲线积分、关节顺序、重复时间戳、时间重置和无效反馈。
2026-10-29 无界面仿真验证通过：五个控制器激活，关节反馈、里程计、TF
均有消息；前进指令 0.1 m/s 对应轮速里程计约 0.1 m/s。
尚未系统比较 Gazebo 真实位移。

## 激光建图

先结束其他仿真实例，再启动带房间场景的建图入口：

```bash
ros2 launch omove_description slam_sim.launch.py gui:=false
```

雷达插件直接发布 `/scan`，无需 relay。项目自带 `config/slam_params.yaml`，
明确使用 `map`、`odom`、`base_footprint` 和 `/scan`。
激光有效处理范围为 0.16–8 m（传感器硬件下限 0.15 m，处理留少量余量），
地图分辨率 0.05 m，移动/转动采样阈值为 0.1 m / 0.1 rad。

在另一个已加载工作空间环境的终端检查和保存地图：

```bash
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo map laser_link
ros2 run nav2_map_server map_saver_cli -f /tmp/omove_room
```

保存命令生成 YAML 和 PGM，不会覆盖工作空间中现有 room 地图。
无界面短程验证已收到 360 线激光、包含空闲和占据栅格的地图，
并验证完整 `map -> odom -> base_footprint -> base_link -> laser_link` TF。
这只验证短程数据链路与保存功能，不代表已完成整个房间探索或回环精度评估。

轮关节的局部 Z 轴通过 -pi/2 绕 X 旋转映射为底盘 +Y 轴，
使正轮速对应前进，与运动学和轮速里程计约定一致。

## 地图定位和导航

停止建图入口后，构建并重新加载工作空间，再启动：

```bash
ros2 launch omove_description nav_sim.launch.py gui:=false
```

该入口启动房间仿真、底盘、AMCL 和 Nav2；机器人出生位置为世界坐标
(1.5, 0)，朝向为 0。不要同时启动 slam_sim.launch.py，避免两个节点发布 map TF。
在 RViz 的 2D Pose Estimate 中设定初始位姿后，再用 Nav2 Goal 发送目标。
使用随包 room 地图时，初始位姿约为 (1.5, 0, 0)。测试时需保持地图与场景一致。

无 RViz 时，可以在另一个已加载工作空间环境的终端初始化：

```bash
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  '{header: {frame_id: map}, pose: {pose: {position: {x: 1.5, y: 0.0}, orientation: {w: 1.0}}, covariance: [0.02, 0, 0, 0, 0, 0, 0, 0.02, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.02]}}'
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}}' --feedback
```

默认地图从包安装目录 maps/room.yaml 加载，可覆盖：

```bash
ros2 launch omove_description nav_sim.launch.py map:=/absolute/path/to/map.yaml
```

已有底盘和传感器运行时可单独启动 nav_bringup.launch.py。
两个导航入口都支持 map、params_file 参数；单独导航入口还支持 use_sim_time。
自定义地图的初始坐标不一定与随包地图相同，必须按新地图设定初始位姿。

Nav2 使用 OmniMotionModel，DWB 与速度平滑器启用横向速度，最大横移速度 0.15 m/s。
代价地图采用包含车轮外沿的 0.54 × 0.46 m 矩形轮廓，再加 0.01 m padding。
局部地图为滚动窗口，使用激光障碍层和膨胀层；全局地图另含静态地图层。

2026-10-29 无界面验证：九个定位/导航生命周期节点均为 active，
收到地图、全局/局部代价地图和 AMCL 位姿，路径规划返回 19 个路径点，
从 (1.5, 0) 附近向 (2.0, 0) 的 NavigateToPose 动作返回 SUCCEEDED。
目标容差沿用 Nav2 默认 0.25 m / 0.25 rad；此结果不代表精确到点。
复杂绕障、长距离和恢复行为仍需验证。

## 指令保护和运动模型限制

运动学节点使用独立单调时钟检查指令超时；默认连续 0.5 秒未收到
`/cmd_vel`，以 0.05 秒检查周期向四轮发布零速。暂停仿真时钟不会暂停此检查。
恢复收到有效指令后可继续运动。NaN、无穷和计算溢出会立即输出零速。
四轮最大转速默认 12 rad/s，超限时等比例缩放，保留运动方向。
可用 ROS 参数 `cmd_timeout`、`max_wheel_speed` 调整。
该保护依赖运动学进程仍在运行，不覆盖进程崩溃、控制器故障或通信中断；
实体底盘还需要控制器端/硬件端的超时保护。

第五轮修改时的旧圆柱轮模型真实位姿对比（历史基线，每方向约 4 秒）：

| 指令 | 实际运动 | 轮速里程计 |
| --- | --- | --- |
| 前进 0.1 m/s | X 约 +0.397 m | X 约 +0.398 m |
| 横移 0.1 m/s | Y 约 -0.0003 m，X 漂移约 +0.061 m | Y 约 +0.302 m |
| 旋转 0.3 rad/s | yaw 约 +0.729 rad | yaw 约 +0.908 rad |

旧圆柱轮模型不能用于全向导航。第五轮曾临时关闭横移；第六轮已替换为被动滚子模型，
并在真实位姿测试通过后恢复横移参数，见下节。
三组运动测试均验证了停止发布指令后四轮命令归零，轮速反馈降至接近零。

测试已接入 CTest；构建后可执行 `colcon test --packages-select omove_description`，
或继续使用前述 unittest 命令运行全部 15 项测试。

## 被动滚子模型（第六轮修改）

每个轮子包含 12 个自由旋转的 45° 圆柱滚子，四轮按 X 型布局设置手性。
轮毂碰撞半径缩小至 0.045 m，不再阻挡滚子接触地面。
滚子中心半径 0.061 m、自身半径 0.015 m、长度 0.04 m；轮组总质量保持 0.5 kg。
48 个滚子关节只提供位置/速度反馈，没有电机命令接口。
GazeboSystem 和 joint_state_broadcaster 提供被动关节状态，供模型 TF 使用。

本轮真实位姿对比（相同方向各约 4 秒墙钟时间）：

| 指令 | Gazebo 真实位姿增量 | 轮速里程计增量 |
| --- | --- | --- |
| 前进 0.1 m/s | X +0.2185 m | X +0.2217 m |
| 横移 0.1 m/s | Y +0.2235 m，X +0.0036 m，yaw +0.0275 rad | Y +0.2346 m |
| 转向 0.3 rad/s | yaw +0.7994 rad | yaw +0.8086 rad |

增加滚子后物理求解更慢，4 秒墙钟不再对应 4 秒仿真时间，不能直接按
指令速度乘墙钟时间评价误差。应比较同一采样窗口的真实位姿与里程计。
本次横移仍有约 5% 的主方向位移差和少量偏航；这是初步验证，不是精度保证。
停车测试预留 3 秒墙钟观察，确认零轮速命令及接近零的反馈。
默认全向参数已恢复。导航回归验证中，九个生命周期节点全部 active，
路径规划返回 19 个路径点，前进目标 (2.0, 0) 和随后侧向目标 (2.0, 0.5)
均返回 SUCCEEDED。导航容差仍为 0.25 m / 0.25 rad，不代表精确到点或全程保持朝向。

圆柱滚子是简化近似，尚未模拟真实鼓形滚子、材料和装配误差。
测试覆盖滚子接触轴方向、惯量合法性、轮毂避让及被动关节无电机命令接口。

## 绕障与连续目标回归（第七轮修改）

新增可重复运行的集成验证脚本（需要本机 Gazebo/ROS 网络权限）：

```bash
ros2 run omove_description validate_navigation.py \
  --output /tmp/omove_navigation_validation.json --domain 97
```

脚本自行启动无界面仿真，使用指定的独立 ROS domain，结束时关闭自己启动的进程。
运行前应退出其他 Gazebo 实例，避免 Gazebo master 端口冲突。
它在临时世界中加入真实位姿观测插件，不修改正常仿真世界。
一次运行通常需要数分钟；每个导航目标默认最多等待 150 秒墙钟时间，
可用 `--goal-timeout` 调整。

流程为：初始化定位；规划绕过中央箱体；连续到达 (-1.2, 0)、(-1.2, 1)、
(1.5, 0)；拒绝箱体中心的占据目标；单独调用 Spin 恢复行为；
再次导航至 (1.5, 0.6)。JSON 保存动作状态、真实/定位轨迹、定位误差、
采样轮廓的几何间距。旁边的同名 .log 保存 ROS 进程日志。
该脚本不属于默认单元测试，不会随 `colcon test` 自动启动 Gazebo。

GridBased 的规划容差由默认 0.5 m 收紧为 0.1 m。
旧容差允许占据目标附近的替代终点，不能把“成功返回路径”理解为目标本身可达。
导航到点容差仍为 0.25 m / 0.25 rad；两者用途不同。

定位误差是本房间地图坐标与 Gazebo 世界坐标对齐后的比较，
不可直接用于坐标原点不同的自定义地图。
几何检查采用采样的保守矩形轮廓，并非 Gazebo 接触力测量，不能证明采样间无碰撞。
本测试单独调用恢复动作，不等同于验证行为树所有自动恢复分支。

本轮完整回归结果：全部检查通过，墙钟耗时约 95.9 秒，122 个定位采样点。
平均位置误差约 0.0417 m，最大约 0.0768 m；采样保守轮廓与障碍物/墙面
最小几何间距约 0.1784 m。三个连续目标、占据点拒绝、Spin 行为和恢复后导航
均通过。该时长仅属短程连续回归，尚未完成小时级耐久测试。

## 避免重复路段：近期轨迹代价层

`global_costmap.visited_trail_layer` 记录地图坐标系下实际经过的位置，
以非致命代价影响 NavFn 全局规划。它只表达优先走新路线，不保证完全不折返。
局部代价地图不加载此层，避障仍由真实障碍数据决定。

默认每移动 0.10 m 记录一个点；半径 0.35 m 内的代价由中心 45 向边缘递减，
并在 180 秒仿真时间内衰减至零。最初 5 秒及车身周围 0.5 m 不增加代价，
重叠轨迹取最大值，不按点数叠加。最多保存 2000 点，停车不会不断生成轨迹。
原有未知、内切障碍和致命障碍代价保持原值，其他格子加代价后最高为 252，仍可通行。
这些参数在初始化时读取，调整 YAML 后需要重启导航。

RViz 添加 Path，Topic 选择 `/global_costmap/global_costmap/visited_trail`，
即可查看当前保留的轨迹；订阅持久性选择 Transient Local 可获取最近一次发布。
轨迹只在内存中保留，跨目标保留，导航重启、仿真时间回退或相邻更新位移大于 1 m
时清空。一般的 Nav2 全局代价地图清理也会清空本层，使恢复动作能够解除偏好限制。
手动清空：

```bash
ros2 service call /global_costmap/global_costmap/clear_visited_trail std_srvs/srv/Empty '{}'
```

全局地图每次完整重算以移除过期代价，适用于目前的小型场地；大地图需要改为增量边界更新。
轨迹只是已访问区域，不能作为静态障碍保存到地图文件。定位的小幅修正会带来一定轨迹误差。
