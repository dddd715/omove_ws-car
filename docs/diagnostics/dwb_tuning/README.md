# world01 全向底盘 DWB 稳定性调参记录

日期：2026-11-05（Asia/Shanghai）。目标：减少直线/横移过程中的多余转向，使用适中速度，并保留倒车、横移和终点转向能力。

## 结果与适用范围

主配置已更新：`/home/deng/omove_ws/src/omove_description/config/nav2_params.yaml`。
最终参数来自 `candidate_02.yaml`，两者有效运行参数一致，主配置额外增加了说明注释，并移除了本机 DWB 未声明的 FollowPath.stateful。

本次使用现有 Gazebo world01、已有地图和 slam_toolbox localization。不是 AMCL，不重新建图。保持仿真和定位运行，只重启 Nav2 比较参数。

最终两轮共 14 个导航任务成功（含两次起点定位任务，12 个运动任务）。覆盖沿车体纵向往返、横移往返、斜向往返及原地转向/恢复朝向。每轮目标位姿相同，实际起点允许有导航终点容差，因此不是严格相同起点的实验。

| 同一目标/路线 | 基线运动中朝向跨度 | 最终参数第一轮 | 最终参数扩展轮 |
|---|---:|---:|---:|
| 纵向去程 | 3.84° | 0.93° | 未重复 |
| 纵向回程 | 12.98° | 0.76° | 未重复 |
| 横移去程 | 84.54° | 1.81° | 1.56° |
| 横移回程 | 88.19° | 0.87° | 0.62° |
| 斜向去程 | 未测 | 未测 | 1.57° |
| 斜向回程 | 未测 | 未测 | 3.02° |

朝向跨度统计平移指令幅值大于 0.025 m/s 时 map→base_footprint 的朝向变化范围。它不是相对目标的绝对朝向误差，也不包括停下后主动调整终点朝向的阶段。角度以目标朝向为中心归一化，测试未跨越 ±π。

最终两轮最大绝对横向路径偏差分别约 0.063 m、0.081 m；最大终点位置误差约 0.130 m、0.134 m；最大终点朝向误差约 5.92°、4.69°。这些为 TF 定位结果，不是独立物理真值。

基线使用开始本轮调参时的参数，已包含此前手动修改的前视距离 0.3 m、横移限速 ±0.05 m/s、前进 0.1 m/s 和转向限速 0.3 rad/s，不是仓库最初配置。基线最后一次恢复朝向任务期间导航进程收到 SIGINT，记录为中断，不将其解释为参数导致导航失败。

## 主要调整原因

1. 原 PathAlign/GoalAlign 对齐评分会鼓励车头与路径方向对齐，横移时可能先转向再移动。全向底盘既能横移又能转向，“能到达”不代表运动姿态符合预期。
2. PathAlign.scale、GoalAlign.scale 设为 0，使用 PathDist/GoalDist 引导机器人中心跟踪路径并接近目标；前视距离保留 0.3，但对应 scale 为 0 时不参与评分。
3. 添加本机 DWB 已提供的 Twirling 评分器，权重 10，惩罚多余旋转。它不硬性锁定车头；RotateToGoal 和目标检查器仍负责终点朝向。
4. 使用前进 0.15 m/s、后退 0.10 m/s、横移 0.10 m/s、旋转 0.35 rad/s 的适中上限。
5. DWB 与 velocity_smoother 使用相同速度/加减速限制，减轻加速阶段模型不一致。没有修改轮胎摩擦、运动学和定位参数。
6. 目标位置容差改为 0.15 m，朝向容差改为 0.10 rad；避免到达时允许约 14° 的朝向误差。

## FollowPath 参数取值

参数不是越小或越大越好。下表包括已调整和经过本轮验证后保留的项；未对所有参数逐一做穷举最优化。

| 参数 | 最终值 | 选择理由 |
|---|---|---|
| plugin | dwb_core::DWBLocalPlanner | 保留当前控制器 |
| debug_trajectory_details | false | 常规运行关闭详细评分调试 |
| min_vel_x / max_vel_x | -0.1 / 0.15 | 倒车限速略低于前进 |
| min_vel_y / max_vel_y | -0.1 / 0.1 | 保留双向横移能力 |
| max_vel_theta | 0.35 | 限制转向速度，约 20°/s |
| min_speed_xy / max_speed_xy | 0.0 / 0.2 | 保留低速及合速度边界；各轴上限仍同时约束 |
| min_speed_theta | 0.0 | 不强制最低旋转速度 |
| acc_lim_x / acc_lim_y | 0.25 / 0.25 | 平缓平移加速 |
| acc_lim_theta | 0.5 | 限制角加速度 |
| decel_lim_x / decel_lim_y | -0.25 / -0.25 | 与平滑器一致 |
| decel_lim_theta | -0.5 | 与平滑器一致 |
| vx_samples / vy_samples / vtheta_samples | 15 / 15 / 21 | 增加候选速度覆盖；对称窗口的奇数采样可覆盖零速度 |
| sim_time | 1.7 | 保留预测时长，未单独优化 |
| linear_granularity | 0.025 | 增加轨迹离散密度 |
| angular_granularity | 0.025 | 保留角度离散步长 |
| transform_tolerance | 0.2 | 保留 TF 容差，不用它掩盖数据延迟 |
| prune_distance | 1.0 | 保留工程已有路径裁剪配置 |
| forward_prune_distance | 0.8 | 保留凹角绕障相关设置 |
| shorten_transformed_plan | true | 保留局部路径截短行为 |
| xy_goal_tolerance | 0.15 | 与 general_goal_checker 位置容差一致 |
| trans_stopped_velocity | 0.02 | 保留停止判定阈值 |
| short_circuit_trajectory_evaluation | true | 保留评分提前终止，减少计算 |
| stateful | 移除该项 | 本机 DWB 未声明此参数；general_goal_checker.stateful=true 保留 |
| critics | 原列表加 Twirling | 保留障碍、路径、目标、振荡和终点旋转评分 |
| Twirling.scale | 10.0 | 减少不必要的旋转 |
| BaseObstacle.scale | 0.02 | 保留原值；本轮没有对避障权重做独立优化 |
| PathAlign.scale / GoalAlign.scale | 0.0 / 0.0 | 不强制车头沿路径方向 |
| PathAlign.forward_point_distance / GoalAlign.forward_point_distance | 0.3 / 0.3 | 保留此前实验值，scale=0 时不产生评分影响 |
| PathDist.scale / GoalDist.scale | 32.0 / 24.0 | 保留机器人中心路径/目标距离引导 |
| RotateToGoal.scale | 32.0 | 保留终点朝向评分 |
| RotateToGoal.slowing_factor | 5.0 | 保留接近目标时减速行为 |
| RotateToGoal.lookahead_time | -1.0 | 保留完整轨迹终点朝向评分 |

速度采样组合上限量级为 15×15×21=4725，实际还受动态窗口和合法性约束。当前测试日志未发现控制循环超时，但不能保证其他 CPU 负载和复杂场景下仍满足周期。

关联修改：general_goal_checker 的 xy_goal_tolerance=0.15、yaw_goal_tolerance=0.10；velocity_smoother 的速度和加减速上下限与上述 DWB 限制匹配。其余导航与代价地图配置保留。

## 启动最终配置

先启动现有仿真（slam:=false）、slam_toolbox localization，并确认定位正确。再启动仅导航入口：

```bash
source /opt/ros/humble/setup.bash
source /home/deng/omove_ws/install/setup.bash
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true autostart:=true use_composition:=False \
  params_file:=/home/deng/omove_ws/src/omove_description/config/nav2_params.yaml
```

不要同时启动另一套 Nav2 或 AMCL。RViz 中使用 Navigation 2 面板和 Nav2 Goal 工具。目标朝向仍需要明确设置；到达终点时必要的转向是预期行为。

## 重复测量

`measure_navigation.py` 会发送真实的仿真导航目标，仅在无人同时遥控/发送目标时运行。它读取 /map，使用包围底盘的 0.395 m 半径圆检查短直线路径净空；未知和占用区域不通过。该检查只用于本次静态短路线筛选，不能替代实时避障。

```bash
source /opt/ros/humble/setup.bash
source /home/deng/omove_ws/install/setup.bash
python3 /home/deng/omove_ws/docs/diagnostics/dwb_tuning/measure_navigation.py \
  --output /home/deng/omove_ws/docs/diagnostics/dwb_tuning/retest_01.json \
  --routes /home/deng/omove_ws/docs/diagnostics/dwb_tuning/routes.json
```

输出文件不可重复覆盖。既有 routes.json 固定了本次地图坐标目标；地图或初始区域改变时需使用新的 routes 文件重新选路。路线名 forward 可能因净空检查退选反方向，不能仅凭标签判断前进/倒车，需看实际速度符号。

每个目标有 80 秒墙钟超时，超时或异常时尝试取消导航。测试中不要主动关闭 Nav2。自动测试不能替代人工急停；取消或进程中断后应确认机器人停止。

## 文件与局限

- baseline.yaml / baseline.json：调参起点配置和基线结果。
- candidate_01.yaml / candidate_01.json：增加 Twirling、修改速度链后的第一组。
- candidate_02.yaml / candidate_02.json：收紧终点容差后的最终组。
- candidate_02_extended.json：最终组斜向和横移重复测试。
- routes.json / extended_routes.json：固定目标位姿。
- summary.json：各次任务的指标汇总。
- active_controller.yaml / active_velocity_smoother.yaml：结束时运行参数快照。

当前 Gazebo 没有发布 /gazebo/model_states，也没有可用的实体状态查询服务，所以没有采集独立物理真值；JSON 中 truth 为 null。摆头改善指标来自定位 TF，不能单凭本报告宣称轮地打滑已消除。

已验证当前开放区域短距离往返、横移、斜向和原地转向。尚未验证完整凹角绕障、动态障碍、长距离和长时间运行、负载变化、真实硬件，也没有做全参数全局最优化。用于本场景的已验证稳定基线，不能视为所有场景都最优的最终参数。

## 后续异常：重复 Nav2 导致恢复打转

用户后续测试再次打转时，发现候选配置与主配置的 navigation_launch.py 同时运行，/follow_path 存在两个 controller_server。日志出现 unknown goal/result response、行为树 invalid status，以及 Spin 恢复行为转动 1.57 rad。此次异常不能作为单套配置摆头退化的证据。

已停止两套导航，确认导航节点退出，再仅使用主配置启动一套。测试脚本或助手留下导航进程后，不应在另一终端重复启动。再次启动前可检查：

```bash
ros2 action info /follow_path
```

正常只应有一个 action server。若需要重启，应先结束原 Nav2 启动进程并等节点退出，再启动新进程。不要通过禁用恢复旋转来掩盖重复服务器问题。

## 当前主配置更新：正向曲线跟踪

用户明确希望沿曲线正向行驶，不使用倒车/横移完成路径跟踪。此前“稳定全向”与此行为目标不同，已备份为 holonomic_stable_before_forward.yaml。

当前主配置使用 forward_candidate.yaml 对应参数：

- FollowPath.min_vel_x=0.0：禁止 DWB 跟踪时倒车。
- FollowPath.min_vel_y=max_vel_y=0.0，vy_samples=1：禁止 DWB 跟踪时横移。
- max_speed_xy=0.15，前进上限仍为 0.15 m/s。
- PathAlign.scale=16.0、GoalAlign.scale=8.0，前视距离保留 0.3 m：引导车头沿路径对齐。
- Twirling.scale=1.0：降低旋转惩罚，允许前进中转弯。
- 保留速度平滑器的底盘速度边界，DWB 自身不再生成负 x 和非零 y 目标；底盘键盘横移能力保留。

边界：这只约束 FollowPath 跟踪指令，BackUp 恢复行为仍可倒车，Spin 恢复及终点旋转仍可发生。DWB 不是车辆曲率约束规划器，可能在急弯停下调整车头，不能保证全程连续前进的光滑曲线。若需要严格不原地转向和指定最小转弯半径，应另行选择匹配的规划器/控制器并验证。

验证：主配置 YAML 解析成功，导航组件激活；自动短路线筛选因当前位姿到旧目标的底盘净空检查未通过而停止，没有发送导航目标（forward_curve_01.json）。因此此前 14 个成功任务只支持旧的全向配置，不能作为当前正向配置的运动验证。当前正向配置还需在空旷处重新做直行、转弯和绕障测试。

## 正向导航转角卡住：完整车身碰撞评分

现场日志在内凹转角出现 Failed to make progress。用户已手动退回起点。主配置的 Obstacle critic 从 BaseObstacle 改为 ObstacleFootprint，scale 保持 0.02，其余正向参数不变。它按每个离散轨迹位姿的车身轮廓边界检查障碍，需配合膨胀层；不是对连续运动扫掠体的精确碰撞证明。禁止倒车/横移的 FollowPath 约束保留。

备份：forward_before_corner.yaml；候选：corner_candidate_01.yaml；结果：corner_candidate_01_result.json。

使用已有的已知凹角回归场景，在独立 ROS domain 94、Gazebo 端口 11361 运行，未移动用户当前仿真小车。起点 Gazebo (-1.16,-3.43)，AMCL 地图起点 (-1.16,-2.43)，目标地图 (0.21,-0.11,0)。结果 NavigateToPose status=4，22 个约 2 秒间隔的运动样本，采样指令无负 x、无横移，未见 Failed to make progress 或 Spin 恢复日志。独立仿真已退出。

可重复脚本：reproduce_forward_corner.py。该脚本复用了既有 AMCL 回归入口，不是当前 slam_toolbox localization，也不是用户刚才精确起点/目标的重放。单次通过不能证明问题已普遍修复，仍需在当前定位流程复测同一转角，并检查 Gazebo 实际车体与墙体接触情况。
