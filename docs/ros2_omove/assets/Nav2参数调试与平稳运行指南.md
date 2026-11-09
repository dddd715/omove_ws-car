# Nav2 参数调试与平稳运行指南
## 一、调参顺序
```text
确认定位与单套 Nav2 正常
    → 备份参数，运行基准路线
    → 明确正向跟踪或全向平移需求
    → 协调速度、加速度和减速度
    → 检查路径对齐与转向评分
    → 检查车身碰撞、转角与路径净空
    → 用固定路线重复验证
    → 保存稳定版本和测试记录
```
每次只改一个目标对应的一组参数。不同时改导航评分、轮胎摩擦、运动学和定位配置，否则很难判断变化来自哪里。
## 二、先排除运行环境问题
开始测试前确认：
- Gazebo 正常运行，仿真未暂停。
- 激光与地图墙体基本对齐，定位正常。
- 仅运行一套 Nav2。
- 自动导航时不同时使用键盘遥控。
检查：
```bash
source /opt/ros/humble/setup.bash
source /home/deng/omove_ws/install/setup.bash
ros2 action info /follow_path
```
正常应有 `Action servers: 1`。本次曾因两套 Nav2 同时运行而出现 action 响应错配、行为树异常，并触发约 90° 的恢复旋转。这类问题应先清理重复节点，不能靠调小转速掩盖。
定位使用独立 slam_toolbox 时，使用仅导航入口 `navigation_launch.py`，避免再启动 AMCL。Navigation 2 面板的 Localization 状态不能直接判断独立 slam_toolbox 定位是否正常。
## 三、备份与建立基准
首次保存当前稳定版本，例如：
```bash
cp /home/deng/omove_ws/src/omove_description/config/nav2_params.yaml \
   /home/deng/omove_ws/src/omove_description/config/nav2_params.stable_018.yaml
```
目标备份已存在时换一个名称，避免覆盖旧记录。
先用未修改参数跑一次固定路线，记录到达结果、耗时、摆头、停车误差、转角净空和恢复次数。之后尽量使用相同起点、目标位置和目标朝向比较。
## 四、明确运动方式：正向曲线还是全向平移
车体坐标系：x 正方向为车头，y 正方向为左侧，正 angular.z 为向左旋转。

| 分量 | 作用 |
|---|---|
| linear.x | 前进或后退 |
| linear.y | 左右横移 |
| angular.z | 左右转向 |

正向走曲线可以由 `linear.x > 0` 和 `angular.z ≠ 0` 实现，不需要非零的横移速度。

当前 FollowPath 采用：

```yaml
      min_vel_x: 0.0
      max_vel_x: 0.18
      min_vel_y: 0.0
      max_vel_y: 0.0
      vy_samples: 1
      max_speed_xy: 0.18
```

这使 DWB 正常跟踪不生成倒车或横移目标。只将 max_vel_y 设为零而保留负的 min_vel_y，仍会允许向右横移。

此限制不改变麦克纳姆轮底盘本身的能力，键盘仍可横移。它也不约束独立的 BackUp、Spin 恢复行为。DWB 仍可能在急弯或终点停下转向，不保证全程连续走光滑圆弧。严格曲率、最小转弯半径和禁止原地转向需要配套规划器/控制器支持。

## 五、当前速度和加减速配置

### 1. FollowPath

以下是现有块中的相关参数摘录，不是完整替换文件：

```yaml
    FollowPath:
      plugin: dwb_core::DWBLocalPlanner
      min_vel_x: 0.0
      min_vel_y: 0.0
      max_vel_x: 0.18
      max_vel_y: 0.0
      max_vel_theta: 0.35
      min_speed_xy: 0.0
      max_speed_xy: 0.18
      min_speed_theta: 0.0

      acc_lim_x: 0.25
      acc_lim_y: 0.25
      acc_lim_theta: 0.5
      decel_lim_x: -0.25
      decel_lim_y: -0.25
      decel_lim_theta: -0.5
```

### 2. velocity_smoother 的四组限制

每个数组依次是 x、y、旋转方向：

```yaml
velocity_smoother:
  ros__parameters:
    max_velocity:
    - 0.18
    - 0.1
    - 0.35

    min_velocity:
    - -0.1
    - -0.1
    - -0.35

    max_accel:
    - 0.25
    - 0.25
    - 0.5

    max_decel:
    - -0.25
    - -0.25
    - -0.5
```

| 参数 | 含义 | 单位 |
|---|---|---|
| max_velocity | 各方向速度上界 | m/s、m/s、rad/s |
| min_velocity | 各方向速度下界，负值允许反方向运动 | m/s、m/s、rad/s |
| max_accel | 加速度限制 | m/s²、m/s²、rad/s² |
| max_decel | 减速度限制，按约定填负值 | m/s²、m/s²、rad/s² |

控制器前进和旋转上限、加减速限制应与平滑器协调。平滑器允许比 DWB 更宽的运动边界，正常跟踪仍由 DWB 的零倒车/横移范围约束。恢复行为是否经过平滑器取决于话题连接，不能仅靠平滑器参数判断。

增加前进速度不要求同步增大倒车范围。当前保留倒车边界 -0.1，而不是改成 -0.18。

## 六、想更平稳，分别调整什么

| 现象或目标 | 优先检查或调整 | 注意 |
|---|---|---|
| 起步过猛 | acc_lim_x、acc_lim_theta 与 max_accel | 分方向降低，避免反应过慢 |
| 减速突然 | decel_lim_* 与 max_decel | 幅值过小会增加停车距离 |
| 转向过快 | max_vel_theta | 只限制转速，不限制总转角 |
| 直线或曲线反复摆头 | angular.z 指令、路径评分、定位和里程计 | 先定位原因，再改权重 |
| 横着走或倒车走路径 | min/max_vel_y、min_vel_x | 先明确是否希望保留全向能力 |
| 弯道卡住 | 路径净空、完整车身检查、路径截取、正向运动可行性 | 不能只提高转速 |
| 到点后转回来 | 目标箭头、目标朝向容差、接近过程 | 可能在满足目标姿态 |
| 想更快 | max_vel_x、max_speed_xy、平滑器 max_velocity[0] | 小幅提速后重新验证 |

当前 0.25 m/s² 的平移加速度与 0.5 rad/s² 的角加速度是本次使用的参考值，不能认为越小越平稳。下降后应同时验证起步、弯道反应和停车距离。

## 七、路径对齐和转向评分

当前正向配置：

```yaml
      PathAlign.scale: 16.0
      PathAlign.forward_point_distance: 0.3
      GoalAlign.scale: 8.0
      GoalAlign.forward_point_distance: 0.3
      Twirling.scale: 1.0
      PathDist.scale: 32.0
      GoalDist.scale: 24.0
      RotateToGoal.scale: 32.0
      RotateToGoal.slowing_factor: 5.0
      RotateToGoal.lookahead_time: -1.0
```

PathAlign 用机器人前方投影点到路径的距离评分，鼓励沿路径对齐；GoalAlign 辅助朝目标接近；Twirling 惩罚旋转速度，降低多余旋转；PathDist、GoalDist 引导靠近路径和目标；RotateToGoal 参与终点旋转。

本次曾使用 PathAlign/GoalAlign 权重为零、Twirling=10 的全向配置，横移摆头明显减小，但会选择保持车头方向的横移/倒车，不符合后续“正向曲线”的要求。不能把不同运动风格的参数直接混用。

当前前视距离 0.3 m 是测试得到的参考值，不是通用最佳值。权重过强可能增加姿态修正；旋转惩罚过强也可能抑制必要转向。

## 八、转角卡住与障碍评分：区分当前文件和历史实验

编写本文时读取的主 YAML 中，实际启用的是：

```yaml
      critics:
      - RotateToGoal
      - Oscillation
      - BaseObstacle
      - GoalAlign
      - PathAlign
      - PathDist
      - GoalDist
      - Twirling
      BaseObstacle.scale: 0.02
```

历史转角实验曾将 BaseObstacle 替换为 ObstacleFootprint：

```yaml
      # 在 critics 列表中将 BaseObstacle 改成 ObstacleFootprint
      # 同时将对应权重项替换为：
      ObstacleFootprint.scale: 0.02
```

两者含义不同：BaseObstacle 按轨迹中心位置的代价评分，依赖合理膨胀层；ObstacleFootprint 按各离散轨迹姿态的车身轮廓边界检查障碍，也需合理配置代价地图和膨胀层。它不是连续运动扫掠体的精确碰撞证明。

历史候选曾在独立仿真的已知转角路线中成功到达，但使用 AMCL，与当前 slam_toolbox 定位流程不同，不能据此保证所有转角都通过。本文不自动把当前 YAML 改回历史候选。

如果再次出现 Failed to make progress，应检查路径是否贴墙、矩形车身转动净空、路径裁剪以及当前位置是否仍有正向可行轨迹。不要缩小真实车身尺寸来掩盖问题。

当前保留路径截取参数：prune_distance=1.0、forward_prune_distance=0.8、shorten_transformed_plan=true。改动这组参数需要重新测试内凹转角。

## 九、终点精度与目标设置

```yaml
    general_goal_checker:
      stateful: true
      plugin: nav2_controller::SimpleGoalChecker
      xy_goal_tolerance: 0.15
      yaw_goal_tolerance: 0.10
```

FollowPath.xy_goal_tolerance 同为 0.15。朝向容差 0.10 rad 约 5.7°。

位置和朝向是独立要求。Nav2 Goal 按下鼠标的位置指定终点，拖出的箭头指定终点朝向。测试直行时使目标朝向与车头一致；测试左转时将目标设在左前方，箭头朝期望的左转后方向。随意拖动箭头可能导致到点后额外大角度转向。

容差过严可能增加修正，过宽会增大停车误差。不要为了减少终点转向而随意放大容差。

## 十、采样与预测参数

当前使用：

```yaml
      vx_samples: 15
      vy_samples: 1
      vtheta_samples: 21
      sim_time: 1.7
      linear_granularity: 0.025
      angular_granularity: 0.025
      transform_tolerance: 0.2
```

采样数量决定候选速度覆盖，sim_time 是轨迹向未来预测的时长，granularity 是离散轨迹点的间隔。增加采样或减小间隔通常增加计算量，不保证更稳。transform_tolerance 是变换时间容差，不应靠增大它掩盖 TF 延迟。

若日志频繁出现 Control loop missed its desired rate，应检查 CPU 负载、控制频率及采样计算量，不继续盲目增加采样数。

## 十一、修改、重启与检查生效

1. 取消当前导航，确认小车停下。
2. 在原 Nav2 启动终端按 Ctrl+C，等待节点退出。
3. 编辑并保存 YAML；保持缩进，修改现有项，不添加重复键。
4. Gazebo、定位和 RViz 可以继续运行。
5. 只启动一套 Nav2，并显式指定源码参数文件。

```bash
source /opt/ros/humble/setup.bash
source /home/deng/omove_ws/install/setup.bash

ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  autostart:=true \
  use_composition:=False \
  params_file:=/home/deng/omove_ws/src/omove_description/config/nav2_params.yaml
```

仅修改这份绝对路径的 YAML，无需重新编译；保存文件不会自动更新正在运行的节点。

检查：

```bash
ros2 param get /controller_server FollowPath.max_vel_x
ros2 param get /controller_server FollowPath.max_vel_theta
ros2 param get /velocity_smoother max_velocity
ros2 action info /follow_path
```

预期前进上限 0.18、转向上限 0.35、平滑器第一项 0.18、路径跟踪服务器只有一个。

## 十二、固定测试路线与判断标准

使用 Nav2 Goal 测试控制器效果。键盘指令不经过 DWB，不能用键盘运动效果代替导航验证。

| 顺序 | 路线 | 重点观察 |
|---|---|---|
| 1 | 约 1 米短直线，目标朝向不变 | 起步、摆头、停车 |
| 2 | 左/右前方约 30°～45° 的目标 | 是否边前进边转向 |
| 3 | 开阔区域较长直线 | 速度和定位稳定性 |
| 4 | 已知内凹转角 | 车身净空、通过能力和恢复行为 |
| 5 | 目标位置相近、朝向不同 | 终点转向和停止 |

每类重复两三次，尽量固定起点、目标和朝向，记录到达情况、耗时、路径偏差、运动中的朝向变化、终点误差及恢复次数。全程观察 Gazebo 实际车体，不能只看轮式里程计。

想进一步提速时，以 0.18 m/s 为基线，下一档可尝试 0.20 m/s，只同步修改 max_vel_x、max_speed_xy 和平滑器 max_velocity 第一项；其他参数先保持不动。每档重新验证直线和转角，退化则恢复上一档。

## 十三、异常时同步录包

只比较同一次运动期间的数据。停车后几乎为零的里程计角速度不能证明此前转向失效。

```bash
ros2 bag record \
  -o /home/deng/omove_ws/docs/diagnostics/navigation_tuning_check_01 \
  /clock /cmd_vel_nav /cmd_vel /odom /joint_states \
  /front_left_velocity_controller/commands \
  /front_right_velocity_controller/commands \
  /rear_left_velocity_controller/commands \
  /rear_right_velocity_controller/commands \
  /tf /tf_static /plan /local_plan
```

开始录制后发送一次目标，停车后 Ctrl+C 保存。输出目录存在时更换编号。

| 同步观察结果 | 排查方向 |
|---|---|
| 转向指令反复变号，实际也反复转 | 轨迹选择、评分、路径和反馈 |
| 指令转向但四轮目标没有差速 | 运动学或速度链路 |
| 四轮目标有差速但轮速反馈不跟随 | 轮速控制和底盘执行 |
| 轮速/里程计显示转动但实际车体不转 | 轮地接触或打滑，需独立物理真值 |
| 靠近终点才明显转向 | 目标姿态与终点控制 |

## 十四、本次结论与适用范围

用户反馈目前 0.18 m/s 运行较平稳。可先保存这套配置作为日常仿真基线，后续以相同路线做对比，而不是继续无目的修改所有参数。

当前主文件与历史转角候选的障碍评分器存在差异，已在第八节明确记录。过去全向配置的自动测试结果不应直接当作当前正向配置、提速版本的验证结果。

本文总结当前仿真中的操作经验，不保证所有狭窄路线、动态障碍、负载或真实机器人上都稳定。更高速度、禁止原地转向、严格曲率跟踪以及真实硬件需要另外验证。

相关历史报告：`/home/deng/omove_ws/docs/diagnostics/dwb_tuning/README.md`。
