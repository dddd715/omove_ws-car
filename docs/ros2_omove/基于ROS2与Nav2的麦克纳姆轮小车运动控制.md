# 基于 ROS 2 与 Nav2 的麦克纳姆轮小车运动控制
## 任务一　ROS 2 系统与工程认知

### 1．工程目录

```text
/home/deng/omove_ws/
├── src/omove_description/
│   ├── launch/              # 仿真、建图、导航入口
│   ├── urdf/                # 车体、轮子、滚子和插件
│   ├── config/              # 控制器、SLAM 与 Nav2 参数
│   ├── omove_description/   # 运动学与里程计 Python 节点
│   ├── worlds/room.world    # 四面墙与中央箱体
│   ├── maps/                # 随包安装的默认 room 地图
│   ├── rviz/                # 模型、地图、激光、导航面板配置
│   ├── scripts/             # 导航集成验证脚本
│   ├── test/                # 单元与模型检查
│   ├── package.xml
│   └── CMakeLists.txt
├── maps/room_new.yaml       # 用户重新建图保存的配置
├── maps/room_new.pgm        # 与 YAML 配套的地图图像
├── build/                  # 构建产物
├── install/                # 运行时加载的安装产物
└── log/                    # 构建与测试日志
```

src 中是可编辑源码；ros2 launch 通常读取 install 中的文件。修改启动文件、模型或参数后需要重新构建并 source。工作空间 maps/room_new 与包内 maps/room 是两组不同文件，使用新地图时必须显式传入 map 参数。

### 3．运行入口及职责

| 入口 | 作用 | 是否包含 SLAM/AMCL |
| --- | --- | --- |
| gazebo.launch.py | 底盘、控制器、里程计及 Gazebo；可指定 world、x、y | 均不包含 |
| slam_sim.launch.py | 房间仿真、底盘与 SLAM | 包含 SLAM，不包含 AMCL |
| nav_bringup.launch.py | 为已有底盘启动地图定位与 Nav2 | 包含 AMCL，不启动 Gazebo |
| nav_sim.launch.py | 一次启动房间、底盘、AMCL 与 Nav2 | 包含 AMCL，不包含 SLAM |

## 任务二　麦克纳姆轮运动原理与仿真模型
### 1．X 型麦轮逆运动学

设车体期望速度为 vx、vy、ω，a=Lx+Ly，则四轮角速度为：

```text
w_FL = (vx - vy - a·ω) / r
w_FR = (vx + vy + a·ω) / r
w_RL = (vx + vy - a·ω) / r
w_RR = (vx - vy + a·ω) / r
```

| 基本运动 | FL | FR | RL | RR |
| --- | --- | --- | --- | --- |
| 前进 vx>0 | + | + | + | + |
| 左移 vy>0 | − | + | + | − |
| 逆时针旋转 ω>0 | − | + | − | + |

例如 vx=0.1 m/s、vy=0、ω=0 时，四轮角速度均约 1.316 rad/s。手性、轮关节轴方向和公式必须同时一致，不能只凭轮子外观判断。

### 3．被动滚子模型

当前轮毂采用半径 0.045 m 的圆柱碰撞体，滚子中心半径为 0.061 m，滚子半径 0.015 m、长度 0.04 m。滚子通过 continuous 关节自由转动，设置小阻尼；前左、后右与前右、后左采用相反手性。

轮关节局部 Z 轴经过绕 X 轴 −π/2 旋转后指向车体 +Y，使正轮速对应前进。48 个被动关节只有 position、velocity 状态接口，没有电机命令接口。轮毂缩小后，地面接触由滚子承担，避免普通圆柱轮毂阻挡横移。

早期普通圆柱轮模型曾出现“轮速里程计横移、真实车体不横移”。因此必须用 Gazebo 真实位姿对比里程计，不能仅看 RViz 模型是否移动。当前模型是简化圆柱滚子，不包含真实鼓形外形、材料和装配误差。

### 4．正运动学与里程计

```text
vx = r·(w_FL + w_FR + w_RL + w_RR) / 4
vy = r·(-w_FL + w_FR + w_RL - w_RR) / 4
ω  = r·(-w_FL + w_FR - w_RL + w_RR) / (4a)
```

mecanum_odometry.py 根据关节名称匹配轮速，使用 joint_states 的时间戳积分。对于恒定车体速度，令 θ=ωΔt，A=sinθ/ω，B=(1−cosθ)/ω，则车体位移增量为 dx=A·vx−B·vy、dy=B·vx+A·vy，再按当前航向旋转到 odom 坐标系。ω 接近零时采用相应极限，避免数值除零。

重复时间戳不积分；时间倒退时重置累计位姿；缺失轮速和非有限数值不用于积分。初始协方差尚未通过实车滑移数据标定。

思考题：轮子发生空转时，轮速里程计是否仍会变化？为什么需要激光定位修正？

## 任务三　ROS 2 通信、控制器与运行准备

### 1．主要数据接口

| 接口 | 类型 | 发布者与用途 |
| --- | --- | --- |
| /cmd_vel | geometry_msgs/msg/Twist | 遥控或导航速度平滑器发布，运动学节点接收 |
| /四轮控制器/commands | std_msgs/msg/Float64MultiArray | 每个单轮控制器接收一个元素的数组 |
| /joint_states | sensor_msgs/msg/JointState | joint_state_broadcaster 发布轮子与滚子状态 |
| /odom | nav_msgs/msg/Odometry | 轮速正运动学与积分输出 |
| /scan | sensor_msgs/msg/LaserScan | 仿真二维激光，每帧 360 线 |
| /map | nav_msgs/msg/OccupancyGrid | SLAM 或地图服务器发布 |
| /initialpose | geometry_msgs/msg/PoseWithCovarianceStamped | RViz 初始定位输入 |
| /plan | nav_msgs/msg/Path | 全局规划路径 |
| /navigate_to_pose | nav2_msgs/action/NavigateToPose | 导航目标、反馈和结果 |

注意：LaserScan 是二维扫描消息，不是 PointCloud2 三维点云。四轮 commands 必须使用 Float64MultiArray，不能替换为 Float64。

### 2．TF 坐标链与时间

```text
map → odom → base_footprint → base_link → laser_link
                                    └→ wheel → roller
```

map→odom 在建图时由 SLAM 发布，在已有地图导航时由 AMCL 发布；odom→base_footprint 由轮速里程计发布；车体与传感器固定变换及轮关节变换由 robot_state_publisher 结合 joint_states 发布。

仿真节点使用 use_sim_time=true 和 Gazebo /clock。重启 Gazebo 会使仿真时间回退，应重新打开 RViz，避免旧缓存干扰。指令超时检查则使用单调时钟，不因 /clock 暂停而停止。

### 3．环境准备与依赖

以下操作以已经安装 ROS 2 Humble 的 Ubuntu 22.04 为前提，不包含操作系统与 ROS 的完整安装过程。已有环境不必重复安装。需要时可使用：

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep \
  ros-humble-desktop ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control ros-humble-ros2-controllers \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-teleop-twist-keyboard
```

依赖补全与构建：

```bash
cd /home/deng/omove_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select omove_description
source install/setup.bash
```

rosdep 需要事先初始化并更新规则；已有本机开发环境通常已经完成。构建正常应显示 Finished 和 Summary，不应在 Failed 状态下继续启动。

### 4．控制器与指令保护

控制器参数位于 config/mecanum_controllers.yaml，controller_manager 更新频率为 100 Hz。四个 JointGroupVelocityController 分别驱动四轮，joint_state_broadcaster 发布反馈；Gazebo 插件加载参数，launch 中的 spawner 等待管理器后加载并激活。

```bash
ros2 control list_controllers
```

应有五个 active 控制器：joint_state_broadcaster，以及 front_left、front_right、rear_left、rear_right 四个 velocity_controller。

运动学节点默认 cmd_timeout=0.5 s，检查周期 0.05 s；指令中断后四轮持续输出零速。max_wheel_speed=12 rad/s，超限时等比例缩放四轮；NaN、无穷或计算溢出立即输出零速。该保护依赖进程仍运行，不能代替控制器端或真实硬件急停。

## 任务四　Gazebo 下四轮机器人运动控制

### 1．终端分工

| 终端 | 用途 | 运行期间 |
| --- | --- | --- |
| 终端 1 | 仿真与底盘 | 保持运行 |
| 终端 2 | 检查话题、TF 或显示 | 按需操作 |
| 终端 3 | 键盘控制 | 保持焦点接收按键 |

每个新终端先执行下面两行。source 的环境不会自动传递到其他已打开终端。

```bash
source /opt/ros/humble/setup.bash
source /home/deng/omove_ws/install/setup.bash
```

### 2．启动基础仿真

终端 1：

```bash
ros2 launch omove_description gazebo.launch.py
```

此入口默认空世界，不加载房间、SLAM 或 AMCL。无图形窗口可增加 gui:=false。若需要检查房间与扫描，应直接使用下一任务的 slam_sim 入口，不要与本入口叠加运行。

终端 2：

```bash
ros2 topic echo /odom --once
ros2 topic echo /scan --once --field header
ros2 run tf2_ros tf2_echo odom base_footprint
```

TF 命令持续运行，按 Ctrl+C 结束当前检查，不影响终端 1。

### 3．键盘控制

终端 3：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p speed:=0.1 -p turn:=0.3
```
按键布局以节点打印的帮助为准；本实验先用前进和旋转完成扫描。持续按住按键依赖系统键盘重复发送，首次重复延迟可能触发 0.5 s 停车。不要把“按一次后持续走”作为预期行为。

单独验证横移可在无导航、无其他指令源的条件下执行下例，停止时 Ctrl+C，等待超时停车：

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {y: 0.1}, angular: {z: 0.0}}'
```

实验观察：必须同时观察 Gazebo 真实车体，不能仅看 RViz 的里程计模型。完成后停止所有该任务进程，再切换下一任务。

## 任务五　SLAM Toolbox 激光建图与地图保存

### 1．启动前准备

关闭原仿真、导航、遥控和 RViz。启动新建图时不加载旧地图，不需要 2D Pose Estimate。房间为约 6×6 m，中央有 0.5×0.5 m 静态箱体。

终端 1（加载环境后）：

```bash
ros2 launch omove_description slam_sim.launch.py
```

终端 2 先检查：

```bash
ros2 node list
ros2 run tf2_ros tf2_echo map laser_link
```

应有 /slam_toolbox，并能够输出 map 到 laser_link 的变换。结束 TF 检查，再启动显示：

```bash
rviz2 \
  -d /home/deng/omove_ws/install/omove_description/share/omove_description/rviz/omove_navigation.rviz \
  --ros-args -p use_sim_time:=true
```

### 2．显示配置与建图参数

| 显示项 | 话题与配置 |
| --- | --- |
| Fixed Frame | map |
| RobotModel | /robot_description，Topic 来源，Transient Local |
| Map | /map，Transient Local，Reliable |
| LaserScan | /scan，Best Effort；红色扫描点 |

地图与模型描述使用保留历史消息，解决“后开 RViz 收不到一次性数据”；激光采用传感器兼容 QoS。建图时 RViz 中的 Nav2 Goal 面板不用于发送导航目标。

config/slam_params.yaml 设置 odom_frame=odom、map_frame=map、base_frame=base_footprint、scan_topic=/scan，采用在线异步 mapping。地图分辨率为 0.05 m，处理激光范围约 0.16–8 m，最小移动与转角阈值为 0.1 m、0.1 rad。传感器本身下限为 0.15 m，扫描频率为 10 Hz。

### 3．移动扫描

终端 3 使用前一任务的低速键盘控制命令。先短距离试走，确认扫描随运动更新；再绕箱体走一圈，慢速观察墙面与角落，尽量返回起点附近。避免快速旋转造成配准困难。

验收要点：红色扫描点与已有墙面基本重合；地图没有明显双墙或撕裂；初始区域和回到起点后的区域能够匹配。不要求扫描点与 5 cm 栅格边缘逐像素重合。

### 4．停车、保存并核对文件

终端 3 按 k 停车，Ctrl+C 结束遥控；终端 1 的 SLAM 保持运行。然后执行：

```bash
mkdir -p /home/deng/omove_ws/maps
ros2 run nav2_map_server map_saver_cli \
  -f /home/deng/omove_ws/maps/room_new \
  --ros-args -p save_map_timeout:=10.0
ls -lh /home/deng/omove_ws/maps/room_new.*
```

出现 Map saved successfully，并同时存在 room_new.yaml 与 room_new.pgm，才算保存完成。同名保存会覆盖旧文件，应按实验批次另命名。地图保存后再关闭建图。

| 地图文件 | 内容 |
| --- | --- |
| PGM | 占据、空闲和未知栅格图像 |
| YAML | 图像引用、resolution、origin、阈值等元数据 |

origin 是图像栅格在地图坐标中的位置，不是机器人当前位姿。只保存 PGM 或移动后破坏 YAML 的相对图像路径，都会影响加载。重新扫描不会改变 Gazebo 的墙和箱体；改变场景必须先修改 world。

思考题：为什么加载新建地图后，不能直接复制旧地图的初始化坐标？

## 任务六　Nav2 下定位、规划与自主导航

### 1．系统组成

Nav2 使用地图服务器加载栅格，AMCL 以激光和里程计估计 map→odom；planner_server 用 NavFn 生成路径，controller_server 用 DWB 评估局部速度轨迹，velocity_smoother 平滑后输出 /cmd_vel。bt_navigator 通过行为树协调规划、跟踪与恢复，生命周期管理器使各节点进入 active。

```text
地图 + /scan + /odom → AMCL → map→odom
目标动作 → BT Navigator → NavFn 路径 → DWB 控制
        → velocity_smoother → /cmd_vel → 四轮控制 → 反馈
```

### 2．全向底盘的关键参数

| 参数/模块 | 当前配置 | 实验意义 |
| --- | --- | --- |
| AMCL robot_model_type | nav2_amcl::OmniMotionModel | 描述含横移的运动噪声 |
| DWB 横向速度 | −0.15～0.15 m/s | 允许横移候选轨迹 |
| DWB 前后速度 | −0.15～0.20 m/s | 实验低速控制 |
| 最大角速度 | 0.6 rad/s | 限制转向 |
| velocity_smoother | Y 方向速度、加减速度非零 | 避免将横移滤成零 |
| 规划容差 | 0.1 m | 减少占据目标被附近替代终点接受 |
| 到点容差 | 0.25 m / 0.25 rad | 位置与朝向成功判定 |
| 底盘 footprint | 0.54×0.46 m 矩形，padding 0.01 m | 包含车轮外沿 |

规划容差和到点容差不是同一参数。侧向目标成功不表示全程纯横移，Nav2 可以组合平移与旋转。若要求航向严格保持，需要单独设计控制与验收条件。

### 3．代价地图

global_costmap 配置层级为 global_costmap→global_costmap→ros__parameters；local_costmap 同理。全局使用静态地图层、激光障碍层和膨胀层；局部使用滚动窗口、激光障碍层和膨胀层。scan 数据源必须明确 topic、LaserScan 类型、marking 和 clearing。

轮廓不能随意设为偏小的圆形半径，否则路径可能看起来可行但真实轮组碰撞。当前矩形是保守近似，仍需结合接触实验验证。

### 4．三终端启动新地图导航

先关闭 SLAM 与旧 Gazebo。终端 1：

```bash
cd /home/deng/omove_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ls -lh maps/room_new.yaml maps/room_new.pgm
ros2 launch omove_description nav_sim.launch.py \
  map:=/home/deng/omove_ws/maps/room_new.yaml
```

终端 2 加载环境后打开 RViz，命令同任务五。终端 3 检查：

```bash
ros2 control list_controllers
ros2 lifecycle get /bt_navigator
ros2 run tf2_ros tf2_echo map base_footprint
```

五个底盘控制器应为 active，bt_navigator 应输出 active [3]。初次定位前 map TF 可能尚未建立，完成下一步后应恢复。

### 5．初始化定位与激光对齐

点击 2D Pose Estimate，在地图中真实机器人对应位置按住鼠标，沿车头方向拖出箭头再松开。新地图的原点可能与 Gazebo 世界原点不同，不能直接把世界坐标 (1.5, 0) 当作新地图坐标。初始判断依据是房间墙、箱体和机器人相对位置。

等待激光与地图基本重合。整体平移偏差通常需要修正位置，沿墙远端逐渐增大的偏差通常需要修正角度。多面墙无法同时对齐时，应检查地图变形和坐标链，不应只靠反复点初始位置掩盖问题。

![图 6-1 用户新地图定位对齐后的 RViz 显示](assets/rviz_localization.png)

图 6-1 为本次操作记录中的用户截图。红色为激光扫描，黑色为地图障碍；该图证明显示与定位可用，不作为厘米级精度测量依据。

### 6．目标发送与导航面板

RViz 必须同时有 nav2_rviz_plugins/GoalTool 和 Navigation 2 面板。若缺面板，选择 Panels→Add New Panel→nav2_rviz_plugins→Navigation 2。正常应能看到导航状态。

| 按钮 | 功能 | 是否驱动 Gazebo |
| --- | --- | --- |
| 2D Pose Estimate | 重新设置机器人当前位姿估计 | 否，只改变定位 |
| Nav2 Goal | 指定要到达的位置和朝向 | 导航执行时驱动 |

点击 Nav2 Goal 后，在空闲目标点按住左键，拖出朝向箭头再松开。只看到 RViz 模型位置跳变而 Gazebo 不动时，先检查是否误点 2D Pose Estimate，以及 Navigation 2 是否确实收到动作目标。

保存显示设置：点击 RViz 窗口使其获得焦点，Ctrl+S 或 File→Save Config。源码配置保存后重新构建，安装目录配置才会更新；日常建议维护源码配置并统一构建。

## 任务七　完整复现、实验记录与项目验收

### 1．从全部关闭状态重跑

关闭所有旧启动进程、Gazebo、遥控和 RViz。依次执行构建、加载 room_new、新开 RViz、手动初始定位、确认 active，再进行以下三项测试。各目标之间不修改代码、不重启节点、不重新设定位姿；必要的首次定位属于正常操作。

| 测试 | 操作 | 通过标准 |
| --- | --- | --- |
| 近距离 | 前方约 0.5 m 设置目标 | 导航成功，真实小车到达附近 |
| 侧向 | 侧方约 0.5 m 设置目标，保持目标朝向相近 | 真实车体到达侧方；不只看 RViz |
| 绕障 | 箱体另一侧设置目标，直线穿过箱体 | 自动绕行，最终成功 |

目标应远离墙和箱体，实验建议留约 0.6 m 以上余量。每次等目标完成并停稳再发送下一个。导航运行时关闭键盘控制，避免两个源争用 /cmd_vel。

实验记录：本次对话中，用户已使用新地图完成从关闭状态重跑，确认近距离、侧向和绕障三个测试成功。该结论属于人工功能验收，不等同于自动测得的定位精度或长时间稳定性。

### 2．自动化验证与单元测试

```bash
cd /home/deng/omove_ws
colcon test --packages-select omove_description
colcon test-result --verbose
python3 -m unittest discover -s src/omove_description/test -v
```

当前已记录 15 项测试通过，涉及指令超时与恢复、异常值停车、轮速等比例限幅、轮子滚子模型、正运动学积分、时间重置和轮廓间距。单元测试不自动启动 Gazebo。

使用默认 room 地图的完整集成回归前，应关闭其他 Gazebo：

```bash
ros2 run omove_description validate_navigation.py \
  --output /tmp/omove_navigation_validation.json --domain 97
```

脚本使用独立 ROS domain，启动自己的仿真，在临时 world 加入真实位姿观测插件，结束后清理自己启动的进程。每个目标默认墙钟超时 150 s，可用 --goal-timeout 修改。该脚本的坐标按默认 room 地图设计，不能直接拿来验收原点不同的 room_new 地图。

### 3．已记录的工程实验结果

| 滚子模型实验 | Gazebo 真实增量 | 轮速里程计增量 |
| --- | --- | --- |
| 前进 | X +0.2185 m | X +0.2217 m |
| 横移 | Y +0.2235 m | Y +0.2346 m |
| 转向 | yaw +0.7994 rad | yaw +0.8086 rad |

横移伴有约 X +0.0036 m 和 yaw +0.0275 rad 的偏移。滚子使仿真速度变慢，上表比较同一采样窗口的真实与估计增量，不能将 4 s 墙钟当作 4 s 仿真时间。数据为此前开发记录，不是本文生成时重新测量。

| 默认地图连续导航回归指标 | 已记录结果 |
| --- | --- |
| 连续目标 | (−1.2,0)、(−1.2,1)、(1.5,0) 成功 |
| 绕障路径最大横向偏移 | 约 0.921 m |
| 箱体中心占据目标 | 规划返回 ABORTED |
| Spin 行为与之后的新导航目标 | 均成功 |
| 运行墙钟时间 / 定位采样数 | 约 95.9 s / 122 个 |
| 平均 / 最大位置误差 | 约 0.0417 / 0.0768 m |
| 采样轮廓最小几何间距 | 约 0.1784 m |

以上误差依赖默认地图与世界坐标对齐，不能直接推广到新地图或实车。采样几何间距不是连续碰撞力检测，单独调用 Spin 也不代表行为树全部自动恢复分支已验证。

### 4．验收材料

交付时保留 src 源码、maps/room_new.yaml 与 PGM、RViz 配置、本指导书、测试输出和演示视频。演示应包含启动、初始定位、近距离、侧向和绕障目标，而非只展示最终静止画面。停止时先结束运动指令，再 Ctrl+C 退出仿真与 RViz。

## 任务八　常见问题分析与排查

| 现象 | 原因检查 | 处理方法 |
| --- | --- | --- |
| Fixed Frame 提示 map 不存在 | 无 SLAM/AMCL 变换、未初始化定位 | 查节点；导航时设置初始位姿；建图时确认 slam_toolbox |
| 地图宽高为 0 | 话题错误、无发布者或错过历史地图 | Map 选 /map，Durability 为 Transient Local |
| RobotModel 不显示 | 未订阅描述或 TF 不完整 | Topic 来源 /robot_description；Transient Local；查 joint_states |
| 激光缺失或队列满 | /scan QoS、map TF 或时间不匹配 | Best Effort；查 map→laser_link；use_sim_time=true |
| 重启后画面异常 | /clock 回退，RViz 保留旧缓存 | 关闭旧 RViz，随新仿真重新打开 |
| Nav2 Goal 无动作 | 缺 Navigation 2 面板、未拖动释放、节点未激活 | 补面板，确认 active 与动作日志 |
| RViz 小车跳动而 Gazebo 不动 | 误用 2D Pose Estimate 或轮速空转 | 查设置位姿日志、真实位置及轮速反馈 |
| 导航到障碍附近也报告规划成功 | 规划容差允许替代终点 | 当前收紧到 0.1 m；区分路径成功和目标可达 |
| 找不到配置或新修改不生效 | 未构建、未 source、加载了旧安装目录 | 重新 colcon build 并逐终端 source |
| /scan 正常但不建图 | 无 odom 动态 TF 或 SLAM 未启动 | 查 slam_toolbox、odom→base、时间戳 |
| 保存地图超时 | /map 未生成或 SLAM 已关闭 | 保持建图运行，确认 /map 后再保存 |
| 四轮计算横移但真实车体不动 | 普通圆柱轮或旧模型阻碍侧向接触 | 确认 48 个被动滚子已加载，比较真实位姿 |

常用检查命令（每次先加载环境）：

```bash
ros2 node list
ros2 topic list
ros2 topic info /scan --verbose
ros2 topic echo /scan --once --field header
ros2 topic echo /odom --once
ros2 control list_controllers
ros2 lifecycle get /bt_navigator
ros2 run tf2_ros tf2_echo map laser_link
```

排查时先验证节点和数据，再查参数，最后才修改模型。不要同时启动多套 Gazebo，避免 master 端口与实体冲突。

## 附录 A　两种运行模式速查

| 模式 | 终端 1 | 终端 2 | 终端 3 |
| --- | --- | --- | --- |
| 重新建图 | slam_sim.launch.py | RViz，确认扫描与地图 | 键盘控制，完成后保存地图 |
| 已有地图导航 | nav_sim.launch.py + map 参数 | RViz，初始定位与 Nav2 Goal | 状态检查，不运行遥控 |
