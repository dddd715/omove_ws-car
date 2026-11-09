# slam_toolbox：mapping 与 localization 对比及切换流程

本文以 `/home/deng/omove_ws` 工作区、ROS 2 Humble 和 `world01` 仿真场景为例，记录从建图模式切换到定位模式的完整操作，包括地图保存、参数编辑、RViz 设置和重启验证。

## 一、两种模式的区别

`slam_toolbox` 是提供建图和定位功能的 ROS 软件包。`mapping` 与 `localization` 是两种工作方式；正确拼写是 `localization`，不是 `location`。

| 对比项 | mapping（建图） | localization（定位） |
|---|---|---|
| 主要任务 | 一边估计机器人位置，一边建立、扩展地图 | 根据已有地图估计机器人位置 |
| 典型用途 | 首次探索环境、补图 | 已有地图中的重复运行 |
| 是否需要已有地图 | 可以从零开始 | 本文流程需要已保存的序列化位姿图 |
| 地图处理 | 将符合条件的扫描用于扩展和优化位姿图 | 利用已有位姿图及临时扫描缓冲区定位，不永久扩展原地图 |
| 启动入口 | `online_async_launch.py` | `localization_launch.py` |
| 节点程序 | `async_slam_toolbox_node` | `localization_slam_toolbox_node` |
| 本工程仿真参数 | `slam:=true`，随仿真启动建图 | `slam:=false`，另开终端启动定位节点 |

原配置中的：

```yaml
mode: mapping #localization
```

实际值是 `mapping`，`#localization` 只是注释。只修改 YAML 中的 `mode`，不足以完成切换；本文同时更换启动入口并加载位姿图。

`mapping → localization` 是改变 slam_toolbox 的工作方式；`AMCL → slam_toolbox` 才是更换定位组件。本文只验证 slam_toolbox 定位，尚未接入 Nav2 自主导航。

## 二、终端环境与地图文件准备

每打开一个用于 ROS 命令的新终端，先执行：

```bash
source /opt/ros/humble/setup.bash
source /home/deng/omove_ws/install/setup.bash
```

### 1. 检查是否已经保存位姿图

```bash
ls -lh /home/deng/omove_ws/maps/world01_slam.posegraph \
       /home/deng/omove_ws/maps/world01_slam.data
```

| 文件 | 用途 |
|---|---|
| `world01_slam.posegraph` + `world01_slam.data` | 本文 slam_toolbox 定位流程需要的序列化位姿图和数据 |
| `world01.yaml` + `world01.pgm` | 栅格地图，可供 AMCL 使用，不能替代上面的位姿图文件 |

如果文件使用了其他名称，应在后续参数中填写对应的路径前缀。

- 两个文件都已存在：可以跳到第四节，准备定位参数。
- 当前仍在建图：先完成第三节中的保存操作。
- 原建图进程已退出且没有位姿图备份：不能通过重新打开 Gazebo 恢复未保存的位姿图，需要重新建图。

## 三、建图并保存序列化位姿图

### 1. 启动建图

终端一执行：

```bash
source /opt/ros/humble/setup.bash
source /home/deng/omove_ws/install/setup.bash

ros2 launch omove_description world01_sim.launch.py \
  slam:=true \
  slam_params_file:=/home/deng/omove_ws/src/omove_description/config/slam_params.yaml
```

此时 `slam:=true` 是正确配置：它启动 Gazebo，并通过在线异步入口启动建图。等待场景、机器人和控制器加载完成。

### 2. 打开 RViz 和键盘遥控

新终端打开 RViz：

```bash
source /opt/ros/humble/setup.bash
source /home/deng/omove_ws/install/setup.bash
rviz2 --ros-args -p use_sim_time:=true
```

按第六节配置地图、激光和机器人模型。建图阶段一般不需要使用 `2D Pose Estimate`。

另开终端启动键盘遥控：

```bash
source /opt/ros/humble/setup.bash
source /home/deng/omove_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

| 按键 | 动作 |
|---|---|
| `i` | 前进 |
| `,` | 后退 |
| `j` / `l` | 原地左转 / 右转 |
| `k` | 停止 |
| `z` | 降低线速度和角速度 |
| `q` | 提高线速度和角速度 |

保持键盘焦点在遥控终端。先按 `z` 降低速度，观察终端提示，可先使用约 0.05～0.1 m/s 的线速度。松开前进键不一定停车，应主动按 `k`。

缓慢走过需要定位的区域，避免快速转向。条件允许时回到经过的位置，观察墙体是否对齐。当前建图配置的地图更新周期为 5 秒，移动后应等待地图刷新。

### 3. 保持建图运行，保存位姿图

地图基本完整且墙体稳定后，先停下小车。不要关闭建图进程，在新终端执行：

```bash
source /opt/ros/humble/setup.bash
source /home/deng/omove_ws/install/setup.bash

ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/deng/omove_ws/maps/world01_slam'}"
```

成功时返回 `result: 0`。确认生成文件：

```bash
ls -lh /home/deng/omove_ws/maps/world01_slam.posegraph \
       /home/deng/omove_ws/maps/world01_slam.data
```

必须确认两个文件都存在，再停止建图。该路径下已有同名文件时，本操作会更新保存结果，使用前确认是否需要保留旧版本。

如果服务不存在，检查：

```bash
ros2 node list
ros2 service list
```

确认建图节点存在，以及服务列表中有 `/slam_toolbox/serialize_map`。

## 四、创建定位专用参数文件

### 1. 复制并编辑

首次创建时执行以下复制命令。若定位配置已经编辑完成，不要再次复制，以免覆盖修改。

```bash
cp /home/deng/omove_ws/src/omove_description/config/slam_params.yaml \
   /home/deng/omove_ws/src/omove_description/config/localization_params.yaml

nano /home/deng/omove_ws/src/omove_description/config/localization_params.yaml
```

### 2. 修改现有参数

以下仅展示需要检查或修改的部分，其余扫描匹配参数保留。修改已有项，避免重复添加同名参数，也不要添加第二组 `slam_toolbox:` 或 `ros__parameters:`。

```yaml
slam_toolbox:
  ros__parameters:
    use_sim_time: true
    odom_frame: odom
    map_frame: map
    base_frame: base_footprint
    scan_topic: /scan

    mode: localization
    use_map_saver: false
    enable_interactive_mode: false

    # 已保存位姿图的路径前缀，不带 .posegraph 或 .data 后缀
    map_file_name: /home/deng/omove_ws/maps/world01_slam

    # 地图坐标系中的初始位姿：[x米, y米, yaw弧度]
    map_start_pose: [0.0, 0.0, 0.0]
```

参数前的 `#` 表示注释，去掉后才生效。参数行与 `odom_frame` 等项保持相同缩进，使用空格，不使用 Tab。

`map_start_pose` 是机器人在 `map` 坐标系中的初始位置估计，不保证等于 Gazebo 出生坐标。示例值不一定适合实际位置，可在启动后通过 RViz 调整。

在 nano 中按 `Ctrl+O`、回车保存，再按 `Ctrl+X` 退出。检查关键项：

```bash
rg -n 'mode:|use_map_saver:|enable_interactive_mode:|map_file_name:|map_start_pose:' \
  /home/deng/omove_ws/src/omove_description/config/localization_params.yaml
```

后面的启动命令直接读取源码目录中的参数文件绝对路径，因此仅修改这份配置不需要先重新编译。

## 五、停止建图并启动定位

### 1. 停止原建图进程

先在遥控终端按 `k` 停车，再关闭遥控。确认位姿图保存成功后，在运行 `world01_sim.launch.py slam:=true` 的终端按 `Ctrl+C`。

该启动文件同时管理仿真和建图，因此 Gazebo 也会一起退出。等待终端回到命令提示符。建议同时关闭旧 RViz，避免仿真重启后的旧时间与 TF 缓存干扰观察。

### 2. 终端一：启动不带建图的仿真

```bash
source /opt/ros/humble/setup.bash
source /home/deng/omove_ws/install/setup.bash

ros2 launch omove_description world01_sim.launch.py slam:=false
```

等待场景和小车加载完成。这里的 `slam:=false` 是关闭仿真入口自动启动的建图节点，定位节点由下一步单独启动。

### 3. 终端二：启动定位节点

```bash
source /opt/ros/humble/setup.bash
source /home/deng/omove_ws/install/setup.bash

ros2 launch slam_toolbox localization_launch.py \
  use_sim_time:=true \
  slam_params_file:=/home/deng/omove_ws/src/omove_description/config/localization_params.yaml
```

日志应显示启动 `localization_slam_toolbox_node`，读取 `world01_slam.posegraph`，并加载 Mapper、Dataset。保持该终端运行，加载后不持续输出日志是可能的，不能仅靠日志是否滚动判断定位状态。

不要同时运行原来的建图入口。当前工程的 `nav_bringup.launch.py` 会启动 AMCL，也不要与这套定位流程直接叠加，以免多个组件同时发布 `map → odom`。

## 六、RViz 显示与初始位姿设置

### 1. 启动并配置显示项

```bash
source /opt/ros/humble/setup.bash
source /home/deng/omove_ws/install/setup.bash
rviz2 --ros-args -p use_sim_time:=true
```

如果左侧面板未显示，选择 `Panels → Displays`。点击面板底部 `Add` 添加显示项。

| 显示项 | 配置 |
|---|---|
| `Global Options` | `Fixed Frame` 设置为 `map` |
| `Map` | Topic 为 `/map`；必要时将 Durability Policy 设为 `Transient Local` |
| `LaserScan` | Topic 为 `/scan` |
| `RobotModel` | 若有 Description Source，选择 Topic；Description Topic 为 `/robot_description`；勾选 Visual Enabled |

如果小车不显示，先确认已经添加并勾选 `RobotModel`，再展开其 Status 查看描述或 TF 错误。模型加载正常但太小时，调整视角和缩放。

### 2. 设置初始位姿

先参考 Gazebo 中小车实际位置和周围环境，再在 RViz 中操作：

1. 点击工具栏的 `2D Pose Estimate`。
2. 在已保存地图中小车实际所在的位置按下鼠标。
3. 拖动箭头，使其指向小车实际朝向。
4. 松开鼠标，观察激光与墙体是否基本重合。

这是告诉定位系统“小车现在在哪里”，不是发送导航目标。如果配置提供的初始位姿已经正确，无需反复点击。

### 3. 保存 RViz 配置

选择 `File → Save Config As`，保存到：

```text
/home/deng/omove_ws/src/omove_description/config/localization.rviz
```

该文件保存成功后，下次可直接加载显示配置：

```bash
rviz2 -d /home/deng/omove_ws/src/omove_description/config/localization.rviz \
  --ros-args -p use_sim_time:=true
```

## 七、定位验收

### 1. 检查节点、参数和 TF

在已加载 ROS 环境的新终端中执行：

```bash
ros2 node list
ros2 param get /slam_toolbox mode
ros2 run tf2_ros tf2_echo map base_footprint
```

`mode` 应返回 `localization`。最后一条命令应持续输出机器人在地图中的位置与朝向，按 `Ctrl+C` 结束查看。

还可以单独检查定位组件提供的变换：

```bash
ros2 run tf2_ros tf2_echo map odom
```

参数值正确不代表已经使用正确节点，需要同时确认第五节启动的是 `localization_launch.py`，且成功加载位姿图。

### 2. 移动验证

按第三节启动键盘遥控，在空旷处缓慢前进一小段，再按 `k` 停止，随后进行小角度转向。

通过标准：

- 地图正常加载，小车初始位置合理。
- 可查询 `map → base_footprint` 位姿。
- 移动时，RViz 中小车的位置和朝向相应变化。
- 激光仍与地图墙体基本对齐，没有明显跳位或持续偏离。

键盘能控制小车，只能证明运动控制链路正常；还要检查地图中的位姿和激光对齐情况，才能判断定位是否正常。

## 八、重启验证与结束顺序

### 1. 结束进程

无需重启电脑。按以下顺序关闭本次运行的进程：

1. 在键盘遥控终端按 `k`，确认小车停止。
2. 在遥控终端按 `Ctrl+C`。
3. 关闭 RViz，或在其启动终端按 `Ctrl+C`。
4. 在定位节点终端按 `Ctrl+C`。
5. 最后在仿真启动终端按 `Ctrl+C`，关闭 Gazebo。

等待各终端回到命令提示符。`tf2_echo` 等查看命令可随时结束。定位退出前不需要再次保存地图，已有位姿图和参数文件会保留。

### 2. 重新启动并验证

依次执行：

1. 按第五节启动仿真，使用 `slam:=false`。
2. 按第五节启动 `localization_launch.py`。
3. 按第六节打开 RViz，加载已保存的显示配置。
4. 必要时设置初始位姿。
5. 按第七节缓慢移动，检查定位。

重新启动后，地图可加载、初始定位可建立、移动后仍对齐，即通过重启验证。允许通过 `2D Pose Estimate` 提供初始位姿；本验证不等同于“无需初始位置提示的全局自动重定位”。

## 九、常见现象与处理

| 现象 | 说明与处理 |
|---|---|
| `.posegraph` 或 `.data` 不存在 | 检查路径前缀；只有栅格地图时需先建图并保存序列化位姿图 |
| 定位加载后终端不再输出 | 可能是正常运行状态，通过 RViz、TF 和移动测试验证 |
| Ceres 提示请求 50 个线程，但最大支持 32 个 | 日志表明已自动限制到上限，此警告本身不表示启动失败 |
| `clipped range threshold` | 内部调整量程阈值；若激光或地图异常，再检查传感器与参数的量程是否一致 |
| `Detected jump back in time` | 重启 Gazebo 会重置仿真时间；旧 RViz 可能清空 TF 缓存。偶发后恢复可继续观察，持续出现则检查仿真时钟 |
| `Trying to create a map of size ...` | RViz 创建地图显示的日志，本身不是错误，不代表重新建图 |
| 地图可见但没有小车 | 添加 RobotModel，检查 `/robot_description` 和模型 Status |
| 建图初期白色区域呈放射状 | 可能与激光射线稀疏及扫描覆盖不足有关；观察移动补充扫描后是否改善，不能仅凭截图断定正常 |
| 激光明显偏离墙体或小车跳位 | 先停车，检查初始位姿、TF、里程计及是否有重复定位/建图节点 |

## 十、完成范围与后续导航

完成本文后，运行方式为：

```text
Gazebo（slam:=false，提供激光和里程计）
    → slam_toolbox 定位节点（加载已保存位姿图）
    → RViz 中观察并验证定位
```

保留以下文件：

- `/home/deng/omove_ws/maps/world01_slam.posegraph`
- `/home/deng/omove_ws/maps/world01_slam.data`
- `/home/deng/omove_ws/src/omove_description/config/localization_params.yaml`
- `/home/deng/omove_ws/src/omove_description/config/localization.rviz`（手动保存后）

模式切换与移动定位验证完成后，无需为了定位再次建图。后续若要通过 `Nav2 Goal` 自动导航，需要另行接入 Nav2 规划与控制，并避免启动 AMCL，让 slam_toolbox 继续负责定位。
