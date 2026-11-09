# world01

`playground.world` 的可搬迁场景资源包。墙体、地面、碰撞体、物理参数、太阳光、保存的场景状态和相机视角均保留。所需的灰色材质、天花板瓷砖材质及 `ceiling_tiled.jpg` 贴图已随包提供，无需下载在线模型，也不依赖原工作空间或用户的 `~/.gazebo/models`。

## 运行环境

Linux、Bash、Gazebo **Classic 11**（打包机器版本为 11.10.2），SDF 1.6。图形界面还需可用的显示服务及 OpenGL 驱动。目标机器须预先安装 Gazebo；此包不包含仿真器及其系统库。新版 Gazebo Sim 的 `gz sim` 不是本包的直接运行目标。为减少物理及渲染差异，建议使用相同的 Gazebo 版本。

## 解压与启动

```bash
tar -xzf world01.tar.gz
cd world01
./run.sh
```

无图形界面运行：

```bash
./run.sh --headless
```

脚本按自身所在位置解析资源，因此可以从任意工作目录调用，移动后的路径也可包含空格。关闭 GUI 或按 Ctrl+C 停止仿真。

## 在现有 ROS 2 仿真启动流程中使用

该 world 自身不包含车辆。原项目的 `qingzhou_sim.launch.py` 会另外通过 `spawn_entity.py` 生成车辆；车辆模型、控制器和导航程序不属于场景资源包。仅重现场地不需要 ROS。

若目标环境另外安装了原项目的 ROS 2 包，可使用：

```bash
source /path/to/ros_workspace/install/setup.bash
source /path/to/world01/setup.bash
ros2 launch qingzhou_gazebo qingzhou_sim.launch.py world_name:=/path/to/world01/worlds/playground.world
```

将 `/path/to/...` 替换为目标机器的实际路径。也可在其他 Gazebo Classic 启动流程中先 source `setup.bash`，再指定 `worlds/playground.world` 的绝对路径。

## 文件与完整性

- `worlds/playground.world`：可移植场景；仅调整资源 URI 和材质名称。
- `models/world01_materials/`：独立的材质资源和原始贴图。
- `run.sh`、`setup.bash`：启动与资源路径配置。
- `PROVENANCE.txt`、`licenses/`：来源、原文件哈希及第三方许可证。
- `SHA256SUMS`：包内各文件校验值；在包根目录执行 `sha256sum -c SHA256SUMS` 检查。

原 world 中 `state` 的保存位置与模型定义的初始 `pose` 不同。本包完整保留两者，交由 Gazebo Classic 按原文件方式恢复，避免改变场地位置。不同仿真器对保存状态的处理可能不同。
