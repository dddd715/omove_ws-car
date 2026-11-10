# omove_ws-car

基于 ROS 2 与 Nav2 的麦克纳姆轮小车运动控制项目，包含 Gazebo 仿真、建图、定位、导航和诊断工具。包含使用 slam_toolbox 定位的场景，具体启动方式见部署手册。

## 项目结构

| 路径 | 用途 |
| --- | --- |
| `src/omove_description/` | ROS 2 功能包：模型、启动文件、配置、节点、插件和测试 |
| `maps/` | world01 栅格地图与 SLAM 位姿图数据 |
| `docs/` | 部署手册、教程、调试记录和诊断资料 |
| `rosgraph.png` | ROS 节点关系图 |
| `build/`、`install/`、`log/` | 本地构建和运行生成的目录，不纳入 Git |

## 文档入口

- [部署与使用手册](docs/ROS2与Nav2麦克纳姆轮小车部署与使用手册.md)
- [功能包说明](src/omove_description/README.md)
- [运动控制教程](docs/ros2_nav2_tutorial/ROS2与Nav2麦克纳姆轮小车运动控制教程.md)
- [ROS 2 与 Nav2 运动控制说明](docs/ros2_omove/基于ROS2与Nav2的麦克纳姆轮小车运动控制.md)
- [Nav2 参数调试与平稳运行指南](docs/ros2_omove/assets/Nav2参数调试与平稳运行指南.md)
- [历史调试记录（2026-10-31）](docs/2026-10-31_ROS2_Nav2麦克纳姆轮小车调试记录.md)
- [历史关键代码快照（2026-10-31）](docs/2026-10-31_ROS2_Nav2关键代码快照.md)

## 开发与维护

- 运行环境、依赖、构建和启动步骤见部署手册及功能包说明。
- `docs/diagnostics/` 保存诊断脚本、参数对照和分析结果，部分路径被脚本和文档引用。
- 源码与文档修改后，通过 Git 提交并推送；编译产物、缓存、诊断日志和录制数据按 `.gitignore` 排除。
- 历史记录和代码快照用于追溯；当前实现以 `src/` 中的源码和配置为准。
- 修改文件位置时，应同步检查文档链接、脚本路径和资源安装规则。
