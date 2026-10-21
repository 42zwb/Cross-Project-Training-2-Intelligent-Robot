# Cross Project Training 2 — Intelligent Robot

本仓库并列收录两套相互独立的 ROS 2 Humble 课程工程：

- [`single_robot/`](single_robot/)：TurtleBot4 单机器人教程工程，包含 `turtlebot4_tutorial` 与 `turtlebot4_cartographer` 两个 ROS 包。
- [`multi_robot/`](multi_robot/)：三台 TurtleBot4 协同建图与队形切换大作业，包含 SLAM、地图融合、Nav2 调度、安全门控、网页控制与离线评估。

## 构建

需要 Ubuntu 22.04、ROS 2 Humble 和各项目 README 中列出的 ROS 依赖。

```bash
source /opt/ros/humble/setup.bash
cd single_robot
colcon build --symlink-install
source install/setup.bash

cd ../multi_robot
./build.sh
```

单机器人工作空间可按 `single_robot/README.md` 中的 launch 命令运行。大作业从 `multi_robot/` 目录执行 `./run.sh`；完整说明见各目录 README。两个项目分别加载自己的环境，不要同时 source 两个工作空间。

本仓库收录源码、配置、构建脚本和 Markdown 文档。ROS 构建目录、仿真日志、地图、PBStream、运行结果和本机核查证据未上传；运行验收须在目标环境重新执行。
