# Cross Project Training 2 — Intelligent Robot

本仓库并列收录两套相互独立的 ROS 2 Humble 课程工程：

- [`single_robot/`](single_robot/)：TurtleBot4 单机器人教程工程，包含 `turtlebot4_tutorial` 与 `turtlebot4_cartographer` 两个 ROS 包。
- [`multi_robot/`](multi_robot/)：三台 TurtleBot4 协同建图与队形切换大作业，包含 SLAM、地图融合、分区航点巡逻、Navfn/Smac2D、队形调度、状态诊断、安全门控、网页控制与离线评估。

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

2026-10-26 优化与新鲜仿真结果见 [多机器人优化与回归报告](multi_robot/docs/优化与回归报告-20261026.md)。

## 15×15 米随机场景预览

当前预览分支为 `preview/seeded-scenes-15m`，增加每轮随机种子、可复现障碍布局、分区受阻时自动跨区及轮流通行调度。使用 `./run.sh --seed 24` 查看本轮场景。最初预览见 [随机场景预览报告](multi_robot/docs/随机场景预览报告-20261026.md)。

后续核查修复了桥接循环引用及退出生命周期、地图重采样孔洞、启动状态竞争和辅助进程异常清理。该轮 80 项多机器人测试、18 项单机器人测试、原生对照、Memcheck、三轮冷启动及真实仿真结果见 [桥接退出与代码修复报告](multi_robot/docs/桥接退出与代码修复报告-20261026.md)。

进一步修复起点附近补盲遗漏与墙后未知区造成的绕行，增加封闭实心障碍内部推断、观测与推断分项，以及 Nav2 当前规划虚线。100 项测试与实际返回补盲结果见 [探索补盲与障碍补全修复](multi_robot/docs/探索补盲与障碍补全修复-20261026.md)。
