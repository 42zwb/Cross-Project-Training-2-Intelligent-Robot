# ROS 2 TurtleBot4 课程项目

本仓库把单机器人教学工程与三机器人协同工程放在两个独立的 colcon 工作空间中。两套工程均在 Ubuntu 22.04 x86_64、ROS 2 Humble、Ignition Gazebo Fortress 下核查，使用不同的 ROS 域和 Gazebo 分区。

| 工程 | 主要内容 | 文档 |
| --- | --- | --- |
| [单机器人](single_robot/) | TurtleBot4 Lite 仓库仿真、遥控、Cartographer 建图、地图保存、AMCL / Nav2、自建地图巡航、速度仲裁与急停 | [快速开始](single_robot/README.md) · [操作手册](single_robot/docs/操作指南.md) · [验证结果](single_robot/docs/验证与范围.md) |
| [三机器人](multi_robot/) | 三车在同一世界中自治建图与导航，通过对等消息完成地图配准、分区探索、巡逻和静态队形切换 | [快速开始](multi_robot/README.md) · [操作手册](multi_robot/docs/操作指南.md) · [工程架构](multi_robot/docs/算法与工程结构.md) · [验证结果](multi_robot/docs/验证与范围.md) |

## 开始使用

```bash
git clone https://github.com/42zwb/Cross-Project-Training-2-Intelligent-Robot.git
cd Cross-Project-Training-2-Intelligent-Robot
```

在各自子目录构建工程。启动及操作终端分别加载对应的 `env.sh`。本机一次运行一套 TurtleBot 仿真，切换工程前在原启动终端按 Ctrl+C，并等待进程退出。仿真入口会按项目归属检查旧进程。

| 工作空间 | 构建目录 | 默认 ROS 域 | 默认 Gazebo 分区 |
| --- | --- | ---: | --- |
| `single_robot` | `single_robot/{build,install,log}/` | 74 | `tb4_single_demo` |
| `multi_robot` | `multi_robot/{build,install,log}/` | 91 | `tb4_course_fleet` |

## 版本与结果

- 本次提交从独立检出的源码重建单机 2 个包和多机 5 个包，运行离线测试和命令入口检查。结果见上表的验证文档。
- 单机运行记录来自 2026-11-04 的仓库仿真，自建地图建图、重载、导航和急停均有实测。多机运行记录来自 2026-11-18 的默认去中心化场景，探索、巡逻、三种队形、地图重载、浏览器和退出均有实测。
- 仓库包含可用的单机示例地图、多机回归测试数据和发布核查工具。运行日志、构建产物及本机环境通过各自的构建和运行命令生成。2026-10 的旧多机边界墙和调度记录可从历史提交查阅。[历史版本](https://github.com/42zwb/Cross-Project-Training-2-Intelligent-Robot/tree/59c807858edf65eaa99537da77635f14c8090124)
