# 单机器人 TurtleBot4 教学工程

在仓库世界中运行 TurtleBot4 Lite，使用 Cartographer 建图，并将保存的地图用于 AMCL 定位和 Nav2 导航。工程还提供键盘、手柄、状态诊断、碰撞保护及急停入口。

## 安装依赖与构建

使用 Ubuntu 22.04 x86_64、ROS 2 Humble 和可用的 ROS 2 apt 源。以下命令从仓库根目录开始，在新 Bash 终端执行。首次使用 rosdep 时先执行一次 `sudo rosdep init`，已初始化的系统直接更新。

```bash
cd single_robot
sudo apt install python3-rosdep python3-colcon-common-extensions \
  ros-humble-turtlebot4-simulator ros-humble-rmw-cyclonedds-cpp
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y
colcon build --base-paths src --symlink-install
source ./env.sh
```

`env.sh` 为 Bash 的 `source` 入口。它加载本工作空间的构建结果，配置 ROS 域 74、CycloneDDS 及 Gazebo 分区 `tb4_single_demo`。启动、控制与诊断终端均从 `single_robot` 目录执行 `source ./env.sh`。

## 快速开始

终端 A 启动仿真和 RViz。

```bash
source ./env.sh
ros2 launch turtlebot4_tutorial tutorial_simulation.launch.py \
  model:=lite gui:=true rviz:=true
```

终端 B 加载同一环境，检查雷达后运行键盘遥控。通过 W/A/S/D 移动、空格停车、Q 退出。

```bash
source ./env.sh
ros2 topic echo /scan --once --qos-reliability best_effort
ros2 run turtlebot4_tutorial keyboard_control
```

结束键盘与仿真后，可直接用仓库中的[示例地图](maps/warehouse/acceptance_map.yaml)演示导航。该地图由仓库场景的局部建图生成，在未插入附加隔断的仓库世界使用。从 `single_robot` 目录运行，地图绝对路径使用英文、数字或下划线。

```bash
ros2 launch turtlebot4_tutorial tutorial_navigation.launch.py \
  "map:=$PWD/maps/warehouse/acceptance_map.yaml"
```

在终端 B 等到 `/bt_navigator` 为 `active [3]` 后发送单目标。自建地图、四点巡航、电量诊断与安全模式见[完整操作手册](docs/操作指南.md)。

```bash
ros2 lifecycle get /bt_navigator
ros2 run turtlebot4_tutorial nav_action_demo 0.5 0.0 0.0 \
  --timeout 120 --ros-args -p use_sim_time:=true
```

## 工程结构

| 目录 | 内容 |
| --- | --- |
| [src/turtlebot4_tutorial](src/turtlebot4_tutorial/) | 仿真、建图和导航 launch，Nav2／RViz 参数，地图、控制、诊断节点与测试 |
| [src/turtlebot4_cartographer](src/turtlebot4_cartographer/) | Cartographer 启动文件及 TurtleBot4 2D 建图配置 |
| [maps/warehouse](maps/warehouse/) | 仓库仿真示例地图 YAML 和 PGM |
| [scripts](scripts/) | 隔离的 YOLO 和 Panda / MoveIt 可选演示环境 |

## 测试与验证

在仿真退出后执行。

```bash
source ./env.sh
colcon test --packages-select turtlebot4_tutorial turtlebot4_cartographer \
  --return-code-on-test-failure
colcon test-result --all --verbose
```

[验证与适用范围](docs/验证与范围.md)记录本次构建、历史仿真实测、地图信息和已知限制。旧版课程延伸结果见[课程延伸实验验证报告](docs/课程延伸实验验证报告.md)。
