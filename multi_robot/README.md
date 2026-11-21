# TurtleBot 三机器人协同建图与队形切换

默认在同一主机和同一 Gazebo 世界中启动三辆机器人，使用种子 24、出生布局 0。每车自行运行 SLAM、Nav2、任务代理和安全节点。机器人通过对等消息配准地图、协商任务与运动授权。网页负责显示与下发操作，终端 CLI 可直接与各车通信。

## 安装依赖与构建

环境为 Ubuntu 22.04 x86_64、ROS 2 Humble、Ignition Gazebo Fortress。以下命令从仓库根目录执行。首次使用 rosdep 时先执行一次 `sudo rosdep init`。

```bash
cd multi_robot
sudo apt install python3-rosdep python3-colcon-common-extensions python3-pytest \
  build-essential cmake pkg-config libignition-gazebo6-dev \
  libignition-transport11-dev libignition-msgs8-dev
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -y
./build.sh
```

构建入口先编译项目内的 tf2 兼容库，再构建五个 ROS 2 包。`source ./env.sh` 将 ROS 域、Gazebo 分区及项目库配置到当前终端。默认 ROS 域为 91，Gazebo 分区为 `tb4_course_fleet`，DDS 和 Gazebo Transport 使用本机回环接口。

## 运行与演示

```bash
./run.sh
```

打开终端显示的网页地址（默认 `http://127.0.0.1:8765`）。等待三个本地链路就绪，点击“开始 / 恢复探索”。机器人利用激光观测建立共同坐标，然后分区探索。探索完成后可设置巡逻目标，切换三角形、横排或纵队，并保存地图。Gazebo 三维窗口可通过 `./run.sh --gui` 打开。

另开终端进入 `multi_robot`。以下命令可配合网页或在 `--no-dashboard` 启动时使用。

```bash
source ./env.sh
ros2 run fleet_control fleetctl status
ros2 run fleet_control fleetctl explore
ros2 run fleet_control fleetctl pause
ros2 run fleet_control fleetctl save --output results/my_peer_map
```

结束手动演示后运行自动演示，脚本会按顺序完成探索、暂停恢复、巡逻、三种队形、急停、地图重载和退出。添加 `--with-ui-checks` 可检查实际浏览器以及监控关闭后的 CLI，依赖安装见[操作手册](docs/操作指南.md)。

```bash
./demo.sh
```

具备 OpenGL EGL 驱动的 OGRE2 环境可运行 `FLEET_EGL_HEADLESS=1 ./demo.sh --headless`，在无窗口时使用 EGL 渲染。

保留显式的集中式兼容入口 `./run.sh --architecture centralized`。其他场景可用 `--seed` 和 `--spawn-layout` 指定。

## 验证与源码

| 入口 | 用途 |
| --- | --- |
| [操作手册](docs/操作指南.md) | 网页、CLI、地图、试验参数、故障验收及排障 |
| [算法与工程结构](docs/算法与工程结构.md) | 私有 TF、配准、对等租约、独立安全、监控与工程分包 |
| [验证与范围](docs/验证与范围.md) | 提交时离线回归、2026-11-18 默认场景实测及专项门禁状态 |
| [src/](src/) | `fleet_interfaces`、`fleet_bridge`、`fleet_bringup`、`fleet_mapping`、`fleet_control` |
| [scripts/](scripts/) · [tests/](tests/) | 运行监督、验收评估脚本及回归样本 |

运行完成后可执行 `./scripts/test.sh` 做离线回归及原生桥接／TF 检查。`runs/`、`results/`、`build/`、`install/`、`vendor/runtime/` 和 `log/` 为本地生成目录。
