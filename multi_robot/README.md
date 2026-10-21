# TurtleBot 三机器人协同建图与队形切换

课程大作业的独立 ROS 2 Humble 工程。场地 12 m × 12 m，三台 TurtleBot4 Lite，使用各自激光雷达和里程计建图。人工分区后自主选择前沿探索，实时融合为 `/map`；可以中断探索，集合为三角形、横排或纵队，再返回各自区域继续探索。

采用已知起点标定、独立 Cartographer SLAM、占据栅格融合和集中任务调度。**没有将场地真值地图输入 SLAM 或控制，也没有实现多机器人联合位姿图优化。**

## 最快开始

只需一个启动终端：

```bash
cd multi_robot
./run.sh
```

浏览器打开 **http://127.0.0.1:8765**。等待“待命”和三车就绪后，依次操作：

1. 可选：展开“开始前调整分区”，填写每车 `xmin ymin xmax ymax`，点“应用区域”。
2. 点“开始 / 恢复探索”。机器人在各自区域里自主选择前沿，界面显示轨迹和各车探索里程。
3. 中央区域变成已观测空闲后，选择队形，点“切换至指定队形”。默认中心 `(0,0)`、朝向 `0 rad`、间距 `1.2 m`。
4. 等待“队形就绪”，查看独立位置 / 朝向误差；可继续切换其他队形。
5. 点“开始 / 恢复探索”，系统先让车返回各自区域，再恢复区域约束和探索。
6. 点“保存地图与运行结果”。
7. 结束时在启动终端按 **Ctrl+C**；关闭网页只关闭操作界面。

首次使用如果尚未构建，`run.sh` 会自动运行 `build.sh`。只依赖 `/opt/ros/humble` 和本工程，不需要 source 旧 `turtlebot4_ws`。

## 命令行操作

网页足够完成正常演示。需要命令行时，再开一个终端：

```bash
cd multi_robot
source ./env.sh
ros2 run fleet_control fleetctl status
ros2 run fleet_control fleetctl explore
ros2 run fleet_control fleetctl pause
ros2 run fleet_control fleetctl formation triangle --x 0 --y 0 --yaw 0 --spacing 1.2
ros2 run fleet_control fleetctl formation row --x 0 --y 0 --yaw 0 --spacing 1.2
ros2 run fleet_control fleetctl formation column --x 0 --y 0 --yaw 0 --spacing 1.2
ros2 run fleet_control fleetctl save --output results/my_map
ros2 run fleet_control fleetctl estop
ros2 run fleet_control fleetctl clear-estop
```

解除急停不会自动恢复运动，必须重新发任务。坐标均为全局 `map` 坐标，单位米；朝向为弧度，从正 X 轴向正 Y 轴逆时针为正。横排沿队形局部 Y 轴，纵队沿局部 X 轴；三角形前顶点朝局部正 X 方向。

开始前分区示例：

```bash
ros2 run fleet_control fleetctl regions -- -6 -6 -2 6 -2 -6 2 6 2 -6 6 6
```

三块区域须覆盖整个场地、不重叠、包含各自起点并留出车体余量。第一次开始探索后分区锁定，要重划分请重启。自定义配置可复制 `src/fleet_bringup/config/fleet.yaml` 后运行 `./run.sh --config /完整路径/fleet.yaml`。

## 启动模式

```bash
./run.sh                         # Gazebo GUI + 后台 + 网页
./run.sh --headless              # 无 Gazebo 窗口，网页仍可用
./run.sh --headless --record-truth # 额外采集独立验收真值，不送入控制节点
```

Gazebo 仅用于查看物理运动；操作任务请使用网页或 `fleetctl`。本工程没有把 Gazebo 自带 Teleop 插件接入三车控制链。正常验收过程中不要移动 / 增删障碍物，保存的地图也不包含 Gazebo 场景编辑结果。

工程默认隔离 ROS 域 91、Gazebo 分区 `tb4_course_fleet`。旧教程仿真可保持独立运行，但会占用额外资源。不能在同一域重复启动本工程；脚本会检查锁。环境只在当前 shell 生效，不修改 `~/.bashrc`。

## 文件与模块

| 路径 | 用途 |
|---|---|
| `src/fleet_interfaces` | 区域、队形和任务自定义服务 |
| `src/fleet_bringup` | 官方 TurtleBot 模型裁剪、场地生成、ROS2 控制器、SLAM / Nav2 启动 |
| `src/fleet_mapping` | 其他车回波过滤、初始地图对齐、共享栅格融合、地图 / PBStream 保存 |
| `src/fleet_control` | 自主前沿探索、队形位置和顺序规划、状态机、安全门控、网页、CLI、独立评估 |
| `scripts` | 启动管理、底座实测、冷启动检查、完整演示、地图重载检查 |
| `tests` | 坐标、地图编码、融合、区域约束、路径和队形算法测试 |
| `runs/<时间>` | 本次启动资源、配置、日志、轨迹及保存结果 |
| `docs` | 算法说明、验收结果和问题排查 |

运行时生成的 `runs/`、`results/`、日志和地图均由 `.gitignore` 排除；启动后可在本机运行目录查看。启用 `--record-truth` 后会生成仅供离线验收器读取的 `truth.jsonl`。

## 保存结果与检查

保存目录包含：

- `shared_map.pgm / shared_map.yaml`：共享地图。
- `robot1.pbstream`、`robot2.pbstream`、`robot3.pbstream`：三套 Cartographer 状态。
- `alignment.yaml`：局部地图到全局坐标的固定变换。
- `fleet.yaml`、`regions.yaml`、`snapshot.json`：配置及任务快照。

PGM 中黑色 `0` 为障碍，白色 `254` 为空闲，灰色 `205` 为未知。YAML 使用 `free_thresh: 0.196`，避免未知像素重载后误变为空闲。

用真实 Nav2 map_server 检查保存 / 加载逐格一致：

```bash
source ./env.sh
python3 scripts/map_roundtrip.py /完整路径/shared_map.yaml
```

PBStream 保存的是 SLAM 内部状态；PGM/YAML 是二维导航地图；它们都不是 Gazebo 场景文件。当前一键启动会从固定起点重新开始建图，不会自动恢复上一次任务。

## 开发与验收

```bash
./build.sh
./scripts/test.sh
```

底座独立测试（会移动机器人，必须在 base-only 模式）：

```bash
./run.sh --headless --base-only
# 另开一个终端
source ./env.sh
./scripts/build_tools.sh
python3 scripts/base_smoke.py
```

完整演示验收：

```bash
./run.sh --headless --record-truth
# 另开一个终端
source ./env.sh
python3 scripts/demo.py
```

完整验收包括分区、三车自主探索、覆盖率与里程检查、保存、三种队形、恢复探索。输出 `runs/<本次>/acceptance.json`。覆盖率由独立评估器与场地真值对比；网页的“已观测栅格比例”不是该覆盖率。

运行状态与已通过 / 未通过的实测项目以 [验收记录](docs/验收记录.md) 为准。

不想分开启动和执行验收时，可用真正的一键演示：

```bash
cd multi_robot
./demo.sh --headless
# 如需同时验证急停、雷达中断和定位中断：
./demo.sh --headless --with-faults
```

该命令启动自己的后台，完成演示、地图重载检查并保存结果，结束后清理后台。请先关闭本工程原有启动实例；它不会接管或结束其他会话。需要看 Gazebo 窗口时去掉 `--headless`。

本机系统 tf2 0.25.23 存在已复现的导航线程死锁，本项目使用本地兼容库修复，详见 [排查与修复](docs/tf2死锁排查与修复.md)。它不覆盖系统 ROS 库。
