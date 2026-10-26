# TurtleBot 三机器人协同建图与队形切换

课程大作业的独立 ROS 2 Humble 工程。场地 15 m × 15 m，三台 TurtleBot4 Lite，使用各自激光雷达和里程计建图。每轮根据种子生成随机障碍物；人工分区后自主选择前沿探索，受分区阻挡时自动跨区补盲，实时融合为 `/map`；可以中断探索，集合为三角形、横排或纵队。

现已结合单机器人课程的航点导航、状态监测与 Smac2D 规划内容，增加三车分区巡逻、标准 ROS 诊断及网页诊断面板，并修复新版浏览器中“开始”按钮的函数名冲突。旧固定场景的核查见 [优化与回归报告](docs/优化与回归报告-20261026.md)。

采用已知起点标定、独立 Cartographer SLAM、占据栅格融合和集中任务调度。**没有将场地真值地图输入 SLAM 或控制，也没有实现多机器人联合位姿图优化。**

## 最快开始

本机既有源码工作空间为 `/home/bwz/turtlebot_fleet_ws`；GitHub 克隆后进入仓库的 `multi_robot/`。下列代码以克隆目录为例，只需一个启动终端：

```bash
cd multi_robot
./run.sh
```

浏览器打开 **http://127.0.0.1:8765**。等待“待命”和三车就绪后，依次操作：

1. 可选：展开“开始前调整分区”，填写每车 `xmin ymin xmax ymax`，点“应用区域”。
2. 点“开始 / 恢复探索”。机器人在各自区域里自主选择前沿，界面显示轨迹和各车探索里程。
3. 中央区域变成已观测空闲后，选择队形，点“切换至指定队形”。默认中心 `(0,0)`、朝向 `0 rad`、间距 `1.2 m`。
4. 等待“队形就绪”，查看独立位置 / 朝向误差；可继续切换其他队形。
5. 点“开始 / 恢复探索”，未解除分区时先返回原区域；已解除分区时直接继续跨区探索。
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
ros2 run fleet_control fleetctl regions -- -7.5 -7.5 -2.5 7.5 -2.5 -7.5 2.5 7.5 2.5 -7.5 7.5 7.5
```

三块区域须覆盖整个场地、不重叠、包含各自起点并留出车体余量。首次探索、巡逻或队形任务后分区锁定，要重划分请重启。自定义配置可复制 `src/fleet_bringup/config/fleet.yaml` 后运行 `./run.sh --config /完整路径/fleet.yaml`。起点不能与场景障碍或外墙相交。

## 随机场景预览版（2026-10-26）

本版实测、迭代问题与功能边界见 [随机场景预览报告](docs/随机场景预览报告-20261026.md)。

```bash
./run.sh                       # 每次使用新种子，终端和网页显示本轮种子
./run.sh --seed 42             # 复现场景；0 也是有效种子
./run.sh --headless --seed 2026 # 只显示网页，不开 Gazebo 窗口
```

场地坐标为 `[-7.5, -7.5, 7.5, 7.5]`，默认起点为 `(-5,-5)`、`(0,-5)`、`(5,-5)`。初始任务区为三个宽 5 m 的竖向矩形。配置 `scene.seed: null` 表示每轮随机，整数表示固定种子；命令行 `--seed` 优先。相同生成器版本（`rectangles-v1`）、种子及起点/障碍数量配置产生相同几何，机器人运动和 SLAM 结果仍会受运行时调度影响。

默认生成 10 个矩形障碍，短边至少 1 m，障碍间和障碍与墙之间至少 1.2 m；预留各起点周围及中央集合空间。生成后在 10 cm 网格上检查离障碍至少 0.5 m 的机器人中心通行空间：必须单连通、包含全部起点且占场地至少 50%。这保证该离散检查下存在通路，不能替代实际导航验证。`scene.obstacle_count` 支持 4–16；过密或不适合的自定义起点配置会明确报错，不会悄悄使用不连通场景。

探索先遵守三矩形。系统检查本区前沿的可达性；发现前沿在分区内不连通、但通过已知空闲区绕出边界可以到达，且这种情况持续至少 8 s 时，先停车、取消原目标，再解除三车 Nav2 分区过滤器及对应边界保护，继续协同探索。网页显示解除原因，`events.jsonl` 记录 `exploration_regions_released`。跨区后按机器人轮流执行目标，同一时间只让一台车移动，规划时与其他车中心至少留 0.8 m；初始分区阶段仍并行探索。本轮后续探索保留跨区权限；暂停、队形切换不会清除它。障碍避让、雷达/定位监测、急停和任务心跳保护继续生效。航点巡逻仍要求路线位于人工分区；若当前车在分区外，巡逻请求会被拒绝，可重新开始一轮。

机器人只读取传感器和实时地图，自动跨区判断不读取生成器障碍几何。`runs/<本轮>/assets/scene.json` 保存种子、生成器版本、几何哈希和连通性检查；`arena.sdf` 是物理场景，`ground_truth.yaml` 只用于离线评估，三者与机器人建出的地图分开保存。

离线查看多个场景（图中障碍是真值预览，不是 SLAM 结果）：

```bash
source ./env.sh
python3 scripts/preview_scene.py --seeds 42 2026 12345 --output docs/random-scenes-preview.png
```

![随机场景示例](docs/random-scenes-preview-20261026.png)

## 启动模式

```bash
./run.sh                         # Gazebo GUI + 后台 + 网页
./run.sh --headless              # 无 Gazebo 窗口，网页仍可用
./run.sh --headless --record-truth # 额外采集独立验收真值，不送入控制节点
```

Gazebo 仅用于查看物理运动；操作任务请使用网页或 `fleetctl`。本工程没有把 Gazebo 自带 Teleop 插件接入三车控制链。正常验收过程中不要移动 / 增删障碍物，保存的地图也不包含 Gazebo 场景编辑结果。

工程默认隔离 ROS 域 91、Gazebo 分区 `tb4_course_fleet`。旧教程仿真可保持独立运行，但会占用额外资源。脚本同时检查 ROS 域锁、Gazebo 分区锁和网页端口；多开实例须分别设置不同的 `FLEET_DOMAIN_ID`、`FLEET_IGN_PARTITION` 和 `dashboard_port`。环境只在当前 shell 生效，不修改 `~/.bashrc`。

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

最近运行目录记录在 `runs/latest.txt`。启动日志在 `launch.log`，任务事件在 `events.jsonl`，状态采样在 `telemetry.jsonl`，资源记录在 `resources.jsonl`。启用 `--record-truth` 后有 `truth.jsonl`，只有独立验收器读取它。这些运行产物由 `.gitignore` 排除，不随源码上传。

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

运行状态与已通过 / 未通过的实测项目以 [验收记录](docs/验收记录.md) 为准。历史复验见 [2026-10-21 核查报告](docs/代码核查与仿真报告-20261021.md)。

不想分开启动和执行验收时，可用真正的一键演示：

```bash
cd multi_robot
./demo.sh --headless
# 如需同时验证急停、雷达中断和定位中断：
./demo.sh --headless --with-faults
```

该命令启动自己的后台，完成演示、地图重载检查并保存结果，结束后清理后台。请先关闭本工程原有启动实例；它不会接管或结束其他会话。需要看 Gazebo 窗口时去掉 `--headless`。

本机系统 tf2 0.25.23 存在已复现的导航线程死锁，本项目使用本地兼容库修复，详见 [排查与修复](docs/tf2死锁排查与修复.md)。它不覆盖系统 ROS 库。

## 新增：分区航点巡逻

等待“三车就绪”后，可直接执行网页默认巡逻示例，也可先探索，再在已观测空闲区配置路线。每个航点是全局 `map` 坐标 `[x, y, yaw]`；可指定一台或多台车。网页提供循环次数、每点停留、单点导航超时与每车完成进度。

命令行读取 [巡逻示例 YAML](src/fleet_control/config/patrol_example.yaml)：

```bash
source ./env.sh
ros2 run fleet_control fleetctl patrol --file src/fleet_control/config/patrol_example.yaml
```

```yaml
cycles: 2
dwell: 0.5
goal_timeout: 120
routes:
  robot1: [[-5.6, -5.0, 0.0], [-5.0, -5.0, 0.0]]
  robot2: [[-0.6, -5.0, 0.0], [0.0, -5.0, 0.0]]
  robot3: [[5.6, -5.0, 0.0], [5.0, -5.0, 0.0]]
```

每车 1–20 个航点、循环 1–20 次、停留 0–30 s、单点超时 5–240 s。机器人和所有航点必须位于所属分区的安全空闲区；系统检查每段路径及循环接回第一点的路径，再交给 Nav2 导航。未知区、障碍、越界和不连通路线不能执行。未解除分区时，队形切换后可恢复探索使车辆返回；若已自动跨区且车辆在区外，需重启后再执行分区巡逻。

“暂停停车”和急停都会取消巡逻；重新提交从第一点开始。任一车巡逻导航失败或超时，全车停车并显示原因。解除急停或故障后仍须重新发任务。重复开始探索不会重置已有探索目标。

## 新增：规划器选择、诊断和实例校验

```bash
./run.sh --planner smac2d
# 默认仍使用 Navfn A*：
./run.sh --planner navfn
```

也可在配置中设置 `planner: navfn` 或 `planner: smac2d`，命令行优先。两种实现均使用 BT 中的 `GridBased` 标识，探索、巡逻和队形都使用所选实现。Smac2D 需要 ROS 包 `nav2_smac_planner`；本次没有接入 D*、RRT* 或机械臂控制。

网页“探索里程”只统计探索期间的里程计平移，“总行驶”还包括巡逻、集合和返回。诊断面板显示原始雷达、里程计、定位的延迟与最近有效回波距离。标准话题为 `/diagnostics`，本项目状态名是 `robot1/fleet_health` 至 `robot3/fleet_health`；Nav2 也会在该话题发布自己的诊断。

启动前检查配置和网页端口。网页和验收脚本携带运行实例标识，避免旧页面向另一轮仿真发任务，并支持中文运行路径。验收脚本从本次配置读取 `dashboard_port`。`demo.py`、`browser_acceptance.py`、`planner_acceptance.py`、`safety_acceptance.py` 等支持 `--run-dir /完整路径`，可显式指定运行目录。

新增 `fleet_control/patrol.py` 检查巡逻参数与路径，`health.py` 检查传感数据；网页 HTML、脚本、样式分开安装，按钮统一使用事件监听器，操作反馈显示在任务状态区。

## 新增功能的验收

对已经启动的实例检查所选规划器、三台车的真实规划请求和标准诊断：

```bash
source ./env.sh
python3 scripts/planner_acceptance.py
```

真实浏览器按钮与两圈巡逻验收需要可选测试依赖 Playwright，不影响正常运行。它会操作仿真，应使用独立测试实例：

```bash
python3 -m pip install --target /tmp/fleet-browser-deps -r requirements-browser.txt
PYTHONPATH=/tmp/fleet-browser-deps python3 -m playwright install chromium --only-shell
PYTHONPATH=/tmp/fleet-browser-deps python3 scripts/browser_acceptance.py
# 已通过 --record-truth 采集真值的实例，还能验证巡逻超时停车：
source ./env.sh
python3 scripts/patrol_failure_acceptance.py
```

结果写入运行目录中的 JSON 文件；浏览器验收另保存 `dashboard-patrol.png`。单元测试不需要 Playwright。修改源码后先执行 `./build.sh`；新环境应按 `package.xml` 安装 ROS 依赖。新增项目的测试结果及边界见本次核查报告。

Ctrl+C 退出时，监督脚本先停车并关闭本次进程组的 Gazebo 数据源，保留桥接节点直到数据源结束并等待最后的回调排空，再由 ROS launch 关闭其余节点。`shutdown_preflight.json` 记录各阶段，`cleanup.json` 记录残留及异常子进程；异常退出会返回失败。重复冷启动与运动退出检查可运行：

```bash
source ./env.sh
python3 scripts/repeat_cold_starts.py --count 3 --planner smac2d --node-fault --move-before-stop
```
