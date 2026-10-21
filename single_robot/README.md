# ROS2 TurtleBot4 地面机器人教程：环境、实现与操作指南

本文记录《仿真环境下基于 ROS2 的地面机器人教程1.docx》在本机上的实现。最新修复与验收日期为 2026-10-21，详见 [修复记录](修复记录.md)。下表的历史验收与本轮复测分别标明。

## 1. 当前环境

| 项目 | 当前配置 |
| --- | --- |
| 操作系统 | Ubuntu 22.04.5 LTS，x86_64 |
| ROS2 | Humble Hawksbill |
| 仿真器 | Ignition Gazebo 6.18.0（Fortress） |
| 机器人 | TurtleBot4 Lite / Standard |
| Python | 3.10.12 |
| DDS | CycloneDDS（新终端自动启用） |
| 工作空间 | `single_robot` |
| 地图目录 | `~/maps` |

已安装 ROS2 Desktop、TurtleBot4 Desktop/Simulator、Navigation2、Nav2 Simple Commander、Cartographer、SLAM Toolbox、RViz2、CycloneDDS、键盘遥控、`joy`、`joy-linux`、`teleop-twist-joy`、colcon 和 ROS 开发工具。

ROS2 apt 源使用清华镜像，签名仍使用 Open Robotics 官方密钥：

```text
deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu jammy main
```

在仓库根目录的新终端中加载 ROS、工作空间 overlay 和 CycloneDDS：

```bash
source /opt/ros/humble/setup.bash
source single_robot/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## 2. 已实现的工程

教程代码已经整理成两个标准 ROS2 包，而不是散放在 `src` 根目录：

```text
single_robot/src/
├── turtlebot4_tutorial/                 # ament_python 功能包
│   ├── config/nav2_params.yaml          # 完整 Nav2 参数
│   ├── launch/
│   │   ├── tutorial_simulation.launch.py
│   │   ├── tutorial_mapping.launch.py
│   │   ├── tutorial_navigation.launch.py
│   │   ├── tutorial_spawn.launch.py
│   │   └── tutorial_rviz.launch.py       # 单独重开可视化
│   ├── worlds/my_obstacles.sdf          # 6 m × 6 m 自定义障碍场景
│   └── turtlebot4_tutorial/
│       ├── keyboard_control.py
│       ├── joystick_control.py
│       ├── collision_monitor.py
│       ├── mapping_patrol.py
│       ├── map_loader.py
│       ├── map_saver.py
│       ├── initial_pose_publisher.py
│       ├── nav_commander_demo.py
│       ├── nav_action_demo.py
│       ├── robot_status_publisher.py
│       ├── multi_robot_patrol.py
│       └── robot_description_filter.py
└── turtlebot4_cartographer/             # ament_cmake 功能包
    ├── config/turtlebot4_2d.lua
    ├── launch/cartographer.launch.py
    ├── CMakeLists.txt
    └── package.xml
```

重新构建工程：

```bash
cd single_robot
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select turtlebot4_tutorial turtlebot4_cartographer
colcon test-result --all --verbose
```

## 3. 快速开始

每个新终端先执行：

```bash
source /opt/ros/humble/setup.bash
cd single_robot
source install/setup.bash
```

### 3.1 只启动仿真

带 Gazebo 图形界面：

```bash
ros2 launch turtlebot4_tutorial tutorial_simulation.launch.py model:=lite
```

仿真、建图、导航三个入口现在统一默认不生成充电座，停靠练习可加
`spawn_dock:=true`。Gazebo GUI 和服务器分为两个进程；关闭 GUI 后服务器仍可
继续工作，结束整套任务应在启动终端按 `Ctrl+C`。

图形窗口默认使用软件渲染，服务器的传感器渲染独立运行；可用
`software_rendering:=false` 切回图形窗口的硬件渲染。完全关闭图形窗口可设置
`gui:=false rviz:=false`。

本机验收所用的无界面模式：

```bash
ros2 launch turtlebot4_tutorial tutorial_simulation.launch.py \
  model:=lite gz_args:='warehouse.sdf -s -r -v 1'
```

这个启动文件包含针对本机 Ignition Fortress 的雷达修复。请优先使用它；直接运行官方启动文件时，本机 `/scan` 可能全部错误地等于 `0.164 m`。

另开终端检查：

```bash
ros2 topic list
ros2 topic echo --once /scan
ros2 topic echo --once /odom
ros2 topic hz /scan
```

直接让机器人前进 3 秒：

```bash
ros2 topic pub -r 10 --times 30 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.15}, angular: {z: 0.0}}'
```

### 3.2 键盘和手柄控制

官方键盘遥控：

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

键盘焦点必须停留在运行该命令的终端内，并切换到英文输入法。该程序使用
`i`、`,`、`j`、`l` 和 `k`，无需按回车；单击一次只会发送一次速度消息，
因此可以长按或连续按 `i` 来明显观察前进。若终端没有打印完整按键说明，
说明它没有在交互式终端中正常启动。

快速确认键盘消息是否发出：先在另一个终端运行下面的命令，再回到键盘终端
按一次 `i`：

```bash
ros2 topic echo --once /cmd_vel
```

正常时应看到 `linear.x: 0.5`。按 `k` 后应看到所有速度恢复为 `0.0`。

教程自定义键盘节点（`W/S/A/D`、空格停止、`Q` 退出）：

```bash
ros2 run turtlebot4_tutorial keyboard_control
```

自定义键盘和手柄节点都有 `0.6 s` 输入看门狗：控制程序失去输入后会自动发送
停车指令。键盘节点需连续按住或重复按 `W/S/A/D`；可用
`--ros-args -p key_timeout:=1.0` 调整保持时间。按 `Q` 或 `Ctrl+C`
退出时会发布停车指令。

手柄控制：

```bash
# 终端 1
ros2 run joy joy_node

# 终端 2
ros2 run turtlebot4_tutorial joystick_control
```

可通过 ROS 参数修改轴、使能键和速度，例如：

```bash
ros2 run turtlebot4_tutorial joystick_control --ros-args \
  -p linear_axis:=1 -p angular_axis:=0 -p enable_button:=4 \
  -p max_linear_speed:=0.35 -p max_angular_speed:=0.8
```

### 3.3 带激光保护的遥控

碰撞监控读取 `/cmd_vel_raw`，根据 `/scan` 限速或停车，再输出 `/cmd_vel`：

```bash
# 终端 1
ros2 run turtlebot4_tutorial collision_monitor

# 终端 2
ros2 run turtlebot4_tutorial keyboard_control --ros-args \
  -r cmd_vel:=cmd_vel_raw
```

默认停车距离为 `0.35 m`，减速距离为 `0.70 m`，雷达超时为 `1.0 s`，
可用 `-p stop_distance:=...`、`-p slow_distance:=...` 和
`-p scan_timeout:=...` 修改。没有雷达、雷达超时、前方扇区没有有效测距，
或参数非法时，监控会禁止前进，但仍保留原地转向和后退能力。

## 4. Cartographer 建图

一条命令启动仿真、Cartographer 和 RViz：

```bash
ros2 launch turtlebot4_tutorial tutorial_mapping.launch.py
```

无界面运行：

```bash
ros2 launch turtlebot4_tutorial tutorial_mapping.launch.py \
  rviz:=false model:=lite gz_args:='warehouse.sdf -s -r -v 1'
```

可选：动态插入教程中的自定义墙体和方块：

```bash
ros2 run ros_gz_sim create -world warehouse \
  -file single_robot/install/turtlebot4_tutorial/share/turtlebot4_tutorial/worlds/my_obstacles.sdf
```

使用键盘探索，或运行自动建图巡航：

```bash
ros2 run turtlebot4_tutorial mapping_patrol
```

巡航先按里程计原地转一周，再走四段约 `0.9 m` 的路线；运动完成由 `/odom`
的实际转角和位移判断，仿真较慢时耗时会增加。若前方受阻或雷达数据失效超过
`3 s`，程序会停车并返回失败，不会把原地等待算作已走完路线。

在 RViz 中把 Fixed Frame 设为 `map`，添加 `Map`、`LaserScan`、`TF`、`Odometry` 和 `RobotModel`。

保存占据栅格地图：

```bash
mkdir -p ~/maps
ros2 run turtlebot4_tutorial map_saver ~/maps/my_map
```

请使用上面的工程保存入口。它固定使用 `free_thresh=0.196`，并在保存后验证
地图；原生保存器的默认阈值 `0.25` 会把本机 PGM 中灰度 205 的未知区域在
重新加载时当作空闲区域。现有三份旧地图的 YAML 已备份并修复；地图加载和
导航启动都会拒绝仍有这个问题的配置。

同时保存 Cartographer 状态，便于以后继续建图：

```bash
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
  "{filename: '~/maps/my_map.pbstream', include_unfinished_submaps: true}"
```

本次已经生成并保存：

```text
~/maps/tutorial_room.pgm
~/maps/tutorial_room.yaml
~/maps/tutorial_room.pbstream
~/maps/warehouse_verified.yaml
~/maps/warehouse_verified.pgm
~/maps/warehouse_verified.pbstream
```

`warehouse_verified` 是 2026-10-21 按默认完整方形路线重新生成的地图，
493×433 像素，0.05 m/像素。加载其原始状态继续建图：

```bash
ros2 launch turtlebot4_tutorial tutorial_mapping.launch.py \
  load_state_filename:=~/maps/warehouse_verified.pbstream \
  load_frozen_state:=false
```

恢复时应使用相同世界；本机验证的是机器人从原点附近重新生成的情况。
这会保留旧地图并创建新轨迹；`load_frozen_state:=true` 可冻结旧图。
已经有仿真运行时，需要再加 `start_sim:=false`。

单独加载地图并自动激活生命周期节点：

```bash
ros2 run turtlebot4_tutorial map_loader ~/maps/tutorial_room.yaml
```

## 5. Nav2 自主导航

默认命令会启动仓库仿真、官方仓库地图、AMCL、Nav2 和 RViz，并自动将仿真起点设置为 `(0, 0, 0)`：

导航启动默认 `spawn_dock:=false`，机器人从地面上直接出发。原先默认在
机器人前方 0.157 米生成充电座，向前导航可能顶住充电座而轮子空转，
使轮式里程计累计距离、导航报告成功，但 Gazebo 中车体实际没有移动。
单独仿真和建图入口也已统一为 `spawn_dock:=false`。对已经运行的仿真使用
`start_sim:=false` 时，导航启动不会移除现有充电座。
停靠实验可显式设置 `spawn_dock:=true`，先退离充电座，并规划绕开它的路径。
充电座的碰撞体低于雷达扫描平面，不能只依靠 `/scan` 判断车前是否可通行。

2026-10-14 实测：修复前，车体真实位置约为 `(0.005, -0.001)`，
轮式里程计却已累计到 `(0.860, 0.077)`；后退测试中车体与里程计均移动
约 0.094 米，说明向前的初始停靠碰撞导致空转。导航默认不生成充电座后，
重新导航到 `(1.0, 0.0, 0.0)`：车体真实位移 0.826 米，里程计位移
0.825 米，实际终点 `(0.825, 0.044)`，与目标距离 0.180 米。
当前到点位置容差为 0.25 米，因此此终点会判为成功。此次包测试为
13 项通过，完整运行数据保存在
`single_robot/log/navigation_undocked_result_20261014.json`。
判断是否真正移动时同时检查 `/sim_ground_truth_pose` 和 `/odom` 的位移，
不能只看导航成功提示或轮式里程计。

```bash
ros2 launch turtlebot4_tutorial tutorial_navigation.launch.py
```

无界面运行：

```bash
ros2 launch turtlebot4_tutorial tutorial_navigation.launch.py \
  rviz:=false model:=lite gz_args:='warehouse.sdf -s -r -v 1'
```

等待第一个终端打印 `AMCL 已确认初始位姿` 和 `AMCL 已就绪，启动 Nav2`，
再检查关键节点；启动时间取决于本机仿真速度：

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
```

它们应显示 `active [3]`。随后发送一个目标：

```bash
ros2 run turtlebot4_tutorial nav_action_demo 0.5 0.0 0.0
```

运行四点巡航示例：

```bash
ros2 run turtlebot4_tutorial nav_commander_demo --loops 1
```

也可以在 RViz 中使用 `2D Goal Pose` 点选目标。目标必须在地图自由区域内。

使用自己保存的地图：

```bash
ros2 launch turtlebot4_tutorial tutorial_navigation.launch.py \
  map:=~/maps/tutorial_room.yaml
```

若仿真机器人从非零位置生成，可同时设置生成位姿和 AMCL 初始位姿：

```bash
ros2 launch turtlebot4_tutorial tutorial_navigation.launch.py \
  x:=1.0 y:=0.5 yaw:=1.57
```

`auto_initial_pose:=true`（默认）会把同一组 `x/y/yaw` 自动发布给 AMCL。
启动文件会等 AMCL 确认定位后才启动 Nav2；45 秒仍未确认时会明确报错并停止，
避免导航节点在缺少 `map` 坐标系时继续接收目标。
若希望在 RViz 中手工使用 `2D Pose Estimate`，再设置
`auto_initial_pose:=false`。

也可用命令发布实际初始位姿：

```bash
ros2 run turtlebot4_tutorial initial_pose_publisher 1.0 0.5 1.57
```

`nav_commander_demo` 默认沿用当前 AMCL 定位，不会把已移动的机器人重置到原点。
仅在尚未设置初始位姿时，加 `--set-initial-pose`；非零起点可同时加
`--initial-x`、`--initial-y`、`--initial-yaw`。

原生动作示例和巡航示例都有目标超时，避免路径规划异常时永久等待：

```bash
ros2 run turtlebot4_tutorial nav_action_demo 0.5 0.0 0.0 --timeout 60
ros2 run turtlebot4_tutorial nav_commander_demo --loops 1 \
  --nav2-timeout 60 --goal-timeout 60
```

真实机器人或已单独启动仿真时，可加 `start_sim:=false`，并应根据真实位置设置初始位姿。

## 6. 状态、诊断和多机器人示例

发布机器人汇总状态：

```bash
ros2 run turtlebot4_tutorial robot_status_publisher
ros2 topic echo /tutorial/robot_status
```

状态中包含里程计位置、线速度、角速度、最近障碍距离、电池百分比和传感器超时信息。

多机器人巡航客户端：

```bash
ros2 run turtlebot4_tutorial multi_robot_patrol robot1 robot2
```

每个目标默认最多等待 180 秒，也可加 `--goal-timeout 60`。命名空间会在启动前
检查，重复、空白或不符合 ROS 命名规则的名称会直接报错。

启动单台带命名空间的仿真示例：

```bash
ros2 launch turtlebot4_tutorial tutorial_simulation.launch.py \
  namespace:=robot1 model:=lite
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/robot1/cmd_vel
```

双机器人仿真的完整启动顺序（每个终端先加载 ROS 与工作空间环境）：

```bash
# 终端 1：启动世界、robot1 及其定位与导航
ros2 launch turtlebot4_tutorial tutorial_navigation.launch.py \
  namespace:=robot1 rviz:=false model:=lite \
  gz_args:='warehouse.sdf -s -r -v 1'

# 终端 2：在同一世界生成 robot2；Sensors 插件只保留一份
ros2 launch turtlebot4_tutorial tutorial_spawn.launch.py \
  namespace:=robot2 model:=lite x:=-1.5 y:=0.0 rviz:=false \
  sensors_plugin:=false

# 终端 3：启动 robot2 的定位与导航；起点必须与终端 2 一致
ros2 launch turtlebot4_tutorial tutorial_navigation.launch.py \
  start_sim:=false namespace:=robot2 rviz:=false x:=-1.5 y:=0.0

# 终端 4：确认两个动作服务器出现后，再并行巡航
ros2 action list | grep navigate_to_pose
ros2 run turtlebot4_tutorial multi_robot_patrol robot1 robot2 \
  --goal-timeout 300
```

同一 Gazebo 世界只能有一个 `Sensors` 系统插件；否则本机的 Ogre2 渲染线程
会因重复创建场景而中止。双机器人运行时仿真速度可能明显低于真实时间，
因此上例为每个目标预留 `300 s`。单机默认仿真不会自动生成第二台机器人。

## 7. 本次修正的教程问题

- Cartographer 的 `pose_publish_period_sec` 和 `trajectory_publish_period_sec` 已从错误的 `5e3`、`30e3` 修正为 `5e-3`、`30e-3`。
- 原教程的 Nav2 YAML 缺少完整服务器和插件参数，已替换为适配当前 TurtleBot4 Humble 的完整配置。
- 地图保存参数拼写已修正为 `--ros-args`。
- 地图加载已由 `map_loader` 自动完成 `configure → activate`，无需手工切换生命周期。
- 没有添加教程中建议的 `base_link → base_footprint` 静态 TF；当前 TurtleBot4 配置以 `base_link` 为机器人基准，额外发布会造成错误或重复 TF。
- 针对本机 Ignition Fortress 的 GPU LiDAR 异常，仿真描述会将 Sensors 渲染器改为 Ogre 2，机器人几何、碰撞和物理属性保持原样。
- Nav2 的定位和导航分阶段启动，避免大地图加载期间生命周期服务超时；仿真起点会自动发布给 AMCL。
- 文档末尾列出的全部脚本都已实现，包括原正文没有提供源码的地图加载、状态发布、多机器人巡航和碰撞监控。
- 修复所有带 `argparse` 的程序与 ROS2 `--ros-args` 混用时的参数解析问题，并增加数值和命名空间校验。
- 碰撞监控和建图巡航现在对缺失、超时及无有效射线的雷达数据采用停车保护；手柄和键盘增加输入超时停车，并拒绝 NaN/Inf 控制值。
- 导航动作、单机巡航和多机巡航增加目标超时及取消路径，避免动作服务器异常时无限挂起。
- 仿真、建图和导航启动文件完整传递 `namespace`、`use_sim_time` 与生成位姿；非零仿真起点会同步给 AMCL。
- 修复 Python 测试依赖声明；默认 `colcon test` 现在会实际运行测试，不再显示“0 tests”。
- 初始位姿发布器改用 AMCL 兼容的可靠 QoS；此前 Best Effort 发布会被 AMCL 拒收，导致 `map → odom` 不生成、costmap 找不到 `map` 坐标系、导航目标被拒绝。
- 导航启动改为等待 AMCL 位姿确认；导航脚本的超时和 `Ctrl+C` 会等待取消确认，避免用户端退出后机器人继续执行原目标。
- 建图巡航以里程计而非墙上时钟判定旋转和行驶进度，受阻超过 3 秒会失败并停车；Simple Commander 默认保留已有定位。
- 第二台机器人生成时可设置 `sensors_plugin:=false`，避免 Ogre2 在重复 Sensors 插件下崩溃。

## 8. 验收结果（含历史记录）

本轮复测：构建、17 项测试、完整默认建图路线、地图保存/加载、自建地图单目标和四点巡航、IMU、图形窗口退出。其余旧条目为历史记录，逐项证据见修复记录。

| 项目 | 结果 |
| --- | --- |
| ROS2 talker/listener | 通过，持续收到消息 |
| 两个自定义包构建 | 通过，`2 packages finished` |
| 自动回归测试 | 2026-10-21：17/17 通过，含真实 map_saver/map_server 往返检查 |
| Python、XML、YAML | 编译/解析通过 |
| 自定义 SDF | `ign sdf -k` 返回 `Valid` |
| 机器人 URDF | `check_urdf` 通过 |
| Gazebo 生成与控制器 | 机器人生成、双轮控制器 active；充电座按 spawn_dock 选项生成 |
| 激光雷达 | 640 条射线，单帧 434 条有限值，约 1.30–11.99 m |
| Cartographer | `/map`、子图和 `map → odom` 正常发布 |
| 自动建图巡航 | 9/9 个动作按里程计达到位移与转角目标 |
| 生成地图 | 原教程地图 446 × 311；里程计版巡航另存 `/tmp/tb4_odom_map.yaml/.pgm` 为 492 × 410，0.05 m/像素 |
| 地图加载 | `map_server` 为 `active [3]`，地图正常发布 |
| Nav2 生命周期 | map_server、AMCL、controller、planner、BT 全部 active |
| 导航闭环 | `(0.5, 0, 0)` 目标被接受并返回“目标导航成功” |
| 非零起点导航 | 生成 x=0.3，AMCL x=0.3066，随后导航到 x=0.65 成功 |
| 默认初始位姿 QoS 修复 | AMCL 收到 `/initialpose`，`map → odom` 正常发布，`(0.5, 0, 0)` 目标接受并导航成功 |
| 导航启动顺序与巡航 | AMCL 确认后 Nav2 才启动；四点巡航全部成功 |
| 超时和中断取消 | 原生动作、Simple Commander、多机客户端均向服务端取消，服务端确认停车 |
| 建图受阻保护 | 人为设置前方受阻，3 秒后停车并返回失败 |
| 命名空间仿真 | `/robot1/scan`、`odom`、`cmd_vel` 正常，里程计实际移动 0.112 m |
| 第二台机器人生成 | `/robot2/scan`、`odom` 正常，两套 AMCL/Nav2 均为 active；单机目标和 `Ctrl+C` 取消通过 |
| 双机器人并行巡航 | robot1 与 robot2 各自的两个目标全部成功，`multi_robot_patrol` 退出码为 0 |
| 键盘节点实际通信 | 交互终端按 `W` 发布 `linear.x=0.2`，按 `Q` 发布 0 并退出 |
| 官方键盘遥控 | 交互终端按 `i` 实测发布 `linear.x=0.5`；若机器人不动，继续检查该话题是否连接到机器人控制节点 |
| 碰撞监控 | 0.20 m 模拟障碍时前进速度降为 0，转向保留 |
| 手柄转换 | 模拟 Joy 输入正确转换为线速度和角速度 |
| ROS2 Doctor | `All 5 checks passed` |

## 9. 已知边界

- 本机未连接实体手柄，因此手柄驱动和转换逻辑已验证，物理按键映射需按具体型号调整。
- 本轮已实际运行 Gazebo/RViz 图形窗口并检查关闭。默认 RViz 配置避开退出时会触发段错误的 Nav2 RViz 扩展组合，采用标准 `2D Goal Pose` 向 `goal_pose` 发布目标；默认软件渲染。原系统自带的 Nav2 RViz 配置未被修改，手工加载它仍可能复现旧问题。
- 控制器频率已从 1000 Hz 改为 100 Hz；它低于仿真步进频率，可能打印预期的 slower 警告。相机或电机模拟服务仍可能打印不可用提示；实测控制器为 active，雷达、里程计、建图和导航均可运行。主动取消导航时 Nav2 的 BT 节点有时打印 `Failed to get result for follow_path in node halt!`，以随后 `Goal canceled` 和控制器停车日志确认取消结果。
- `Ctrl+C` 关闭整套仿真时，Ignition 可能显示 `exit code -2`，表示收到中断信号；运行阶段的测试结果不受影响。
- 当前 Humble 二进制的 AMCL 在整套 Nav2 同时 `Ctrl+C` 退出时曾出现一次
  `exit code -11`；它发生在目标成功且 AMCL 已完成 deactivate/cleanup 之后。
  若在运行中出现该错误才需要重启并排查，单纯关闭阶段出现不影响已完成结果。
- `ros2 doctor` 只提示本地 TF2 0.25.23 低于索引中的 0.25.24；当前构建、TF、建图和导航均通过。
- rosdep 已在用户目录 `~/.config/rosdep/sources.list.d` 初始化，`.bashrc` 已设置 `ROSDEP_SOURCE_PATH`，并完成 Humble 索引更新。新终端执行 `rosdep check --from-paths single_robot/src --ignore-src --rosdistro humble` 应显示依赖满足；不需要再运行 sudo rosdep init。
- RViz 第一次收到地图时仍可能打印一次 `active samplers ... same texture image unit`。本轮地图实际显示并更新；这是 [RViz 上游问题 #463](https://github.com/ros2/rviz/issues/463) 中记录的提示，不能把它与退出段错误混为一谈。
- 教程末尾的 PyTorch/YOLO 和 MoveIt 是延伸方向，不属于 TurtleBot4 移动、建图和导航主实验。本机没有安装这些大型可选组件；文档所写 `ros-humble-yolov8-ros` 包名在当前 Humble 软件源中也不存在。

## 10. 故障排查

单独重开 RViz，不会重新启动仿真：

```bash
ros2 launch turtlebot4_tutorial tutorial_rviz.launch.py mode:=navigation
# 建图时改为 mode:=mapping；仅仿真改为 mode:=simulation
```

导航视图用工具栏 `2D Pose Estimate` 设置初始位姿，用 `2D Goal Pose` 点击拖拽
设置目标位置和朝向。旧 Nav2 扩展中的 Pause/Reset 面板不再默认加载；
命令行 Action、四点巡航及地图上点击目标均可使用。
RViz 视图配置存放在 `~/.ros/turtlebot4_tutorial/<命名空间>/`，可正常保存；
关闭时的“保存更改”只涉及视图设置，地图应使用 `map_saver` 另行保存。

检查关键话题：

```bash
ros2 topic list | grep -E '^/(clock|cmd_vel|odom|scan|tf|map)$'
```

如果 `ros2 node list` 显示不完整，可刷新 ROS2 daemon：

```bash
ros2 daemon stop
ros2 daemon start
```

如果导航没有开始，按顺序检查：

```bash
ros2 lifecycle get /amcl
ros2 run tf2_ros tf2_echo map odom
ros2 action list | grep navigate_to_pose
ros2 topic echo --once /scan
```

如果 costmap 连续提示 `frame "map" does not exist`，再检查 AMCL 是否收到了初始位姿：

```bash
ros2 topic info /initialpose -v
ros2 topic echo --once /amcl_pose
ros2 run tf2_ros tf2_echo map odom
```

`/initialpose` 的 AMCL 订阅应为 `RELIABLE`；`/amcl_pose` 应持续发布，
`tf2_echo` 应能查到 `map → odom`。若 AMCL 日志出现 `incompatible QoS`，
请重新构建并 source 工作空间，确保运行的是修复后的初始位姿发布器。

如果关闭后再次启动提示控制器已存在，先确认没有遗留仿真进程：

```bash
pgrep -af 'ign gazebo|ros2 launch'
```

如果只剩 `ign gazebo server`，却没有对应的 `ros2 launch` 进程和
`/cmd_vel` 话题，说明仿真已经残缺退出。确认列表中没有需要保留的仿真后，
清理遗留服务器并重新启动：

```bash
pkill -f 'ign gazebo server'
ros2 daemon stop
ros2 daemon start
```

随后重新运行 `tutorial_simulation.launch.py`，等待
`diffdrive_controller` 显示 `Configured and activated`，再启动键盘遥控。

正常使用时在主启动终端按一次 `Ctrl+C`，等待所有子进程退出后再重新启动。
