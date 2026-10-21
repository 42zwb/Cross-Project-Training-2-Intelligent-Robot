# tf2 0.25.23 导航位姿冻结：证据与工程内修复

## 本机现象与排查结果

系统版本：Ubuntu 22.04、ROS2 Humble，`ros-humble-tf2 / tf2-ros` 为 `0.25.23-1jammy.20260907`，Nav2 costmap 为 `1.1.20`。

初始实现中，独立观察者看到全局 TF 持续更新，但个别 Nav2 的 `published_footprint` 和控制器使用的位姿冻结在旧时间。车辆由此原地持续转向，或者规划起点停留在旧位置。仅看外部 `tf2_echo` 无法排除此问题。

曾比较以下措施：

- 导航等待完整 TF 后再启动：仍可复现。
- 将 TF 合并为单一发布通道：仍可复现，已撤除该试验代码。
- CycloneDDS 改为 FastDDS：仍可复现，最终保持 CycloneDDS。

这些对照不能代替根因证据。最终抓取冻结控制器的线程栈，发现：

```text
TF listener:
  BufferCore::testTransformableRequests
  → tf2_ros callback
  → waits on timer_to_request_map_mutex_

LaserScan / obstacle-layer callback:
  Buffer::waitForTransform
  → BufferCore::addTransformableRequest
  → waits on transformable_requests_mutex_
```

两线程互相持有对方需要的锁。冻结缓冲区转储中，所有动态 TF 保持在 44.38 秒，而同一时刻的新观察者收到约 327 秒的实时 TF。

本机证据位于 `runs/full-06-fastdds/controller2_threads.txt` 和 `buffer_dump.txt`。上游已有与该版本和线程栈吻合的报告：[geometry2 #995](https://github.com/ros2/geometry2/issues/995)。

## 修复边界

`vendor/tf2_compat` 保留 0.25.23 的源文件和头文件，仅修改 `BufferCore::testTransformableRequests()`：在锁内移除已完成请求并复制回调参数，释放请求锁和回调锁后，再调用外部回调。这样不再持有核心锁进入 tf2_ros 的定时器锁。

未改动公开头文件、类布局或导出接口。构建产物放在本工程 `vendor/runtime/lib/libtf2.so`，由 `env.sh` 的私有 `LD_LIBRARY_PATH` 选择。没有覆盖 `/opt/ros`、没有全局降级 apt 包，也没有修改旧教程源码或用户启动文件。

正常启动不加载调试用 `allow_ptrace.so / debug_tf.so`；它们只在排查时用于获取本项目进程的线程栈和缓冲区。

## 验证方法

`tests/tf2_lock_regression.cpp` 用两线程确定性地制造反向锁顺序：

- 系统原库：3 秒内不能完成，`timeout` 返回 124。
- 本地修复库：完成，返回 0。

结果在 `docs/tf2-lock-regression.json`。这项回归测试会随 `./scripts/test.sh` 检查修复库。

多机器人实际运行的验收记录另见 `docs/验收记录.md`。单独的锁测试不能证明整个导航系统已正常运行。

## 如果再次出现“导航说在走、车却不动”

1. 先看网页当前状态和三车门控原因；不要只看剩余距离。
2. 运行 `source ./env.sh` 后查看 `fleetctl status` 中的 `nav_pose_age`。
3. 对比 `/clock`、三车 `/odom`、TF 和六张代价地图 `published_footprint` 的时间戳。
4. 检查 `/proc/<导航进程 PID>/maps` 是否加载本工程 `vendor/runtime/lib/libtf2.so`。
5. 用独立 `truth_probe` 对照实际位置，避免把轮子空转或定位漂移误判为移动。
6. 保留 `runs/<本次>`，在原启动终端 Ctrl+C 后再重启；不要用无范围的 `pkill` 清掉其他机器人项目。
