# Project-local tf2 compatibility patch

Source: ROS 2 geometry2 tag **0.25.23** (BSD-3-Clause core and original per-file licenses are retained).

- https://github.com/ros2/geometry2/tree/0.25.23/tf2
- Relevant upstream report: https://github.com/ros2/geometry2/issues/995

The installed tf2 / tf2_ros 0.25.23 can invert `transformable_requests_mutex_` and tf2_ros's `timer_to_request_map_mutex_` while receiving lidar messages. The exact two blocked stacks were captured in `runs/full-06-fastdds/controller2_threads.txt`.

`deadlock.patch` moves completed callbacks out of the core request/callback locks, after removing completed entries under the existing locks. Callback arguments and functions are copied before unlock. No public headers, class layouts or exported interfaces are changed. The resulting ABI-compatible `libtf2.so` is installed under `vendor/runtime/lib` and selected only by this project's `env.sh`. Nothing in `/opt/ros` is overwritten or downgraded.

`tests/tf2_lock_regression.cpp` deterministically reproduces the inverse lock acquisition with two threads. The unpatched library hangs; the patched library must complete. Full three-robot runtime acceptance is separate from that regression test.
