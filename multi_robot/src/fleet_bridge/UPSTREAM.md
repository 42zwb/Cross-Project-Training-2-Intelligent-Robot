# ROS Gazebo bridge lifecycle compatibility entrypoint

The argument parser in `src/parameter_bridge.cpp` is derived from Apache-2.0
`gazebosim/ros_gz` tag **0.244.26**, `ros_gz_bridge/src/parameter_bridge.cpp`:
https://github.com/gazebosim/ros_gz/blob/0.244.26/ros_gz_bridge/src/parameter_bridge.cpp
Original copyright and license notices are retained.

This package links the installed ROS Humble `libros_gz_bridge.so` for all message
factories and conversions. It manages lifecycle ownership and validates arguments:

1. Cancel the bridge heartbeat and detach Gazebo subscriptions before releasing
   factories, then clear the node-owned handles/services and transport reference.
   This explicitly breaks the node/handle and subscriber/node ownership cycles.
2. Call `rclcpp::shutdown()` after spin returns, including error exits, to join the
   deferred signal-handler thread before C/C++ process-exit finalizers run.
3. Reject incomplete topic specifications and unsupported topic/service mappings
   instead of spinning indefinitely with no working bridge.

Related upstream fixes (different release branch; not assumed ABI-compatible):
- https://github.com/gazebosim/ros_gz/pull/952
- https://github.com/gazebosim/ros_gz/pull/977

The compatibility class uses the protected lifecycle fields present in the
installed 0.244.26 API. No system ROS/Gazebo library is replaced or preloaded.
`bridge_lifecycle_probe --stock` diagnoses retained nodes with the installed
entrypoint's ownership model; without that option it checks this package's
explicit release in all three topic directions.
