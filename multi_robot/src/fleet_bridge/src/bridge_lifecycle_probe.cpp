// Copyright 2026 bwz
// SPDX-License-Identifier: Apache-2.0
#include <iostream>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "fleet_bridge/managed_bridge.hpp"

int main(int argc, char ** argv)
{
  const bool stock = argc > 1 && std::string(argv[1]) == "--stock";
  rclcpp::init(argc, argv);
  int retained = 0;
  for (int round = 0; round < 3; ++round) {
    for (auto direction : {ros_gz_bridge::BridgeDirection::GZ_TO_ROS,
      ros_gz_bridge::BridgeDirection::ROS_TO_GZ,
      ros_gz_bridge::BridgeDirection::BIDIRECTIONAL})
    {
      std::shared_ptr<ros_gz_bridge::RosGzBridge> node;
      if (stock) {
        node = std::make_shared<ros_gz_bridge::RosGzBridge>();
      } else {
        node = std::make_shared<fleet_bridge::ManagedBridge>();
      }
      ros_gz_bridge::BridgeConfig config;
      config.ros_type_name = "rosgraph_msgs/msg/Clock";
      config.gz_type_name = "ignition.msgs.Clock";
      config.ros_topic_name = config.gz_topic_name = "/fleet_lifecycle_probe";
      config.direction = direction;
      node->add_bridge(config);
      std::weak_ptr<ros_gz_bridge::RosGzBridge> lifetime = node;
      if (!stock) {
        std::static_pointer_cast<fleet_bridge::ManagedBridge>(node)->close();
        std::static_pointer_cast<fleet_bridge::ManagedBridge>(node)->close();
      }
      node.reset();
      if (!lifetime.expired()) {
        ++retained;
      }
    }
  }
  rclcpp::shutdown();
  std::cout << "retained bridge nodes: " << retained << "/9" << std::endl;
  return retained == 0 ? 0 : 3;
}
