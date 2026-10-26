// Copyright 2026 bwz
// SPDX-License-Identifier: Apache-2.0
#ifndef FLEET_BRIDGE__MANAGED_BRIDGE_HPP_
#define FLEET_BRIDGE__MANAGED_BRIDGE_HPP_

#include <ros_gz_bridge/ros_gz_bridge.hpp>

namespace fleet_bridge
{
// ros_gz_bridge 0.244.26 keeps strong node references in its bridge handles.
// Explicit release breaks those cycles without modifying the installed library.
// Call only after removing the node from its executor / spin has returned.
class ManagedBridge : public ros_gz_bridge::RosGzBridge
{
public:
  using RosGzBridge::RosGzBridge;

  std::size_t bridge_count() const {return handles_.size();}
  std::size_t service_count() const {return services_.size();}

  void close()
  {
    if (closed_) {
      return;
    }
    if (heartbeat_timer_) {
      heartbeat_timer_->cancel();
      heartbeat_timer_.reset();
    }
    // Gazebo callbacks capture the factory's raw `this`. Detach inputs before
    // dropping handles/factories, even when the producer is still running.
    if (gz_node_) {
      for (const auto & topic : gz_node_->SubscribedTopics()) {
        gz_node_->Unsubscribe(topic);
      }
    }
    services_.clear();
    handles_.clear();
    gz_node_.reset();
    closed_ = true;
  }

private:
  bool closed_ = false;
};
}  // namespace fleet_bridge
#endif
