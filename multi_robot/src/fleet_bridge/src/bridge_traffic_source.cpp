// Copyright 2026 bwz
// SPDX-License-Identifier: Apache-2.0
// Dedicated synthetic traffic for the isolated bridge lifecycle regression.
#include <atomic>
#include <chrono>
#include <csignal>
#include <thread>
#include <gz/transport/Node.hh>
#include <gz/msgs/clock.pb.h>
#include <gz/msgs/laserscan.pb.h>

namespace { volatile std::sig_atomic_t stopped = 0; }
void stop(int) {stopped = 1;}

int main()
{
  std::signal(SIGINT, stop);
  std::signal(SIGTERM, stop);
  gz::transport::Node node;
  auto clock_pub = node.Advertise<gz::msgs::Clock>("/fleet_bridge_probe/clock");
  auto scan_pub = node.Advertise<gz::msgs::LaserScan>("/fleet_bridge_probe/scan");
  gz::msgs::LaserScan scan;
  scan.set_frame("probe/rplidar_link");
  scan.set_angle_min(-3.141592653589793);
  scan.set_angle_max(3.141592653589793);
  scan.set_angle_step(6.283185307179586 / 640);
  scan.set_range_min(0.12);
  scan.set_range_max(12.0);
  scan.set_count(640);
  scan.set_vertical_count(1);
  auto frame = scan.mutable_header()->add_data();
  frame->set_key("frame_id");
  frame->add_value("probe/rplidar_link");
  for (int i=0; i<640; ++i) {scan.add_ranges(6.0);}
  uint64_t tick = 0;
  while (!stopped) {
    gz::msgs::Clock clock;
    clock.mutable_sim()->set_sec(tick / 200);
    clock.mutable_sim()->set_nsec((tick % 200) * 5000000);
    clock_pub.Publish(clock);
    if (tick % 8 == 0) {
      *scan.mutable_header()->mutable_stamp() = clock.sim();
      scan_pub.Publish(scan);
    }
    ++tick;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return 0;
}
