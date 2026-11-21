// Independent evaluator only. Runtime ROS graph never receives these poses.
#include <ignition/transport/Node.hh>
#include <ignition/msgs/pose_v.pb.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <optional>
#include <thread>

namespace {
constexpr auto sample_period = std::chrono::milliseconds(50);
std::atomic<bool> running{true};
}

int main(int argc, char **argv) {
  std::ofstream file;
  if (argc > 1) {
    file.open(argv[1]);
    if (!file) {
      std::cerr << "could not open truth output: " << argv[1] << '\n';
      return 1;
    }
  }
  std::ostream &out = argc > 1 ? file : std::cout;
  out << std::setprecision(17);
  std::signal(SIGINT, [](int) { running = false; });
  std::signal(SIGTERM, [](int) { running = false; });

  std::optional<std::chrono::nanoseconds> last_sample;
  std::function<void(const ignition::msgs::Pose_V &)> callback =
      [&](const ignition::msgs::Pose_V &message) {
        const auto received = std::chrono::steady_clock::now();
        const auto &stamp = message.header().stamp();
        const auto sim_time = std::chrono::seconds(stamp.sec()) +
                              std::chrono::nanoseconds(stamp.nsec());
        if (last_sample && sim_time - *last_sample < sample_period) {
          return;
        }
        last_sample = sim_time;

        // Preserve native message stamps and actual receipt times. A stalled
        // publisher leaves a gap in the capture, even with denser sampling.
        const auto wall_time = std::chrono::system_clock::now().time_since_epoch();
        const auto steady_time = received.time_since_epoch();
        out << "{\"sim_time\":" << std::chrono::duration<double>(sim_time).count()
            << ",\"wall_time\":" << std::chrono::duration<double>(wall_time).count()
            << ",\"steady_time\":" << std::chrono::duration<double>(steady_time).count()
            << ",\"robots\":{";
        bool found = false;
        for (const auto &pose : message.pose()) {
          const auto &name = pose.name();
          if (name != "robot1" && name != "robot2" && name != "robot3") {
            continue;
          }
          if (found) {
            out << ',';
          }
          found = true;
          const auto &q = pose.orientation();
          const double yaw = std::atan2(
              2 * (q.w() * q.z() + q.x() * q.y()),
              1 - 2 * (q.y() * q.y() + q.z() * q.z()));
          const auto &p = pose.position();
          out << '"' << name << "\":[" << p.x() << ',' << p.y() << ',' << yaw
              << ',' << p.z() << ']';
        }
        out << "}}\n";
        out.flush();
      };

  ignition::transport::Node node;
  if (!node.Subscribe("/world/fleet_arena/dynamic_pose/info", callback)) {
    std::cerr << "subscribe failed\n";
    return 1;
  }
  while (running) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return 0;
}
