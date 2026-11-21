// Own the simulator and ROS context in a deterministic shutdown order.
// Robot algorithms still run in their separate, independently guarded nodes.
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string_view>
#include <thread>
#include <ignition/common/Console.hh>
#include <ignition/gazebo/Server.hh>
#include <ignition/gazebo/ServerConfig.hh>
#include <rclcpp/rclcpp.hpp>

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void request_stop(int) { stop_requested = 1; }
}

int main(int argc, char **argv) {
  if (argc != 2 || !std::filesystem::is_regular_file(argv[1])) {
    std::cerr << "Usage: fleet_sim_server /absolute/path/to/world.sdf\n";
    return 2;
  }
  ignition::common::Console::SetVerbosity(2);
  // gz_ros2_control otherwise installs a second process-wide signal handler.
  // Keep its context valid until the world, executors and controllers are gone.
  rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  int result = 0;
  try {
    ignition::gazebo::ServerConfig config;
    config.SetSdfFile(argv[1]);
    const char *egl_headless = std::getenv("FLEET_EGL_HEADLESS");
    if (egl_headless && std::string_view(egl_headless) == "1") {
      config.SetHeadlessRendering(true);
      std::cout << "fleet_sim_server: requested headless EGL\n" << std::flush;
    }
    auto server = std::make_unique<ignition::gazebo::Server>(config);
    if (!server->EntityCount().has_value()) {
      throw std::runtime_error("world could not be loaded");
    }
    // Install after Server's own handler, before any model is spawned. Only
    // this main thread calls Stop; a signal handler only changes a sig_atomic_t.
    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);
    std::signal(SIGHUP, request_stop);
    if (!server->Run(false, 0, false)) {
      throw std::runtime_error("simulator failed to start");
    }
    std::cout << "fleet_sim_server: running\n" << std::flush;
    while (!stop_requested && server->Running()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    server->Stop();
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
    while (server->Running() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (server->Running()) {
      // A failed Configure callback may be waiting for a dead ROS service.
      // Break that wait, report failure, and let the supervisor bound cleanup.
      std::cerr << "fleet_sim_server: simulation thread failed to stop\n";
      rclcpp::shutdown();
      result = 1;
    }
    // Server's destructor joins the run thread and releases all system plugins.
    // No asynchronous /server_control stop request races with this destruction.
    server.reset();
    std::cout << "fleet_sim_server: world released\n" << std::flush;
  } catch (const std::exception &error) {
    std::cerr << "fleet_sim_server: " << error.what() << '\n';
    result = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return result;
}
