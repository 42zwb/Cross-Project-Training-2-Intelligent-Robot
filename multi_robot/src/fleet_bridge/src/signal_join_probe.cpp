// Copyright 2026 bwz
// SPDX-License-Identifier: Apache-2.0
#include <atomic>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv)
{
  const bool stock = argc > 1 && std::string(argv[1]) == "--stock";
  rclcpp::init(argc, argv);
  auto complete = std::make_shared<std::atomic<bool>>(false);
  rclcpp::on_shutdown([complete]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    complete->store(true);
  });
  auto node = std::make_shared<rclcpp::Node>("fleet_signal_join_probe");
  std::cout << "ready" << std::endl;
  rclcpp::spin(node);
  if (!stock) {
    rclcpp::shutdown();
  }
  const bool done = complete->load();
  const bool handlers = rclcpp::signal_handlers_installed();
  std::cout << "shutdown callback joined: " << (done ? "yes" : "no") << std::endl;
  std::cout << "signal handlers still installed: " << (handlers ? "yes" : "no") << std::endl;
  return done && !handlers ? 0 : 4;
}
