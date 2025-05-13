/**
 * @file camera_manager.cpp
 * @author tlr
 * @brief The main, just instanciate a CameraManager obj
 * @version 0.2.1
 * @date 2025-04-30
 * 
 * @copyright Copyright (c) 2025
 * 
 */

 // STL includes
#include <cstdio>
#include <memory>
#include <chrono>

// Lib includes
#include <rclcpp/rclcpp.hpp>

// User includes
#include "cameraManager.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  CameraManager* node = new CameraManager;

  std::thread spinner([&]() {
    rclcpp::spin(node->GetNodeHandle());
  });

  node->Run();

  spinner.join();
  return 0;
}

// gdb debug stuff
// set args --ros-args -r __node:=camera_manager --params-file config/camera.yaml