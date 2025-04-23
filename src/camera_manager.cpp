#include <cstdio>
#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <cameraManager.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  CameraManager* node = new CameraManager;
  (void)node; // complains it's not used
  while(1) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  return 0;
}

// --ros-args -r __node:=camera_manager --params-file config/camera.yaml