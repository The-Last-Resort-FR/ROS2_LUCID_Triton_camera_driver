cmake_minimum_required(VERSION 3.8)
project(camera_manager)

set(CMAKE_CXX_FLAGS "-g3 -O3 -Wall -Wextra")


set(sources
  "src/camera_manager.cpp"
  "src/cameraManager.cpp"
  "src/camera.cpp")

set(includes
  "include/cameraManager.hpp"
  "include/camera.hpp"
  "include/config.hpp")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(image_transport REQUIRED)
#find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED 408)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/FindARENA.cmake")
link_directories(${arena_sdk_LIBRARIES})

add_executable(camera_manager ${sources} ${includes})
target_include_directories(camera_manager
  SYSTEM
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${arena_sdk_INCLUDES}
  ${OpenCV_INCLUDE_DIRS})


# OpenCV and Arenasdk
target_link_libraries(camera_manager
  ${OpenCV_LIBRARIES}
  ${arena_sdk_LIBRARIES}
)

ament_target_dependencies(camera_manager "rclcpp" "image_transport" "cv_bridge") # ROS2 dependencies
target_compile_features(camera_manager PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17

install(TARGETS camera_manager
  DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
