# ROS2 Triton camera driver

A simple camera driver for the LUCID's Triton cameras using ROS2

## Description

This program takes images from one or more cameras on an external trigger then send those images to topics in ROS2

## Getting Started

### Dependencies

- ROS2 Humble
- LUCID Arena SDK

### Installing

Clone the repository  
```bash
git clone https://github.com/The-Last-Resort-FR/ROS2_LUCID_Triton_camera_driver.git
cd ROS2_LUCID_Triton_camera_driver
```
Build  
```bash
colcon build --symlink-install
```
Source the environement  
```bash
source install/setup.bash
```
The trigger source should be connected to line0 on the cameras (green wire on official M8 cable)

### Executing program

Run the node  
```bash
ros2 launch camera_manager launch.py
```
Make sure the trigger source is actually triggering the cameras  
You can visualize the images on the topics depending on the configuration of the camera master
```bash
ros2 run image_view image_view image:=/cameraN0 _image_transport:=theora # SINGLE mode
ros2 run image_view image_view image:=/cameraNO _image_transport:=theora &  ros2 run image_view image_view image:=/cameraN1 _image_transport:=theora # DUAL mode

```  
If the node is crashing from a timeout exception it means your cameras aren't in the same subnet as your host! They will be detected but attempting anything else results in a crash

## TODO

- Exposure time and gain feedback from a process node or commands
- External trigger starting with stm_comm

## Tested Hardware

- Host: Jetson AGX Orin
- Cameras : TRI054S-CC, TRI050S1-QC

## Version History

- V0.1.0 : First working version
- V0.2.0 : Rework and resiliance to disconnect or errors
- V0.2.1 : Camera calibration file, white balance and code ducomentation

## License

This project is licensed under the "The Unlicense" License - see the LICENSE.md file for details

## Acknowledgments

-  LUCID Arena SDK