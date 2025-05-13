/**
 * @file cameraManager.hpp
 * @author tlr
 * @brief Takes the Camera instances, feed them with their config, handles device management and publishing on topics
 * @version 0.2.1
 * @date 2025-04-30
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

// STL includes
#include <memory>
#include <vector>

// Lib includes
#include <rclcpp/rclcpp.hpp>
#include <Arena/ArenaApi.h>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/mat.hpp>
#include <camera_info_manager/camera_info_manager.hpp>

// User includes
#include "camera.hpp"
#include "custom_msg/srv/stcommand.hpp"


// Enable the mode where pretty much everything is base on the cam's user defined name
#define MODE_USRNAME

// Error checking macro
#define ECHECK(x) if(x) printf("The function %s returned with an error\n", #x)

// Queue size for ROS2 stuff
constexpr uint32_t QUEUE_SIZE = 1;

class CameraManager : public rclcpp::Node {
private:
    rclcpp::Node::SharedPtr mNodeHandle;
    uint64_t mDeviceUpdateTimeout;
    uint64_t mAquisitionTimeout;
    NodeParameters mNodeParams;
    Arena::ISystem* mpSystem;
    std::vector<Arena::DeviceInfo> mDevicesInfo;
    std::vector<Arena::IDevice*> mDevices;
    std::vector<Camera*> mCameras;
    image_transport::ImageTransport* mpIt;
    std::vector<image_transport::Publisher> mPublishers;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> mInfoPublishers;
    bool mError;
    bool mShouldStop;
    uint8_t mCamCount;
    std::shared_ptr<camera_info_manager::CameraInfoManager> mCamInfoL;
    std::shared_ptr<camera_info_manager::CameraInfoManager> mCamInfoR;
    sensor_msgs::msg::CameraInfo mCamMsgL;
    sensor_msgs::msg::CameraInfo mCamMsgR;
    rclcpp::Client<custom_msg::srv::Stcommand>::SharedPtr mpClient;

public:
    CameraManager();
    ~CameraManager();
    void SetDeviceUpdateTimeout(uint64_t deviceUpdateTimeout);
    void SetAquisitionTimeout(uint64_t aquisitionTimeout);
    bool TriggerSetup(uint16_t freq);
    bool TriggerControl(bool startStop);
    void DeclareNodeParams();
    void GetNodeParams();
    bool InitSystem();
    bool InitCameras();
    bool PublishingLoop();
    void Recovery();
    void Run();
    void Purge();
    rclcpp::Node::SharedPtr GetNodeHandle();
};
