#pragma once

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <Arena/ArenaApi.h>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/mat.hpp>

#include "camera.hpp"

#define ECHECK(x) if(x) printf("The function %s returned with an error\n", #x)

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
    bool mError;
    bool mShouldStop;
    uint8_t mCamCount;

public:
    CameraManager();
    ~CameraManager();
    void SetDeviceUpdateTimeout(uint64_t deviceUpdateTimeout);
    void SetAquisitionTimeout(uint64_t aquisitionTimeout);
    void DeclareNodeParams();
    void GetNodeParams();
    bool InitSystem();
    bool InitCameras();
    bool PublishingLoop();
    void Recovery();
    void Run();
    void Purge();
};
