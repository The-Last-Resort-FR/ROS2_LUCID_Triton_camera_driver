/**
 * @file cameraManager.cpp
 * @author tlr
 * @brief Implements the CameraManager class
 * @version 0.2.1
 * @date 2025-04-30
 * 
 * @copyright Copyright (c) 2025
 * 
 */

 // User includes
#include "cameraManager.hpp"

/**
 * @brief Initializes members, instanciates an image transport and runs the manager
 * 
 */
CameraManager::CameraManager()
: rclcpp::Node("camera_manager"), mNodeHandle((rclcpp::Node::SharedPtr)this), mDeviceUpdateTimeout(1000), mAquisitionTimeout(1000), mNodeParams(), mpSystem(nullptr), mpIt(nullptr), mError(false), mShouldStop(false), mCamCount(0) {
    mpIt = new image_transport::ImageTransport(mNodeHandle);
    mpClient = mNodeHandle->create_client<custom_msg::srv::Stcommand>("/send_stm_commands");
}

/**
 * @brief Destroy the Camera Manager:: Camera Manager object
 * 
 */
CameraManager::~CameraManager() {

}

/**
 * @brief Sets the timeout for the device discovery
 * 
 * @param deviceUpdateTimeout time in ms
 */
void CameraManager::SetDeviceUpdateTimeout(uint64_t deviceUpdateTimeout) {
    mDeviceUpdateTimeout = deviceUpdateTimeout;
}

/**
 * @brief Set the timeout for getting a camera's buffer
 * 
 * @param aquisitionTimeout time in ms
 */
void CameraManager::SetAquisitionTimeout(uint64_t aquisitionTimeout) {
    mAquisitionTimeout = aquisitionTimeout;
}

/**
 * @brief Discovers the devices
 * 
 * @return cam state
 */
bool CameraManager::InitSystem() {
    try {
        mpSystem = Arena::OpenSystem();
        mpSystem->UpdateDevices(mDeviceUpdateTimeout);
        mDevicesInfo = mpSystem->GetDevices();
        if(mDevicesInfo.size() == 0 ) throw std::runtime_error("No cameras detected");
    }
    catch (std::exception& ex) {
        RCLCPP_ERROR(mNodeHandle->get_logger(), "%s\n", ex.what());
        return CAM_ERROR;
    }
    catch (...) {
        RCLCPP_ERROR(mNodeHandle->get_logger(), "An error happened while Initializing the system\n");
        return CAM_ERROR;
    }
    mError = false;
    return CAM_OK;
}

// The opposite isn't implemented
#ifdef MODE_USRNAME

/**
 * @brief Creates the devices and Camera instances then feed them with the required parameters, also create the topics
 * 
 * @return cam state
 */
bool CameraManager::InitCameras() {
    if(mError || mCamCount != 0) return CAM_ERROR;
    
    for(Arena::DeviceInfo devi: mDevicesInfo) {
        mDevices.push_back(mpSystem->CreateDevice(devi));
        mCameras.push_back(new Camera(mNodeHandle, mAquisitionTimeout, mShouldStop, mNodeParams, devi.UserDefinedName().c_str()));
        ECHECK(mCameras[mCamCount]->SetDevice(mDevices[mCamCount]));
        ECHECK(mCameras[mCamCount]->SetParameters());
        mPublishers.push_back(mpIt->advertise(devi.UserDefinedName().c_str(), QUEUE_SIZE));
        char buff[32];
        sprintf(buff, "%s_info", devi.UserDefinedName().c_str());
        mInfoPublishers.push_back(this->create_publisher<sensor_msgs::msg::CameraInfo>(buff, QUEUE_SIZE));
        mCamCount++;
        if(devi.UserDefinedName() == "cam_rgb_left") {
            mCamInfoL = std::shared_ptr<camera_info_manager::CameraInfoManager>( new camera_info_manager::CameraInfoManager(this));
            mCamInfoL->setCameraName(devi.UserDefinedName().c_str());
            char buff[64];
            sprintf(buff, "package://camera_manager/config/calibration_%s.yaml", devi.UserDefinedName().c_str());
            mCamInfoL->validateURL(buff);
            mCamInfoL->loadCameraInfo(buff);
            mCamMsgL = mCamInfoL->getCameraInfo();
        }
        else {
            mCamInfoR = std::shared_ptr<camera_info_manager::CameraInfoManager>( new camera_info_manager::CameraInfoManager(this));
            mCamInfoR->setCameraName(devi.UserDefinedName().c_str());
            char buff[64];
            sprintf(buff, "package://camera_manager/config/calibration_%s.yaml", devi.UserDefinedName().c_str());
            mCamInfoR->validateURL(buff);
            mCamInfoR->loadCameraInfo(buff);
            mCamMsgR = mCamInfoR->getCameraInfo();
        }
    }
    RCLCPP_INFO(mNodeHandle->get_logger(), "%u cameras found and initiated\n", mCamCount);
    return CAM_OK;
}

#else

/**
 * @brief Deprectated
 * 
 * @return cam state
 */
bool CameraManager::InitCameras() {
    std::throw std::runtime_error("Not Implemented");
    if(mError || mCamCount != 0) return CAM_ERROR;
    for(Arena::DeviceInfo devi: mDevicesInfo) {
        char publisherName[32];
        mDevices.push_back(mpSystem->CreateDevice(devi));
        mCameras.push_back(new Camera(mNodeHandle, mAquisitionTimeout, mShouldStop, mNodeParams));
        ECHECK(mCameras[mCamCount]->SetDevice(mDevices[mCamCount]));
        ECHECK(mCameras[mCamCount]->SetParameters());
        sprintf(publisherName, "cameraN%u", mCamCount);
        mPublishers.push_back(mpIt->advertise(publisherName, QUEUE_SIZE));
        mCamCount++;
    }
    RCLCPP_INFO(mNodeHandle->get_logger(), "%u cameras found and initiated\n", mCamCount);
    return CAM_OK;
}
#endif


/**
 * @brief Starts then loops through all the cameras and see if they have an image to publish or in an error state
 * 
 * @return true 
 * @return false 
 */
bool CameraManager::PublishingLoop() {
    if(mError || mCamCount < 1) return CAM_ERROR;
    RCLCPP_INFO(mNodeHandle->get_logger(), "Publishing loop started\n");
    uint64_t frameId = 0;
    uint8_t indexIt = 0;
    ECHECK(TriggerSetup(20));
    ECHECK(TriggerControl(1));

    for(Camera* cam: mCameras) {
        mDevices[indexIt++]->StartStream();
        cam->Run();
    }
    while (!mShouldStop)
    {
        for(indexIt = 0; indexIt < mCamCount; indexIt++) {
            if(mCameras[indexIt]->GetImageQueue().size() > 0) {
                RCLCPP_INFO(mNodeHandle->get_logger(), "Frame found\n");
                std_msgs::msg::Header hdr;
                char ids[40];
                snprintf(ids, 40, "id%ld", frameId);
                hdr.stamp = mNodeHandle->now();
                hdr.set__frame_id(ids);

                Arena::IImage* img = mCameras[indexIt]->GetImageQueue().front();
                mCameras[indexIt]->GetImageQueue().pop();
                cv::Mat imageCv = cv::Mat(img->GetHeight(), img->GetWidth(), CV_8UC1, (uint8_t *)img->GetData());
                cv::Mat imageBgr(imageCv.rows, imageCv.cols, CV_8UC3);
                cvtColor(imageCv, imageBgr, cv::COLOR_BayerBG2BGR);
                cv::Mat msgImg = imageBgr.clone();
                sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(hdr, "bgr8", msgImg).toImageMsg();
                mPublishers[indexIt].publish(msg);
                if(mCameras[indexIt]->GetName() == "cam_rgb_left") {
                    mInfoPublishers[indexIt]->publish(mCamMsgL);
                }
                else {
                    mInfoPublishers[indexIt]->publish(mCamMsgR);
                }
                Arena::ImageFactory::Destroy(img);
            }
            if(mCameras[indexIt]->GetStatus() == CAM_ERROR) {
                RCLCPP_INFO(mNodeHandle->get_logger(), "A camera has an error status");
                mError = true;
                for(uint8_t j = 0; j < mCamCount; j++) {
                    mDevices[j]->StopStream();
                }
                return CAM_ERROR;
            }
        }
    }

    for(uint8_t j = 0; j < mCamCount; j++) {
        mDevices[j]->StopStream();
    }
    return CAM_OK;
    
}

/**
 * @brief Handles recovery afer a camera was detected as in an error state
 * 
 */
void CameraManager::Recovery() {
    Purge();
    mCamCount = 0;
    mDevicesInfo.clear();
}

/**
 * @brief Runs the node
 * 
 */
void CameraManager::Run() {
    DeclareNodeParams();
    GetNodeParams();
    while (!mShouldStop)
    {
        InitSystem();
        InitCameras();
        PublishingLoop();
        Recovery();
    } 
}

/**
 * @brief Tries to delete and close everything
 * 
 */
void CameraManager::Purge() {   // remove from vector
    for(Camera* cam: mCameras) {
        try {
            if(cam != nullptr)
                delete cam;
        }
        catch(...) {
            RCLCPP_ERROR(mNodeHandle->get_logger(), "Invalid cam to purge\n");
        }
    }
    for(Arena::IDevice* dev: mDevices) {
        try {
            if(mpSystem != nullptr) {
                mpSystem->DestroyDevice(dev);
            }
        }
        catch (...) {
            RCLCPP_ERROR(mNodeHandle->get_logger(), "empty mpSystem\n");
        }
    }
    mCameras.clear();
    mDevices.clear();
    Arena::CloseSystem(mpSystem);
    mpSystem = nullptr;
}

/**
 * @brief Get all the parameters provided by the ROS2 API though our .yaml
 * 
 */
void CameraManager::GetNodeParams() {
#define X(field, type) GET_PARAMS(mNodeParams, field, mNodeHandle);
    PARAM_FIELDS_DEC
#undef X
}

/**
 * @brief Declare all the parameters the node intends on getting from the ROS2 API
 * 
 */
void CameraManager::DeclareNodeParams() {
#define X(field, type) DECLARE_PARAM(mNodeParams, field, mNodeHandle);
    PARAM_FIELDS_DEC
#undef X
}

rclcpp::Node::SharedPtr CameraManager::GetNodeHandle() {
    return mNodeHandle;
}

bool CameraManager::TriggerSetup(uint16_t freq) {
    using namespace std::chrono_literals;

    // maybe deport to a function to avoid
    auto request = std::make_shared<custom_msg::srv::Stcommand::Request>();
    request->command = 0x0001;
    request->arg = freq;
    request->type = 1;
        while (!mpClient->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the stm_comm service. Exiting.");
          return CAM_ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "stm_comm not yet available");
    }

    auto result = mpClient->async_send_request(request);

    if (result.wait_for(2s) == std::future_status::ready)
    {
        std::array<uint8_t, 9UL> r = result.get()->response;
        if(r[0] == 0) {
            return CAM_OK;
        }
        else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "STM32 Replied with an error");
            return CAM_ERROR;
        }
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call stcommand");
        return CAM_ERROR;
    }
}

bool CameraManager::TriggerControl(bool startStop) {
    using namespace std::chrono_literals;

    auto request = std::make_shared<custom_msg::srv::Stcommand::Request>();
    request->command = startStop ? 0x0002 : 0x0003;
    request->arg = 00;
    request->type = 1;
        while (!mpClient->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the stm_comm service. Exiting.");
          return CAM_ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "stm_comm not yet available");
    }

    auto result = mpClient->async_send_request(request);

    if (result.wait_for(2s) == std::future_status::ready)
    {
        std::array<uint8_t, 9UL> r = result.get()->response;
        if(r[0] == 0) {
            return CAM_OK;
        }
        else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "STM32 Replied with an error");
            return CAM_ERROR;
        }
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call stcommand");
        return CAM_ERROR;
    }
}