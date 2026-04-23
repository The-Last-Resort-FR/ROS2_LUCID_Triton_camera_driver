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
        GenICam_3_3_LUCID::gcstring name = "lucid_" + devi.UserDefinedName();
        mPublishers.push_back(mpIt->advertise(name.c_str(), QUEUE_SIZE));
        char buff[32];
        sprintf(buff, "%s_info", name.c_str());
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

void CameraManager::CameraPublishingWorker(int index) {
    uint64_t frameId = 0;
    RCLCPP_INFO(this->get_logger(), "Worker thread for %s started", mCameras[index]->GetName().c_str());

    while (!mShouldStop && !mError) {
        Arena::IImage* img = nullptr;

        // 1. Thread-safe extraction from the camera queue
        {
            std::lock_guard<std::mutex> lock(mCameras[index]->mQueueMtx);
            if (!mCameras[index]->GetImageQueue().empty()) {
                img = mCameras[index]->GetImageQueue().front();
                mCameras[index]->GetImageQueue().pop();
            }
        }

        if (img) {
            std::chrono::high_resolution_clock::time_point _start = std::chrono::high_resolution_clock::now();

            std_msgs::msg::Header hdr;
            char ids[40];
            snprintf(ids, 40, "%s_id%ld", mCameras[index]->GetName().c_str(), frameId++);
            hdr.stamp = mNodeHandle->now();
            hdr.set__frame_id(ids);

            cv::Mat imageCv = cv::Mat(img->GetHeight(), img->GetWidth(), CV_8UC1, (uint8_t *)img->GetData());
            cv::Mat imageBgr; 
            cv::cvtColor(imageCv, imageBgr, cv::COLOR_BayerBG2BGR);
            
            sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(hdr, "bgr8", imageBgr).toImageMsg();

            // 32ms bottleneck
            mPublishers[index].publish(msg);
            
            if (mCameras[index]->GetName() == "cam_rgb_left") {
                mInfoPublishers[index]->publish(mCamMsgL);
            } else {
                mInfoPublishers[index]->publish(mCamMsgR);
            }

            if (!(frameId % 300)) {
                auto duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - _start).count();
                RCLCPP_INFO(this->get_logger(), "%s work + publish took %ld us", mCameras[index]->GetName().c_str(), duration);
            }

            Arena::ImageFactory::Destroy(img);
        } else {
            // void spinning at 100% CPU
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        if (mCameras[index]->GetStatus() == CAM_ERROR) {
            RCLCPP_ERROR(this->get_logger(), "Camera %s entered error state", mCameras[index]->GetName().c_str());
            mError = true;
        }
    }
}


/**
 * @brief Starts then loops through all the cameras and see if they have an image to publish or in an error state
 * 
 * @return true 
 * @return false 
 */
bool CameraManager::PublishingLoop() {
    if (mError || mCamCount < 1) return CAM_ERROR;

    RCLCPP_INFO(mNodeHandle->get_logger(), "Preparing threads...");
    ECHECK(TriggerSetup(20));

    for (uint8_t i = 0; i < mCamCount; i++) {
        mDevices[i]->StartStream();
        mCameras[i]->Run();
    }

    ECHECK(TriggerControl(1));

    for (uint8_t i = 0; i < mCamCount; i++) {
        mPublishingThreads.emplace_back(&CameraManager::CameraPublishingWorker, this, i);
    }

    while (!mShouldStop && !mError) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    for (auto& t : mPublishingThreads) {
        if (t.joinable()) t.join();
    }
    mPublishingThreads.clear();

    for (uint8_t j = 0; j < mCamCount; j++) {
        mDevices[j]->StopStream();
    }

    return mError ? CAM_ERROR : CAM_OK;
}

/**
 * @brief Handles recovery afer a camera was detected as in an error state
 * 
 */
void CameraManager::Recovery() {
    ECHECK(TriggerControl(0));
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
void CameraManager::Purge() {
    // Ensure threads are dead before cleaning up pointers
    mShouldStop = true; 
    for (auto& t : mPublishingThreads) {
        if (t.joinable()) t.join();
    }
    mPublishingThreads.clear();

    for (Camera* cam : mCameras) {
        if (cam != nullptr) delete cam;
    }
    for (Arena::IDevice* dev : mDevices) {
        if (mpSystem != nullptr) mpSystem->DestroyDevice(dev);
    }
    mCameras.clear();
    mDevices.clear();
    if (mpSystem != nullptr) {
        Arena::CloseSystem(mpSystem);
        mpSystem = nullptr;
    }
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