#include "cameraManager.hpp"

CameraManager::CameraManager()
: rclcpp::Node("camera_manager"), mNodeHandle((rclcpp::Node::SharedPtr)this), mDeviceUpdateTimeout(1000), mAquisitionTimeout(1000), mNodeParams(), mpSystem(nullptr), mpIt(nullptr), mError(false), mShouldStop(false), mCamCount(0) {
    mpIt = new image_transport::ImageTransport(mNodeHandle);
    Run();
}

CameraManager::~CameraManager() {

}

void CameraManager::SetDeviceUpdateTimeout(uint64_t deviceUpdateTimeout) {
    mDeviceUpdateTimeout = deviceUpdateTimeout;
}

void CameraManager::SetAquisitionTimeout(uint64_t aquisitionTimeout) {
    mAquisitionTimeout = aquisitionTimeout;
}

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

#ifdef MODE_USRNAME

bool CameraManager::InitCameras() {
    if(mError || mCamCount != 0) return CAM_ERROR;
    
    for(Arena::DeviceInfo devi: mDevicesInfo) {
        mDevices.push_back(mpSystem->CreateDevice(devi));
        mCameras.push_back(new Camera(mNodeHandle, mAquisitionTimeout, mShouldStop, mNodeParams));
        ECHECK(mCameras[mCamCount]->SetDevice(mDevices[mCamCount]));
        ECHECK(mCameras[mCamCount]->SetParameters());
        mPublishers.push_back(mpIt->advertise(devi.UserDefinedName().c_str(), QUEUE_SIZE));
        mCamCount++;
    }
    RCLCPP_INFO(mNodeHandle->get_logger(), "%u cameras found and initiated\n", mCamCount);
    return CAM_OK;
}

#else

bool CameraManager::InitCameras() {
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


bool CameraManager::PublishingLoop() {
    if(mError || mCamCount < 1) return CAM_ERROR;
    RCLCPP_INFO(mNodeHandle->get_logger(), "Publishing loop started\n");
    uint64_t frameId = 0;
    uint8_t indexIt = 0;

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

void CameraManager::Recovery() {
    Purge();
    mCamCount = 0;
    mDevicesInfo.clear();
}

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

void CameraManager::GetNodeParams() {
#define X(field, type) GET_PARAMS(mNodeParams, field, mNodeHandle);
    PARAM_FIELDS_DEC
#undef X
}

void CameraManager::DeclareNodeParams() {
#define X(field, type) DECLARE_PARAM(mNodeParams, field, mNodeHandle);
    PARAM_FIELDS_DEC
#undef X
}