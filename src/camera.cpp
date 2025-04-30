/**
 * @file camera.cpp
 * @author tlr
 * @brief Implementation of the Camera class
 * @version 0.2.1
 * @date 2025-04-30
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "camera.hpp"

/**
 * @brief Initialize members
 * 
 */
Camera::Camera(rclcpp::Node::SharedPtr nodeHandle, const uint64_t& timeout, const bool& pExtShouldStop, const NodeParameters& nodeParameters, std::string name) \
: mNodeHandle(nodeHandle), mTimeout(timeout), mExtShouldStop(pExtShouldStop), mNodeParameters(nodeParameters), mpDevice(nullptr), mHasCrashed(false), mName(name) {

}

/**
 * @brief Attempt to reset the devices and leaves some time for threads to finish and die
 * 
 */
Camera::~Camera() {
    mHasCrashed = true;
    ResetDevice();
    std::this_thread::sleep_for(std::chrono::milliseconds(mTimeout + 200));
}

/**
 * @brief Self explanatory
 * 
 * @return true 
 * @return false 
 */
const bool& Camera::GetStatus() {
    return mHasCrashed;
}

/**
 * @brief Set a device created externally for internal use
 * 
 * @param pDevice 
 * @return cam status
 */
bool Camera::SetDevice(Arena::IDevice* pDevice) {
    if(pDevice == nullptr) return CAM_ERROR;
    mpDevice = pDevice;
    return CAM_OK;
}

/**
 * @brief Returns the internal image queue, free the image after use
 * @attention If the queue isn't watched and emptied very regularly it can eat all the ram very fast
 * 
 * @return std::queue<Arena::IImage*>& 
 */
std::queue<Arena::IImage*>& Camera::GetImageQueue() {
    return mImages;
}

/**
 * @brief Set all the paramters recovered externally from the .yaml 
 * 
 * @return cam status
 */
bool Camera::SetParameters() {
    try {
        if(mHasCrashed || mpDevice == nullptr) {
            throw std::runtime_error("Setting parameters of a crashed or empty camera");
        }

        Arena::SetNodeValue<bool>(mpDevice->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
        Arena::SetNodeValue<bool>(mpDevice->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);
        Arena::SetNodeValue<GenICam::gcstring>(mpDevice->GetNodeMap(), "TriggerSelector", GenICam::gcstring(mNodeParameters.TriggerSelector.c_str()));
        Arena::SetNodeValue<GenICam::gcstring>(mpDevice->GetNodeMap(), "TriggerMode", GenICam::gcstring(mNodeParameters.TriggerMode.c_str()));
        Arena::SetNodeValue<GenICam::gcstring>(mpDevice->GetNodeMap(), "TriggerSource", GenICam::gcstring(mNodeParameters.TriggerSource.c_str()));
        Arena::SetNodeValue<GenICam::gcstring>(mpDevice->GetNodeMap(), "TriggerActivation", GenICam::gcstring(mNodeParameters.TriggerActivation.c_str()));
        Arena::SetNodeValue<GenICam::gcstring>(mpDevice->GetNodeMap(), "TriggerOverlap", GenICam::gcstring(mNodeParameters.TriggerOverlap.c_str()));
        Arena::SetNodeValue<GenICam::gcstring>(mpDevice->GetNodeMap(), "ExposureAuto", GenICam::gcstring(mNodeParameters.ExposureAuto.c_str()));
        Arena::SetNodeValue<double>(mpDevice->GetNodeMap(), "ExposureTime", mNodeParameters.ExposureTime);
        Arena::SetNodeValue<GenICam::gcstring>(mpDevice->GetNodeMap(), "GainAuto", GenICam::gcstring(mNodeParameters.GainAuto.c_str()));
        Arena::SetNodeValue<double>(mpDevice->GetNodeMap(), "Gain", mNodeParameters.Gain);
        Arena::SetNodeValue<GenICam::gcstring>(mpDevice->GetNodeMap(), "BalanceWhiteAuto", "Continuous");
        Arena::SetNodeValue<bool>(mpDevice->GetNodeMap(), "BalanceWhiteEnable", true);
        Arena::SetNodeValue<int64_t>(mpDevice->GetNodeMap(), "OffsetX", mNodeParameters.OffsetX);
        Arena::SetNodeValue<int64_t>(mpDevice->GetNodeMap(), "OffsetY", mNodeParameters.OffsetY);
        Arena::SetNodeValue<GenICam::gcstring>(mpDevice->GetNodeMap(), "LineMode", GenICam::gcstring(mNodeParameters.LineMode.c_str()));
        Arena::SetNodeValue<GenICam::gcstring>(mpDevice->GetNodeMap(), "PixelFormat", GenICam::gcstring(mNodeParameters.PixelFormat.c_str()));

        RCLCPP_ERROR(mNodeHandle->get_logger(), "Parameter set with success\n");
        printf("Parameter set with success\n");
    }
    catch (GenICam::GenericException& ge) {
        RCLCPP_ERROR(mNodeHandle->get_logger(), "Parameter set error %s\n", ge.what());
        printf("Parameter set error %s\n", ge.what());
        mHasCrashed = true;
        return CAM_ERROR;
    }
    catch(...) {
        RCLCPP_ERROR(mNodeHandle->get_logger(), "Parameter set error\n");
        printf("Parameter set error\n");
        mHasCrashed = true;
        return CAM_ERROR;
    }
    return CAM_OK;
}

/**
 * @brief Launches the worker thread that aquires images and push them onto the queue
 * 
 */
void Camera::Run() {
    std::thread(AquireLoop, this).detach();
}

/**
 * @brief Worker thread that aquires images and push them onto the queue
 * 
 * @param pInstantiator pointer to this
 */
void Camera::AquireLoop(Camera* pInstantiator) {
    Arena::IImage* pBuff;
    while (!(pInstantiator->mHasCrashed || pInstantiator->mExtShouldStop)) {
        try {
            pBuff = pInstantiator->mpDevice->GetImage(pInstantiator->mTimeout);
            if(pBuff->HasImageData()) {
                std::thread(ProcessImage, pInstantiator, pBuff).detach();
            }
            else {
                // TODO log
            }
        }
        catch(...) {
            pInstantiator->mHasCrashed = true;
            // TODO log
            return;
        }
    }
    
}

/**
 * @brief Actually just pushes the image on the queue
 * 
 * @param pInstantiator pointer to this
 * @param pBuff buffer to process
 */
void Camera::ProcessImage(Camera* pInstantiator, Arena::IImage* pBuff) {
    if(pInstantiator->mHasCrashed || pInstantiator->mExtShouldStop) return;
    Arena::IImage* pImg = Arena::ImageFactory::Copy(pBuff);

    std::unique_lock<std::mutex> lc(pInstantiator->mQueueMtx);
    pInstantiator->mImages.push(pImg);
    lc.unlock();

    pInstantiator->mpDevice->RequeueBuffer(pBuff);
}

/**
 * @brief Attempts to reset a camera to a normal state
 * 
 */
void Camera::ResetDevice() {
    if(mpDevice != nullptr) {
        try {
            // To test
            if(mpDevice != nullptr)
                Arena::SetNodeValue<bool>(mpDevice->GetNodeMap(), "DeviceReset", true);
        }
        catch (...) {

        }
    }
}

/**
 * @brief Return the camera's name
 * 
 * @return const std::string& 
 */
const std::string& Camera::GetName() {
    return mName;
}