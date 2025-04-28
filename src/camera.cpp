#include "camera.hpp"
Camera::Camera(rclcpp::Node::SharedPtr nodeHandle, const uint64_t& timeout, const bool& pExtShouldStop, const NodeParameters& nodeParameters, std::string name) \
: mNodeHandle(nodeHandle), mTimeout(timeout), mExtShouldStop(pExtShouldStop), mNodeParameters(nodeParameters), mpDevice(nullptr), mHasCrashed(false), mName(name) {

}

Camera::~Camera() {
    mHasCrashed = true;
    ResetDevice();
    std::this_thread::sleep_for(std::chrono::milliseconds(mTimeout + 200));
}

const bool& Camera::GetStatus() {
    return mHasCrashed;
}

bool Camera::SetDevice(Arena::IDevice* pDevice) {
    if(pDevice == nullptr) return CAM_ERROR;
    mpDevice = pDevice;
    return CAM_OK;
}

std::queue<Arena::IImage*>& Camera::GetImageQueue() {
    return mImages;
}

bool Camera::SetParameters() {
    try {
        if(mHasCrashed || mpDevice == nullptr) {
            throw std::runtime_error("Setting parameters of a crashed or empty camera");
        }
        /*
        #define X(field, type) SET_PARAMS(mNodeParameters, field, type, mpDevice);
                PARAM_FIELDS_ARENA
        #undef X
        */
       //BalanceWhiteAuto
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
        Arena::SetNodeValue<GenICam::gcstring>(mpDevice->GetNodeMap(), "BalanceWhiteAuto", "Off");
        //Arena::SetNodeValue<int64_t>(mpDevice->GetNodeMap(), "Width", mNodeParameters.Width);
        //Arena::SetNodeValue<int64_t>(mpDevice->GetNodeMap(), "Height", mNodeParameters.Height);
        Arena::SetNodeValue<int64_t>(mpDevice->GetNodeMap(), "OffsetX", mNodeParameters.OffsetX);
        Arena::SetNodeValue<int64_t>(mpDevice->GetNodeMap(), "OffsetY", mNodeParameters.OffsetY);

        
        /*
        #define X(field, type) SET_PARAMS_GCSTRING(mNodeParameters, field, type, mpDevice);
            PARAM_FIELDS_ARENA_GCSTRING
        #undef X
        */
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

void Camera::Run() {
    std::thread(AquireLoop, this).detach();
}

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

void Camera::ProcessImage(Camera* pInstantiator, Arena::IImage* pBuff) {
    if(pInstantiator->mHasCrashed || pInstantiator->mExtShouldStop) return;
    Arena::IImage* pImg = Arena::ImageFactory::Copy(pBuff);

    std::unique_lock<std::mutex> lc(pInstantiator->mQueueMtx);
    pInstantiator->mImages.push(pImg);
    lc.unlock();

    pInstantiator->mpDevice->RequeueBuffer(pBuff);
}

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

const std::string& Camera::GetName() {
    return mName;
}