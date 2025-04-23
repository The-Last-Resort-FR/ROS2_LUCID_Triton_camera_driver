#pragma once

#define CAM_OK false
#define CAM_ERROR true

#include <cstdint>
#include <string>
#include <thread>
#include <mutex>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <Arena/ArenaApi.h>

#include <config.hpp>

#define X(name, type) type name;
struct NodeParameters {
    PARAM_FIELDS_DEC
};
#undef X


class Camera {
private:
    rclcpp::Node::SharedPtr mNodeHandle;
    const uint64_t& mTimeout;
    const bool& mExtShouldStop;
    const NodeParameters& mNodeParameters;
    Arena::IDevice* mpDevice;
    bool mHasCrashed;
    std::mutex mQueueMtx;
    std::queue<Arena::IImage*> mImages;

public:
    Camera(rclcpp::Node::SharedPtr nodeHandle, const uint64_t& timeout, const bool& pExtShouldStop, const NodeParameters& nodeParameters);
    ~Camera();
    const bool& GetStatus();
    bool SetDevice(Arena::IDevice* pDevice);
    std::queue<Arena::IImage*>& GetImageQueue();
    bool SetParameters();
    void Run();
    static void AquireLoop(Camera* pInstantiator);
    static void ProcessImage(Camera* pInstantiator, Arena::IImage* pBuff);
    void ResetDevice();
};

