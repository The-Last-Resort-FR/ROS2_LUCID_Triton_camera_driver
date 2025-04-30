/**
 * @file camera.hpp
 * @author tlr
 * @brief Camera class that gets images into a queue and check for errors
 * @version 0.2.1
 * @date 2025-04-30
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#pragma once

// Return constants
constexpr bool CAM_OK = false;
constexpr bool  CAM_ERROR = true;

// STL includes
#include <cstdint>
#include <string>
#include <thread>
#include <mutex>
#include <queue>

// Lib includes
#include <rclcpp/rclcpp.hpp>
#include <Arena/ArenaApi.h>

// User includes
#include "config.hpp"

// NodeParameters structure for the paramters from .yaml
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
    std::string mName;
public:
    Camera(rclcpp::Node::SharedPtr nodeHandle, const uint64_t& timeout, const bool& pExtShouldStop, const NodeParameters& nodeParameters, std::string name);
    ~Camera();
    const bool& GetStatus();
    bool SetDevice(Arena::IDevice* pDevice);
    std::queue<Arena::IImage*>& GetImageQueue();
    bool SetParameters();
    void Run();
    static void AquireLoop(Camera* pInstantiator);
    static void ProcessImage(Camera* pInstantiator, Arena::IImage* pBuff);
    void ResetDevice();
    const std::string& GetName();
};

