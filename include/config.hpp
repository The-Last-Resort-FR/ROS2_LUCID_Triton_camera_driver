#pragma once

#define PARAM_FIELDS_DEC \
    X(mode, uint8_t)\
    X(TriggerSelector, std::string) \
    X(TriggerMode, std::string) \
    X(TriggerSource, std::string) \
    X(TriggerActivation, std::string) \
    X(TriggerOverlap, std::string) \
    X(LineMode, std::string) \
    X(ExposureAuto, std::string) \
    X(ExposureTime, double) \
    X(Gain, double) \
    X(GainAuto, std::string) \
    X(Width, int64_t) \
    X(Height, int64_t) \
    X(OffsetX, int64_t) \
    X(OffsetY, int64_t) \
    X(PixelFormat, std::string)



#define PARAM_FIELDS_ARENA \
    X(ExposureTime, double) \
    X(Gain, double) \
    X(Width, int64_t) \
    X(Height, int64_t) \
    X(OffsetX, int64_t) \
    X(OffsetY, int64_t) \

#define PARAM_FIELDS_ARENA_GCSTRING \
    X(TriggerSelector, GenICam::gcstring) \
    X(TriggerMode, GenICam::gcstring) \
    X(TriggerSource, GenICam::gcstring) \
    X(TriggerActivation, GenICam::gcstring) \
    X(TriggerOverlap, GenICam::gcstring) \
    X(LineMode, GenICam::gcstring) \
    X(ExposureAuto, GenICam::gcstring) \
    X(GainAuto, GenICam::gcstring) \
    X(PixelFormat, GenICam::gcstring)



#define DECLARE_PARAM(obj, field, nh) nh->declare_parameter(#field, obj.field)
#define GET_PARAMS(obj, field, nh) nh->get_parameter(#field, obj.field)
#define SET_PARAMS(obj, field, type, dev) Arena::SetNodeValue<type>( dev->GetNodeMap(), #field, obj.field)
#define SET_PARAMS_GCSTRING(obj, field, type, dev) Arena::SetNodeValue<type>( dev->GetNodeMap(), #field, GenICam::gcstring(obj.field.c_str()))
    