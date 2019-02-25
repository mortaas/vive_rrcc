#ifndef _SURVIVE_INTERFACE_H_
#define _SURVIVE_INTERFACE_H_

#include <openvr.h>

#include <survive.h>
#include <survive_api.h>
#include "vive_bridge/survive_types.h"

// Boost
#include <boost/algorithm/string/predicate.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

// STL
#include <algorithm>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>
#include <queue>

// ROS logging
typedef boost::function<void(const std::string&)> DebugMsgCallback;
typedef boost::function<void(const std::string&)> InfoMsgCallback;
typedef boost::function<void(const std::string&)> WarnMsgCallback;
typedef boost::function<void(const std::string&)> ErrorMsgCallback;
typedef boost::function<void(const std::string&)> FatalMsgCallback;

class ViveInterface {
    // OpenVR
    vr::TrackedDevicePose_t device_poses_[8];

    // libsurvive
    SurviveSimpleContext *actx_;

    SurvivePose survive_pose_;
    FLT ogl_pose[16];

    std::array<int, 8> device_classes_;
    std::array<std::string, 8> device_names_;
    std::array<std::string, 8> device_serials_;

    // Callback functions for ROS logging
    DebugMsgCallback VR_DEBUG;
    InfoMsgCallback VR_INFO;
    WarnMsgCallback VR_WARN;
    ErrorMsgCallback VR_ERROR;
    FatalMsgCallback VR_FATAL;

    void PollNextMessage();

    public:
        ViveInterface();
        ~ViveInterface();

        bool Init(int argc, char **argv);
        void Update();
        void Shutdown();

        // libsurvive
        int SurviveClassToOpenVR(const std::string &device_name);

        bool PoseIsValid(const int &device_index);
        void GetControllerState(const int &device_index, std::vector<float> &axes, std::vector<int> &buttons);
        int GetDeviceClass(const int &device_index);
        void GetDevicePose(const int &device_index, float m[3][4]);
        void GetDeviceSN(const int &device_index, std::string &device_sn);
        void GetDeviceVelocity(const int &device_index, float linear[3], float angular[3]);
        bool PollNextEvent(int &event_type, int &device_index);
        void TriggerHapticPulse(const int &device_index, const int &axis_id, int duration);

        // ROS logging
        void SetDebugMsgCallback(DebugMsgCallback fn);
        void SetInfoMsgCallback(InfoMsgCallback fn);
        void SetWarnMsgCallback(WarnMsgCallback fn);
        void SetErrorMsgCallback(ErrorMsgCallback fn);
        void SetFatalMsgCallback(FatalMsgCallback fn);
};

#endif  // _SURVIVE_INTERFACE_H_