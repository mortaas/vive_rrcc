#ifndef _SURVIVE_INTERFACE_H_
#define _SURVIVE_INTERFACE_H_

#include <openvr.h>
#include <libsurvive/survive.h>
#include <libsurvive/survive_api.h>

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
#include <map>
#include <queue>

// ROS logging
typedef boost::function<void(const std::string&)> DebugMsgCallback;
typedef boost::function<void(const std::string&)> InfoMsgCallback;
typedef boost::function<void(const std::string&)> WarnMsgCallback;
typedef boost::function<void(const std::string&)> ErrorMsgCallback;
typedef boost::function<void(const std::string&)> FatalMsgCallback;

class ViveInterface {
    // OpenVR
    // vr::IVRSystem *pHMD_;
    // vr::VREvent_t event_;
    vr::VRControllerState_t controller_states_[8];
    vr::TrackedDevicePose_t device_poses_[8];

    // std::string GetStringProperty(vr::TrackedDeviceIndex_t unDeviceIndex, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *pError);

    // libsurvive
    SurviveSimpleContext *actx_;

    SurvivePose survive_pose_;
    FLT ogl_pose[16], ogl_transpose[16];

    std::array<int, 8> device_classes_;
    std::array<std::string, 8> device_names_;

    // Callback functions
    // void button_process(SurviveObject * so, uint8_t eventType, uint8_t buttonId, uint8_t axis1Id, uint16_t axis1Val, uint8_t axis2Id, uint16_t axis2Val);

    // Callback functions for ROS logging
    DebugMsgCallback VR_DEBUG;
    InfoMsgCallback VR_INFO;
    WarnMsgCallback VR_WARN;
    ErrorMsgCallback VR_ERROR;
    FatalMsgCallback VR_FATAL;

    public:
        ViveInterface();
        ~ViveInterface();

        bool Init(int argc, char **argv);
        void Update();
        void Shutdown();

        // libsurvive
        // bool ConvertSurvivePose(SurvivePose &pose_, float m[3][4]);
        int GetSurviveClass(const std::string &device_name);

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