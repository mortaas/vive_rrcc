#ifndef _VR_INTERFACE_H_
#define _VR_INTERFACE_H_

#include <openvr.h>
#include <boost/function.hpp>

// STL
#include <iostream>
#include <string>
#include <vector>
#include <map>


// ROS logging
typedef boost::function<void(const std::string&)> DebugMsgCallback;
typedef boost::function<void(const std::string&)> InfoMsgCallback;
typedef boost::function<void(const std::string&)> WarnMsgCallback;
typedef boost::function<void(const std::string&)> ErrorMsgCallback;
typedef boost::function<void(const std::string&)> FatalMsgCallback;


class ViveInterface {
    // OpenVR
    vr::IVRSystem *pHMD_;
    vr::IVRTrackedCamera *pCamera_;
    vr::VREvent_t event_;
    vr::VRControllerState_t controller_state_;
    vr::TrackedDevicePose_t device_poses_[vr::k_unMaxTrackedDeviceCount];
    
    std::string GetStringProperty(vr::TrackedDeviceIndex_t unDeviceIndex, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *pError);

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

        // OpenVR
        bool PollNextEvent(unsigned int &event_type, unsigned int &device_index);
        void GetControllerState(const unsigned int &device_index, std::vector<float> &axes, std::vector<int> &buttons);
        void TriggerHapticPulse(const unsigned int &device_index, const unsigned short &duration);

        bool PoseIsValid(const unsigned int &device_index);
        unsigned char GetDeviceClass(const unsigned int &device_index);
        void GetDevicePose(const unsigned int &device_index, float m[3][4]);
        unsigned char GetControllerRole(const unsigned int &device_index);
        void GetDeviceSN(const unsigned int &device_index, std::string &device_sn);
        void GetDeviceVelocity(const unsigned int &device_index, float linear[3], float angular[3]);

        // ROS logging
        void SetDebugMsgCallback(DebugMsgCallback fn);
        void SetInfoMsgCallback(InfoMsgCallback fn);
        void SetWarnMsgCallback(WarnMsgCallback fn);
        void SetErrorMsgCallback(ErrorMsgCallback fn);
        void SetFatalMsgCallback(FatalMsgCallback fn);
};

#endif  // _VR_INTERFACE_H_