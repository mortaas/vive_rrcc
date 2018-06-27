#ifndef _VR_INTERFACE_H_
#define _VR_INTERFACE_H_

#include <openvr.h>
#include <boost/function.hpp>

// #include <array>
#include <string>
#include <vector>
#include <iostream>

// ROS logging
typedef boost::function<void(const std::string&)> DebugMsgCallback;
typedef boost::function<void(const std::string&)> InfoMsgCallback;
typedef boost::function<void(const std::string&)> WarnMsgCallback;
typedef boost::function<void(const std::string&)> ErrorMsgCallback;
typedef boost::function<void(const std::string&)> FatalMsgCallback;

class VRInterface {
    // OpenVR
    vr::IVRSystem *pHMD_;
    vr::TrackedDevicePose_t device_poses_[vr::k_unMaxTrackedDeviceCount];

    std::string GetStringProperty(vr::TrackedDeviceIndex_t unDeviceIndex, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *pError);

    // Private callback functions for ROS logging
    DebugMsgCallback VR_DEBUG;
    InfoMsgCallback VR_INFO;
    WarnMsgCallback VR_WARN;
    ErrorMsgCallback VR_ERROR;
    FatalMsgCallback VR_FATAL;

    public:
        VRInterface();
        ~VRInterface();

        bool Init();
        void Update();
        void Shutdown();

        bool PoseIsValid(int device_index);
        void GetControllerState(int device_index, std::vector<float> &axes, std::vector<int> &buttons);
        int GetDeviceClass(int device_index);
        void GetDevicePose(int device_index, float m[3][4]);
        std::string GetDeviceSN(int device_index);
        void GetDeviceVelocity(int device_index, float linear[3], float angular[3]);

        // ROS logging
        void SetDebugMsgCallback(DebugMsgCallback fn);
        void SetInfoMsgCallback(InfoMsgCallback fn);
        void SetWarnMsgCallback(WarnMsgCallback fn);
        void SetErrorMsgCallback(ErrorMsgCallback fn);
        void SetFatalMsgCallback(FatalMsgCallback fn);
};

#endif  // _VR_INTERFACE_H_