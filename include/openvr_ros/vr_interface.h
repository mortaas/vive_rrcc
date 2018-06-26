#ifndef _VR_INTERFACE_H_
#define _VR_INTERFACE_H_

#include <openvr.h>

#include <array>
#include <string>
#include <vector>
#include <iostream>

class VRInterface {
        // OpenVR
        vr::IVRSystem *pHMD_;
        vr::TrackedDevicePose_t device_poses_[vr::k_unMaxTrackedDeviceCount];
        uint32_t device_count;

        std::string GetStringProperty(vr::TrackedDeviceIndex_t unDeviceIndex, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *pError = nullptr);

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
};

#endif  // _VR_INTERFACE_H_