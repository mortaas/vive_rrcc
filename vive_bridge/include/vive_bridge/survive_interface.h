#ifndef _SURVIVE_INTERFACE_H_
#define _SURVIVE_INTERFACE_H_

// STL
#include <algorithm>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>
#include <queue>

// Boost
#include <boost/algorithm/string/predicate.hpp>
#include <boost/function.hpp>
#include <boost/bind.hpp>

// libsurvive
#include <survive.h>
#include <survive_api.h>


// Maximum number of tracked devices to keep track of
const unsigned char MAX_TRACKED_DEVICES = 8;

struct TrackedDeviceData
{
    // Meta
    std::string survive_name;
    unsigned char device_class;
    std::string serial_number;

    // Pose and velocities
	float pose[3][4];
	float linear_vel[3];
	float angular_vel[3];

    // Controller input
    unsigned long button_flags;
    float axes[10];
};


// ROS logging
typedef boost::function<void(const std::string&)> DebugMsgCallback;
typedef boost::function<void(const std::string&)> InfoMsgCallback;
typedef boost::function<void(const std::string&)> WarnMsgCallback;
typedef boost::function<void(const std::string&)> ErrorMsgCallback;
typedef boost::function<void(const std::string&)> FatalMsgCallback;

class ViveInterface {
    // libsurvive
    SurviveSimpleContext *actx_;

    // Tracked device data
    TrackedDeviceData tracked_devices_[MAX_TRACKED_DEVICES];

    FLT pose[16];
    SurvivePose survive_pose_;
    SurviveVelocity survive_velocity_;

    unsigned char SurviveClassToOpenVR(const std::string &device_name);

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
        bool PollNextEvent(unsigned int &event_type, unsigned int &device_index);
        void GetControllerState(const unsigned int &device_index, std::vector<float> &axes, std::vector<int> &buttons);
        void TriggerHapticPulse(const unsigned int &device_index, const unsigned short &duration);

        bool PoseIsValid(const unsigned int &device_index);
        unsigned char GetDeviceClass(const unsigned int &device_index);
        void GetDevicePose(const unsigned int &device_index, float m[3][4]);
        void GetDeviceSN(const unsigned int &device_index, std::string &device_sn);
        void GetDeviceVelocity(const unsigned int &device_index, float linear[3], float angular[3]);

        // ROS logging
        void SetDebugMsgCallback(DebugMsgCallback fn);
        void SetInfoMsgCallback(InfoMsgCallback fn);
        void SetWarnMsgCallback(WarnMsgCallback fn);
        void SetErrorMsgCallback(ErrorMsgCallback fn);
        void SetFatalMsgCallback(FatalMsgCallback fn);
};

#endif  // _SURVIVE_INTERFACE_H_