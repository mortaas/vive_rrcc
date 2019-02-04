#include "vive_bridge/survive_interface.h"
#include "vive_bridge/survive_types.h"


// Default functions for logging
inline void DefaultDebugMsgCallback(const std::string &msg) {
    std::cerr << "VR Debug: " << msg << std::endl;
}
inline void DefaultInfoMsgCallback(const std::string &msg) {
    std::cerr << "VR Info: " << msg << std::endl;
}
inline void DefaultWarnMsgCallback(const std::string &msg) {
    std::cerr << "VR Warn: " << msg << std::endl;
}
inline void DefaultErrorMsgCallback(const std::string &msg) {
    std::cerr << "VR Error: " << msg << std::endl;
}
inline void DefaultFatalMsgCallback(const std::string &msg) {
    std::cerr << "VR Fatal: " << msg << std::endl;
}

int controller_event_type, controller_device_index;
// struct SurviveObject *controller_so_;

// std::map<vr::TrackedPropertyError, std::string> TrackedPropErrorStrings
// {
//   { vr::TrackedProp_Success, "The property request was successful"}, 
//   { vr::TrackedProp_WrongDataType, "The property was requested with the wrong typed function." },
//   { vr::TrackedProp_WrongDeviceClass, "The property was requested on a tracked device with the wrong class." },
//   { vr::TrackedProp_BufferTooSmall, "The string property will not fit in the provided buffer. The buffer size needed is returned." },
//   { vr::TrackedProp_UnknownProperty, "The property enum value is unknown." },
//   { vr::TrackedProp_InvalidDevice, "The tracked device index was invalid." },
//   { vr::TrackedProp_CouldNotContactServer, "OpenVR could not contact vrserver to query the device for this property." },
//   { vr::TrackedProp_ValueNotProvidedByDevice, "The driver for this device returned that it does not provide this specific property for this device." },
//   { vr::TrackedProp_StringExceedsMaximumLength, "The string property value returned by a driver exceeded the maximum property length of 32k." }
// };

void button_process(SurviveObject * so, uint8_t eventType, uint8_t buttonId, uint8_t axis1Id, uint16_t axis1Val, uint8_t axis2Id, uint16_t axis2Val)
{
	survive_default_button_process(so, eventType, buttonId, axis1Id, axis1Val, axis2Id, axis2Val);

    // controller_device_index = (int)so->codename[2] - '0';
    controller_device_index = 3;

    switch(eventType) {
        case BUTTON_EVENT_BUTTON_DOWN : 
            controller_event_type = 200; // ButtonPress
            break;
        case BUTTON_EVENT_BUTTON_UP :
            controller_event_type = 201; // ButtonUnpress
            break;
        case BUTTON_EVENT_AXIS_CHANGED :
            if (axis1Val + axis2Val > 0) {
                controller_event_type = 202; // ButtonTouch
            } else {
                controller_event_type = 203; // ButtonUntouch
            }
            break;
        default:
            controller_event_type = 0;
    }

	// printf("ButtonEntry: eventType:%x, buttonId:%d, axis1:%d, axis1Val:%8.8x, axis2:%d, axis2Val:%8.8x\n",
	// 	eventType,
	// 	buttonId,
	// 	axis1Id,
	// 	axis1Val,
	// 	axis2Id,
	// 	axis2Val);
    
	// // Note: the code for haptic feedback is not yet written to support wired USB connections to the controller.  
	// // Work is still needed to reverse engineer that USB protocol.

	// // let's do some haptic feedback if the trigger is pushed
	// if (buttonId == 24 && eventType == 1) // trigger engage
	// {
	// 	for (int j = 0; j < 6; j++)
	// 	{
	// 		for (int i = 0; i < 0x5; i++)
	// 		{
	// 			survive_haptic(so, 0, 0xf401, 0xb5a2, 0x0100);
	// 			//survive_haptic(so, 0, 0xf401, 0xb5a2, 0x0100);
	// 			OGUSleep(1000);
	// 		}
	// 		OGUSleep(20000);
	// 	}
	// }

	// // let's do some haptic feedback if the touchpad is pressed.
	// if (buttonId == 2 && eventType == 1) // trigger engage
	// {
	// 	for (int j = 0; j < 6; j++)
	// 	{
	// 		for (int i = 0; i < 0x1; i++)
	// 		{
	// 			survive_haptic(so, 0, 0xf401, 0x05a2, 0xf100);
	// 			//survive_haptic(so, 0, 0xf401, 0xb5a2, 0x0100);
	// 			OGUSleep(5000);
	// 		}
	// 		OGUSleep(20000);
	// 	}
	// }
}

ViveInterface::ViveInterface()
    : VR_DEBUG(DefaultDebugMsgCallback),
      VR_INFO(DefaultInfoMsgCallback),
      VR_WARN(DefaultWarnMsgCallback),
      VR_ERROR(DefaultErrorMsgCallback),
      VR_FATAL(DefaultFatalMsgCallback)
{
    device_classes_.fill(0);
    device_names_.fill("UNKNOWN");
}
ViveInterface::~ViveInterface() {
}

bool ViveInterface::Init(int argc, char **argv) {
      /**
     * Initialize the OpenVR API and get access to the vr::IVRSystem interface.
     * The vr::IVRSystem interface provides access to display configuration information, 
     * tracking data, distortion functions, controller state, events, and device properties.
     */

	// vr::EVRInitError peError = vr::VRInitError_None;
	// pHMD_ = vr::VR_Init(&peError, vr::VRApplication_Background);
    
	// if (peError != vr::VRInitError_None) {
	// 	pHMD_ = nullptr;

    //     VR_FATAL("OpenVR API initialization failed: " + std::string(vr::VR_GetVRInitErrorAsEnglishDescription(peError) ) );
	// 	return false;
	// }

    // VR_INFO("OpenVR API initialization succeeded");
    // return true;

    actx_ = survive_simple_init(argc, argv);

    survive_install_button_fn(actx_->ctx, button_process);

    // Check if libsurvive is initialized properly
    if (actx_ == nullptr) {
        VR_FATAL("libsurvive initialization failed");

        return false;
    }

    survive_simple_start_thread(actx_);

    if (survive_simple_is_running(actx_) ) {
        const SurviveSimpleObject *fsao_= survive_simple_get_first_object(actx_);

        for (const SurviveSimpleObject *it_ = fsao_; it_ != nullptr;
                                        it_ = survive_simple_get_next_object(actx_, it_) )
        {
            std::ptrdiff_t i = std::distance(fsao_, it_);

            device_names_[i] = survive_simple_object_name(it_);
            device_classes_[i] = GetSurviveClass(device_names_[i] );
        }
    } else {
        VR_FATAL("libsurvive initialization failed");

        return false;
    }

    VR_INFO("libsurvive initialization succeeded");
}

void ViveInterface::Shutdown() {
      /**
     * Shuts down the connection to the VR hardware and cleans up the OpenVR API. 
     * The vr::IVRSystem pointer returned by vr::VR_Init will be invalid after this call is made.
     */
    
    // Check if the OpenVR API is initialized
    // if (pHMD_) {
	// 	vr::VR_Shutdown();
	// 	pHMD_ = nullptr;
	// }

    if (actx_) {
        survive_simple_close(actx_);
        actx_ = nullptr;
    }
}

// std::string ViveInterface::GetStringProperty(vr::TrackedDeviceIndex_t unDeviceIndex, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *pError) {
//       /**
//      * Returns the static string property of a tracked device.
//      *  
//      * vr::TrackedDeviceIndex unDeviceIndex - Index of the device to get the property for.
//      * vr::TrackedDeviceProperty prop - Which property to get.
//      */
    
//     uint32_t unBufferSize = pHMD_->GetStringTrackedDeviceProperty(unDeviceIndex, prop, nullptr, 0, pError);
    
//     // Return empty string if the property is empty
//     if (unBufferSize == 0) {
//         return "";
//     }
    
//     // Get string property
//     char *pchValue = new char[unBufferSize];
//     unBufferSize = pHMD_->GetStringTrackedDeviceProperty(unDeviceIndex, prop, pchValue, unBufferSize, pError);
//     std::string strValue = pchValue;
//     delete [] pchValue;

//     return strValue;
// }

void ViveInterface::GetControllerState(const int &device_index, std::vector<float> &axes, std::vector<int> &buttons) {
      /**
     * Get controller state and map it to axes and buttons
     */

    // pHMD_->GetControllerState(device_index, &controller_state_, sizeof(vr::VRControllerState_t) );

    if (survive_simple_is_running(actx_) ) {
        const struct SurviveObject *so_ = actx_->objects[device_index].data.so;

        axes[0] = so_->axis1 / 32640.;
        axes[1] = so_->axis2 / 32767.;
        axes[2] = so_->axis3 / 32767.;

        buttons.assign(13, 0);
        if ( so_->buttonmask &  1)       { buttons[3] = 1; } // Trigger
        if ((so_->buttonmask &  6) == 6) { buttons[2] = 1; } // Touchpad
        if ((so_->buttonmask & 16) >> 4) { buttons[1] = 1; } // Grip
        if ((so_->buttonmask & 32) >> 5) { buttons[0] = 1; } // Menu
    } else {
        axes.assign(3, 0.);
        buttons.assign(13, 0);
    }
}

bool ViveInterface::PollNextEvent(int &event_type, int &device_index) {
      /**
     * Returns true if there is any event waiting in the event queue,
     * and also returns the event type and device index of this event.
     * Returns event type VREvent_None (0) and device index k_unTrackedDeviceIndexInvalid (4294967295)
     * if there is no event waiting in the event queue.
     */

    // if (!pHMD_->PollNextEvent(&event_, sizeof(event_) ) ) {
    //     event_type = vr::VREvent_None;
    //     device_index = vr::k_unTrackedDeviceIndexInvalid;

    //     return false;
    // } else {
    //     event_type = event_.eventType;
    //     device_index = event_.trackedDeviceIndex;

    //     return true;
    // }

    // std::ptrdiff_t i = std::distance(controller_so_->ctx->objs[0], controller_so_);
    // VR_INFO(std::to_string(actx_->objects[3].data.so->) );

    event_type = controller_event_type;
    device_index = controller_device_index;

    controller_event_type = 0;
    controller_device_index = 0xFFFFFFFF;

    return false;
}

int ViveInterface::GetSurviveClass(const std::string &device_name) {
    if (boost::algorithm::contains(device_name, "HMD") ||
        boost::algorithm::contains(device_name, "T20") )
    {
        return 1; // HMD
    }
    else if (boost::algorithm::contains(device_name, "WM") ||
             boost::algorithm::contains(device_name, "WW") )
    {
        return 2; // Controller (WW is USB wired)
    }
    else if (boost::algorithm::contains(device_name, "TR") ) {
        return 3; // Tracker
    } else if (boost::algorithm::contains(device_name, "LH") ) {
        return 4; // Lighthouse
    } else if (boost::algorithm::contains(device_name, "RD") ) {
        return 5; // Redirected display
    } else {
        return 0; // No device
    }
}

void ViveInterface::GetDeviceSN(const int &device_index, std::string &device_sn) {
    /**
     * Get the serial number of a tracked device
     */

    // vr::TrackedPropertyError pError = vr::TrackedProp_UnknownProperty;

    // device_sn = GetStringProperty(device_index, vr::Prop_SerialNumber_String, &pError);
    // if (pError != vr::TrackedProp_Success) {
    //     VR_ERROR("Error occurred when getting serial number from tracked device: " + TrackedPropErrorStrings[pError] );
    // }

    device_sn = device_names_[device_index];

    // const struct SurviveObject *so_ = actx_->objects[device_index].data.so;
}

void ViveInterface::GetDevicePose(const int &device_index, float m[3][4]) {
    /**
     * Get the pose of a tracked device
     * This pose is represented as the top 3 rows of a homogeneous transformation matrix
     */

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 4; j++)
            m[i][j] = device_poses_[device_index].mDeviceToAbsoluteTracking.m[i][j];
}
void ViveInterface::GetDeviceVelocity(const int &device_index, float linear[3], float angular[3]) {
    /**
     * Get the linear and angular velocity (twist) of a tracked device
     */

    for (int i = 0; i < 3; i++) {
        linear[i] = device_poses_[device_index].vVelocity.v[i];
        angular[i] = device_poses_[device_index].vAngularVelocity.v[i];
    }
}

int ViveInterface::GetDeviceClass(const int &device_index) {
    /**
     * Get the class of a tracked device
     */

    // return pHMD_->GetTrackedDeviceClass(device_index);

    return GetSurviveClass(device_names_[device_index] );
}

bool ViveInterface::PoseIsValid(const int &device_index) {
    /**
     * Check if the pose of a tracked device is valid
     */

    // return (device_poses_[device_index].eTrackingResult == vr::TrackingResult_Running_OK && 
    //         device_poses_[device_index].bPoseIsValid &&
    //         device_poses_[device_index].bDeviceIsConnected);

    // if (device_names_[device_index] != "UNKNOWN") {
    //     return true;
    // } else {
    //     return false;
    // }

    // Check if the device is populated
    if ((actx_->object_ct - device_index) >= 0) {
        return true;
    } else {
        return false;
    }
}

void ViveInterface::Update() {
    /*
    * Calculates updated poses for all tracked devices
    * TrackingUniverseRawAndUncalibrated - provides poses relative to the hardware-specific coordinate system in the driver
    */
    
    // pHMD_->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseSeated, 0, device_poses_, vr::k_unMaxTrackedDeviceCount);

    if (survive_simple_is_running(actx_) ) {
        const SurviveSimpleObject *fsao_;
        fsao_ = survive_simple_get_first_object(actx_);

        for (const SurviveSimpleObject *it_ = survive_simple_get_next_updated(actx_); it_ != nullptr;
			                            it_ = survive_simple_get_next_updated(actx_) )
        {
            std::ptrdiff_t i = std::distance(fsao_, it_);

            uint32_t timecode = survive_simple_object_get_latest_pose(it_, &survive_pose_);

            // Convert survive pose to OpenVR 3x4 matrix
            PoseToMatrix(ogl_pose, &survive_pose_);
            std::copy(ogl_pose, ogl_pose + 16, &device_poses_[i].mDeviceToAbsoluteTracking.m[0][0]);

            const SurvivePose *imu_pose_ = &it_->data.so->OutPoseIMU;
            printf("%s (%u): %f %f %f %f %f %f %f\n", survive_simple_object_name(it_), timecode, imu_pose_->Pos[0],
			       imu_pose_->Pos[1], imu_pose_->Pos[2], imu_pose_->Rot[0], imu_pose_->Rot[1], imu_pose_->Rot[2], imu_pose_->Rot[3]);
        }
    }
}

void ViveInterface::TriggerHapticPulse(const int &device_index, const int &axis_id, int duration) {
    /*
    * Triggers a single haptic pulse on a controller given its device index, and the pulse duration 0-3999 µs ("strength")
    */

    // if (GetDeviceClass(device_index) == vr::TrackedDeviceClass_Controller) {
    //     // 
    //     duration = std::min(duration, 3999);
    //     duration = std::max(duration, 0);

    //     pHMD_->TriggerHapticPulse(device_index, axis_id, duration);
    // }
}

// Logging to ROS
void ViveInterface::SetDebugMsgCallback(DebugMsgCallback fn) { VR_DEBUG = fn; }
void ViveInterface::SetInfoMsgCallback(InfoMsgCallback fn) { VR_INFO = fn; }
void ViveInterface::SetWarnMsgCallback(WarnMsgCallback fn) { VR_WARN = fn; }
void ViveInterface::SetErrorMsgCallback(ErrorMsgCallback fn) { VR_ERROR = fn; }
void ViveInterface::SetFatalMsgCallback(FatalMsgCallback fn) { VR_FATAL = fn; }