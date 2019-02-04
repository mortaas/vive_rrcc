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

void ViveInterface::button_process(SurviveObject * so, uint8_t eventType, uint8_t buttonId, uint8_t axis1Id, uint16_t axis1Val, uint8_t axis2Id, uint16_t axis2Val)
{
	survive_default_button_process(so, eventType, buttonId, axis1Id, axis1Val, axis2Id, axis2Val);

    // eventType 1: button press
    // eventType 2: button unpress
    // eventType 3: axis interact

    // [BUTTONS]
    // Trackpad - buttonId 1
    // System - buttonId 3
    // Grip - buttonId 4
    // Menu - buttonId 5
    // Trigger - buttonId 24

    // [AXIS]
    // axis1:1 Trigger
    // axis1:2 Trackpad x
    // axis2:3 Trackpad y

    // std::ptrdiff_t i = std::distance(*so_->ctx->objs, so_);
    // controller_states_[i].ulButtonPressed = 0;

    // Axes
    // axes[0] = controller_state_.rAxis[0].x;
    // axes[1] = controller_state_.rAxis[0].y;
    // axes[2] = controller_state_.rAxis[1].x;

    // Buttons
    // inline uint64_t ButtonMaskFromId( EVRButtonId id ) { return 1ull << id; }
    // if (vr::ButtonMaskFromId(vr::k_EButton_ApplicationMenu) & controller_state_.ulButtonPressed) {
    //     buttons[0] = 1;
    // }
    // if (vr::ButtonMaskFromId(vr::k_EButton_Dashboard_Back) & controller_state_.ulButtonPressed) {
    //     buttons[1] = 1;
    // }
    // if (vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Touchpad) & controller_state_.ulButtonPressed) {
    //     buttons[2] = 1;
    // }
    // if (vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Trigger) & controller_state_.ulButtonPressed) {
    //     buttons[3] = 1;
    // }

	// do nothing.
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

    // auto mem_button_process = std::mem_fn(&ViveInterface::button_process);
    // button_process_func *button_process_ptr = *this->button_process;
    // survive_install_button_fn(actx_->ctx, &ViveInterface::button_process);

    // Logging to ROS
    // survive_install_error_fn(actx_->ctx, VR_ERROR);

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

    // Axes
    // axes[0] = controller_state_.rAxis[0].x;
    // axes[1] = controller_state_.rAxis[0].y;
    // axes[2] = controller_state_.rAxis[1].x;

    axes[0] = 0.;
    axes[1] = 0.;
    axes[2] = 0.;

    // Buttons
    buttons.assign(13, 0);
    // if (vr::ButtonMaskFromId(vr::k_EButton_ApplicationMenu) & controller_state_.ulButtonPressed) {
    //     buttons[0] = 1;
    // }
    // if (vr::ButtonMaskFromId(vr::k_EButton_Dashboard_Back) & controller_state_.ulButtonPressed) {
    //     buttons[1] = 1;
    // }
    // if (vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Touchpad) & controller_state_.ulButtonPressed) {
    //     buttons[2] = 1;
    // }
    // if (vr::ButtonMaskFromId(vr::k_EButton_SteamVR_Trigger) & controller_state_.ulButtonPressed) {
    //     buttons[3] = 1;
    // }
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

    event_type = 0;
    device_index = 0xFFFFFFFF;

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

    // device_sn = device_names_[device_index];

    const struct SurviveObject *so_ = actx_->objects[device_index].data.so;
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
            PoseToMatrix((double*) ogl_pose, &survive_pose_);
            matrix44transpose((double *) ogl_transpose, (double *) ogl_pose);
            memcpy(device_poses_[i].mDeviceToAbsoluteTracking.m, ogl_transpose, sizeof(float) * 16);
        }
    }
}

// bool ViveInterface::ConvertSurvivePose(SurvivePose &pose_, float m[3][4]) {
//     /*
//     * Convert SurvivePose (quaternion) to 3x4 transformation matrix
//     */

//     double d = pose_.Rot[0] * pose_.Rot[0] +
//                pose_.Rot[1] * pose_.Rot[1] +
//                pose_.Rot[2] * pose_.Rot[2] +
//                pose_.Rot[3] * pose_.Rot[3];

//     if (d != 0.) {
//         double s = 2. / d;

//         double xs = pose_.Rot[1] * s,   ys = pose_.Rot[2] * s,   zs = pose_.Rot[3] * s;
//         double wx = pose_.Rot[0] * xs,  wy = pose_.Rot[0] * ys,  wz = pose_.Rot[0] * zs;
//         double xx = pose_.Rot[1] * xs,  xy = pose_.Rot[1] * ys,  xz = pose_.Rot[1] * zs;
//         double yy = pose_.Rot[2] * ys,  yz = pose_.Rot[2] * zs,  zz = pose_.Rot[3] * zs;

//         m[0][0] = 1.0 - (yy + zz);  m[0][1] =         xy - wz;  m[0][2] =         xz + wy;
//         m[1][0] =         xy + wz;  m[1][1] = 1.0 - (xx + zz);  m[1][2] =         yz - wx;
//         m[2][0] =         xz - wy;  m[2][1] =         yz + wx;  m[2][2] = 1.0 - (xx + yy);

//         m[0][3] = pose_.Pos[0];
//         m[1][3] = pose_.Pos[1];
//         m[2][3] = pose_.Pos[2];
//     } else {
//         m[0][0] = 1; m[0][1] = 0; m[0][2] = 0;
//         m[1][0] = 0; m[1][1] = 1; m[1][2] = 0;
//         m[2][0] = 0; m[2][1] = 0; m[2][2] = 1;

//         m[0][3] = 0;
//         m[1][3] = 0;
//         m[2][3] = 0;
//     }
// }

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