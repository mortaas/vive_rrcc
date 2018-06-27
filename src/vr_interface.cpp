#include "openvr_ros/vr_interface.h"

inline void defaultDebugMsgCallback(const std::string &msg) {
    std::cerr << "VR Debug: " << msg << std::endl;
}
inline void defaultInfoMsgCallback(const std::string &msg) {
    std::cerr << "VR Info: " << msg << std::endl;
}
inline void defaultWarnMsgCallback(const std::string &msg) {
    std::cerr << "VR Warning: " << msg << std::endl;
}
inline void defaultErrorMsgCallback(const std::string &msg) {
    std::cerr << "VR Error: " << msg << std::endl;
}
inline void defaultFatalMsgCallback(const std::string &msg) {
    std::cerr << "VR Fatal: " << msg << std::endl;
}

VRInterface::VRInterface() 
    : debug_(defaultDebugMsgCallback),
      info_(defaultInfoMsgCallback),
      warn_(defaultWarnMsgCallback),
      error_(defaultErrorMsgCallback),
      fatal_(defaultFatalMsgCallback)
{
    device_count = vr::k_unMaxTrackedDeviceCount;
}
VRInterface::~VRInterface() {
}

bool VRInterface::Init() {
      /**
     * Initialize the OpenVR API and get access to the vr::IVRSystem interface.
     * The vr::IVRSystem interface provides access to display configuration information, 
     * tracking data, distortion functions, controller state, events, and device properties.
     */

	vr::EVRInitError peError = vr::VRInitError_None;
	pHMD_ = vr::VR_Init(&peError, vr::VRApplication_Background);
    
	if (peError != vr::VRInitError_None) {
		pHMD_ = nullptr;

        fatal_("OpenVR API initialization failed: " + std::string(vr::VR_GetVRInitErrorAsEnglishDescription(peError) ) );
		return false;
	}

    info_("OpenVR API initialization succeeded");
    return true;
}

void VRInterface::Shutdown() {
      /**
     * Shuts down the connection to the VR hardware and cleans up the OpenVR API. 
     * The vr::IVRSystem pointer returned by vr::VR_Init will be invalid after this call is made.
     */
    
    // Check if the OpenVR API is initialized, else do nothing
    if (pHMD_) {
		vr::VR_Shutdown();
		pHMD_ = nullptr;
	} else {
        warn_("Attempted to shut down the OpenVR API, but it is not initialized");
    }
}

std::string VRInterface::GetStringProperty(vr::TrackedDeviceIndex_t unDeviceIndex, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *pError) {
      /**
     * Returns a static string property for a tracked device.
     *  
     * vr::TrackedDeviceIndex unDeviceIndex - Index of the device to get the property for.
     * vr::TrackedDeviceProperty prop - Which property to get.
     */

    /**
    * uint32_t GetStringTrackedDeviceProperty(vr::TrackedDeviceIndex_t unDeviceIndex, TrackedDeviceProperty prop,
    *                                         char *pchValue, uint32_t unBufferSize, TrackedPropertyError *pError = 0L)
    * char *pchValue - The buffer to store string properties in.
    * uint32_t unBufferSize - The size of the buffer pointed to by pchValue.
    * TrackedPropertyError *pError - The error returned when attempting to fetch this property.
    * 
    * GetStringTrackedDeviceProperty returns the number of bytes necessary to hold the string, including the trailing null.
    * If the buffer wasn't large enough it passes back TrackedProp_BufferTooSmall as the error and doesn't fill the string into the buffer.
    */
    
    uint32_t unBufferSize = pHMD_->GetStringTrackedDeviceProperty(unDeviceIndex, prop, nullptr, 0, pError);
    
    // Return empty string if the property is empty
    if (unBufferSize == 0) {
        return "";
    }
    
    // Get string property
    char *pchValue = new char[unBufferSize];
    unBufferSize = pHMD_->GetStringTrackedDeviceProperty(unDeviceIndex, prop, pchValue, unBufferSize, pError);
    std::string strValue = pchValue;
    delete [] pchValue;

    return strValue;
}

void VRInterface::GetControllerState(int device_index, std::vector<float> &axes, std::vector<int> &buttons) {
    vr::VRControllerState_t state;
    pHMD_->GetControllerState(device_index, &state, sizeof(vr::VRControllerState_t) );

    for (int i = 0; i < 3; i++) {
        if((1LL << vr::k_EButton_ApplicationMenu) & state.ulButtonPressed)
          buttons[0] = 1;
        if((1LL << vr::k_EButton_SteamVR_Trigger) & state.ulButtonPressed)
          buttons[1] = 1;
        if((1LL << vr::k_EButton_SteamVR_Touchpad) & state.ulButtonPressed)
          buttons[2] = 1;
        if((1LL << vr::k_EButton_Grip) & state.ulButtonPressed)
          buttons[3] = 1;
        // TrackPad axis
        axes[0] = state.rAxis[0].x;
        axes[1] = state.rAxis[0].y;
        // Trigger axis
        axes[2] = state.rAxis[1].x;
    }
}

std::string VRInterface::GetDeviceSN(int device_index) {
    /**
     * Get the serial number of a tracked device
     */

    vr::TrackedPropertyError pError = vr::TrackedProp_UnknownProperty;

    std::string device_sn = GetStringProperty(device_index, vr::Prop_SerialNumber_String, &pError);
    if (pError != vr::TrackedProp_Success) {
        error_("Error occurred when getting serial number from tracked device: " + std::string(pHMD_->GetPropErrorNameFromEnum(pError) ) );
    }

    // !! TODO !!: return tracked property error description instead of name from enum

    // TrackedProp_Success - The property request was successful.
    // TrackedProp_WrongDataType - The property was requested with the wrong typed function.
    // TrackedProp_WrongDeviceClass - The property was requested on a tracked device with the wrong class.
    // TrackedProp_BufferTooSmall - The string property will not fit in the provided buffer. The buffer size needed is returned.
    // TrackedProp_UnknownProperty - The property enum value is unknown.
    // TrackedProp_InvalidDevice - The tracked device index was invalid.
    // TrackedProp_CouldNotContactServer - OpenVR could not contact vrserver to query the device for this property.
    // TrackedProp_ValueNotProvidedByDevice - The driver for this device returned that it does not provide this specific property for this device.
    // TrackedProp_StringExceedsMaximumLength - The string property value returned by a driver exceeded the maximum property length of 32k.

    return device_sn;
}

void VRInterface::GetDevicePose(int device_index, float m[3][4]) {
    /**
     * Get the pose of a tracked device
     * This pose is represented as the top 3 rows of a homogeneous transformation matrix
     */

    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 4; j++)
            m[i][j] = device_poses_[device_index].mDeviceToAbsoluteTracking.m[i][j];
}
void VRInterface::GetDeviceVelocity(int device_index, float linear[3], float angular[3]) {
    /**
     * Get the linear and angular velocity (twist) of a tracked device
     */

    for (int i = 0; i < 3; i++) {
        linear[i] = device_poses_[device_index].vVelocity.v[i];
        angular[i] = device_poses_[device_index].vAngularVelocity.v[i];
    }
}

int VRInterface::GetDeviceClass(int device_index) {
    /**
     * Get the class of a tracked device
     */

    return pHMD_->GetTrackedDeviceClass(device_index);
}

bool VRInterface::PoseIsValid(int device_index) {
    /**
     * Check if the pose of a tracked device is valid
     */

    return (device_poses_[device_index].eTrackingResult == vr::TrackingResult_Running_OK && 
            device_poses_[device_index].bPoseIsValid &&
            device_poses_[device_index].bDeviceIsConnected);
}

void VRInterface::Update() {
    /*
    * Calculates updated poses for all tracked devices
    * TrackingUniverseRawAndUncalibrated - provides poses relative to the hardware-specific coordinate system in the driver
    */
    
    pHMD_->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0, device_poses_, device_count);
}

// ROS logging
void VRInterface::SetDebugMsgCallback(DebugMsgCallback fn) { debug_ = fn; }
void VRInterface::SetInfoMsgCallback(InfoMsgCallback fn) { info_ = fn; }
void VRInterface::SetWarnMsgCallback(WarnMsgCallback fn) { warn_ = fn; }
void VRInterface::SetErrorMsgCallback(ErrorMsgCallback fn) { error_ = fn; }
void VRInterface::SetFatalMsgCallback(FatalMsgCallback fn) { fatal_ = fn; }