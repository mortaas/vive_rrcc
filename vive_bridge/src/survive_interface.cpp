#include "vive_bridge/survive_interface.h"


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


struct SurviveExternalObject {
	SurvivePose pose;
	SurviveVelocity velocity;
};

struct SurviveLighthouseData {
	int lighthouse;
	char serial_number[16];
};

struct SurviveSimpleObject {
	struct SurviveSimpleContext *actx;

	enum SurviveSimpleObject_type {
		SurviveSimpleObject_LIGHTHOUSE,
		SurviveSimpleObject_OBJECT,
		SurviveSimpleObject_EXTERNAL
	} type;

	union {
		struct SurviveLighthouseData lh;
		struct SurviveObject *so;
		struct SurviveExternalObject seo;
	} data;

	char name[32];
	bool has_update;
};


ViveInterface::ViveInterface()
    : VR_DEBUG(DefaultDebugMsgCallback),
      VR_INFO(DefaultInfoMsgCallback),
      VR_WARN(DefaultWarnMsgCallback),
      VR_ERROR(DefaultErrorMsgCallback),
      VR_FATAL(DefaultFatalMsgCallback)
{
    // Fill device info with default values
    for (int i = 0; i < MAX_TRACKED_DEVICES; i++) {
        tracked_devices_[i].device_class = 0;
        tracked_devices_[i].survive_name = "UNKNOWN"; 
    }
}
ViveInterface::~ViveInterface() {
}

bool ViveInterface::Init(int argc, char **argv) {
      /**
     * Initializes libsurvive
     */

    actx_ = survive_simple_init(argc, argv);

    // Check if libsurvive was initialized properly
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

            // Populate device info
            tracked_devices_[i].survive_name = survive_simple_object_name(it_);
            tracked_devices_[i].device_class = SurviveClassToOpenVR(tracked_devices_[i].survive_name);
            tracked_devices_[i].serial_number = survive_simple_serial_number(it_);
        }
    } else {
        VR_FATAL("libsurvive initialization failed");

        return false;
    }
    VR_INFO("libsurvive initialization succeeded");

    return true;
}

void ViveInterface::Shutdown() {
      /**
     * Shuts down the connection to the VR hardware and cleans up the OpenVR API. 
     * The vr::IVRSystem pointer returned by vr::VR_Init will be invalid after this call is made.
     */

    if (actx_) {
        survive_simple_close(actx_);
        actx_ = nullptr;
    }
}

void ViveInterface::GetControllerState(const unsigned int &device_index, std::vector<float> &axes, std::vector<int> &buttons) {
      /**
     * Get controller state and map it to axes and buttons
     */

    // Axes
    axes[0] = tracked_devices_[device_index].axes[0];
    axes[1] = tracked_devices_[device_index].axes[1];
    axes[2] = tracked_devices_[device_index].axes[2];

    // Buttons
    buttons.assign(13, 0);
    if ((1ull << 1) & tracked_devices_[device_index].button_flags) {
        buttons[0] = 1; // Menu button
    }
    if ((1ull << 2) & tracked_devices_[device_index].button_flags) {
        buttons[1] = 1; // Grip button
    }
    if ((1ull << 32) & tracked_devices_[device_index].button_flags) {
        buttons[2] = 1; // Touchpad button
    }
    if ((1ull << 33) & tracked_devices_[device_index].button_flags) {
        buttons[3] = 1; // Trigger button
    }
}

bool ViveInterface::PollNextEvent(unsigned int &event_type, unsigned int &device_index) {
      /**
     * Returns true if there is any event waiting in the event queue,
     * and also returns the event type and device index of this event.
     * Returns event type VREvent_None (0) and device index k_unTrackedDeviceIndexInvalid (4294967295)
     * if there is no event waiting in the event queue.
     */

    SurviveSimpleEvent event_ = {};
    survive_simple_next_event(actx_, &event_);

    switch (event_.event_type) {
        case SurviveSimpleEventType_ButtonEvent: {
            const struct SurviveSimpleButtonEvent *button_event_ = survive_simple_get_button_event(&event_);
            const SurviveSimpleObject *fsao_ = survive_simple_get_first_object(actx_);
            const SurviveSimpleObject *sao = button_event_->object;

            std::ptrdiff_t i = std::distance(fsao_, sao);
            device_index = (int) i;

            switch(button_event_->event_type) {
                case BUTTON_EVENT_BUTTON_DOWN:
                    event_type = 200; // ButtonPress

                    if (button_event_->button_id == 2) { // Touchpad button
                        tracked_devices_[i].button_flags |= 1ull << 32;
                    }
                    if (button_event_->button_id == 4) { // Grip button
                        tracked_devices_[i].button_flags |= 1ull << 2;
                    }
                    if (button_event_->button_id == 5) { // Menu button
                        tracked_devices_[i].button_flags |= 1ull << 1;
                    }
                    if (button_event_->button_id == 24) { // Trigger button
                        tracked_devices_[i].button_flags |= 1ull << 33;
                    }

                    break;
                case BUTTON_EVENT_BUTTON_UP:
                    event_type = 201; // ButtonUnpress

                    if (button_event_->button_id == 2) { // Touchpad button
                        tracked_devices_[i].button_flags &= !(1ull << 32);
                    }
                    if (button_event_->button_id == 4) { // Grip button
                        tracked_devices_[i].button_flags &= !(1ull <<  2);
                    }
                    if (button_event_->button_id == 5) { // Menu button
                        tracked_devices_[i].button_flags &= !(1ull <<  1);
                    }
                    if (button_event_->button_id == 24) { // Trigger button
                        tracked_devices_[i].button_flags &= !(1ull << 33);
                    }

                    break;
                case BUTTON_EVENT_AXIS_CHANGED:
                    if (button_event_->axis_val[0] + button_event_->axis_val[1] != 0.) {
                        event_type = 202; // ButtonTouch
                    } else {
                        event_type = 203; // ButtonUntouch
                    }
                    break;
                case BUTTON_EVENT_BUTTON_NONE:
                    device_index = 0xFFFFFFFF;
                    event_type = 0;
                    break;
            }

            if (event_type == 202 || event_type == 203)
            {
                if (button_event_->axis_ids[0] == 1) {
                    tracked_devices_[i].axes[2] = button_event_->axis_val[0] / 32640.; // Trigger
                } else {
                    tracked_devices_[i].axes[0] = (int16_t) button_event_->axis_val[0] / 32767.; // Touchpad x
                    tracked_devices_[i].axes[1] = (int16_t) button_event_->axis_val[1] / 32767.; // Touchpad y
                }
            }

            break;
        }
        case SurviveSimpleEventType_None:
            device_index = 0xFFFFFFFF;
            event_type = 0;
            break;
    }
}

unsigned char ViveInterface::SurviveClassToOpenVR(const std::string &device_name) {
      /**
     * Converts libsurvive's device names to OpenVR classes
     */

         if (boost::algorithm::contains(device_name, "HMD") ||
             boost::algorithm::contains(device_name, "T20") )
    {
        return 1; // HMD
    }
    else if (boost::algorithm::contains(device_name, "WM") ||
             boost::algorithm::contains(device_name, "WW") )
    {
        return 2; // Controller (WW is USB wired)
    } else if (boost::algorithm::contains(device_name, "TR") ) {
        return 3; // Tracker
    } else if (boost::algorithm::contains(device_name, "LH") ) {
        return 4; // Lighthouse
    } else if (boost::algorithm::contains(device_name, "RD") ) {
        return 5; // Redirected display
    } else {
        return 0; // No device
    }
}

void ViveInterface::GetDeviceSN(const unsigned int &device_index, std::string &device_sn) {
    /**
     * Get the serial number of a tracked device.
     */

    device_sn = tracked_devices_[device_index].serial_number;
}

void ViveInterface::GetDevicePose(const unsigned int &device_index, float m[3][4]) {
    /**
     * Get the pose of a tracked device
     * This pose is represented as the top 3 rows of a homogeneous transformation matrix
     */

    float *mptr = &tracked_devices_[device_index].pose[0][0];
    std::copy(mptr, mptr + 16, &m[0][0]);
}
void ViveInterface::GetDeviceVelocity(const unsigned int &device_index, float linear[3], float angular[3]) {
    /**
     * Get the linear and angular velocity (twist) of a tracked device
     */

    for (int i = 0; i < 3; i++) {
        linear[i] = tracked_devices_[device_index].linear_vel[i];
        angular[i] = tracked_devices_[device_index].angular_vel[i];
    }
}

unsigned char ViveInterface::GetDeviceClass(const unsigned int &device_index) {
    /**
     * Get the class of a tracked device
     */

    return tracked_devices_[device_index].device_class;
}

bool ViveInterface::PoseIsValid(const unsigned int &device_index) {
    /**
     * Check if the pose of a tracked device is valid
     */

    // Check if the device is populated
    if ((survive_simple_get_object_count(actx_) - device_index) >= 0) {
        return true;
    } else {
        return false;
    }
}

void ViveInterface::Update() {
    /*
    * Calculates updated poses and velocities for all tracked devices
    */

    if (survive_simple_is_running(actx_) ) {
        const SurviveSimpleObject *fsao_ = survive_simple_get_first_object(actx_);

        for (const SurviveSimpleObject *it_ = survive_simple_get_next_updated(actx_); it_ != nullptr;
			                            it_ = survive_simple_get_next_updated(actx_) )
        {
            std::ptrdiff_t i = std::distance(fsao_, it_);

            survive_simple_object_get_latest_pose(it_, &survive_pose_);
            survive_simple_object_get_latest_velocity(it_, &survive_velocity_);

            // Convert pose to OpenGL matrix
            PoseToMatrix(pose, &survive_pose_);

            std::copy(pose, pose + 16,
                      &tracked_devices_[i].pose[0][0]);
            std::copy(survive_velocity_.Pos, survive_velocity_.Pos + 3,
                      &tracked_devices_[i].linear_vel[0]);
            std::copy(survive_velocity_.AxisAngleRot, survive_velocity_.AxisAngleRot + 3,
                      &tracked_devices_[i].angular_vel[0]);
        }
    }
}

void ViveInterface::TriggerHapticPulse(const unsigned int &device_index, const unsigned short &duration) {
    /*
    * Triggers a single haptic pulse on a controller given its device index, and the pulse duration 0-3999 µs ("strength")
    */

    // for (int i = 0; i < 0x5; i++)
    // {
    //     survive_haptic(actx_->objects[device_index].data.so, 0, 0xf401, 0xb5a2, 0x0100);
    //     OGUSleep(1000);
    // }
}

// Logging to ROS
void ViveInterface::SetDebugMsgCallback(DebugMsgCallback fn) { VR_DEBUG = fn; }
void ViveInterface::SetInfoMsgCallback(InfoMsgCallback fn) { VR_INFO = fn; }
void ViveInterface::SetWarnMsgCallback(WarnMsgCallback fn) { VR_WARN = fn; }
void ViveInterface::SetErrorMsgCallback(ErrorMsgCallback fn) { VR_ERROR = fn; }
void ViveInterface::SetFatalMsgCallback(FatalMsgCallback fn) { VR_FATAL = fn; }