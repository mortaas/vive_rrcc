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

static std::queue<int> controller_event_type, controller_device_index;

// static std::queue<std::string> error_queue, info_queue;
// static void error_fn(SurviveContext * ctx, const char * fault) {
//     struct SurviveSimpleContext *actx = (struct SurviveSimpleContext *) ctx->user_ptr;
// 	OGLockMutex(actx->poll_mutex);

//     error_queue.push(std::string(fault) );
//     printf("[VR] %s \n", fault);

//     OGUnlockMutex(actx->poll_mutex);
// }
// static void info_fn(SurviveContext * ctx, const char * fault) {
//     struct SurviveSimpleContext *actx = (struct SurviveSimpleContext *) ctx->user_ptr;
// 	OGLockMutex(actx->poll_mutex);

//     info_queue.push(std::string(fault) );

//     OGUnlockMutex(actx->poll_mutex);
// }

static void light_process( SurviveObject * so, int sensor_id, int acode, int timeinsweep, uint32_t timecode, uint32_t length, uint32_t lh)
{
    survive_default_light_process(so, sensor_id, acode, timeinsweep, timecode, length, lh);

    if (std::string(so->codename) == "T20") {
        printf("Light: [%u][lh %u][%s][acode %u][sensor %u][time in sweep %u][length %u]\n", timecode,
            lh, so->codename, acode, sensor_id, timeinsweep, length);
    }
}

static void imu_process(SurviveObject * so, int mask, FLT * accelgyromag, uint32_t timecode, int id)
{
    struct SurviveSimpleContext *actx = (struct SurviveSimpleContext *) so->ctx->user_ptr;

	OGLockMutex(actx->poll_mutex);

	    survive_default_imu_process(so, mask, accelgyromag, timecode, id);

    OGUnlockMutex(actx->poll_mutex);
}

static void button_process(SurviveObject *so, uint8_t eventType, uint8_t buttonId, uint8_t axis1Id, uint16_t axis1Val, uint8_t axis2Id, uint16_t axis2Val)
{
    survive_default_button_process(so, eventType, buttonId, axis1Id, axis1Val, axis2Id, axis2Val);

    void *ptr = so;

    // Check if pointer to the SurviveObject is corrupt
    //if (ptr != nullptr) {
        struct SurviveSimpleContext *actx = (struct SurviveSimpleContext *) so->ctx->user_ptr;
        intptr_t i = (intptr_t) so->user_ptr;

        OGLockMutex(actx->poll_mutex);

        switch(eventType) {
            case BUTTON_EVENT_BUTTON_DOWN :
                controller_device_index.push(i);
                controller_event_type.push(200); // ButtonPress
                break;
            case BUTTON_EVENT_BUTTON_UP :
                controller_device_index.push(i);
                controller_event_type.push(201); // ButtonUnpress
                break;
            case BUTTON_EVENT_AXIS_CHANGED :
                if (axis1Val + axis2Val != 0.) {
                    controller_device_index.push(i);
                    controller_event_type.push(202); // ButtonTouch
                } else {
                    controller_device_index.push(i);
                    controller_event_type.push(203); // ButtonUntouch
                }
                break;
            case BUTTON_EVENT_BUTTON_NONE :
                break;
        }

        OGUnlockMutex(actx->poll_mutex);
    //}
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
     * Initializes libsurvive
     */

    actx_ = survive_simple_init(argc, argv);

    // Install callback functions
    survive_install_button_fn(actx_->ctx, button_process);
    survive_install_imu_fn(actx_->ctx, imu_process);

    // survive_install_light_fn(actx_->ctx, light_process);

    // survive_install_error_fn(actx_->ctx, error_fn);
    // survive_install_info_fn(actx_->ctx, info_fn);

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
            device_names_[i] = survive_simple_object_name(it_);
            device_classes_[i] = SurviveClassToOpenVR(device_names_[i]);
            // device_serials_[i] = survive_simple_serial_number(it_);
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

void ViveInterface::GetControllerState(const int &device_index, std::vector<float> &axes, std::vector<int> &buttons) {
      /**
     * Get controller state and map it to axes and buttons
     */

    OGLockMutex(actx_->poll_mutex);

        const struct SurviveObject *so_ = actx_->objects[device_index].data.so;

        axes.assign(3, 0.);
        axes[0] = so_->axis1 / 32640.;
        axes[1] = so_->axis2 / 32767.;
        axes[2] = so_->axis3 / 32767.;

        buttons.assign(13, 0);
        if ( so_->buttonmask &  1)       { buttons[3] = 1; } // Trigger
        if ((so_->buttonmask &  6) == 6) { buttons[2] = 1; } // Touchpad
        if ((so_->buttonmask & 16) >> 4) { buttons[1] = 1; } // Grip
        if ((so_->buttonmask & 32) >> 5) { buttons[0] = 1; } // Menu

    OGUnlockMutex(actx_->poll_mutex);
}

// void ViveInterface::PollNextMessage() {
//       /**
//      * Get and display next message from libsurvive
//      */

//     struct SurviveSimpleContext *actx = (struct SurviveSimpleContext *) actx_->ctx->user_ptr;

//     OGLockMutex(actx->poll_mutex);

//     while (!error_queue.empty() ) {
//         VR_ERROR(error_queue.front() );
//         info_queue.pop();
//     }
//     while (!info_queue.empty() ) {
//         VR_INFO(info_queue.front() );
//         info_queue.pop();
//     }

//     OGUnlockMutex(actx->poll_mutex);
// }

bool ViveInterface::PollNextEvent(int &event_type, int &device_index) {
      /**
     * Returns true if there is any event waiting in the event queue,
     * and also returns the event type and device index of this event.
     * Returns event type VREvent_None (0) and device index k_unTrackedDeviceIndexInvalid (4294967295)
     * if there is no event waiting in the event queue.
     */

    OGLockMutex(actx_->poll_mutex);

    if (!controller_event_type.empty() ) {
        event_type = controller_event_type.front();
        device_index = controller_device_index.front();

        controller_event_type.pop();
        controller_device_index.pop();
    } else {
        event_type = 0;
        device_index = 0xFFFFFFFF;
    }

    OGUnlockMutex(actx_->poll_mutex);

    if (!(event_type == 0 && device_index == 0xFFFFFFFF) ) {
        return true;
    } else {
        return false;
    }
}

int ViveInterface::SurviveClassToOpenVR(const std::string &device_name) {
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

void ViveInterface::GetDeviceSN(const int &device_index, std::string &device_sn) {
    /**
     * Should ideally get the serial number of a tracked device.
     * The serial number is not easily available in libsurvive,
     * and the device name is therefore returned as a UID instead
     */

    // if (!(device_classes_[device_index] == 4) ) {
    //     device_sn = device_serials_[device_index];
    // } else {
    //     device_sn = device_names_[device_index][2];
    // }

    device_sn = device_names_[device_index][2];
}

void ViveInterface::GetDevicePose(const int &device_index, float m[3][4]) {
    /**
     * Get the pose of a tracked device
     * This pose is represented as the top 3 rows of a homogeneous transformation matrix
     */

    float *mptr = &device_poses_[device_index].mDeviceToAbsoluteTracking.m[0][0];
    std::copy(mptr, mptr + 16, &m[0][0]);
}
void ViveInterface::GetDeviceVelocity(const int &device_index, float linear[3], float angular[3]) {
    /**
     * Get the linear and angular velocity (twist) of a tracked device
     */

    OGLockMutex(actx_->poll_mutex);

        struct SurviveObject *so_ = actx_->objects[device_index].data.so;
        SurviveSensorActivations *ssa_ = &so_->activations;

        for (int i = 0; i < 3; i++) {
            // Compute calibrated linear and angular velocities
            linear[i]  = so_->acc_scale[i]  * ssa_->accel[i] + so_->acc_bias[i];
            angular[i] = so_->gyro_scale[i] * ssa_->gyro[i]  + so_->gyro_bias[i];
        }
    
    OGUnlockMutex(actx_->poll_mutex);
}

int ViveInterface::GetDeviceClass(const int &device_index) {
    /**
     * Get the class of a tracked device
     */

    return device_classes_[device_index];
}

bool ViveInterface::PoseIsValid(const int &device_index) {
    /**
     * Check if the pose of a tracked device is valid
     */

    // Check if the device is populated
    if ((actx_->object_ct - device_index) >= 0) {
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
        const SurviveSimpleObject *fsao_;
        fsao_ = survive_simple_get_first_object(actx_);

        for (const SurviveSimpleObject *it_ = survive_simple_get_next_updated(actx_); it_ != nullptr;
			                            it_ = survive_simple_get_next_updated(actx_) )
        {
            std::ptrdiff_t i = std::distance(fsao_, it_);

            uint32_t timecode = survive_simple_object_get_latest_pose(it_, &survive_pose_);

            PoseToMatrix(ogl_pose, &survive_pose_);
            std::copy(ogl_pose, ogl_pose + 16, &device_poses_[i].mDeviceToAbsoluteTracking.m[0][0]);
        }
    }
}

void ViveInterface::TriggerHapticPulse(const int &device_index, const int &axis_id, int duration) {
    /*
    * Triggers a single haptic pulse on a controller given its device index, and the pulse duration 0-3999 µs ("strength")
    */

    for (int i = 0; i < 0x5; i++)
    {
        survive_haptic(actx_->objects[device_index].data.so, 0, 0xf401, 0xb5a2, 0x0100);
        OGUSleep(1000);
    }
}

// Logging to ROS
void ViveInterface::SetDebugMsgCallback(DebugMsgCallback fn) { VR_DEBUG = fn; }
void ViveInterface::SetInfoMsgCallback(InfoMsgCallback fn) { VR_INFO = fn; }
void ViveInterface::SetWarnMsgCallback(WarnMsgCallback fn) { VR_WARN = fn; }
void ViveInterface::SetErrorMsgCallback(ErrorMsgCallback fn) { VR_ERROR = fn; }
void ViveInterface::SetFatalMsgCallback(FatalMsgCallback fn) { VR_FATAL = fn; }