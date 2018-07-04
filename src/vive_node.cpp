#include "vive_bridge/vive_node.h"

ViveNode::ViveNode(int frequency)
    : loop_rate_(frequency),
      tf_broadcaster_(),
      static_tf_broadcaster_(),
      vr_()
{
    // Set callback functions for logging OpenVR messages to ROS
    vr_.SetDebugMsgCallback(HandleDebugMsgs);
    vr_.SetInfoMsgCallback(HandleInfoMsgs);
    vr_.SetWarnMsgCallback(HandleWarnMsgs);
    vr_.SetErrorMsgCallback(HandleErrorMsgs);
    vr_.SetFatalMsgCallback(HandleFatalMsgs);

    // Set dynamic reconfigure callback function
    callback_type_ = boost::bind(&ViveNode::ReconfCallback, this, _1, _2);
    reconf_server_.setCallback(callback_type_);

    for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
        button_touched[i] = false;
        controller_interaction[i] = false;
    }
}

ViveNode::~ViveNode() {
}

bool ViveNode::InitParams() {
     /**
      * Initialize parameters from parameter server.
      * Return true if the parameters was retrieved from the server, false otherwise.
      */
    
    bool params_retrieved (nh_.param("/vive_node/publish_joy",    publish_joy,    false) &&
                           nh_.param("/vive_node/publish_twist",  publish_twist,  false) &&
                           nh_.param("/vive_node/x_offset",       x_offset,       0.0) &&
                           nh_.param("/vive_node/y_offset",       y_offset,       0.0) &&
                           nh_.param("/vive_node/z_offset",       z_offset,       0.0) &&
                           nh_.param("/vive_node/yaw_offset",     yaw_offset,     0.0) &&
                           nh_.param("/vive_node/pitch_offset",   pitch_offset,   0.0) &&
                           nh_.param("/vive_node/roll_offset",    roll_offset,    0.0));

    return params_retrieved;
}
void ViveNode::ReconfCallback(vive_bridge::ViveConfig &config, uint32_t level) {
     /**
      * Dynamic reconfigure callback for changing parameters during runtime
      */

    publish_joy     = config.publish_joy;
    publish_twist   = config.publish_twist;
    x_offset        = config.x_offset;
    y_offset        = config.y_offset;
    z_offset        = config.z_offset;
    yaw_offset      = config.yaw_offset;
    pitch_offset    = config.pitch_offset;
    roll_offset     = config.roll_offset;

    SendOffsetTransform();
}

void ViveNode::SendOffsetTransform() {
      /**
     * Update and send static offset transform from fixed world frame
     * to world_vr frame (i.e. the fixed VR frame)
     */

    origin_offset_.setValue(x_offset,
                            y_offset,
                            z_offset);
    rotation_offset_.setEuler(yaw_offset,
                              pitch_offset,
                              roll_offset);
    tf_offset_.setOrigin(origin_offset_);
    tf_offset_.setRotation(rotation_offset_);

    offset_msg_.header.stamp = ros::Time::now();
    tf2::convert(tf_offset_, offset_msg_.transform);
    static_tf_broadcaster_.sendTransform(offset_msg_);
}

void ViveNode::ConvertTransform(const float m[3][4], tf2::Transform &tf_m_) {
      /**
     * Convert pose from OpenVR to tf2 transform
     */

    tf_m_basis.setValue(m[0][0], m[0][1], m[0][2],
                        m[1][0], m[1][1], m[1][2],
                        m[2][0], m[2][1], m[2][2]);
    tf_m_origin.setValue(m[0][3], m[1][3], m[2][3]);

    tf_m_.setBasis(tf_m_basis);
    tf_m_.setOrigin(tf_m_origin);
}
void ViveNode::ConvertVector(const float v[3], tf2::Vector3 &tf_v_) {
     /**
      * Convert vector from OpenVR to tf2 vector
      */

    tf_v_.setValue(v[0], v[1], v[2]);
}

bool ViveNode::Init() {
      /**
     * Initialize the node and OpenVR
     */

    offset_msg_.header.frame_id = "world";
    offset_msg_.child_frame_id = "world_vr";
    transform_msg_.header.frame_id = "world_vr";

    // Corrective transform to make the VIVE trackers follow the 
    // coordinate system conventions for VIVE devices
    origin_tracker_.setZero();
    rotation_tracker_.setEuler(M_PI, M_PI_2, 0.0);
    tf_tracker_.setOrigin(origin_tracker_);
    tf_tracker_.setRotation(rotation_tracker_);

    if (!InitParams()) {
        ROS_WARN("Failed to get parameters from the parameter server");
    }
    SendOffsetTransform();

    return vr_.Init();
}
void ViveNode::Shutdown() {
      /**
     * Shuts down the connection to the VR hardware and cleans up the OpenVR API
     */

    vr_.Shutdown();
}

void ViveNode::Loop() {
    // Calculate updated poses for all tracked devices
    vr_.Update();

    // Check if there are any events in the OpenVR queue
    vr_.PollNextEvent(event_type, event_device_index);
    // Handle controller events
    for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
        controller_interaction[i] = false || button_touched[i];
    }
    if (event_type != vr::VREvent_None && event_device_index != vr::k_unTrackedDeviceIndexInvalid) {
        switch (event_type) {
            case vr::VREvent_ButtonPress:   controller_interaction[event_device_index] = true;
                                            break;
            case vr::VREvent_ButtonUnpress: controller_interaction[event_device_index] = true;
                                            break;
            case vr::VREvent_ButtonTouch:   button_touched[event_device_index] = true;
                                            break;
            case vr::VREvent_ButtonUntouch: button_touched[event_device_index] = false;
                                            break;
        }

        controller_interaction[event_device_index] = controller_interaction[event_device_index];
    }

    // Loop through all possible device slots (indices)
    for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
        // Get device class of current device
        int device_class = vr_.GetDeviceClass(i);

        // Every device with something other than TrackedDevice_Invalid is associated with an actual physical device
        if (device_class != vr::TrackedDeviceClass_Invalid) {
            if (vr_.PoseIsValid(i) ) {
                // Get serial number of current tracked device
                std::string device_sn = vr_.GetDeviceSN(i);
                std::replace(device_sn.begin(), device_sn.end(), '-', '_');
                // Get manufacturer name of current tracked device
                // std::string device_name = vr_.GetDeviceManufacturerName(i);

                // Get pose of current tracked device
                vr_.GetDevicePose(i, current_pose);

                // Populate and send transform message with pose of current tracked device
                transform_msg_.header.stamp = ros::Time::now();
                std::string device_frame;
                switch(device_class) {
                    case vr::TrackedDeviceClass_HMD:               device_frame = "hmd_" + device_sn;
                                                                   break;
                    case vr::TrackedDeviceClass_Controller:        device_frame = "controller_" + device_sn;
                                                                   break;
                    case vr::TrackedDeviceClass_GenericTracker:    device_frame = "tracker_" + device_sn;
                                                                   break;
                    case vr::TrackedDeviceClass_TrackingReference: device_frame = "lighthouse_" + device_sn;
                                                                   break;
                }
                transform_msg_.child_frame_id = device_frame;
                ConvertTransform(current_pose, tf_current_pose_);
                if (device_class != vr::TrackedDeviceClass_GenericTracker) {
                    tf2::convert(tf_current_pose_, transform_msg_.transform);
                } else {
                    // Correct pose if current device is a tracker
                    tf2::convert(tf_current_pose_ * tf_tracker_, transform_msg_.transform);
                }

                tf_broadcaster_.sendTransform(transform_msg_);

                // Check if current tracked device is not a lighthouse (i.e. twist is available)
                if (device_class != vr::TrackedDeviceClass_TrackingReference && publish_twist) {
                    // Get linear and angular velocity of current device
                    vr_.GetDeviceVelocity(i, current_linvel, current_angvel);

                    // Populate and publish twist message
                    twist_msg_.header.stamp = ros::Time::now();
                    twist_msg_.header.frame_id = device_frame;
                    ConvertVector(current_linvel, tf_current_linvel_);
                    ConvertVector(current_angvel, tf_current_angvel_);
                    tf2::convert(tf_current_linvel_, twist_msg_.twist.linear);
                    tf2::convert(tf_current_angvel_, twist_msg_.twist.angular);

                    // Advertise twist topic if the tracked device is new
                    if (twist_pubs_map_.count(device_sn) == 0) {
                        switch(device_class) {
                            case vr::TrackedDeviceClass_HMD:
                                twist_pubs_map_[device_sn] = nh_.advertise<geometry_msgs::TwistStamped>("/vive/twist/hmd_"        + device_sn, 10);
                                break;
                            case vr::TrackedDeviceClass_Controller:
                                twist_pubs_map_[device_sn] = nh_.advertise<geometry_msgs::TwistStamped>("/vive/twist/controller_" + device_sn, 10);
                                break;
                            case vr::TrackedDeviceClass_GenericTracker:
                                twist_pubs_map_[device_sn] = nh_.advertise<geometry_msgs::TwistStamped>("/vive/twist/tracker_"    + device_sn, 10);
                                break;
                        }
                    }
                    twist_pubs_map_[device_sn].publish(twist_msg_);
                }

                if (controller_interaction[i] && publish_joy) {
                    // Populate and publish joy message
                    joy_msg_.header.stamp = ros::Time::now();
                    joy_msg_.header.frame_id = device_frame;
                    vr_.GetControllerState(i, joy_msg_.axes, joy_msg_.buttons);

                    // Advertise joy topic if the tracked device is new
                    if (joy_pubs_map_.count(device_sn) == 0) {
                        joy_pubs_map_[device_sn] = nh_.advertise<sensor_msgs::Joy>("/vive/joy/controller_" + device_sn, 10); break;
                    }
                    joy_pubs_map_[device_sn].publish(joy_msg_);
                }
            }
        } else {
            // Contine to next device if device class of the current device is invalid
            continue;
        }
    }

    ros::spinOnce();
    loop_rate_.sleep();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vive_node");

    ViveNode node_(120);

    if (!node_.Init() ) {
        // node_.Shutdown();
        return 0;
    }

    while (ros::ok() ) {
        node_.Loop();
    }

    node_.Shutdown();
    return 0;
}