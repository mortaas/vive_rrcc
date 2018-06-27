#include "vr_ros/vr_node.h"

VRNode::VRNode(int frequency)
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
    callback_type_ = boost::bind(&VRNode::ReconfCallback, this, _1, _2);
    reconf_server_.setCallback(callback_type_);

    if (!InitParams()) {
        ROS_WARN("Failed to get parameters from the parameter server");
    }
    SendOffsetTransform();
}
VRNode::~VRNode() {
}

bool VRNode::InitParams() {
     /**
      * Initialize parameters from parameter server.
      * Return true if the parameters was retrieved from the server, false otherwise.
      */
    
    bool params_retrieved (nh_.param("/vr_node/x_offset",       x_offset,       0.0) &&
                           nh_.param("/vr_node/y_offset",       y_offset,       0.0) &&
                           nh_.param("/vr_node/z_offset",       z_offset,       0.0) &&
                           nh_.param("/vr_node/yaw_offset",     yaw_offset,     0.0) &&
                           nh_.param("/vr_node/pitch_offset",   pitch_offset,   0.0) &&
                           nh_.param("/vr_node/roll_offset",    roll_offset,    0.0));
    SendOffsetTransform();

    return params_retrieved;
}
void VRNode::ReconfCallback(vr_ros::VRConfig &config, uint32_t level) {
     /**
      * Dynamic reconfigure callback for changing parameters during runtime
      */

    x_offset        = config.x_offset;
    y_offset        = config.y_offset;
    z_offset        = config.z_offset;
    yaw_offset      = config.yaw_offset;
    pitch_offset    = config.pitch_offset;
    roll_offset     = config.roll_offset;

    SendOffsetTransform();
}

void VRNode::SendOffsetTransform() {
      /**
     * Update and send static offset transform from fixed world frame
     * to world_vr frame (i.e. the fixed VR frame).
     */

    origin_offset_.setValue(x_offset,
                            y_offset,
                            z_offset);
    rotation_offset_.setEuler(yaw_offset,
                              pitch_offset,
                              roll_offset);
    tf_offset_.setOrigin(origin_offset_);
    tf_offset_.setRotation(rotation_offset_);

    geometry_msgs::TransformStamped world_vr_msg_;
    world_vr_msg_.header.stamp = ros::Time::now();
    world_vr_msg_.header.frame_id = "world";
    world_vr_msg_.child_frame_id = "world_vr";
    tf2::convert(tf_offset_, world_vr_msg_.transform);
    static_tf_broadcaster_.sendTransform(world_vr_msg_);
}

tf2::Transform VRNode::ConvertTransform(float m[3][4]) {
      /**
     * Convert pose from OpenVR to tf2 transform
     */

    tf2::Matrix3x3 m_basis = tf2::Matrix3x3(m[0][0], m[0][1], m[0][2],
                                            m[1][0], m[1][1], m[1][2],
                                            m[2][0], m[2][1], m[2][2]);
    tf2::Vector3 m_origin = tf2::Vector3(m[0][3], m[1][3], m[2][3]);

    return tf2::Transform(m_basis, m_origin);
}
tf2::Vector3 VRNode::ConvertVector(float v[3]) {
     /**
      * Convert vector from OpenVR to tf2 vector
      */

    return tf2::Vector3(v[0], v[1], v[2]);
}

bool VRNode::Init() {
      /**
     * Initialize the OpenVR API
     */

    return vr_.Init();
}
void VRNode::Shutdown() {
      /**
     * Shuts down the connection to the VR hardware and cleans up the OpenVR API.
     */

    vr_.Shutdown();
}

void VRNode::Loop() {
    // Calculate updated poses for all tracked devices
    vr_.Update();

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

                // Get pose of current tracked device
                vr_.GetDevicePose(i, current_pose);

                // Declare, populate and send transform message with pose of current tracked device
                geometry_msgs::TransformStamped transform_msg_;

                transform_msg_.header.stamp = ros::Time::now();
                transform_msg_.header.frame_id = "world_vr";
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
                tf2::convert(ConvertTransform(current_pose), transform_msg_.transform);

                tf_broadcaster_.sendTransform(transform_msg_);

                // Check if current tracked device is not a lighthouse (i.e. twist is available)
                if (device_class != vr::TrackedDeviceClass_TrackingReference) {
                    // Get linear and angular velocity of current device
                    vr_.GetDeviceVelocity(i, current_linvel, current_angvel);

                    // Declare, populate and publish twist message
                    geometry_msgs::TwistStamped twist_msg_;

                    twist_msg_.header.stamp = ros::Time::now();
                    twist_msg_.header.frame_id = device_frame;
                    tf2::convert(ConvertVector(current_linvel), twist_msg_.twist.linear);
                    tf2::convert(ConvertVector(current_angvel), twist_msg_.twist.angular);

                    // Advertise twist topic if the tracked device is new
                    if (twist_pubs_map_.count(device_sn) == 0) {
                        switch(device_class) {
                            case vr::TrackedDeviceClass_HMD:
                                twist_pubs_map_[device_sn] = nh_.advertise<geometry_msgs::TwistStamped>("/vr/twist/hmd_"        + device_sn, 10);
                                break;
                            case vr::TrackedDeviceClass_Controller:
                                twist_pubs_map_[device_sn] = nh_.advertise<geometry_msgs::TwistStamped>("/vr/twist/controller_" + device_sn, 10);
                                break;
                            case vr::TrackedDeviceClass_GenericTracker:
                                twist_pubs_map_[device_sn] = nh_.advertise<geometry_msgs::TwistStamped>("/vr/twist/tracker_"    + device_sn, 10);
                                break;
                        }
                    }
                    twist_pubs_map_[device_sn].publish(twist_msg_);
                }

                if (device_class == vr::TrackedDeviceClass_Controller) {
                    // Declare, populate and publish joy message
                    sensor_msgs::Joy joy_msg_;

                    joy_msg_.header.stamp = ros::Time::now();
                    joy_msg_.header.frame_id = device_frame;
                    vr_.GetControllerState(i, joy_msg_.axes, joy_msg_.buttons);

                    // Advertise joy topic if the tracked device is new
                    if (joy_pubs_map_.count(device_sn) == 0) {
                        joy_pubs_map_[device_sn] = nh_.advertise<sensor_msgs::Joy>("/vr/joy/controller_" + device_sn, 10); break;
                    }
                    // !!TODO!! Publish only when values are non-zero
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
    ros::init(argc, argv, "vr_node");

    VRNode node_(120);

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