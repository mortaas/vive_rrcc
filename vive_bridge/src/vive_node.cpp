#include "vive_bridge/vive_node.h"

ViveNode::ViveNode(int frequency)
    : nh_(ros::NodeHandle("vive_node") ),
      loop_rate_(frequency),
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

    // Publishers
    devices_pub_ = nh_.advertise<vive_bridge::TrackedDevicesStamped>("tracked_devices", 10, true);
    // Services
    devices_service_ = nh_.advertiseService("tracked_devices", &ViveNode::ReturnTrackedDevices, this);

    offset_msg_.header.frame_id = "world";
    offset_msg_.child_frame_id = "world_vr";
    // Set dynamic reconfigure callback function
    callback_type_ = boost::bind(&ViveNode::ReconfCallback, this, _1, _2);
    reconf_server_.setCallback(callback_type_);

    // Initialize rviz_visual_tools for publishing meshes of tracked devices to RViz
    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world", "rviz_mesh_markers") );
    visual_tools_->loadMarkerPub(false, true);
    visual_tools_->enableFrameLocking();
}

int ViveNode::FindDpadState(float x, float y) {
     /**
      * Find the D-pad state of a controller given it's x and y touch coordinates.
      * The D-pad states corresponds to a numpad, i.e. up is 8, right is 6, down-left is 1...
      */

    if (x >= 0.5) {
        if (y >= 0.5) {
            return 9;
        } else if (y <= -0.5) {
            return 3;
        } else {
            return 6;
        }
    } else if (x <= -0.5) {
        if (y >= 0.5) {
            return 7;
        } else if (y <= -0.5) {
            return 1;
        } else {
            return 4;
        }
    } else {
        if (y >= 0.5) {
            return 8;
        } else if (y <= -0.5) {
            return 2;
        } else {
            return 5;
        }
    }
}

// int ViveNode::HandleDpadState(int dpad_state, float x, float y) {
//      /**
//       * Handle mirrored swipe motions for emulating D-pad buttons
//       */
    
//     ROS_INFO_STREAM(x);
//     ROS_INFO_STREAM(y);
//     int current_dpad_state = FindDpadState(x, y);

//     ROS_INFO_STREAM(current_dpad_state);
//     switch (current_dpad_state) {
//         case 5: return 5;
//                 break;
//         case 1: if (dpad_state == 9) {
//                     return 1;
//                 }
//                 break;
//         case 2: if (dpad_state == 8) {
//                     return 2;
//                 }
//                 break;
//         case 3: if (dpad_state == 7) {
//                     return 3;
//                 }
//                 break;
//         case 4: if (dpad_state == 6) {
//                     return 4;
//                 }
//                 break;
        
//         case 6: if (dpad_state == 4) {
//                     return 6;
//                 }
//                 break;
//         case 7: if (dpad_state == 3) {
//                     return 7;
//                 }
//                 break;
//         case 8: if (dpad_state == 2) {
//                     return 8;
//                 }
//                 break;
//         case 9: if (dpad_state == 1) {
//                     return 9;
//                 }
//                 break;
//     }
// }

bool ViveNode::ReturnTrackedDevices(vive_bridge::GetTrackedDevicesRequest &req,
                                    vive_bridge::GetTrackedDevicesResponse &res)
{
     /**
      * Service callback for getting tracked devices
      */
    
    UpdateTrackedDevices();
    PublishMeshes();

    res.frame_id = "world_vr";
    res.device_count = device_count;
    res.device_classes.resize(device_count);
    res.device_frames.resize(device_count);
    for (int i = 0; i < device_count; i++) {
        res.device_classes[i] = device_classes[i];
        res.device_frames[i] = device_frames[i];
    }

    return true;
}

bool ViveNode::PublishMeshes() {
     /**
      * Publish meshes of tracked devices to visualize them as a MarkerArray in RViz
      */
    
    visual_tools_->resetMarkerCounts();

    for (int i = 0; i < device_count; i++) {
        visual_tools_->setBaseFrame(device_frames[i]);

        switch (device_classes[i]) {
            case vr::TrackedDeviceClass_Invalid:
                break;
            case vr::TrackedDeviceClass_HMD:
                visual_tools_->publishMesh(Eigen::Affine3d::Identity(),
                                           hmd_mesh_path,
                                           rviz_visual_tools::WHITE,
                                           1,
                                           device_frames[i]);
                break;
            case vr::TrackedDeviceClass_Controller:
                visual_tools_->publishMesh(Eigen::Affine3d::Identity(),
                                           controller_mesh_path,
                                           rviz_visual_tools::WHITE,
                                           1,
                                           device_frames[i]);
                break;
            case vr::TrackedDeviceClass_GenericTracker:
                visual_tools_->publishMesh(Eigen::Affine3d::Identity(),
                                           tracker_mesh_path,
                                           rviz_visual_tools::BLACK,
                                           1,
                                           device_frames[i]);
                break;
            case vr::TrackedDeviceClass_TrackingReference:
                visual_tools_->publishMesh(Eigen::Affine3d::Identity(),
                                           lighthouse_mesh_path,
                                           rviz_visual_tools::WHITE,
                                           1,
                                           device_frames[i]);
                break;
        }
    }

    return visual_tools_->trigger();
}

ViveNode::~ViveNode() {
}

bool ViveNode::InitParams() {
     /**
      * Initialize parameters from the parameter server.
      * Return true if the parameters was retrieved from the server, false otherwise.
      */
    
    return (nh_.param("/vive_node/publish_joy",     publish_joy,    false) &&
            nh_.param("/vive_node/publish_twist",   publish_twist,  false) &&
            
            nh_.param<std::string>("/vive_node/hmd_mesh_path",           hmd_mesh_path,
                                   "package://vive_bridge/meshes/vr_hmd_vive_2_0/vr_hmd_vive_2_0.dae") &&
            nh_.param<std::string>("/vive_node/controller_mesh_path",    controller_mesh_path,
                                   "package://vive_bridge/meshes/vr_controller_vive_1_5/vr_controller_vive_1_5.dae") &&
            nh_.param<std::string>("/vive_node/tracker_mesh_path",       tracker_mesh_path,
                                   "package://vive_bridge/meshes/TRACKER-3D.dae") &&
            nh_.param<std::string>("/vive_node/lighthouse_mesh_path",    lighthouse_mesh_path,
                                   "package://vive_bridge/meshes/lh_basestation_vive/lh_basestation_vive.dae") &&
            
            nh_.param("/vive_node/x_offset",        x_offset,       0.0) &&
            nh_.param("/vive_node/y_offset",        y_offset,       0.0) &&
            nh_.param("/vive_node/z_offset",        z_offset,       0.0) &&
            nh_.param("/vive_node/yaw_offset",      yaw_offset,     0.0) &&
            nh_.param("/vive_node/pitch_offset",    pitch_offset,   0.0) &&
            nh_.param("/vive_node/roll_offset",     roll_offset,    0.0));
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
    tf_offset_.setOrigin(origin_offset_);

    rotation_offset_.setRPY(roll_offset,
                            pitch_offset,
                            yaw_offset);
    tf_offset_.setRotation(rotation_offset_);

    tf2::convert(tf_offset_, offset_msg_.transform);

    offset_msg_.header.stamp = ros::Time::now();
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
     /**tracker
      * Convert vector from OpenVR to tf2 vector
      */

    tf_v_.setValue(v[0], v[1], v[2]);
}

void ViveNode::UpdateTrackedDevices() {
      /**
     * Update tracked device count and update arrays for keeping track of these devices
     */

    device_count = 0;

    // Loop through all possible device slots (indices)
    for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
        device_classes[i] = vr_.GetDeviceClass(i);

        if (device_classes[i] != vr::TrackedDeviceClass_Invalid) {
            device_count++;

            // Get serial number of current tracked device as unique identifier (UID)
            vr_.GetDeviceSN(i, device_sns[i]);
            std::replace(device_sns[i].begin(), device_sns[i].end(), '-', '_');
            
            // Find name of tracked device frame
            switch(device_classes[i]) {
                case vr::TrackedDeviceClass_HMD:                device_frames[i] = "hmd_" + device_sns[i];
                                                                break;
                case vr::TrackedDeviceClass_Controller:         device_frames[i] = "controller_" + device_sns[i];
                                                                break;
                case vr::TrackedDeviceClass_GenericTracker:     device_frames[i] = "tracker_" + device_sns[i];
                                                                break;
                case vr::TrackedDeviceClass_TrackingReference:  device_frames[i] = "lighthouse_" + device_sns[i];
                                                                break;
            }
        }
    }
}

bool ViveNode::Init() {
      /**
     * Initialize the node and OpenVR
     */

    if (vr_.Init() ) {
        devices_msg_.header.frame_id = "world_vr";
        transform_msg_.header.frame_id = "world_vr";

        joy_msg_.axes.resize(3);
        joy_msg_.buttons.resize(13);

        // Corrective transform to make the VIVE trackers follow the 
        // coordinate system conventions for VIVE devices
        origin_tracker_.setZero();
        rotation_tracker_.setEuler(0.0, M_PI, 0.0);
        tf_tracker_.setOrigin(origin_tracker_);
        tf_tracker_.setRotation(rotation_tracker_);

        if (!InitParams() ) {
            ROS_WARN_STREAM("Failed to get parameters from the parameter server.");
            ROS_INFO_STREAM("Using default parameters.");
        }
        SendOffsetTransform();

        for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
            button_touched[i] = false;
            controller_interaction[i] = false;
            dpad_state[i] = 5;

            device_classes[i] = vr::TrackedDeviceClass_Invalid;
        }

        UpdateTrackedDevices();
        PublishTrackedDevices();
        PublishMeshes();

        return true;
    }

    return false;
}

void ViveNode::PublishTrackedDevices() {
      /**
     * Publish information about the currently tracked devices
     */
    
    devices_msg_.device_count = device_count;

    devices_msg_.device_classes.resize(device_count);
    devices_msg_.device_frames.resize(device_count);
    for (int i = 0; i < device_count; i++) {
        devices_msg_.device_classes[i] = device_classes[i];
        devices_msg_.device_frames[i] = device_frames[i];
    }

    devices_msg_.header.stamp = ros::Time::now();
    devices_pub_.publish(devices_msg_);
}

void ViveNode::Shutdown() {
      /**
     * Shuts down the connection to the VR hardware and cleans up the OpenVR API
     */

    vr_.Shutdown();
}

void ViveNode::Loop() {
      /**
     * Main loop of the node
     */

    // Calculate updated poses for all tracked devices
    vr_.Update();

    // Check if there are any events in the OpenVR queue
    vr_.PollNextEvent(event_type, event_device_index);

    if (publish_joy) {
        // Update controller interaction flags (clears flag if button is not touched)
        for (int i = 0; i < device_count; i++) {
            controller_interaction[i] = false || button_touched[i];
        }
    }

    if (event_type != vr::VREvent_None &&
        event_device_index != vr::k_unTrackedDeviceIndexInvalid)
    {
        // Update and publish if there are changes to the tracked devices
        if (event_type == vr::VREvent_TrackedDeviceActivated ||
            event_type == vr::VREvent_TrackedDeviceDeactivated)
        {
            UpdateTrackedDevices();
            PublishTrackedDevices();
            PublishMeshes();
        }

        if (publish_joy) {
            // Handle controller events
            switch (event_type) {
                case vr::VREvent_ButtonPress:   controller_interaction[event_device_index] = true;
                                                break;
                case vr::VREvent_ButtonUnpress: controller_interaction[event_device_index] = true;
                                                break;
                case vr::VREvent_ButtonTouch:   button_touched[event_device_index] = true;
                                                vr_.GetControllerState(event_device_index, joy_msg_.axes, joy_msg_.buttons);
                                                dpad_state[event_device_index] = FindDpadState(joy_msg_.axes[0], joy_msg_.axes[1]);
                                                break;
                case vr::VREvent_ButtonUntouch: button_touched[event_device_index] = false;
                                                dpad_state[event_device_index] = 0;
                                                break;
            }
        }
    }

    // Loop through all possible device slots (indices)
    for (int i = 0; i < device_count; i++) {
        // Every device with something other than TrackedDevice_Invalid is associated with an actual physical device
        if (device_classes[i] != vr::TrackedDeviceClass_Invalid) {
            if (vr_.PoseIsValid(i) ) {
                // Get pose of current tracked device
                vr_.GetDevicePose(i, current_pose);

                // Populate and send transform message with pose of current tracked device
                transform_msg_.child_frame_id = device_frames[i];
                
                ConvertTransform(current_pose, tf_current_pose_);
                if (device_classes[i] != vr::TrackedDeviceClass_GenericTracker) {
                    tf2::convert(tf_current_pose_, transform_msg_.transform);
                } else {
                    // Correct pose if current device is a tracker
                    tf2::convert(tf_current_pose_ * tf_tracker_, transform_msg_.transform);
                }

                transform_msg_.header.stamp = ros::Time::now();
                tf_broadcaster_.sendTransform(transform_msg_);

                // Check if current tracked device is not a lighthouse (i.e. twist is available)
                if (device_classes[i] != vr::TrackedDeviceClass_TrackingReference && publish_twist) {
                    // Get linear and angular velocity of current device
                    vr_.GetDeviceVelocity(i, current_linvel, current_angvel);

                    // Populate and publish twist message
                    twist_msg_.header.frame_id = device_frames[i];

                    ConvertVector(current_linvel, tf_current_linvel_);
                    ConvertVector(current_angvel, tf_current_angvel_);
                    tf2::convert(tf_current_linvel_, twist_msg_.twist.linear);
                    tf2::convert(tf_current_angvel_, twist_msg_.twist.angular);

                    // Advertise twist topic if the tracked device is new
                    if (twist_pubs_map_.count(device_sns[i]) == 0) {
                        switch(device_classes[i]) {
                            case vr::TrackedDeviceClass_HMD:
                                twist_pubs_map_[device_sns[i]] = nh_.advertise<geometry_msgs::TwistStamped>("twist/hmd_"        + device_sns[i], 10);
                                break;
                            case vr::TrackedDeviceClass_Controller:
                                twist_pubs_map_[device_sns[i]] = nh_.advertise<geometry_msgs::TwistStamped>("twist/controller_" + device_sns[i], 10);
                                break;
                            case vr::TrackedDeviceClass_GenericTracker:
                                twist_pubs_map_[device_sns[i]] = nh_.advertise<geometry_msgs::TwistStamped>("twist/tracker_"    + device_sns[i], 10);
                                break;
                        }
                    }
                    twist_msg_.header.stamp = ros::Time::now();
                    twist_pubs_map_[device_sns[i]].publish(twist_msg_);
                }

                if (controller_interaction[i] && publish_joy) {
                    // Populate and publish joy message
                    joy_msg_.header.frame_id = device_frames[i];

                    vr_.GetControllerState(i, joy_msg_.axes, joy_msg_.buttons);
                    if (dpad_state[i] > 0) {
                        // Emulate D-pad if the touchpad button is pressed
                        if (joy_msg_.buttons[2]) {
                            joy_msg_.buttons[3 + dpad_state[i]] = 1;
                        }
                    }

                    // Advertise joy topic if the tracked device is new
                    if (joy_pubs_map_.count(device_sns[i]) == 0) {
                        joy_pubs_map_[device_sns[i]] = nh_.advertise<sensor_msgs::Joy>("joy/controller_" + device_sns[i], 10); break;
                    }
                    joy_msg_.header.stamp = ros::Time::now();
                    joy_pubs_map_[device_sns[i]].publish(joy_msg_);
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