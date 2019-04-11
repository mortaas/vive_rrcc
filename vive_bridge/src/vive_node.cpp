#include "vive_bridge/vive_node.h"

// Handle signal [ctrl + c]
bool sigint_flag = true;

void IntHandler(int signal) {
    sigint_flag = false;
}


ViveNode::ViveNode(int frequency)
    : pvt_nh_("~"),
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

    PACKAGE_PATH = ros::package::getPath("vive_bridge");
    NODE_NAME = ros::this_node::getName();

    // System commands for dumping and loading dynamic reconfigure parameters
    // Temporary workaround as there is no C++ API for dynamic reconfigure
    cmd_dynparam_dump = "rosrun dynamic_reconfigure dynparam dump " + NODE_NAME + 
                        " " + PACKAGE_PATH + "/cfg/dynparam.yaml";

    // Publisher and service for info about tracked devices
    devices_pub_ = pvt_nh_.advertise<vive_bridge::TrackedDevicesStamped>("tracked_devices", 10, true);
    devices_service_ = pvt_nh_.advertiseService("tracked_devices", &ViveNode::ReturnTrackedDevices, this);

    joy_feedback_sub_ = pvt_nh_.subscribe("joy/haptic_feedback", 10, &ViveNode::HapticFeedbackCallback, this);

    // Initialize rviz_visual_tools for publishing tracked device meshes to RViz
    rviz_tools_.reset(new rviz_visual_tools::RvizVisualTools(world_frame, "rviz_mesh_markers") );
    rviz_tools_->loadMarkerPub(false, true);
    rviz_tools_->enableFrameLocking();
    rviz_tools_->setLifetime(0);

    // Initialize data for tracked devices
    for (int i = 0; i < MAX_TRACKED_DEVICES; i++) {
        TrackedDevices[i].serial_number = "";
        TrackedDevices[i].button_touched = false;
        TrackedDevices[i].controller_interaction = false;
        TrackedDevices[i].numpad_state = 5;
    }
    
    // Initialize message structures
    devices_msg_.device_classes.resize(MAX_TRACKED_DEVICES);
    devices_msg_.device_roles.resize(MAX_TRACKED_DEVICES);
    devices_msg_.device_frames.resize(MAX_TRACKED_DEVICES);

    joy_msg_.axes.resize(3);
    joy_msg_.buttons.resize(13);

    looped_once = false;
}

unsigned char ViveNode::FindEmulatedNumpadState(const float &x, const float &y) {
     /**
      * Find the D-pad state of a controller given it's x and y touch coordinates.
      * The D-pad states corresponds to numpad keys, i.e. up is 8, right is 6, down-left is 1 and so on.
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

bool ViveNode::TrackedDeviceIsChanged(const unsigned int &event_type) {
     /**
      * Returns true if there are changes to the state of a tracked device
      */
    
    return (event_type == vr::VREvent_TrackedDeviceActivated ||
            event_type == vr::VREvent_TrackedDeviceDeactivated ||
            event_type == vr::VREvent_TrackedDeviceUpdated ||
            event_type == vr::VREvent_TrackedDeviceRoleChanged);
}

bool ViveNode::EventIsAvailable(unsigned int &event_type, unsigned int &device_index) {
     /**
      * Returns true if there is an event available in the event queue
      */
    
    return (event_type != vr::VREvent_None &&
            event_device_index != vr::k_unTrackedDeviceIndexInvalid);
}

void ViveNode::HapticFeedbackCallback(const sensor_msgs::JoyFeedback &msg_) {
     /**
      * Callback for triggering haptic feedback on a VIVE controller
      */
    
    if (msg_.type == msg_.TYPE_RUMBLE) {
        // Check if device is a controller
        if (devices_msg_.device_classes[msg_.id] == devices_msg_.CONTROLLER) {
            TrackedDevices[msg_.id].haptic_enabled = true;
            TrackedDevices[msg_.id].haptic_end_time = ros::Time::now() + ros::Duration(msg_.intensity);
        }
    }
}

bool ViveNode::ReturnTrackedDevices(vive_bridge::GetTrackedDevicesRequest &req,
                                    vive_bridge::GetTrackedDevicesResponse &res)
{
     /**
      * Service callback for getting info about tracked devices
      */
    
    // Ensure that we respond with current info
    UpdateTrackedDevices();

    // Fill response message with info about tracked devices
    res.frame_id = vr_frame;
    res.device_count = devices_msg_.device_count;
    res.device_classes.resize(devices_msg_.device_count);
    res.device_roles.resize(devices_msg_.device_count);
    res.device_frames.resize(devices_msg_.device_count);
    for (int i = 0; i < devices_msg_.device_count; i++) {
        res.device_classes[i] = devices_msg_.device_classes[i];
        res.device_roles[i] = devices_msg_.device_roles[i];
        res.device_frames[i]  = devices_msg_.device_frames[i];
    }

    return true;
}

bool ViveNode::PublishMeshes() {
     /**
      * Publish meshes of tracked devices to visualize them as a MarkerArray in RViz
      */
    
    rviz_tools_->resetMarkerCounts();

    for (int i = 0; i < devices_msg_.device_count; i++) {
        rviz_tools_->setBaseFrame(devices_msg_.device_frames[i]);

        switch (devices_msg_.device_classes[i]) {
            case vr::TrackedDeviceClass_Invalid:
                break;
            case vr::TrackedDeviceClass_HMD:
                rviz_tools_->publishMesh(Eigen::Affine3d::Identity(),
                                         hmd_mesh_path,
                                         rviz_visual_tools::WHITE,
                                         1,
                                         devices_msg_.device_frames[i]);
                break;
            case vr::TrackedDeviceClass_Controller:
                rviz_tools_->publishMesh(Eigen::Affine3d::Identity(),
                                         controller_mesh_path,
                                         rviz_visual_tools::WHITE,
                                         1.0,
                                         devices_msg_.device_frames[i]);
                break;
            case vr::TrackedDeviceClass_GenericTracker:
                rviz_tools_->publishMesh(Eigen::Affine3d::Identity(),
                                         tracker_mesh_path,
                                         rviz_visual_tools::BLACK,
                                         1,
                                         devices_msg_.device_frames[i]);
                break;
            case vr::TrackedDeviceClass_TrackingReference:
                rviz_tools_->publishMesh(Eigen::Affine3d::Identity(),
                                         lighthouse_mesh_path,
                                         rviz_visual_tools::WHITE,
                                         1,
                                         devices_msg_.device_frames[i]);
                break;
        }
    }

    rviz_tools_->trigger();
    return true;
}

ViveNode::~ViveNode() {
}

bool ViveNode::InitParams() {
     /**
      * Retrieve parameters from the parameter server.
      * Returns true if the parameters was retrieved from the server, false otherwise.
      */
    
    return (pvt_nh_.param<std::string>("hmd_mesh_path",           hmd_mesh_path,
                                       "package://vive_bridge/meshes/vr_hmd_vive_2_0/vr_hmd_vive_2_0.dae")               &&
            pvt_nh_.param<std::string>("controller_mesh_path",    controller_mesh_path,
                                       "package://vive_bridge/meshes/vr_controller_vive_1_5/vr_controller_vive_1_5.dae") &&
            pvt_nh_.param<std::string>("tracker_mesh_path",       tracker_mesh_path,
                                       "package://vive_bridge/meshes/TRACKER-3D.dae")                                    &&
            pvt_nh_.param<std::string>("lighthouse_mesh_path",    lighthouse_mesh_path,
                                       "package://vive_bridge/meshes/lh_basestation_vive/lh_basestation_vive.dae")       &&
            
            pvt_nh_.param<std::string>("world_frame", world_frame,       "root")     &&
            pvt_nh_.param<std::string>("vr_frame",    vr_frame,          "world_vr") );
}

void ViveNode::ReconfCallback(vive_bridge::ViveConfig &config, uint32_t level) {
     /**
      * Dynamic reconfigure callback for changing parameters during runtime
      */

    vr_x_offset        = config.vr_x_offset;
    vr_y_offset        = config.vr_y_offset;
    vr_z_offset        = config.vr_z_offset;
    vr_yaw_offset      = config.vr_yaw_offset;
    vr_pitch_offset    = config.vr_pitch_offset;
    vr_roll_offset     = config.vr_roll_offset;
    
    SendOffsetTransform();
}

void ViveNode::SendOffsetTransform() {
      /**
     * Update and send a static offset transform from our inertial world frame
     * to the world_vr frame (i.e. the inertial VR frame)
     */

    orientation_offset_.setRPY(vr_roll_offset,
                            vr_pitch_offset,
                            vr_yaw_offset);

    tf_offset_.setOrigin(tf2::Vector3(vr_x_offset,
                                      vr_y_offset,
                                      vr_z_offset) );
    tf_offset_.setRotation(orientation_offset_);

    tf2::convert(tf_offset_, vr_offset_msg_.transform);

    vr_offset_msg_.header.stamp = ros::Time::now();
    static_tf_broadcaster_.sendTransform(vr_offset_msg_);
}

void ViveNode::ConvertTransform(const float m[3][4], tf2::Transform &tf_m_) {
      /**
     * Convert pose from OpenVR to tf2 transform
     */

    tf_m_.setBasis(tf2::Matrix3x3(m[0][0], m[0][1], m[0][2],
                                  m[1][0], m[1][1], m[1][2],
                                  m[2][0], m[2][1], m[2][2]) );
    tf_m_.setOrigin( tf2::Vector3(m[0][3], m[1][3], m[2][3]) );
}
void ViveNode::ConvertVector(const float v[3], tf2::Vector3 &tf_v_) {
     /**
      * Convert vector from OpenVR to tf2 vector
      */

    tf_v_.setValue(v[0], v[1], v[2]);
}

void ViveNode::UpdateTrackedDevices() {
      /**
     * Update tracked device count and update arrays for keeping track of these devices
     */

    devices_msg_.device_count = 0;

    // Loop through all possible device slots (indices)
    for (int i = 0; i < MAX_TRACKED_DEVICES; i++) {
        devices_msg_.device_classes[i] = vr_.GetDeviceClass(i);
        devices_msg_.device_roles[i] = vr_.GetControllerRole(i);

        if (devices_msg_.device_classes[i] != vr::TrackedDeviceClass_Invalid) {
            devices_msg_.device_count++;

            // Get serial number of current tracked device as unique identifier (UID)
            vr_.GetDeviceSN(i, TrackedDevices[i].serial_number);
            std::replace(TrackedDevices[i].serial_number.begin(), TrackedDevices[i].serial_number.end(), '-', '_');
            
            // Find name of tracked device frame
            switch(devices_msg_.device_classes[i]) {
                case vr::TrackedDeviceClass_HMD:                devices_msg_.device_frames[i] =
                                                                    "hmd_" + TrackedDevices[i].serial_number;
                                                                break;
                case vr::TrackedDeviceClass_Controller:         devices_msg_.device_frames[i] =
                                                                    "controller_" + TrackedDevices[i].serial_number;
                                                                break;
                case vr::TrackedDeviceClass_GenericTracker:     devices_msg_.device_frames[i] =
                                                                    "tracker_" + TrackedDevices[i].serial_number;
                                                                break;
                case vr::TrackedDeviceClass_TrackingReference:  devices_msg_.device_frames[i] =
                                                                    "lighthouse_" + TrackedDevices[i].serial_number;
                                                                break;
            }
        } else {
            devices_msg_.device_frames[i] = "empty" + TrackedDevices[i].serial_number;
        }
    }
}

bool ViveNode::Init(int argc, char **argv) {
      /**
     * Initialize OpenVR and the node
     */

    if (vr_.Init(argc, argv) ) {
        // Initialize parameters
        if (!InitParams() ) {
            ROS_WARN_STREAM("Failed to get parameters from the parameter server. \n" <<
                            "Using default parameters.");
        }

        // Update message headers
        devices_msg_.header.frame_id = vr_frame;
        transform_msg_.header.frame_id = vr_frame;
        vr_offset_msg_.header.frame_id = world_frame;
        vr_offset_msg_.child_frame_id = vr_frame;

        // Update and publish info about tracked devices
        vr_.Update();
        UpdateTrackedDevices();
        PublishTrackedDevices();

        // Set dynamic reconfigure callback function
        callback_type_ = boost::bind(&ViveNode::ReconfCallback, this, _1, _2);
        reconf_server_.setCallback(callback_type_);

        return true;
    }

    return false;
}

void ViveNode::PublishTrackedDevices() {
      /**
     * Publish information about the currently tracked devices
     */

    devices_msg_.header.stamp = ros::Time::now();
    devices_pub_.publish(devices_msg_);

    // Publish meshes in case of changes to the tracked devices
    PublishMeshes();
}

void ViveNode::Shutdown() {
      /**
     * Shuts down the connection to the VR hardware and cleans up the OpenVR API
     */

    // Dump dynamic reconfigure parameters
    system(cmd_dynparam_dump.c_str() );

    vr_.Shutdown();
    ros::shutdown();
}

void ViveNode::Loop() {
      /**
     * Main loop of the node
     */

    // Calculate updated poses for all tracked devices
    vr_.Update();

    // Check if there are any events in the OpenVR queue
    vr_.PollNextEvent(event_type, event_device_index);

    // Update controller interaction flags (clears flag if the buttons is not touched)
    for (int i = 0; i < devices_msg_.device_count; i++) {
        TrackedDevices[i].controller_interaction = false || TrackedDevices[i].button_touched;

        // Haptic feedback
        if (TrackedDevices[i].haptic_enabled) {
            if (ros::Time::now() < TrackedDevices[i].haptic_end_time) {
                vr_.TriggerHapticPulse(i, 3999);
            }
        }
    }

    if (EventIsAvailable(event_type, event_device_index) )
    {
        // Update info about tracked devices if there are changes
        if (TrackedDeviceIsChanged(event_type) )
        {
            UpdateTrackedDevices();
        }

        // Handle controller events
        switch (event_type) {
            case vr::VREvent_ButtonPress:   TrackedDevices[event_device_index].controller_interaction = true;
                                            break;
            case vr::VREvent_ButtonUnpress: TrackedDevices[event_device_index].controller_interaction = true;
                                            break;
            case vr::VREvent_ButtonTouch:   TrackedDevices[event_device_index].button_touched = true;
                                            vr_.GetControllerState(event_device_index,
                                                                    joy_msg_.axes,
                                                                    joy_msg_.buttons);
                                            TrackedDevices[event_device_index].numpad_state =
                                                FindEmulatedNumpadState(joy_msg_.axes[0], joy_msg_.axes[1]);
                                            break;
            case vr::VREvent_ButtonUntouch: TrackedDevices[event_device_index].button_touched = false;
                                            TrackedDevices[event_device_index].numpad_state = 0;
                                            break;
        }
    }

    // Loop through all tracked devices (indices)
    for (unsigned char i = 0; i < devices_msg_.device_count; i++) {
        // Every device with something other than TrackedDevice_Invalid is associated with a physical device
        if (devices_msg_.device_classes[i] != vr::TrackedDeviceClass_Invalid) {
            if (vr_.PoseIsValid(i) ) {
                // Get pose of current tracked device
                vr_.GetDevicePose(i, current_pose);

                // Populate and send transform message with pose of current tracked device
                transform_msg_.child_frame_id = devices_msg_.device_frames[i];
                
                ConvertTransform(current_pose, tf_current_pose_);
                tf2::convert(tf_current_pose_, transform_msg_.transform);

                // Check if the pose has a valid unit quaternion
                // tf2Scalar sq_length = tf_current_pose_.getRotation().length2();
                // if (sq_length >= 0.9999 && sq_length <= 1.0001) {
                //     transform_msg_.header.stamp = ros::Time::now();
                //     tf_broadcaster_.sendTransform(transform_msg_);
                // }

                transform_msg_.header.stamp = ros::Time::now();
                tf_broadcaster_.sendTransform(transform_msg_);

                // Check if current tracked device is not a lighthouse (i.e. twist is available)
                if (devices_msg_.device_classes[i] != vr::TrackedDeviceClass_TrackingReference) {
                    // Get linear and angular velocity of current device
                    vr_.GetDeviceVelocity(i, current_linvel, current_angvel);

                    // Populate and publish twist message
                    twist_msg_.header.frame_id = devices_msg_.device_frames[i];

                    ConvertVector(current_linvel, tf_current_linvel_);
                    ConvertVector(current_angvel, tf_current_angvel_);
                    tf2::convert(tf_current_linvel_, twist_msg_.twist.linear);
                    tf2::convert(tf_current_angvel_, twist_msg_.twist.angular);

                    // Advertise twist topic if the tracked device is new
                    if (twist_pubs_map_.count(TrackedDevices[i].serial_number) == 0) {
                        switch(devices_msg_.device_classes[i]) {
                            case vr::TrackedDeviceClass_HMD:
                                twist_pubs_map_[TrackedDevices[i].serial_number] =
                                    pvt_nh_.advertise<geometry_msgs::TwistStamped>("twist/hmd_" +
                                                                               TrackedDevices[i].serial_number, 10);
                                break;
                            case vr::TrackedDeviceClass_Controller:
                                twist_pubs_map_[TrackedDevices[i].serial_number] =
                                    pvt_nh_.advertise<geometry_msgs::TwistStamped>("twist/controller_" +
                                                                               TrackedDevices[i].serial_number, 10);
                                break;
                            case vr::TrackedDeviceClass_GenericTracker:
                                twist_pubs_map_[TrackedDevices[i].serial_number] =
                                    pvt_nh_.advertise<geometry_msgs::TwistStamped>("twist/tracker_" +
                                                                               TrackedDevices[i].serial_number, 10);
                                break;
                        }
                    }
                    twist_msg_.header.stamp = ros::Time::now();
                    twist_pubs_map_[TrackedDevices[i].serial_number].publish(twist_msg_);
                } else {
                    // Compute difference between current and previous pose
                    tf_difference_poses_[i] = tf_current_pose_.inverseTimes(tf_previous_poses_[i]);
                    tf_previous_poses_[i] = tf_current_pose_;

                    // Lighthouse pose monitor
                    if (tf_difference_poses_[i].getOrigin().length() >= 1e-6 && looped_once)
                    {
                        ROS_WARN_STREAM("[LIGHTHOUSE MONITOR] Pose of " << devices_msg_.device_frames[i] <<
                                        " has changed! \nThe pose of tracked devices has likely also changed.");
                    }
                }

                // Handle controller input
                if (TrackedDevices[i].controller_interaction) {
                    // Populate and publish joy message
                    joy_msg_.header.frame_id = devices_msg_.device_frames[i];

                    vr_.GetControllerState(i, joy_msg_.axes, joy_msg_.buttons);
                    if (TrackedDevices[i].numpad_state > 0) {
                        // Emulate D-pad if the touchpad button is pressed
                        if (joy_msg_.buttons[2]) {
                            joy_msg_.buttons[3 + TrackedDevices[i].numpad_state] = 1;
                        }
                    }

                    // Advertise joy topic if the tracked device is new
                    if (joy_pubs_map_.count(TrackedDevices[i].serial_number) == 0) {
                        joy_pubs_map_[TrackedDevices[i].serial_number] =
                            pvt_nh_.advertise<sensor_msgs::Joy>("joy/controller_" +
                                                            TrackedDevices[i].serial_number, 10);
                        break;
                    }
                    joy_msg_.header.stamp = ros::Time::now();
                    joy_pubs_map_[TrackedDevices[i].serial_number].publish(joy_msg_);
                }
            }
        } else {
            // Contine to next device if device class of the current device is invalid
            continue;
        }
    }

    if (EventIsAvailable(event_type, event_device_index) )
    {
        // Publish info about tracked devices if there are changes
        if (TrackedDeviceIsChanged(event_type) )
        {
            PublishTrackedDevices();
            PublishMeshes();
        }
    }

    looped_once = true;

    ros::spinOnce();
    loop_rate_.sleep();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vive_node");

    ViveNode node_(240);

    // Handle signal [ctrl + c]
    signal(SIGINT, IntHandler);

    if (!node_.Init(argc, argv) ) {
        node_.Shutdown();

        // Handle sigint
        if (sigint_flag) {
            exit(EXIT_SUCCESS);
        } else {
            exit(EXIT_FAILURE);
        }
    }

    while (ros::ok() && sigint_flag) {
        node_.Loop();
    }

    node_.Shutdown();
    exit(EXIT_SUCCESS);
}