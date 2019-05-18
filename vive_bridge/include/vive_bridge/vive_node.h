#pragma once

#include <signal.h>

// Boost
#include <boost/date_time/posix_time/posix_time.hpp>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
// ROS msgs
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedback.h>

#include <vive_bridge/TrackedDevicesStamped.h>
#include <vive_bridge/GetTrackedDevices.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include "vive_bridge/ViveConfig.h"

// RViz
#include <rviz_visual_tools/rviz_visual_tools.h>

// tf2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

// OpenVR
#include "vive_bridge/vive_interface.h"


// Functions for logging OpenVR messages to ROS
void HandleDebugMsgs(const std::string &msg) {ROS_DEBUG(" [VR] %s", msg.c_str() ); }
void HandleInfoMsgs (const std::string &msg) {ROS_INFO (" [VR] %s", msg.c_str() ); }
void HandleWarnMsgs (const std::string &msg) {ROS_WARN (" [VR] %s", msg.c_str() ); }
void HandleErrorMsgs(const std::string &msg) {ROS_ERROR(" [VR] %s", msg.c_str() ); }
void HandleFatalMsgs(const std::string &msg) {ROS_FATAL(" [VR] %s", msg.c_str() ); }


// Maximum number of tracked devices to keep track of
const unsigned char MAX_TRACKED_DEVICES = 8;

struct TrackedDevice {
      /**
     * Contains values for keeping track of tracked devices and controller interactions
     */

    std::string serial_number;

    // Controller user interaction
    bool button_touched;
    bool controller_interaction;
    // Controller haptic feedback
    bool haptic_enabled;
    ros::Time haptic_end_time;
    // Emulated numpad state
    unsigned char numpad_state;
};


class ViveNode {
    ros::NodeHandle nh_, pvt_nh_;
    ros::Rate loop_rate_;

    std::string PACKAGE_PATH, NODE_NAME;

    // Parameters
    std::string          world_frame,          vr_frame,
                lighthouse_mesh_path, tracker_mesh_path,
                controller_mesh_path,     hmd_mesh_path;
    double   vr_x_offset,     vr_y_offset,    vr_z_offset,
           vr_yaw_offset, vr_pitch_offset, vr_roll_offset;
    bool InitParams();

    // Dynamic reconfigure
    dynamic_reconfigure::Server<vive_bridge::ViveConfig> reconf_server_;
    dynamic_reconfigure::Server<vive_bridge::ViveConfig>::CallbackType callback_type_;
    void ReconfCallback(vive_bridge::ViveConfig &config, uint32_t level);

    // Command for dumping dynamic reconfigure parameters
    std::string cmd_dynparam_dump;

    // Publishers
    ros::Publisher devices_pub_;
    // Associate ROS publishers with the serial numbers of tracked devices ("UID")
    std::map<std::string, ros::Publisher> twist_pubs_map_;
    std::map<std::string, ros::Publisher> joy_pubs_map_;

    // Subscribers
    ros::Subscriber joy_feedback_sub_;
    void HapticFeedbackCallback(const sensor_msgs::JoyFeedback &msg_);

    // Services
    ros::ServiceServer devices_service_;
    bool ReturnTrackedDevices(vive_bridge::GetTrackedDevicesRequest  &req,
                              vive_bridge::GetTrackedDevicesResponse &res);

    // Messages
    sensor_msgs::Joy joy_msg_;
    geometry_msgs::TwistStamped twist_msg_;
    geometry_msgs::TransformStamped transform_msg_, vr_offset_msg_;

    // RViz
    rviz_visual_tools::RvizVisualToolsPtr rviz_tools_;
    bool PublishMeshes();

    // tf2
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

    // Transform from fixed world frame to world_vr frame (i.e. fixed VR frame)
    tf2::Quaternion orientation_offset_;
    tf2::Transform tf_offset_;

    void ConvertVector(const float v[3], tf2::Vector3 &tf_v_);
    void ConvertTransform(const float m[3][4], tf2::Transform &tf_m_);

    void SendOffsetTransform();

    // Temporary values for getting poses and velocities from tracked devices
    float current_pose[3][4];
    tf2::Transform tf_current_pose_;
    float current_linvel[3], current_angvel[3];
    tf2::Vector3 tf_current_linvel_, tf_current_angvel_;

    // Lighthouse position monitor
    bool looped_once;
    tf2::Transform tf_previous_poses_[MAX_TRACKED_DEVICES],
                   tf_difference_poses_[MAX_TRACKED_DEVICES];
    geometry_msgs::Transform tf_msg_difference_pose_;

    // OpenVR interface
    ViveInterface vr_;

    // State values for handling VR events
    unsigned int event_type, event_device_index;
    // State values for keeping track of tracked devices
    vive_bridge::TrackedDevicesStamped devices_msg_;
    TrackedDevice TrackedDevices[MAX_TRACKED_DEVICES];

    void UpdateTrackedDevices();
    void PublishTrackedDevices();

    unsigned char FindEmulatedNumpadState(const float &x, const float &y);
    bool TrackedDeviceIsChanged(const unsigned int &event_type);
    bool EventIsAvailable(unsigned int &event_type, unsigned int &device_index);
    
    public:
        ViveNode(int frequency);
        ~ViveNode();

        bool Init(int argc, char **argv);
        void Loop();
        void Shutdown();
};