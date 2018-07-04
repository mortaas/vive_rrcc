// ROS
#include <ros/ros.h>
// ROS msgs
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedback.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include "vive_bridge/ViveConfig.h"

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

class ViveNode {
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;

    // Parameters
    bool publish_joy, publish_twist;
    double x_offset, y_offset, z_offset, yaw_offset, pitch_offset, roll_offset;
    bool InitParams();

    // Dynamic reconfigure
    dynamic_reconfigure::Server<vive_bridge::ViveConfig> reconf_server_;
    dynamic_reconfigure::Server<vive_bridge::ViveConfig>::CallbackType callback_type_;
    void ReconfCallback(vive_bridge::ViveConfig &config, uint32_t level);

    // Associate ROS publishers with the serial numbers of tracked devices ("UID")
    std::map<std::string, ros::Publisher> twist_pubs_map_;
    std::map<std::string, ros::Publisher> joy_pubs_map_;

    // Messages
    sensor_msgs::Joy joy_msg_;
    geometry_msgs::TwistStamped twist_msg_;
    geometry_msgs::TransformStamped offset_msg_;
    geometry_msgs::TransformStamped transform_msg_;

    // tf2
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

    // Transform from fixed world frame to world_vr frame (i.e. fixed VR frame)
    tf2::Vector3 origin_offset_;
    tf2::Quaternion rotation_offset_;
    tf2::Transform tf_offset_;
    // Corrective transform for the tracker coordinate system
    tf2::Vector3 origin_tracker_;
    tf2::Quaternion rotation_tracker_;
    tf2::Transform tf_tracker_;

    void ConvertVector(const float v[3], tf2::Vector3 &tf_v_);
    void ConvertTransform(const float m[3][4], tf2::Transform &tf_m_);
    void SendOffsetTransform();

    // OpenVR interface
    ViveInterface vr_;
    // Temporary values for event handling
    int event_type, event_device_index;
    bool button_touched[vr::k_unMaxTrackedDeviceCount];
    bool controller_interaction[vr::k_unMaxTrackedDeviceCount];

    // Temporary values for getting poses and velocities from tracked devices
    float current_pose[3][4];
    float current_linvel[3], current_angvel[3];

    tf2::Matrix3x3 tf_m_basis;
    tf2::Vector3 tf_m_origin;

    tf2::Transform tf_current_pose_;
    tf2::Vector3 tf_current_linvel_, tf_current_angvel_;
    
    public:
        ViveNode(int frequency);
        ~ViveNode();

        bool Init();
        void Loop();
        void Shutdown();
};