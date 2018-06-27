// ROS
#include <ros/ros.h>
// ROS msgs
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedback.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include "vr_ros/VRConfig.h"

// tf2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>


// OpenVR
#include "vr_ros/vr_interface.h"

// Functions for logging OpenVR messages to ROS
void HandleDebugMsgs(const std::string &msg) {ROS_DEBUG(" [VR] %s", msg.c_str() ); }
void HandleInfoMsgs (const std::string &msg) {ROS_INFO (" [VR] %s", msg.c_str() ); }
void HandleWarnMsgs (const std::string &msg) {ROS_WARN (" [VR] %s", msg.c_str() ); }
void HandleErrorMsgs(const std::string &msg) {ROS_ERROR(" [VR] %s", msg.c_str() ); }
void HandleFatalMsgs(const std::string &msg) {ROS_FATAL(" [VR] %s", msg.c_str() ); }

class VRNode {
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;

    // Parameters
    double x_offset, y_offset, z_offset, yaw_offset, pitch_offset, roll_offset;
    bool InitParams();

    // Associate ROS publishers with the serial numbers of tracked devices ("UID")
    std::map<std::string, ros::Publisher> twist_pubs_map_;
    std::map<std::string, ros::Publisher> joy_pubs_map_;

    // Dynamic reconfigure
    dynamic_reconfigure::Server<vr_ros::VRConfig> reconf_server_;
    dynamic_reconfigure::Server<vr_ros::VRConfig>::CallbackType callback_type_;
    void ReconfCallback(vr_ros::VRConfig &config, uint32_t level);

    // tf2
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    tf2::Vector3 ConvertVector(float v[3]);
    tf2::Transform ConvertTransform(float m[3][4]);
    // Transform from fixed world frame to world_vr frame (i.e. fixed VR frame)
    tf2::Vector3 origin_offset_;
    tf2::Quaternion rotation_offset_;
    tf2::Transform tf_offset_;
    void SendOffsetTransform();

    // OpenVR interface
    VRInterface vr_;

    // Temporary values for getting poses and velocities from tracked devices
    float current_pose[3][4];
    float current_linvel[3];
    float current_angvel[3];

    public:
        VRNode(int frequency);
        ~VRNode();

        bool Init();
        void Loop();
        void Shutdown();
};