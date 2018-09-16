// ROS
#include <ros/ros.h>
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

struct TrackedDevice {
      /**
     * Contains values for keeping track of tracked devices and controller user interaction
     */

    std::string serial_number;
    std::string frame_id;
    int device_class;

    // Controller user interaction
    bool button_touched;
    bool controller_interaction;
    // Emulated numpad state
    int numpad_state;
};

class ViveNode {
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;

    // Parameters
    bool send_tf, publish_joy, publish_twist;
    std::string vr_frame, robot_frame;
    double vr_x_offset, vr_y_offset, vr_z_offset, vr_yaw_offset, vr_pitch_offset, vr_roll_offset;
    double robot_x_offset, robot_y_offset, robot_z_offset, robot_yaw_offset, robot_pitch_offset, robot_roll_offset;
    bool InitParams();

    // Dynamic reconfigure
    dynamic_reconfigure::Server<vive_bridge::ViveConfig> reconf_server_;
    dynamic_reconfigure::Server<vive_bridge::ViveConfig>::CallbackType callback_type_;
    void ReconfCallback(vive_bridge::ViveConfig &config, uint32_t level);

    // Publishers
    ros::Publisher devices_pub_;
    // Associate ROS publishers with the serial numbers of tracked devices ("UID")
    std::map<std::string, ros::Publisher> twist_pubs_map_;
    std::map<std::string, ros::Publisher> joy_pubs_map_;

    // Services
    ros::ServiceServer devices_service_;
    bool ReturnTrackedDevices(vive_bridge::GetTrackedDevicesRequest &req,
                              vive_bridge::GetTrackedDevicesResponse &res);

    // Messages
    sensor_msgs::Joy joy_msg_;
    geometry_msgs::TwistStamped twist_msg_;
    geometry_msgs::TransformStamped vr_offset_msg_, robot_offset_msg_;
    geometry_msgs::TransformStamped transform_msg_;

    vive_bridge::TrackedDevicesStamped devices_msg_;
    void PublishTrackedDevices();

    // RViz
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;
    std::string lighthouse_mesh_path, tracker_mesh_path, controller_mesh_path, hmd_mesh_path;
    bool PublishMeshes();

    // tf2
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

    // Transform from fixed world frame to world_vr frame (i.e. fixed VR frame)
    tf2::Vector3 origin_offset_;
    tf2::Quaternion rotation_offset_;
    tf2::Transform tf_offset_;
    // Corrective transform for the tracker frames
    tf2::Vector3 origin_tracker_;
    tf2::Quaternion rotation_tracker_;
    tf2::Transform tf_tracker_;

    void ConvertVector(const float v[3], tf2::Vector3 &tf_v_);
    void ConvertTransform(const float m[3][4], tf2::Transform &tf_m_);
    void SendOffsetTransform();

    // OpenVR interface
    ViveInterface vr_;
    
    // Temporary values for keeping track of tracked devices
    int device_count;
    TrackedDevice TrackedDevices[vr::k_unMaxTrackedDeviceCount];
    void UpdateTrackedDevices();

    // Temporary values for handling events
    int event_type, event_device_index;

    int FindEmulatedNumpadState(float x, float y);

    // Temporary values for getting poses and velocities from tracked devices
    float current_pose[3][4];
    tf2::Matrix3x3 tf_m_basis;
    tf2::Vector3 tf_m_origin;
    tf2::Transform tf_current_pose_;

    float current_linvel[3], current_angvel[3];
    tf2::Vector3 tf_current_linvel_, tf_current_angvel_;
    
    public:
        ViveNode(int frequency);
        ~ViveNode();

        bool Init();
        void Loop();
        void Shutdown();
};