#pragma once

#include <signal.h>

// ROS
#include <ros/ros.h>
// ROS msgs
#include <vive_bridge/TrackedDevicesStamped.h>
#include <sensor_msgs/JoyFeedback.h>

// MoveIt!
// #include <moveit_msgs/ApplyPlanningScene.h>
// #include <moveit_msgs/PlanningScene.h>

// tf2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// Eigen
#include "tf2_eigen/tf2_eigen.h"
#include <Eigen/Dense>

// RViz
#include <rviz_visual_tools/rviz_visual_tools.h>

// SDFormat
#include "vive_planning_scene/sdf_interface.h"

// Ceres NLS solver solver
#include <ceres/ceres.h>
#include "ceres_cost_functors.h"

// Sophus - C++ implementation of Lie Groups using Eigen
#include <sophus/se3.hpp>

#include "sophus_ros_conversions/eigen.hpp"
#include "sophus_ros_conversions/geometry.hpp"


// Handle signal [ctrl + c]
bool sigint_flag = true;

void IntHandler(int signal) {
    sigint_flag = false;
}


enum E_DefineStates {
    STATE_DEFINE_POINT = 0,
    STATE_DEFINE_LINE,
    STATE_DEFINE_PLANE,
    STATE_DEFINE_BOX,
    STATE_DEFINE_BOX_POINT,
    STATE_DEFINE_BOX_LINE,
    STATE_DEFINE_BOX_PLANE,
    STATE_DEFINE_BOX_HEIGHT,
    STATE_DEFINE_SPHERE,
    STATE_DEFINE_CYLINDER,
    STATE_DEFINE_CYLINDER_BOTTOM,
    STATE_DEFINE_CYLINDER_TOP,
    STATE_DEFINE_CONE,
    STATE_DEFINE_CONE_HEIGHT
};

class SceneNode {
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;

    // Publishers
    ros::Publisher joy_feedback_pub_;

    // Subscribers
    ros::Subscriber devices_sub_;
    ros::Subscriber joy_sub_;

    void DevicesCb(const vive_bridge::TrackedDevicesStamped& msg_);
    void JoyCb(const sensor_msgs::Joy& msg_);

    // ROS msgs
    geometry_msgs::TransformStamped tf_msg_;
    sensor_msgs::JoyFeedback joy_feedback_msg_;

    // ros::ServiceClient planning_scene_client_;
    // moveit_msgs::ApplyPlanningScene planning_scene_srv_;
    // moveit_msgs::PlanningScene planning_scene_;

    // RViz
    rviz_visual_tools::RvizVisualToolsPtr rviz_tools_, rviz_infinite_tools_;

    // SDFormat interface
    SDFinterface* sdf_;

    // tf2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener *tf_listener_;

    tf2::Transform tf_tracker_;
    std::string controller_frame, tool_frame, tracker_frame, world_frame;
    unsigned int controller_id, tracker_id;

    // Eigen
    Eigen::Affine3d eigen_msg_, eigen_pose_, eigen_controller_offset_;

    std::vector<Eigen::Vector3d> points_;
    Eigen::Vector3d vecs_[6], basis_[3], projected_point_, rpy_angles_;
    Eigen::Vector3d eigen_a_, eigen_b_, eigen_point_;
    double angle, height, radius, scale;

    unsigned int state;

    public:
        SceneNode(int frequency);
        ~SceneNode();

        bool Init();
        void Loop();
        void Shutdown();
};