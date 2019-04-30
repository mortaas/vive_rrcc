#pragma once

// ROS
#include <ros/ros.h>

// msgs
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <moveit_msgs/Constraints.h>
#include <moveit_msgs/RobotTrajectory.h>

#include <trajectory_msgs/JointTrajectoryPoint.h>

// tf2
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include "moveit_msgs/Constraints.h"

// Eigen
#include <Eigen/Geometry>

// STL
#include <iterator>


class RobotInterface {
    // msgs
    geometry_msgs::TransformStamped tf_msg_;
    geometry_msgs::PoseStamped pose_msg_;

    // MoveIt!
    moveit::planning_interface::MoveGroupInterface move_group_;
    // RobotState
    robot_model::RobotModelConstPtr kinematic_model_;
    robot_state::RobotStatePtr kinematic_state_;

    std::vector<std::string> joint_names;

    // Eigen
    Eigen::Affine3d eigen_FK_, eigen_model_planning_;

    // tf2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

    tf2::Transform tf_pose_;
    
    public:
        RobotInterface(const std::string &PLANNING_GROUP,
                       const std::string &PLANNING_FRAME);
        ~RobotInterface();

        void SetPoseTarget(const geometry_msgs::PoseStamped &pose_);
        void SetJointValueTarget(const std::vector<double> &joint_state_);
        void SetStartStateToCurrentState();

        void GetCurrentPose(geometry_msgs::PoseStamped &pose_);

        bool MoveIt();
        bool GetPlan(moveit::planning_interface::MoveGroupInterface::Plan &plan_);
        bool ExecutePlan(const moveit::planning_interface::MoveGroupInterface::Plan &plan_);
};