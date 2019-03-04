#include <signal.h>

// ROS
#include <ros/ros.h>
#include <rosbag/bag.h>

#include <actionlib/client/simple_action_client.h>

// ROS msgs
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>
// Service msgs
#include <vive_calibrating/AddSample.h>
#include <vive_calibrating/ComputeCalibration.h>

// Action client
#include "vive_bridge/TrackedDevicesStamped.h"

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Config.h>

// tf2
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

// Eigen
#include <Eigen/Geometry>
#include <Eigen/SVD>

// dr_eigen
#include <average.h>

// MoveIt!
// #include <moveit/move_group_interface/move_group_interface.h>

// #include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/robot_model/robot_model.h>
// #include <moveit/robot_state/robot_state.h>

// #include "moveit_msgs/Constraints.h"

#include "robot_interface.h"

// Boost
#include <boost/algorithm/string/predicate.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// STL
#include <cmath>
#include <iterator>
#include <random>


class CalibratingNode {
    // ROS
    ros::NodeHandle nh_, pvt_nh_;
    ros::Rate loop_rate_;

    rosbag::Bag bag_;

    // Subscribers
    ros::Subscriber device_sub_;
    // Callback functions
    void DevicesCb(const vive_bridge::TrackedDevicesStamped& msg_);

    // Service
    ros::ServiceClient sample_client, compute_client;
    // Service msgs
    vive_calibrating::AddSample sample_srv;
    vive_calibrating::ComputeCalibration compute_srv;

    // msgs
    geometry_msgs::TransformStamped tf_msg_, tf_msg_pose_;

    geometry_msgs::TransformStamped tf_msg_tool0_, tf_msg_sensor_, tf_msg_diff_;
    geometry_msgs::TransformStamped tf_msg_A_, tf_msg_B_, tf_msg_X_, tf_msg_X_inv_;

    geometry_msgs::PoseStamped pose_msg_;

    // Parameters
    bool calibrate_flag, test_flag;
    int averaging_samples, calibration_stations;
    double sleep_duration, yaw_offset, pitch_offset, roll_offset;
    double radius_lower_bound,              radius_upper_bound,
           phi_position_lower_bound,        phi_position_upper_bound,
           phi_orientation_lower_bound,     phi_orientation_upper_bound,
           theta_position_lower_bound,      theta_position_upper_bound,
           theta_orientation_lower_bound,   theta_orientation_upper_bound;
    std::string vr_frame,   controller_frame, test_frame,
                base_frame, tool_frame,       world_frame;

    std::vector<double> joints_home;
    std::string planning_group;

    bool InitParams();

    // Reconfigure request for changing frame offset, e.g. changing world_vr frame w.r.t. world
    dynamic_reconfigure::ReconfigureRequest srv_reconf_req_;
    dynamic_reconfigure::ReconfigureResponse srv_reconf_resp_;

    RobotInterface* robot_;

    // MoveIt! plans
    std::vector<moveit::planning_interface::MoveGroupInterface::Plan> calibration_plans_, test_plans_;
    moveit::planning_interface::MoveGroupInterface::Plan plan_;

    // tf2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    tf2_ros::TransformListener *tf_listener_;
    // transforms
    tf2::Transform tf_tool0_[2], tf_sensor_[2];
    tf2::Transform tf_X_, tf_X_inv_;
    
    tf2::Transform tf_pose_, tf_controller_;

    bool CalibrateViveNode();
    void MeasureRobot(const int &N);

    bool SampleSensor(const std::string &target_frame, const std::string &source_frame,
                      const int &N, const int &F, geometry_msgs::TransformStamped &tf_msg_avg_);
    std::vector<Eigen::Vector3d> eigen_translations_;
    std::vector<Eigen::Quaterniond> eigen_rotations_;

    void ParkMartinExample();

    // Random number generator (RNG) for generating random poses
    std::uniform_real_distribution<double> r_dist, theta_dist1, phi_dist1, theta_dist2, phi_dist2;
    std::random_device random_seed1, random_seed2;
    std::mt19937_64 rng1, rng2;
    // Functions for generating random poses on a sphere
    geometry_msgs::Pose SphereNormalPose(double r, double theta, double phi, geometry_msgs::Pose &pose_);
    geometry_msgs::Pose GenerateRandomPose(geometry_msgs::Pose &pose_);

    // Test poses
    void FillTestPlanePlans(std::vector<moveit::planning_interface::MoveGroupInterface::Plan> &plans_, std::string frame_id, 
                            double L, double W, int n, int m, double x_offset, double y_offset, double z_offset, bool reverse_order);
    void ExecuteTestPlans(std::vector<moveit::planning_interface::MoveGroupInterface::Plan> &plans_);
    
    public:
        CalibratingNode(int frequency);
        ~CalibratingNode();

        bool Init();
        void Loop();
        void Shutdown();
};