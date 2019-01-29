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

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include "moveit_msgs/Constraints.h"

// Boost
#include <boost/algorithm/string/predicate.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

// STL
#include <cmath>
#include <random>


class CalibratingNode {
    // ROS
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;

    // Subscribers
    ros::Subscriber joy_sub_;
    ros::Subscriber device_sub_;
    // Callback functions
    void JoyCb(const sensor_msgs::Joy& msg_);
    void DevicesCb(const vive_bridge::TrackedDevicesStamped& msg_);

    // Publisher
    ros::Publisher traj_pub_;

    // Service
    ros::ServiceClient sample_client, compute_client;

    // Action client
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *action_client_;

    // msgs
    control_msgs::FollowJointTrajectoryGoal traj_goal_msg_;
    geometry_msgs::TransformStamped tf_msg_;

    geometry_msgs::TransformStamped tf_msg_tool0_, tf_msg_sensor_;
    geometry_msgs::TransformStamped tf_msg_A_, tf_msg_B_, tf_msg_X_inv_;
    // std::vector<geometry_msgs::Transform> tf_Avec_, tf_Bvec_;
    
    geometry_msgs::TransformStamped tf_msg_pose_;
    geometry_msgs::PoseStamped pose_msg_;

    rosbag::Bag bag_;

    // Parameters
    double yaw_offset, pitch_offset, roll_offset;
    std::string controller_frame;

    bool InitParams();

    // Reconfigure request for changing frame offset, e.g. changing world_vr frame w.r.t. world
    dynamic_reconfigure::ReconfigureRequest srv_reconf_req_;
    dynamic_reconfigure::ReconfigureResponse srv_reconf_resp_;

    // MoveIt!
    moveit::planning_interface::MoveGroupInterface move_group_;
    static const std::string PLANNING_GROUP;
    // RobotState
    robot_model_loader::RobotModelLoader robot_model_loader_;
    robot_model::RobotModelPtr kinematic_model_;
    robot_state::RobotStatePtr kinematic_state_;
    const robot_state::JointModelGroup* joint_model_group_;

    bool MoveRobot(const geometry_msgs::PoseStamped &pose_);
    std::vector<double> joint_values, joint_folded;

    bool CalibrateViveNode();

    // tf2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    tf2_ros::TransformListener *tf_listener_;

    // transforms
    tf2::Transform tf_tool0_[2], tf_sensor_[2];
    tf2::Transform tf_X_, tf_X_inv_;
    
    tf2::Transform tf_pose_, tf_controller_;

    void MeasureRobot(const int &N);

    // Eigen
    Eigen::Affine3d eigen_Ta_, eigen_Tb_;
    Eigen::Matrix3d eigen_Rx_, eigen_M_;
    Eigen::Vector3d eigen_tx_;

    Eigen::Vector3d RotationMatrixLogarithm(const Eigen::Matrix3d &rotmat_);

    geometry_msgs::TransformStamped ParkMartin(const geometry_msgs::Transform tf_Ta_[],
                                               const geometry_msgs::Transform tf_Tb_[],
                                               const int &size);
    void ParkMartinExample();

    // Random number generator (RNG) for generating random poses
    std::uniform_real_distribution<double> r_dist, theta_dist1, phi_dist1, theta_dist2, phi_dist2;
    std::random_device random_seed1, random_seed2;
    std::mt19937_64 rng1, rng2;

    geometry_msgs::Pose SphereNormalPose(double r, double theta, double phi, geometry_msgs::Pose &pose_);
    geometry_msgs::Pose GenerateRandomPose(geometry_msgs::Pose &pose_);

    void FillTestPlanePoses(std::vector<geometry_msgs::PoseStamped> &poses_, std::string frame_id, 
                            double L, int n, double x_offset, double y_offset, double z_offset);
    void ExecuteTestPoses(std::vector<geometry_msgs::PoseStamped> &poses_);

    // bool GetJointPositionsFromIK(const geometry_msgs::PoseStamped &pose_, std::vector<double> joint_values);
    
    public:
        CalibratingNode(int frequency);
        ~CalibratingNode();

        bool Init();
        void Loop();
        void Shutdown();
};
const std::string CalibratingNode::PLANNING_GROUP = "floor_manipulator";