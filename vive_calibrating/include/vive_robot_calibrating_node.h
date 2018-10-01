// ROS
#include <ros/ros.h>
#include <rosbag/bag.h>

// ROS msgs
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>

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

    // msgs
    geometry_msgs::TransformStamped tf_msg_;

    geometry_msgs::TransformStamped tf_msg_tool0_, tf_msg_sensor_;
    geometry_msgs::TransformStamped tf_msg_A_, tf_msg_B_;
    std::vector<geometry_msgs::Transform> tf_Avec_, tf_Bvec_;
    
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

    bool MoveRobot(const geometry_msgs::PoseStamped &pose_);

    // tf2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    tf2_ros::TransformListener *tf_listener_;

    tf2::Transform tf_tool0_[2], tf_sensor_[2];
    tf2::Transform tf_X_;
    
    tf2::Transform tf_pose_;

    // Eigen
    Eigen::Affine3d eigen_Ta_, eigen_Tb_;
    Eigen::Matrix3d eigen_Rx_, eigen_M_;
    Eigen::Vector3d eigen_tx_;

    Eigen::Vector3d RotationMatrixLogarithm(const Eigen::Matrix3d &rotmat_);

    geometry_msgs::TransformStamped ParkMartin(const geometry_msgs::Transform tf_Ta_[],
                                               const geometry_msgs::Transform tf_Tb_[],
                                               const int &size);
    void ParkMartinExample();

    Eigen::Matrix3d AntisymmetricMatrix(const Eigen::Vector3d &eigen_v_);
    Eigen::Matrix<double, 6, 8> ConstructSMatrix(const Eigen::Quaterniond &eigen_qrA_,
                                                 const Eigen::Quaterniond &eigen_qdA_,
                                                 const Eigen::Quaterniond &eigen_qrB_,
                                                 const Eigen::Quaterniond &eigen_qdB_);
    
    void TransformToDualQuaternion(const geometry_msgs::Transform &tf_msg_,
                                   Eigen::Quaterniond &eigen_qr_,
                                   Eigen::Quaterniond &eigen_qd_);
    Eigen::MatrixXd ConstructTMatrix(const geometry_msgs::Transform tf_Ta_[],
                                     const geometry_msgs::Transform tf_Tb_[],
                                     const int &size);

    geometry_msgs::Pose SphereNormalPose(double r, double theta, double phi, geometry_msgs::Pose &pose_);
    geometry_msgs::Pose GenerateRandomPose(geometry_msgs::Pose &pose_);
    void MeasureRobot(const int &N);

    // Random number generator (RNG) for generating random poses
    std::uniform_real_distribution<double> r_dist, theta_dist, phi_dist;
    std::random_device random_seed;
    std::mt19937_64 rng;

    public:
        CalibratingNode(int frequency);
        ~CalibratingNode();

        bool Init();
        void Loop();
        void Shutdown();
};
const std::string CalibratingNode::PLANNING_GROUP = "floor_manipulator";