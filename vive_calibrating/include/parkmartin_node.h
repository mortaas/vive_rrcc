// ROS
#include <ros/ros.h>

// ROS msgs
#include <geometry_msgs/PoseStamped.h>
// Service msgs
#include <vive_calibrating/AddSample.h>
#include <vive_calibrating/ComputeCalibration.h>

// tf2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

// Eigen
#include <Eigen/Geometry>
#include <Eigen/SVD>


class ParkMartinNode {
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;

    // Service
    ros::ServiceServer sample_service_, compute_service_;

    // tf2
    geometry_msgs::TransformStamped tf_X_;

    // Eigen
    std::vector<Eigen::Matrix3d> eigen_Ra_, eigen_Rb_;
    std::vector<Eigen::Vector3d> eigen_ta_, eigen_tb_;

    // Temporary matrices for solving the calibration problem
    Eigen::Affine3d eigen_Ta_, eigen_Tb_;
    Eigen::Matrix3d eigen_Rx_, eigen_M_;
    Eigen::Vector3d eigen_tx_;

    int n_samples;
    
    bool AddSample(vive_calibrating::AddSample::Request  &req,
                   vive_calibrating::AddSample::Response &res);
    bool ComputeCalibration(vive_calibrating::ComputeCalibration::Request  &req,
                            vive_calibrating::ComputeCalibration::Response &res);

    Eigen::Vector3d RotationMatrixLogarithm(const Eigen::Matrix3d &rotmat_);
    bool ParkMartin();
    void ParkMartinExample();

    public:
        ParkMartinNode(int frequency);
        ~ParkMartinNode();

        bool Init();
        void Loop();
        void Shutdown();
};