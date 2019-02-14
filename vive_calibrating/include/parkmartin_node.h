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
// Sophus - C++ implementation of Lie Groups using Eigen
#include <sophus/se3.hpp>

#include "sophus_ros_conversions/eigen.hpp"
#include "sophus_ros_conversions/geometry.hpp"

// Ceres solver
#include <ceres/ceres.h>


class ParkMartinNode {
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;

    // Service
    ros::ServiceServer sample_service_, compute_service_;

    // tf2
    geometry_msgs::TransformStamped tf_X_;

    // Sophus
    std::vector<Sophus::SE3f> sophus_Ta_, sophus_Tb_;
    // Solution
    Sophus::SO3f sophus_Rx_;
    Sophus::Vector3f sophus_tx_;
    Sophus::SE3f sophus_Tx_;

    int n_samples;
    
    bool AddSample(vive_calibrating::AddSample::Request  &req,
                   vive_calibrating::AddSample::Response &res);
    bool ComputeCalibration(vive_calibrating::ComputeCalibration::Request  &req,
                            vive_calibrating::ComputeCalibration::Response &res);
    
    bool ParkMartin();
    bool HoraudDornaika();
    void ParkMartinExample();

    public:
        ParkMartinNode(int frequency);
        ~ParkMartinNode();

        bool Init();
        void Loop();
        void Shutdown();
};