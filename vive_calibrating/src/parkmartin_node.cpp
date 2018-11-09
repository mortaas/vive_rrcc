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

    public:
        ParkMartinNode(int frequency);
        ~ParkMartinNode();

        bool Init();
        void Loop();
        void Shutdown();
};

ParkMartinNode::ParkMartinNode(int frequency)
    : loop_rate_(frequency)
{   
    // Number of sampled pairs (A, B)
    n_samples = 0;

    // Advertise services
    sample_service_  = nh_.advertiseService("/vive_calibration/add_sample",
                                            &ParkMartinNode::AddSample, this);
    compute_service_ = nh_.advertiseService("/vive_calibration/compute_calibration",
                                            &ParkMartinNode::ComputeCalibration, this);
}
ParkMartinNode::~ParkMartinNode() {
}

bool ParkMartinNode::Init() {
      /**
     * Initialize the node
     */

    return true;
}

void ParkMartinNode::Loop() {
      /**
     * Main loop of the node
     */
    
    ros::spinOnce();
    loop_rate_.sleep();
}

void ParkMartinNode::Shutdown() {
      /**
     * Runs before shutting down the node
     */

}

bool ParkMartinNode::AddSample(vive_calibrating::AddSample::Request  &req,
                               vive_calibrating::AddSample::Response &res)
{
      /**
     * Adds the provided pair (A, B) to an internal vector.
     * Both transformations are decomposed into their internal representations:
     * Rotations    (Eigen::Matrix3d) - Ra[], Rb[]
     * Translations (Eigen::Vector3d) - ta[], tb[]
     */

    // Convert tf msgs to Eigen's 4x4 transform matrices (OpenGL)
    eigen_Ta_ = tf2::transformToEigen(req.A.transform);
    eigen_Tb_ = tf2::transformToEigen(req.B.transform);
    // Get rotation matrices from transforms
    eigen_Ra_.push_back(eigen_Ta_.matrix().block<3, 3>(0, 0) );
    eigen_Rb_.push_back(eigen_Tb_.matrix().block<3, 3>(0, 0) );
    // Get translation vectors from transforms
    eigen_ta_.push_back(eigen_Ta_.matrix().col(3).head<3>() );
    eigen_tb_.push_back(eigen_Tb_.matrix().col(3).head<3>() );

    // Respond with number of sampled pairs (A, B)
    n_samples++;
    res.n = n_samples;
}

bool ParkMartinNode::ComputeCalibration(vive_calibrating::ComputeCalibration::Request  &req,
                                        vive_calibrating::ComputeCalibration::Response &res)
{
      /**
     * Responds with the computed transformation from wrist to sensor,
     * but only iff more than two pairs of (A, B) have been sampled.
     * Responds with success = false otherwise.
     */

    if (n_samples >= 2) {
        // Compute the transformation from wrist to sensor
        ParkMartin();

        res.success = true;
        res.X = tf_X_;
    } else {
        res.success = false;
    };
}

Eigen::Vector3d ParkMartinNode::RotationMatrixLogarithm(const Eigen::Matrix3d &rotmat_) {
     /**
      * Computes the logarithm of a 3x3 rotation matrix.
      * Returns a 3x1 vector form of the skew symmetric matrix logarithm.
      */

    // Check if the logarithm is uniquely defined
    if (rotmat_.trace() != -1.) {
        // Axis-angle magnitude
        const double theta = std::acos((rotmat_.trace() - 1.) / 2.);

        return theta / (2. * sin(theta) ) * Eigen::Vector3d(rotmat_(2,1) - rotmat_(1,2),
                                                            rotmat_(0,2) - rotmat_(2,0),
                                                            rotmat_(1,0) - rotmat_(0,1) );
    } else {
        ROS_ERROR_STREAM("Error occurred when computing logarithm of rotation matrix:" << std::endl <<
                         rotmat_.matrix() << std::endl << "This rotation matrix has a trace equal to: " <<
                         rotmat_.trace()  << std::endl << ", and therefore results in a logarithm that is not uniquely defined.");

        return Eigen::Vector3d(0., 0., 1.);
    }
}

bool ParkMartinNode::ParkMartin()
{
     /**
      * Solves Ta * Tx = Tx * Tb based on a closed-form least squares solution from:
      * Robot sensor calibration: solving AX=XB on the Euclidean group
      * https://ieeexplore.ieee.org/document/326576/
      * 
      * "The equation AX = XB on the Euclidean group, where A and
      *  B are known and X is unknown, is of fundamental importance in the
      *  problem of calibrating wrist-mounted robotic sensors."
      * 
      * Input:
      * n_samples - Number of sampled pairs (A, B)
      * 
      * Output:
      * Tx - Transformation between end effector and sensor (solution)
      */

    Eigen::MatrixXd eigen_C_(3 * n_samples, 3);
    Eigen::VectorXd eigen_d_(3 * n_samples, 1);
    eigen_M_.setZero();
    // eigen_C_.setZero();
    // eigen_d_.setZero();

    for (int i = 0; i < n_samples; i++) {
        eigen_M_ += RotationMatrixLogarithm(eigen_Rb_[i]) *
                    RotationMatrixLogarithm(eigen_Ra_[i]).transpose();
    }

    if (n_samples == 2) {
        eigen_M_ += RotationMatrixLogarithm(eigen_Rb_[1]).cross(RotationMatrixLogarithm(eigen_Rb_[0]) ) *
                    RotationMatrixLogarithm(eigen_Ra_[1]).cross(RotationMatrixLogarithm(eigen_Ra_[0]) ).transpose();
    }

    // Compute the optimal rotation matrix R_x = (M^T * M)^(-1/2) * M^T with SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd_(eigen_M_.transpose() * eigen_M_,
                                           Eigen::ComputeFullU | Eigen::ComputeFullV);
    eigen_Rx_ = svd_.matrixU() *
                ((((svd_.singularValues() ).cwiseSqrt() ).cwiseInverse() ).asDiagonal() ).toDenseMatrix() *
                svd_.matrixV().transpose() *
                eigen_M_.transpose();

    for (int i = 0; i < n_samples; i++) {
        eigen_C_.block<3, 3>(3*i, 0) = Eigen::Matrix3d::Identity() - eigen_Ra_[i];
        eigen_d_.segment(3*i, 3) = eigen_ta_[i] - eigen_Rx_ * eigen_tb_[i];
    }
    // Compute the optimal translation vector t_x = C^T / (C^T * C) * d
    eigen_tx_ = (eigen_C_.transpose() * eigen_C_).inverse() * eigen_C_.transpose() * eigen_d_;

    // Create and return the resulting 4x4 transform as geometry_msgs::TransformStamped
    Eigen::Affine3d eigen_Tx_;
    eigen_Tx_.matrix().block<3, 3>(0, 0) = eigen_Rx_;
    eigen_Tx_.matrix().col(3).head<3>()  = eigen_tx_;

    ROS_INFO_STREAM(std::endl << eigen_Tx_.matrix() );
    tf_X_ = tf2::eigenToTransform(eigen_Tx_);
    tf_X_.header.stamp = ros::Time::now();
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "parkmartin");

    ParkMartinNode node_(30);

    if (!node_.Init() ) {
        exit(EXIT_FAILURE);
    }

    while (ros::ok() ) {
        node_.Loop();
    }

    node_.Shutdown();
    exit(EXIT_SUCCESS);
}