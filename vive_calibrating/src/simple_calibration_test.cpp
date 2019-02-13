// ROS
#include <ros/ros.h>

// ROS msgs
#include <geometry_msgs/PoseStamped.h>

// tf2
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class CalibrationTestNode
{
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;

    // Publisher
    ros::Publisher pose_pub_;

    // tf2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener *tf_listener_;

    tf2::Transform tf_diff_;
    geometry_msgs::TransformStamped tf_msg_diff_;
    geometry_msgs::PoseStamped pose_diff_;

    public:
        CalibrationTestNode(int frequency);
        ~CalibrationTestNode();

        bool Init();
        void Loop();
        void Shutdown();
};

CalibrationTestNode::CalibrationTestNode(int frequency):
    loop_rate_(frequency),
    tf_listener_(new tf2_ros::TransformListener(tf_buffer_) )
{   
    // Publisher
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose_diff", 10, true);

    pose_diff_.header.frame_id = "controller_test";
}
CalibrationTestNode::~CalibrationTestNode() {
}

bool CalibrationTestNode::Init() {
      /**
     * Initialize the node
     */

    while (!tf_buffer_.canTransform("controller_test", "controller_LHR_FDB9BFC4", ros::Time(0) ) ) {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }

    return true;
}

void CalibrationTestNode::Loop() {
      /**
     * Main loop of the node
     */

    tf_msg_diff_ = tf_buffer_.lookupTransform("controller_LHR_FDB9BFC4", "controller_test", ros::Time(0) );
    tf2::fromMsg(tf_msg_diff_.transform, tf_diff_);
    tf2::toMsg(tf_diff_, pose_diff_.pose);

    pose_diff_.header.stamp = ros::Time::now();
    pose_pub_.publish(pose_diff_);

    ros::spinOnce();
    loop_rate_.sleep();
}

void CalibrationTestNode::Shutdown() {
      /**
     * Runs before shutting down the node
     */

}

// bool CalibrationTestNode::AddSample(vive_calibrating::AddSample::Request  &req,
//                                vive_calibrating::AddSample::Response &res)
// {
//       /**
//      * Adds the provided pair (A, B) to an internal vector.
//      * Both transformations are decomposed into their internal representations:
//      * Rotations    (Eigen::Matrix3d) - Ra[], Rb[]
//      * Translations (Eigen::Vector3d) - ta[], tb[]
//      */

//     // Convert tf msgs to Eigen's 4x4 transform matrices (OpenGL)
//     eigen_Ta_ = tf2::transformToEigen(req.A.transform);
//     eigen_Tb_ = tf2::transformToEigen(req.B.transform);
//     // Get rotation matrices from transforms
//     eigen_Ra_.push_back(eigen_Ta_.matrix().block<3, 3>(0, 0) );
//     eigen_Rb_.push_back(eigen_Tb_.matrix().block<3, 3>(0, 0) );
//     // Get translation vectors from transforms
//     eigen_ta_.push_back(eigen_Ta_.matrix().col(3).head<3>() );
//     eigen_tb_.push_back(eigen_Tb_.matrix().col(3).head<3>() );

//     // Respond with number of sampled pairs (A, B)
//     n_samples++;
//     res.n = n_samples;
// }

// bool CalibrationTestNode::ComputeCalibration(vive_calibrating::ComputeCalibration::Request  &req,
//                                         vive_calibrating::ComputeCalibration::Response &res)
// {
//       /**
//      * Responds with the computed transformation from wrist to sensor,
//      * but only iff more than two pairs of (A, B) have been sampled.
//      * Responds with success = false otherwise.
//      */

//     if (n_samples >= 2) {
//         // Solve AX=XB for transformation X from wrist to sensor
//         ParkMartin();

//         // Clean up
//         n_samples = 0;
//         eigen_Ra_.clear();
//         eigen_Rb_.clear();
//         eigen_ta_.clear();
//         eigen_tb_.clear();

//         res.success = true;
//         res.X = tf_X_;
//     } else {
//         res.success = false;
//     };
// }

// Eigen::Vector3d CalibrationTestNode::RotationMatrixLogarithm(const Eigen::Matrix3d &rotmat_) {
//      /**
//       * Computes the logarithm of a 3x3 rotation matrix.
//       * Returns a 3x1 vector form of the skew symmetric matrix logarithm.
//       */

//     // Check if the logarithm is uniquely defined
//     if (rotmat_.trace() != -1.) {
//         // Axis-angle magnitude
//         const double theta = std::acos((rotmat_.trace() - 1.) / 2.);

//         return theta / (2. * sin(theta) ) * Eigen::Vector3d(rotmat_(2,1) - rotmat_(1,2),
//                                                             rotmat_(0,2) - rotmat_(2,0),
//                                                             rotmat_(1,0) - rotmat_(0,1) );
//     } else {
//         ROS_ERROR_STREAM("Error occurred when computing logarithm of rotation matrix:" << std::endl <<
//                          rotmat_.matrix() << std::endl << "This rotation matrix has a trace equal to: " <<
//                          rotmat_.trace()  << std::endl << ", and therefore results in a logarithm that is not uniquely defined.");

//         return Eigen::Vector3d(0., 0., 1.);
//     }
// }

// bool CalibrationTestNode::ParkMartin()
// {
//      /**
//       * Solves Ta * Tx = Tx * Tb based on a closed-form least squares solution from:
//       * Robot sensor calibration: solving AX=XB on the Euclidean group
//       * https://ieeexplore.ieee.org/document/326576/
//       * 
//       * "The equation AX = XB on the Euclidean group, where A and
//       *  B are known and X is unknown, is of fundamental importance in the
//       *  problem of calibrating wrist-mounted robotic sensors."
//       * 
//       * Input:
//       * n_samples - sampled pairs (A, B)
//       * 
//       * Output:
//       * Tx - Transformation between end effector and sensor (solution)
//       */

//     Eigen::MatrixXd eigen_C_(3 * n_samples, 3);
//     Eigen::VectorXd eigen_d_(3 * n_samples, 1);
//     eigen_M_.setZero();
//     // eigen_C_.setZero();
//     // eigen_d_.setZero();

//     for (int i = 0; i < n_samples; i++) {
//         eigen_M_ += RotationMatrixLogarithm(eigen_Rb_[i]) *
//                     RotationMatrixLogarithm(eigen_Ra_[i]).transpose();
//     }

//     if (n_samples == 2) {
//         eigen_M_ += RotationMatrixLogarithm(eigen_Rb_[1]).cross(RotationMatrixLogarithm(eigen_Rb_[0]) ) *
//                     RotationMatrixLogarithm(eigen_Ra_[1]).cross(RotationMatrixLogarithm(eigen_Ra_[0]) ).transpose();
//     }

//     // Compute the optimal rotation matrix R_x = (M^T * M)^(-1/2) * M^T with SVD
//     Eigen::JacobiSVD<Eigen::Matrix3d> svd_(eigen_M_.transpose() * eigen_M_,
//                                            Eigen::ComputeFullU | Eigen::ComputeFullV);
//     eigen_Rx_ = svd_.matrixU() *
//                 ((((svd_.singularValues() ).cwiseSqrt() ).cwiseInverse() ).asDiagonal() ).toDenseMatrix() *
//                 svd_.matrixV().transpose() *
//                 eigen_M_.transpose();

//     for (int i = 0; i < n_samples; i++) {
//         eigen_C_.block<3, 3>(3*i, 0) = Eigen::Matrix3d::Identity() - eigen_Ra_[i];
//         eigen_d_.segment(3*i, 3) = eigen_ta_[i] - eigen_Rx_ * eigen_tb_[i];
//     }
//     // Compute the optimal translation vector t_x = C^T / (C^T * C) * d
//     eigen_tx_ = (eigen_C_.transpose() * eigen_C_).inverse() * eigen_C_.transpose() * eigen_d_;

//     // Create and return the resulting 4x4 transform as geometry_msgs::TransformStamped
//     Eigen::Affine3d eigen_Tx_;
//     eigen_Tx_.matrix().block<3, 3>(0, 0) = eigen_Rx_;
//     eigen_Tx_.matrix().col(3).head<3>()  = eigen_tx_;

//     ROS_INFO_STREAM(std::endl << eigen_Tx_.matrix() );
//     tf_X_ = tf2::eigenToTransform(eigen_Tx_);
//     tf_X_.header.stamp = ros::Time::now();
// }

// void CalibrationTestNode::ParkMartinExample() {
//       /**
//      * Test method for solving AX = BX with example from report:
//      * Robot sensor calibration: solving AX=XB on the Euclidean group
//      * https://ieeexplore.ieee.org/document/326576/
//      * This method is intended to run as a one-shot test
//      * (that is, by itself and not multiple times).
//      */

//     // Define homogenous transformation matrices from example in report
//     tf2::Transform tf_A1_(tf2::Matrix3x3(-0.989992, -0.141120, 0., 0.141120, -0.989992, 0., 0., 0., 1.), tf2::Vector3(0., 0., 0.) );
//     tf2::Transform tf_A2_(tf2::Matrix3x3(0.070737, 0., 0.997495, 0., 1., 0., -0.997495, 0., 0.070737), tf2::Vector3(-400., 0., 400.) );
//     tf2::Transform tf_B1_(tf2::Matrix3x3(-0.989992, -0.138307, 0.028036,  0.138307, -0.911449,  0.387470, -0.028036,  0.387470, 0.921456), tf2::Vector3(- 26.9559, -96.1332,  19.4872) );
//     tf2::Transform tf_B2_(tf2::Matrix3x3( 0.070737,  0.198172, 0.997612, -0.198172,  0.963323, -0.180936, -0.977612, -0.180936, 0.107415), tf2::Vector3(-309.5430,  59.0244, 291.1770) );

//     geometry_msgs::Transform tf_msg_A_[2];
//     tf2::convert(tf_A1_, tf_msg_A_[0]);
//     tf2::convert(tf_A2_, tf_msg_A_[1]);
//     geometry_msgs::Transform tf_msg_B_[2];
//     tf2::convert(tf_B1_, tf_msg_B_[0]);
//     tf2::convert(tf_B2_, tf_msg_B_[1]);

//     for (int i = 0; i < 2; i++) {
//         // Convert tf msgs to Eigen's 4x4 transform matrices (OpenGL)
//         eigen_Ta_ = tf2::transformToEigen(tf_msg_A_[i]);
//         eigen_Tb_ = tf2::transformToEigen(tf_msg_B_[i]);
//         // Get rotation matrices from transforms
//         eigen_Ra_.push_back(eigen_Ta_.matrix().block<3, 3>(0, 0) );
//         eigen_Rb_.push_back(eigen_Tb_.matrix().block<3, 3>(0, 0) );
//         // Get translation vectors from transforms
//         eigen_ta_.push_back(eigen_Ta_.matrix().col(3).head<3>() );
//         eigen_tb_.push_back(eigen_Tb_.matrix().col(3).head<3>() );
//     }

//     // Solve AX=XB for transformation X from wrist to sensor
//     ParkMartin();

//     ROS_INFO_STREAM(tf_X_);
// }


int main(int argc, char** argv) {
    ros::init(argc, argv, "cal_test");

    CalibrationTestNode node_(240);

    if (!node_.Init() ) {
        exit(EXIT_FAILURE);
    }

    while (ros::ok() ) {
        node_.Loop();
    }

    node_.Shutdown();
    exit(EXIT_SUCCESS);
}