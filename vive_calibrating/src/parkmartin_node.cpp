#include "parkmartin_node.h"
#include "test/ceres/local_parameterization_se3.hpp"


ParkMartinNode::ParkMartinNode(int frequency)
    : pvt_nh_("~"),
      loop_rate_(frequency)
{   
    // Number of sampled pairs (A, B)
    n_samples = 0;

    // Advertise services
    sample_service_  = pvt_nh_.advertiseService("add_sample",
                                                &ParkMartinNode::AddSample, this);
    compute_service_ = pvt_nh_.advertiseService("compute_calibration",
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
     */

    // Convert tf msgs to Sophus's SE3 group
    sophus_Ta_.push_back(sophus_ros_conversions::transformMsgToSophus(req.A.transform) );
    sophus_Tb_.push_back(sophus_ros_conversions::transformMsgToSophus(req.B.transform) );

    // Respond with number of sampled pairs (A, B)
    res.n = ++n_samples;
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
        // Solve AX=XB for transformation X from wrist to sensor
        ParkMartin();
        HoraudDornaika();

        // Clean up
        n_samples = 0;
        sophus_Ta_.clear();
        sophus_Tb_.clear();

        // Respond with computed transformation
        res.success = true;
        res.X = tf_X_;
    } else {
        res.success = false;
    };
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
      * n_samples - sampled pairs (A, B)
      * 
      * Output:
      * Tx - Transformation between end effector and sensor (solution)
      */

    Eigen::MatrixXf eigen_C_(3 * n_samples, 3);
    Eigen::VectorXf eigen_d_(3 * n_samples, 1);
    Eigen::Matrix3f eigen_M_ = Eigen::Matrix3f::Zero();

    for (int i = 0; i < n_samples; i++) {
        eigen_M_ += sophus_Tb_[i].so3().log() *
                    sophus_Ta_[i].so3().log().transpose();
    }

    // Special case for 2 samples
    if (n_samples == 2) {
        eigen_M_ += sophus_Tb_[1].so3().log().cross(sophus_Tb_[0].so3().log() ) *
                    sophus_Ta_[1].so3().log().cross(sophus_Ta_[0].so3().log() ).transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3f> svd_(eigen_M_.transpose() * eigen_M_,
                                           Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3f eigen_Rx_ = svd_.matrixU() * ((((
                                svd_.singularValues() ).cwiseSqrt() ).cwiseInverse() ).asDiagonal() ).toDenseMatrix() *
                                svd_.matrixV().transpose() *
                                eigen_M_.transpose();

    // Ensure that the rotation matrix is orthogonal with determinant 1
    Eigen::Quaternionf eigen_Qx_ = Eigen::Quaternionf(eigen_Rx_);
    sophus_Rx_ = Sophus::SO3f(eigen_Qx_);

    for (int i = 0; i < n_samples; i++) {
        eigen_C_.block<3, 3>(3*i, 0) = Eigen::Matrix3f::Identity() - sophus_Ta_[i].so3().matrix();
        eigen_d_.segment(3*i, 3) = sophus_Ta_[i].translation() - sophus_Rx_ * sophus_Tb_[i].translation();
    }
    // Compute the optimal translation vector t_x = C^T / (C^T * C) * d
    sophus_tx_ = (eigen_C_.transpose() * eigen_C_).inverse() * eigen_C_.transpose() * eigen_d_;

    // Create and return the resulting 4x4 transform as geometry_msgs::TransformStamped
    Sophus::SE3f sophus_Tx_(sophus_Rx_, sophus_tx_);

    ROS_INFO_STREAM("Before optimization:" << std::endl << sophus_Tx_.matrix() );
    tf_X_.transform = sophus_ros_conversions::sophusToTransformMsg(sophus_Tx_);
    tf_X_.header.stamp = ros::Time::now();
}

void ParkMartinNode::ParkMartinExample() {
      /**
     * Test method for solving AX = BX with example from report:
     * Robot sensor calibration: solving AX=XB on the Euclidean group
     * https://ieeexplore.ieee.org/document/326576/
     */

    // Define homogenous transformation matrices from example in report
    tf2::Transform tf_A1_(tf2::Matrix3x3(-0.989992, -0.141120, 0., 0.141120, -0.989992, 0., 0., 0., 1.),
                          tf2::Vector3(0., 0., 0.) );
    tf2::Transform tf_A2_(tf2::Matrix3x3(0.070737, 0., 0.997495, 0., 1., 0., -0.997495, 0., 0.070737),
                          tf2::Vector3(-400., 0., 400.) );
    tf2::Transform tf_B1_(tf2::Matrix3x3(-0.989992, -0.138307, 0.028036,  0.138307, -0.911449,  0.387470, -0.028036,  0.387470, 0.921456),
                          tf2::Vector3(- 26.9559, -96.1332,  19.4872) );
    tf2::Transform tf_B2_(tf2::Matrix3x3( 0.070737,  0.198172, 0.997612, -0.198172,  0.963323, -0.180936, -0.977612, -0.180936, 0.107415),
                          tf2::Vector3(-309.5430,  59.0244, 291.1770) );

    geometry_msgs::Transform tf_msg_A_[2];
    tf2::convert(tf_A1_, tf_msg_A_[0]);
    tf2::convert(tf_A2_, tf_msg_A_[1]);
    geometry_msgs::Transform tf_msg_B_[2];
    tf2::convert(tf_B1_, tf_msg_B_[0]);
    tf2::convert(tf_B2_, tf_msg_B_[1]);

    sophus_Ta_.push_back(sophus_ros_conversions::transformMsgToSophus(tf_msg_A_[0]) );
    sophus_Tb_.push_back(sophus_ros_conversions::transformMsgToSophus(tf_msg_B_[0]) );
    sophus_Ta_.push_back(sophus_ros_conversions::transformMsgToSophus(tf_msg_A_[1]) );
    sophus_Tb_.push_back(sophus_ros_conversions::transformMsgToSophus(tf_msg_B_[1]) );

    n_samples = 2;

    // Solve AX=XB for transformation X from wrist to sensor
    ParkMartin();
    HoraudDornaika();

    ROS_INFO_STREAM(tf_X_);
}


namespace Eigen {
    namespace internal {
        template <class T, int N, typename NewType>
            struct cast_impl<ceres::Jet<T, N>, NewType> {
            EIGEN_DEVICE_FUNC
            
            static inline NewType run(ceres::Jet<T, N> const& x) {
                return static_cast<NewType>(x.a);
            }
        };
    }  // namespace internal
}  // namespace Eigen

struct TestCostFunctor {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    TestCostFunctor(Sophus::SE3d T_a, Sophus::SE3d T_b) : T_a(T_a), T_b(T_b) {}

    template <class T>
    bool operator()(T const* const sT_x, T* sResiduals) const
    {
        Eigen::Map<Sophus::SE3<T> const> const T_x(sT_x);
        Eigen::Map<Eigen::Matrix<T, 6, 1> > residuals(sResiduals);

        residuals = ((T_a.cast<T>()*T_x).inverse()*(T_x*T_b.cast<T>() ) ).log();

        return true;
    }

    Sophus::SE3d T_a, T_b;
};

bool ParkMartinNode::HoraudDornaika() {
    // Parameters
    Sophus::SE3d T_x = sophus_Tx_.cast<double>();

    ceres::Problem problem;

    problem.AddParameterBlock(T_x.data(), Sophus::SE3d::num_parameters,
                              new Sophus::test::LocalParameterizationSE3);

    for (int i = 0; i < n_samples; i++) {
        ceres::CostFunction* cost_function =
            new ceres::AutoDiffCostFunction<TestCostFunctor, Sophus::SE3d::DoF,
                                            Sophus::SE3d::num_parameters>
                                           (new TestCostFunctor(sophus_Ta_[i].cast<double>(),
                                                                sophus_Tb_[i].cast<double>() ) );
        problem.AddResidualBlock(cost_function, NULL, T_x.data() );
    }

    // Set solver options (precision / method)
    ceres::Solver::Options options;
    options.gradient_tolerance = 0.1 * Sophus::Constants<double>::epsilon();
    options.function_tolerance = 0.1 * Sophus::Constants<double>::epsilon();
    options.linear_solver_type = ceres::DENSE_QR;

    // Solve
    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);

    ROS_INFO_STREAM("After optimization:" << std::endl << T_x.matrix() );
    ROS_INFO_STREAM(summary.FullReport() << std::endl);

    tf_X_.transform = sophus_ros_conversions::sophusToTransformMsg(T_x.cast<float>() );
    tf_X_.header.stamp = ros::Time::now();

    return true;
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