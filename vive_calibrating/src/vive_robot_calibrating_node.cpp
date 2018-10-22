#include "vive_robot_calibrating_node.h"

CalibratingNode::CalibratingNode(int frequency)
    : loop_rate_(frequency),
      tf_listener_(new tf2_ros::TransformListener(tf_buffer_) ),
      move_group_(moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP) ),
      robot_model_loader_(robot_model_loader::RobotModelLoader("/robot_description") ),
      kinematic_model_(robot_model_loader_.getModel() ),
      kinematic_state_(new robot_state::RobotState(kinematic_model_) ),
      joint_model_group_(kinematic_model_->getJointModelGroup("floor_manipulator") ),
      action_client_(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
                    ("/floor/joint_trajectory_action", true) ),
      rng(random_seed() ),
      r_dist(1.2, 1.4),
      theta_dist(5. * M_PI_4, 7. * M_PI_4),
      phi_dist(M_PI_4, 1.5*M_PI_4)
{
    // Subscribers
    device_sub_ = nh_.subscribe("/vive_node/tracked_devices", 1, &CalibratingNode::DevicesCb, this);

    // Define dynamic reconfigure message for calibrating frames
    srv_reconf_req_.config.doubles.resize(6);
    srv_reconf_req_.config.doubles[0].name = "vr_x_offset";
    srv_reconf_req_.config.doubles[1].name = "vr_y_offset";
    srv_reconf_req_.config.doubles[2].name = "vr_z_offset";
    srv_reconf_req_.config.doubles[3].name = "vr_yaw_offset";
    srv_reconf_req_.config.doubles[4].name = "vr_pitch_offset";
    srv_reconf_req_.config.doubles[5].name = "vr_roll_offset";
    
    // Set the planning parameters of the move group (MoveIt!)
    move_group_.setPoseReferenceFrame("floor_base");
    move_group_.setMaxVelocityScalingFactor(0.05);

    ROS_INFO_STREAM(kinematic_model_->getModelFrame().c_str() );

    // std::vector<double> joint_values;
    // const std::vector<std::string>& joint_names = joint_model_group_->getVariableNames();
    // kinematic_state_->copyJointGroupPositions(joint_model_group_, joint_values);
    // for (std::size_t i = 0; i < joint_names.size(); ++i)
    // {
    // ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    // }
}
CalibratingNode::~CalibratingNode() {
}

bool CalibratingNode::InitParams() {
     /**
      * Initialize parameters from the parameter server.
      * Returns true if the parameters were retrieved from the server, false otherwise.
      */

    if (nh_.param<std::string>("/vive_calibrating_node/controller", controller_frame, "") ) {
        joy_sub_ = nh_.subscribe("/vive_node/joy/" + controller_frame, 1, &CalibratingNode::JoyCb, this);
        
        return true;
    } else {
        ROS_WARN("Failed to get parameters from the parameter server.");

        return false;
    }
}

bool CalibratingNode::Init() {
      /**
     * Initialize the node and check if the necessary transforms are available
     */

    // ParkMartinExample();
    // return false;

    // if (!InitParams() ) {
    //     while (controller_frame.empty() ) {
    //         ROS_INFO("Waiting for available VIVE controller..");
            
    //         ros::spinOnce();
    //         ros::Duration(5.0).sleep();
    //     }
    // }
    // ROS_INFO_STREAM("Using " + controller_frame + " for calibration");

    // joy_sub_ = nh_.subscribe("/vive_node/joy/" + controller_frame, 1, &CalibratingNode::JoyCb, this);

    // std::string pError;
    // if (!tf_buffer_.canTransform("world_vr", controller_frame, ros::Time(0),
    //                              ros::Duration(5.0), &pError) )
    // {
    //     ROS_ERROR_STREAM("Can't transform from world_vr to " + controller_frame + ": " + pError);

    //     return false;
    // }

    // Wait for action server
    ROS_INFO("Waiting for action server...");
    action_client_->waitForServer();
    ROS_INFO("Action server ready");

    tf_X_.setOrigin(tf2::Vector3(2., -1., 0.5) );
    tf_X_.setRotation(tf2::Quaternion(0., 0., std::sin(M_PI_4), std::cos(M_PI_4) ) );

    tf_msg_pose_.header.frame_id = "floor_base";
    tf_msg_pose_.child_frame_id = "desired_pose";
    pose_msg_.header.frame_id = "floor_base";

    MeasureRobot(10);
    return false;
    
    // return true;
}

void CalibratingNode::Loop() {
      /**
     * Main loop of the node
     */

    ros::waitForShutdown();
}

void CalibratingNode::Shutdown() {
      /**
     * Runs before shutting down the node
     */

    
}

void CalibratingNode::JoyCb(const sensor_msgs::Joy& msg_) {
      /**
     * Handle VIVE Controller inputs
     */

    // Grip button - Runs a calibration routine with measurements from the robot
    // if (msg_.buttons[1]) {
    //     MeasureRobot(50);
    // }
}

void CalibratingNode::DevicesCb(const vive_bridge::TrackedDevicesStamped& msg_) {
      /**
     * Update controller and tracker frames of tracked devices.
     */

    for (int i = 0; i < msg_.device_count; i++) {
        if (msg_.device_classes[i] == msg_.CONTROLLER) {
            controller_frame = msg_.device_frames[i];
        }
    }
}


geometry_msgs::Pose CalibratingNode::SphereNormalPose(double r, double theta, double phi, geometry_msgs::Pose &pose_) {
      /**
     * Find pose based on normal vector of sphere with radius r, polar angle theta and azimuthal angle phi
     */

    tf2::Transform tf_rot_ = tf2::Transform(tf2::Quaternion(0., 0., std::sin(-M_PI/8), std::cos(-M_PI/8) ),
                                            tf2::Vector3(0., 0., 0.) );

    tf2::Matrix3x3 tf_rotmat_;
    tf_rotmat_.setEulerYPR(theta, phi, 0.);

    tf2::Transform tf_pose_(tf_rotmat_, r * (tf_rotmat_ * tf2::Vector3(0., 0., 1.) ) );
    tf_pose_ *= tf_rot_;

    tf2::toMsg(tf_pose_, pose_);

    return pose_;
}

geometry_msgs::Pose CalibratingNode::GenerateRandomPose(geometry_msgs::Pose &pose_) {
     /**
      * Generate random pose from pre-defined uniform distributions r_dist, theta_dist, phi_dist
      * Two random poses are generated, and position is used from the first pose and orientation is used from the second pose
      */

    geometry_msgs::Pose pose_pos_;
    SphereNormalPose(r_dist(rng), theta_dist(rng), phi_dist(rng), pose_pos_ );
    SphereNormalPose(r_dist(rng), theta_dist(rng), phi_dist(rng), pose_ );
    
    // Set position from the first random pose
    pose_.position = pose_pos_.position;

    return pose_;
}

bool CalibratingNode::MoveRobot(const geometry_msgs::PoseStamped &pose_) {
     /**
      * Move robot to provided pose, stop, and wait for the dynamics to settle down.tf_buffer_.waitForTransform("floor_tool0", "controller_frame", ros::Time(0), ros::Duration(10.) );
      * Returns true if trajectory execution succeeded.
      */
    
    geometry_msgs::TransformStamped tf_root_ = tf_buffer_.lookupTransform("root", pose_.header.frame_id, ros::Time(0) );
    geometry_msgs::PoseStamped pose_root_;
    tf2::doTransform(pose_, pose_root_, tf_root_);
    

    if (kinematic_state_->setFromIK(joint_model_group_, pose_root_.pose, 10, 0.1) ) {
        std::vector<double> joint_values;
        kinematic_state_->copyJointGroupPositions(joint_model_group_, joint_values);
        const std::vector<std::string>& joint_names = joint_model_group_->getVariableNames();

        control_msgs::FollowJointTrajectoryGoal traj_goal_msg_;
        traj_goal_msg_.trajectory.header.frame_id = pose_.header.frame_id;
        traj_goal_msg_.trajectory.joint_names = joint_names;
        traj_goal_msg_.trajectory.points.push_back(trajectory_msgs::JointTrajectoryPoint() );
        traj_goal_msg_.trajectory.points[0].positions = joint_values;
        traj_goal_msg_.trajectory.points[0].velocities = {0., 0., 0., 0., 0., 0.};
        traj_goal_msg_.trajectory.points[0].accelerations = {0., 0., 0., 0., 0., 0.};
        traj_goal_msg_.trajectory.points[0].time_from_start = ros::Duration(10.);

        action_client_->sendGoalAndWait(traj_goal_msg_);

        // move_group_.setPoseTarget(pose_);

        // if (move_group_.move() ) {
        //     ROS_INFO_STREAM("Trajectory execution succeeded");

        //     move_group_.stop();
        //     ros::Duration(0.5).sleep();
        //     return true;
        // } else {
        //     ROS_WARN_STREAM("Trajectory execution failed with pose:" << std::endl
        //                                                             << pose_.pose);

        //     return false;
        // }
    } else {
        ROS_INFO("Unable to find IK solution");
    }
}
void CalibratingNode::MeasureRobot(const int &N) {
     /**
      * Runs a calibration routine by moving the robot to N random poses.
      * The poses are generated by computing the normal vector of a sphere with random radius and angles.
      * 
      * Measurements for each pose are saved to a bag file in the home folder and consists of tf messages:
      * - tool0
      * - sensor
      * - (N - 1) transforms between both poses
      * Each pose is w.r.t. the robot's base frame
      * 
      * The transformation from tool0 to sensor is then calculated based on the measurements.
      */

    // Open a bag file for recording poses (named with iso-date)
    bag_.open("calib_data_" + boost::posix_time::to_iso_string(ros::Time::now().toBoost() ) + ".bag",
              rosbag::bagmode::Write);

    ros::Duration(3.).sleep();

    GenerateRandomPose(pose_msg_.pose);
    pose_msg_.header.stamp = ros::Time::now();
    bag_.write("desired_pose", ros::Time::now(), pose_msg_);

    ROS_INFO_STREAM("0/" << N << " Moving robot to pose:" << std::endl << pose_msg_.pose);
    MoveRobot(pose_msg_);

    std::string pError;
    if (tf_buffer_.canTransform("floor_tool0", "floor_base", ros::Time(0), &pError) &&
        tf_buffer_.canTransform(controller_frame, "world_vr", ros::Time(0), &pError) )
    {
        // Lookup and convert necessary transforms from msgs
        tf_msg_tool0_ = tf_buffer_.lookupTransform("floor_base", ros::Time(0), "floor_tool0", ros::Time(0), "floor_base");
        tf_msg_sensor_ = tf_buffer_.lookupTransform("world_vr", ros::Time(0), controller_frame, ros::Time(0), "world_vr");
        bag_.write("tool0", ros::Time::now(), tf_msg_tool0_);
        bag_.write("sensor", ros::Time::now(), tf_msg_sensor_);

        tf2::convert(tf_msg_tool0_.transform, tf_tool0_[0]);
        // tf2::convert(tf_msg_sensor_.transform, tf_sensor_[0]);
        tf_sensor_[0] = tf_tool0_[0]*tf_X_;

        for (int i = 1; i <= N; i++) {
            GenerateRandomPose(pose_msg_.pose);
            pose_msg_.header.stamp = ros::Time::now();
            bag_.write("desired_pose", ros::Time::now(), pose_msg_);

            // Visualize desired pose as a transform in RViz
            tf2::convert(pose_msg_.pose, tf_pose_);
            tf2::convert(tf_pose_, tf_msg_pose_.transform);
            tf_msg_pose_.header.stamp = ros::Time::now();
            static_tf_broadcaster_.sendTransform(tf_msg_pose_);

            ROS_INFO_STREAM(i << "/" << N << " Moving robot to pose:" << std::endl << pose_msg_.pose);
            if (MoveRobot(pose_msg_) ) {
                if (tf_buffer_.canTransform("floor_tool0", "floor_base", ros::Time(0), &pError) )//&&
                    //tf_buffer_.canTransform(controller_frame, "world_vr", ros::Time(0), &pError) )
                {
                    // Lookup and convert necessary transforms
                    tf_msg_tool0_ = tf_buffer_.lookupTransform("floor_base", ros::Time(0), "floor_tool0", ros::Time(0), "floor_base");
                    tf_msg_sensor_ = tf_buffer_.lookupTransform("world_vr", ros::Time(0), controller_frame, ros::Time(0), "world_vr");
                    bag_.write("tool0", ros::Time::now(), tf_msg_tool0_);
                    bag_.write("sensor", ros::Time::now(), tf_msg_sensor_);

                    tf2::convert(tf_msg_tool0_.transform, tf_tool0_[1]);
                    tf2::convert(tf_msg_sensor_.transform, tf_sensor_[1]);
                    // tf_sensor_[1] = tf_tool0_[1]*tf_X_;

                    // Compute transforms between poses
                    tf2::convert(tf_tool0_[0].inverseTimes(tf_tool0_[1]), tf_msg_A_.transform);
                    tf2::convert(tf_sensor_[0].inverseTimes(tf_sensor_[1]), tf_msg_B_.transform);
                    bag_.write("A", ros::Time::now(), tf_msg_A_);
                    bag_.write("B", ros::Time::now(), tf_msg_B_);

                    tf_Avec_.push_back(tf_msg_A_.transform);
                    tf_Bvec_.push_back(tf_msg_B_.transform);

                    // Set current pose as reference for next pose
                    tf_tool0_[0]  = tf_tool0_[1];
                    tf_sensor_[0] = tf_sensor_[1];
                } else {
                    ROS_WARN_STREAM("0/" << N << ": " << pError);
                }
            }
        }
    } else {
        ROS_WARN_STREAM("0/" << N << ": " << pError);
    }

    geometry_msgs::TransformStamped tf_msg_Tx_ = ParkMartin(tf_Avec_.data(),
                                                            tf_Bvec_.data(),
                                                            tf_Avec_.size() );
    ROS_INFO_STREAM(tf_msg_Tx_);
    bag_.write("X", ros::Time::now(), tf_msg_Tx_);

    // ConstructTMatrix(tf_Avec_.data(), tf_Bvec_.data(), tf_Avec_.size() );
    bag_.close();
}

Eigen::Vector3d CalibratingNode::RotationMatrixLogarithm(const Eigen::Matrix3d &rotmat_) {
     /**
      * Compute the logarithm of a 3x3 rotation matrix
      */

    // Check if logarithm is uniquely defined
    if (rotmat_.trace() != -1.) {
        // Axis-angle magnitude
        const double theta = std::acos((rotmat_.trace() - 1.) / 2.);
        return theta / (2. * sin(theta) ) * Eigen::Vector3d(rotmat_(2,1) - rotmat_(1,2),
                                                            rotmat_(0,2) - rotmat_(2,0),
                                                            rotmat_(1,0) - rotmat_(0,1) );
    } else {
        ROS_ERROR_STREAM("Error occurred when computing logarithm of rotation matrix:" << std::endl <<
                         rotmat_.matrix() << std::endl <<
                         "This rotation matrix has a trace equal to: " << rotmat_.trace() <<
                         ", and therefore results in a logarithm that is not uniquely defined.");

        return Eigen::Vector3d::Constant(0.);
    }
}

geometry_msgs::TransformStamped CalibratingNode::ParkMartin(const geometry_msgs::Transform tf_Ta_[],
                                                            const geometry_msgs::Transform tf_Tb_[],
                                                            const int &size)
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
      * A helpful illustration of this problem is found at:
      * https://torsteinmyhre.name/snippets/robcam_calibration.html
      * 
      * Input:
      * Ta - TF message array of tool0 poses
      * Tb - TF message array of sensor poses
      * size - Size of both arrays (should be the same)
      * 
      * Output:
      * Tx - Transformation between end effector and sensor (solution)
      */

    Eigen::Matrix3d eigen_Ra_[size], eigen_Rb_[size];
    Eigen::Vector3d eigen_ta_[size], eigen_tb_[size];

    Eigen::MatrixXd eigen_C_(3 * size, 3);
    Eigen::VectorXd eigen_d_(3 * size, 1);
    eigen_M_.setZero();
    eigen_C_.setZero();
    eigen_d_.setZero();

    for (int i = 0; i < size; i++) {
        // Convert tf msgs to Eigen's 4x4 transform matrices (OpenGL)
        eigen_Ta_ = tf2::transformToEigen(tf_Ta_[i]);
        eigen_Tb_ = tf2::transformToEigen(tf_Tb_[i]);
        // Get rotation matrices from transforms
        eigen_Ra_[i] = eigen_Ta_.matrix().block<3, 3>(0, 0);
        eigen_Rb_[i] = eigen_Tb_.matrix().block<3, 3>(0, 0);
        // Get translation vectors from transforms
        eigen_ta_[i] = eigen_Ta_.matrix().col(3).head<3>();
        eigen_tb_[i] = eigen_Tb_.matrix().col(3).head<3>();

        eigen_M_ += RotationMatrixLogarithm(eigen_Rb_[i]) *
                    RotationMatrixLogarithm(eigen_Ra_[i]).transpose();
    }

    if (size == 2) {
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

    for (int i = 0; i < size; i++) {
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
    return tf2::eigenToTransform(eigen_Tx_);
}

void CalibratingNode::ParkMartinExample() {
      /**
     * Unit test method for solving AX = BX with example from report:
     * Robot sensor calibration: solving AX=XB on the Euclidean group
     * https://ieeexplore.ieee.org/document/326576/
     */

    // Define homogenous transformation matrices from example in report
    tf2::Transform tf_A1_(tf2::Matrix3x3(-0.989992, -0.141120, 0., 0.141120, -0.989992, 0., 0., 0., 1.), tf2::Vector3(0., 0., 0.) );
    tf2::Transform tf_A2_(tf2::Matrix3x3(0.070737, 0., 0.997495, 0., 1., 0., -0.997495, 0., 0.070737), tf2::Vector3(-400., 0., 400.) );
    tf2::Transform tf_B1_(tf2::Matrix3x3(-0.989992, -0.138307, 0.028036,  0.138307, -0.911449,  0.387470, -0.028036,  0.387470, 0.921456), tf2::Vector3(- 26.9559, -96.1332,  19.4872) );
    tf2::Transform tf_B2_(tf2::Matrix3x3( 0.070737,  0.198172, 0.997612, -0.198172,  0.963323, -0.180936, -0.977612, -0.180936, 0.107415), tf2::Vector3(-309.5430,  59.0244, 291.1770) );

    geometry_msgs::Transform tf_msg_A_[2];
    tf2::convert(tf_A1_, tf_msg_A_[0]);
    tf2::convert(tf_A2_, tf_msg_A_[1]);
    geometry_msgs::Transform tf_msg_B_[2];
    tf2::convert(tf_B1_, tf_msg_B_[0]);
    tf2::convert(tf_B2_, tf_msg_B_[1]);

    //ParkMartin(tf_msg_A_, tf_msg_B_, 2);
    ConstructTMatrix(tf_msg_A_, tf_msg_B_, 2);
}


void CalibratingNode::TransformToDualQuaternion(const geometry_msgs::Transform &tf_msg_,
                                                Eigen::Quaterniond &eigen_qr_,
                                                Eigen::Quaterniond &eigen_qd_)
{
     /**
      * Compute the screw representation of a transform as a dual quaternion.
      * 
      * qr - real quaternion (rotation)
      * qd - dual quaternion (translation)
      */
    
    // The real quaternion is simply the rotation part of the transform
    tf2::fromMsg(tf_msg_.rotation, eigen_qr_);

    Eigen::Vector3d eigen_t_;
    tf2::fromMsg(tf_msg_.translation, eigen_t_);

    eigen_qd_.w() = -0.5*(eigen_qr_.vec() ).transpose()*eigen_t_;
    eigen_qd_.vec() = 0.5*(eigen_qr_.w()*eigen_t_ + eigen_t_.cross(eigen_qr_.vec() ) );
}

Eigen::Matrix3d CalibratingNode::AntisymmetricMatrix(const Eigen::Vector3d &eigen_v_) {
     /**
      * Compute the antisymmetric matrix corresponding to the 
      * cross-product with the provided vector.
      */
    
    Eigen::Matrix3d eigen_asmat_;
    eigen_asmat_ <<             0 , -eigen_v_(2),  eigen_v_(1),
                     eigen_v_(2),             0 , -eigen_v_(0),
                    -eigen_v_(1),  eigen_v_(0),             0 ;
    
    return eigen_asmat_;
}

Eigen::Matrix<double, 6, 8> CalibratingNode::ConstructSMatrix(const Eigen::Quaterniond &eigen_qrA_,
                                                              const Eigen::Quaterniond &eigen_qdA_,
                                                              const Eigen::Quaterniond &eigen_qrB_,
                                                              const Eigen::Quaterniond &eigen_qdB_)
{
     /**
      * Construct a 6x8 matrix corresponding to a single instance 
      * of the hand-eye calibration problem with dual quaternions.
      * 
      * The S matrix is given by equation 31 in Daniilidis (1999):
      * Hand-Eye Calibration Using Dual Quaternions
      */
    
    Eigen::Matrix<double, 3, 4> eigen_Sdiag_;
    eigen_Sdiag_ << eigen_qrA_.vec() - eigen_qrB_.vec(),
                    AntisymmetricMatrix(eigen_qrA_.vec() + eigen_qrB_.vec() );
    
    Eigen::Matrix<double, 6, 8> eigen_S_;
    eigen_S_ << eigen_Sdiag_, Eigen::Matrix<double, 3, 4>::Constant(0.),
                eigen_qdA_.vec() - eigen_qdB_.vec(),
                AntisymmetricMatrix(eigen_qdA_.vec() + eigen_qdB_.vec() ),
                eigen_Sdiag_;
    
    return eigen_S_;
}

Eigen::MatrixXd CalibratingNode::ConstructTMatrix(const geometry_msgs::Transform tf_Ta_[],
                                                  const geometry_msgs::Transform tf_Tb_[],
                                                  const int &size)
{
     /**
      * Construct a 6*nx8 matrix T corresponding to the complete
      * hand-eye calibration problem with dual quaternions.
      * 
      * The T matrix is given as equation 33 in Daniilidis (1999):
      * Hand-Eye Calibration Using Dual Quaternions
      */
    
    Eigen::Quaterniond eigen_qrA_, eigen_qdA_, eigen_qrB_, eigen_qdB_;
    Eigen::MatrixXd eigen_T_(6*size, 8);

    for (int i = 0; i < size; i++) {
        TransformToDualQuaternion(tf_Ta_[i], eigen_qrA_, eigen_qdA_);
        TransformToDualQuaternion(tf_Tb_[i], eigen_qrB_, eigen_qdB_);

        eigen_T_.block<6, 8>(6*i, 0) = ConstructSMatrix(eigen_qrA_, eigen_qdA_,
                                                        eigen_qrB_, eigen_qdB_);
    }

    Eigen::JacobiSVD<Eigen::MatrixXd> svd_(eigen_T_, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Vector4d eigen_u_[2] = {svd_.matrixV().matrix().block<4, 1>(0, 6),
                                   svd_.matrixV().matrix().block<4, 1>(0, 7) };
    Eigen::Vector4d eigen_v_[2] = {svd_.matrixV().matrix().block<4, 1>(4, 6),
                                   svd_.matrixV().matrix().block<4, 1>(4, 7) };

    ROS_INFO_STREAM(svd_.singularValues() );

    double a = eigen_u_[0].dot(eigen_v_[0]);
    double b = eigen_u_[0].dot(eigen_v_[1]) + eigen_u_[1].dot(eigen_v_[0]);
    double c = eigen_u_[1].dot(eigen_v_[1]);

    double s[2] = {(-b + std::sqrt(b*b - 4*a*c) )/(2*a),
                   (-b - std::sqrt(b*b - 4*a*c) )/(2*a) };

    a = eigen_u_[0].dot(eigen_u_[0]);
    b = 2*eigen_u_[0].dot(eigen_u_[1]);
    c = eigen_u_[1].dot(eigen_u_[1]);

    double d[2] = {s[0]*s[0]*a + 2*b*s[0] + c,
                   s[1]*s[1]*a + 2*b*s[1] + c};
    
    double lambda[2];
    // Pick solution s corresponding to the smallest lambda value
    if (d[0] >= d[1]) {
        lambda[1] = 1/std::sqrt(d[0]);
        lambda[0] = s[0]*lambda[1];
    } else {
        lambda[1] = 1/std::sqrt(d[1]);
        lambda[0] = s[1]*lambda[1];
    }

    Eigen::Quaterniond eigen_qrX_(lambda[0]*eigen_u_[0] + lambda[1]*eigen_u_[1]);
    eigen_qrX_.normalize();

    Eigen::Matrix3d eigen_rotmat_;
    eigen_rotmat_ << 0, 1, 0, 0, 0, 1, -1, 0, 0; 

    Eigen::Quaterniond eigen_qdX_(lambda[0]*eigen_v_[0] + lambda[1]*eigen_v_[1]);
    Eigen::Vector3d eigen_t_ = 2.*(eigen_qdX_*eigen_qrX_.conjugate() ).vec();

    ROS_INFO_STREAM(std::endl << eigen_qrX_.toRotationMatrix()*eigen_rotmat_ << std::endl << eigen_rotmat_*eigen_t_);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vive_robot_calibrating_node");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CalibratingNode node_(120);

    if (!node_.Init() ) {
        exit(EXIT_FAILURE);
    }

    while (ros::ok() ) {
        node_.Loop();
    }

    node_.Shutdown();
    exit(EXIT_SUCCESS);
}