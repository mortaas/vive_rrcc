// ROS
#include <ros/ros.h>
#include <rosbag/bag.h>
// ROS msgs
#include <vive_bridge/TrackedDevicesStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include "vive_calibrating/ViveConfig.h"

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

// tf2
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

// Eigen
#include <Eigen/Dense>

// #include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <unsupported/Eigen/MatrixFunctions>

// MoveIt!
#include <moveit/move_group_interface/move_group_interface.h>

// Boost
#include <boost/algorithm/string/predicate.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"

// STL
#include <cmath>
#include <random>



class CalibratingNode {
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;

    // Subscribers
    ros::Subscriber joy_sub_;
    ros::Subscriber devices_sub_;

    void JoyCb(const sensor_msgs::Joy& msg_);

    void DevicesCb(const vive_bridge::TrackedDevicesStamped& msg_);
    bool ValidateDeviceID(std::string device_id);

    // ROS msgs
    geometry_msgs::TransformStamped tf_msg_;

    // Parameters
    bool InitParams();

    // Dynamic reconfigure
    dynamic_reconfigure::Server<vive_calibrating::ViveConfig> reconf_server_;
    dynamic_reconfigure::Server<vive_calibrating::ViveConfig>::CallbackType callback_type_;
    void ReconfCallback(vive_calibrating::ViveConfig &config, uint32_t level);

    // Reconfigure request for changing frame offset, e.g. changing world_vr frame w.r.t. world
    dynamic_reconfigure::ReconfigureRequest srv_reconf_req_;
    dynamic_reconfigure::ReconfigureResponse srv_reconf_resp_;
    vive_calibrating::ViveConfig conf_;

    // MoveIt!
    static const std::string PLANNING_GROUP;
    moveit::planning_interface::MoveGroupInterface move_group_;

    // tf2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
    tf2_ros::TransformListener *tf_listener_;

    std::string controller_frame, tracker1_frame, tracker2_frame;
    tf2::Transform tf_tracker1_[2], tf_tracker2_[2];
    tf2::Transform tf_tool0_[2];

    double yaw_offset, pitch_offset, roll_offset;

    // Random number generator (RNG) for generating random poses
    std::random_device random_seed;
    std::mt19937_64 rng;
    std::uniform_real_distribution<double> r_dist, theta_dist, phi_dist;

    geometry_msgs::Pose GenerateRandomPose();
    geometry_msgs::Pose SphereNormalPose(double r, double theta, double phi);

    geometry_msgs::TransformStamped ComputeTransformFromPoses(const geometry_msgs::TransformStamped &tf_msg_T0_,
                                                              const geometry_msgs::TransformStamped &tf_msg_T1_);
    Eigen::Matrix3d RotationMatrixLogarithm(const Eigen::Matrix3d &rotmat_);

    void MeasureRobot();
    void CalibrateWorld();
    void CalibrateRobotRough();
    geometry_msgs::TransformStamped CalibrateRobot(geometry_msgs::TransformStamped A[],
                                                   geometry_msgs::TransformStamped B[],
                                                   int size);

    public:
        CalibratingNode(int frequency);
        ~CalibratingNode();

        bool Init();
        void Loop();
        void Shutdown();
};
const std::string CalibratingNode::PLANNING_GROUP = "manipulator";

geometry_msgs::Pose CalibratingNode::GenerateRandomPose() {
     /**
      * Generate random pose from pre-defined uniform distributions r_dist, theta_dist, phi_dist
      * Two random poses are generated, and position is used from the first pose and orientation is used from the second pose
      */

    geometry_msgs::Pose pose_position_ = SphereNormalPose(r_dist(rng), theta_dist(rng), phi_dist(rng) );
    geometry_msgs::Pose pose_ = SphereNormalPose(r_dist(rng), theta_dist(rng), phi_dist(rng) );
    
    // Set position from the first random pose
    pose_.position.x = pose_position_.position.x;
    pose_.position.y = pose_position_.position.y;
    pose_.position.z = pose_position_.position.z;

    return pose_;
}

geometry_msgs::TransformStamped CalibratingNode::ComputeTransformFromPoses(const geometry_msgs::TransformStamped &tf_msg_T0_,
                                                                           const geometry_msgs::TransformStamped &tf_msg_T1_)
{
     /**
      * Compute the transform from pose T0 to pose T1.
      */
    
    // Convert transform messages to Eigen's 4x4 transformation matrices (OpenGL)
    Eigen::Affine3d eigen_T0_ = tf2::transformToEigen(tf_msg_T0_);
    Eigen::Affine3d eigen_T1_ = tf2::transformToEigen(tf_msg_T1_);
    // Get rotation matrices from poses
    Eigen::Matrix3d eigen_R0_ = eigen_T0_.matrix().block<3, 3>(0, 0);
    Eigen::Matrix3d eigen_R1_ = eigen_T1_.matrix().block<3, 3>(0, 0);
    // Get translation vectors from poses
    Eigen::Vector3d eigen_t0_ = eigen_T0_.matrix().col(3).head<3>();
    Eigen::Vector3d eigen_t1_ = eigen_T1_.matrix().col(3).head<3>();

    // Compute transform
    Eigen::Affine3d eigen_Tx_;
    eigen_Tx_.matrix().block<3, 3>(0, 0) = eigen_R0_.transpose() * eigen_R1_;
    eigen_Tx_.matrix().col(3).head<3>() = eigen_t1_ - eigen_t0_;

    return tf2::eigenToTransform(eigen_Tx_);
}

Eigen::Matrix3d CalibratingNode::RotationMatrixLogarithm(const Eigen::Matrix3d &rotmat_) {
     /**
      * Compute the logarithm of a 3x3 rotation matrix
      */
    
    // Axis-angle magnitude
    double theta = std::acos((rotmat_.trace() - 1.) / 2.);

    // Check if logarithm is uniquely defined
    if (rotmat_.trace() != -1.) {
        return theta / (2. * sin(theta) ) * (rotmat_ - rotmat_.transpose() );
    } else {
        ROS_ERROR_STREAM("Error occurred when computing logarithm of rotation matrix:" << std::endl <<
                         rotmat_.matrix() << std::endl <<
                         "Which has a trace equal to " << rotmat_.trace() <<
                         ", and results in a logarithm that is not uniquely defined.");

        // Return NaN (Not-A-Number) matrix
        return Eigen::Matrix3d::Constant(0./0.);
    }
}

geometry_msgs::TransformStamped CalibratingNode::CalibrateRobot(geometry_msgs::TransformStamped tf_msg_Ta_[],
                                                                geometry_msgs::TransformStamped tf_msg_Tb_[],
                                                                int size)
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
      * Ta - TF messages array of end effector poses
      * Tb - TF messages array of VIVE Tracker poses
      * size - Size of both arrays (should be the same)
      * 
      * Output:
      * Tx - Transformation between end effector and VIVE Tracker (solution)
      */

    Eigen::Affine3d eigen_Ta_, eigen_Tb_;
    Eigen::Matrix3d eigen_Ra_[size], eigen_Rb_[size], eigen_Rx_, eigen_M_;
    Eigen::Vector3d eigen_ta_[size], eigen_tb_[size], eigen_tx_;

    Eigen::MatrixXd eigen_C_(3 * size, 3);
    Eigen::VectorXd eigen_d_(3 * size, 1);
    eigen_M_.setZero();
    eigen_C_.setZero();
    eigen_d_.setZero();

    for (int i = 0; i < size; i++) {
        // Convert tf messages to Eigen's 4x4 transformation matrices (OpenGL)
        eigen_Ta_ = tf2::transformToEigen(tf_msg_Ta_[i]);
        eigen_Tb_ = tf2::transformToEigen(tf_msg_Tb_[i]);
        // Get rotation matrices from transforms
        eigen_Ra_[i] = eigen_Ta_.matrix().block<3, 3>(0, 0);
        eigen_Rb_[i] = eigen_Tb_.matrix().block<3, 3>(0, 0);
        // Get translation vectors from transforms
        eigen_ta_[i] = eigen_Ta_.matrix().col(3).head<3>();
        eigen_tb_[i] = eigen_Tb_.matrix().col(3).head<3>();

        eigen_M_ += RotationMatrixLogarithm(eigen_Rb_[i]) *
                    RotationMatrixLogarithm(eigen_Ra_[i]).transpose();
    }

    // Compute the optimal rotation matrix R_x = (M^T * M)^(-1/2) * M^T with SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd_(eigen_M_.transpose() * eigen_M_,
                                           Eigen::ComputeFullU | Eigen::ComputeFullV);
    eigen_Rx_ = svd_.matrixU() * ((svd_.singularValues() ).cwiseSqrt().cwiseInverse() ).asDiagonal() *
                svd_.matrixV() * eigen_M_.transpose();
    
    // Eigen::EigenSolver<Eigen::Matrix3d> eigen_solver_(eigen_M_.transpose() * eigen_M_);
    // eigen_Rx_ = eigen_solver_.eigenvectors() * ((eigen_solver_.eigenvalues() ).cwiseSqrt().cwiseInverse() ).asDiagonal() *
    //             eigen_solver_.eigenvectors().inverse() * eigen_M_.transpose();

    for (int i = 0; i < size; i++) {
        eigen_C_.block<3, 3>(3*i, 0) = Eigen::Matrix3d::Identity() - eigen_Ra_[i];
        eigen_d_.segment(3*i, 3) = eigen_ta_[i] - eigen_Rx_ * eigen_tb_[i];
    }
    // Compute the optimal translation vector t_x = C^T / (C^T * C) * d
    eigen_tx_ = (eigen_C_.transpose() * eigen_C_).inverse() * eigen_C_.transpose() * eigen_d_;

    // Create and return the resulting 4x4 transformation matrix as geometry_msgs::TransformStamped
    Eigen::Affine3d eigen_Tx_;
    eigen_Tx_.matrix().block<3, 3>(0, 0) = eigen_Rx_;
    eigen_Tx_.matrix().col(3).head<3>()  = eigen_tx_;

    ROS_INFO_STREAM(eigen_Tx_.matrix() );
    return tf2::eigenToTransform(eigen_Tx_);
}

bool CalibratingNode::InitParams() {
     /**
      * Initialize parameters from the parameter server.
      * Return true if the parameters were retrieved from the server, false otherwise.
      */
    
    std::string controller, tracker;

    bool init_success (nh_.param<std::string>("/vive_calibrating_node/controller", controller, "") &&
                       nh_.param<std::string>("/vive_calibrating_node/tracker",    tracker,    "") );
    
    // Use controller and/or tracker from parameter server if their IDs are valid
    if (ValidateDeviceID(controller) ) {
        controller_frame = controller;
    }
    if (ValidateDeviceID(tracker) ) {
        tracker1_frame = tracker;
        joy_sub_ = nh_.subscribe("/vive_node/joy/" + controller_frame, 1, &CalibratingNode::JoyCb, this);
    }

    return init_success;
}

void CalibratingNode::ReconfCallback(vive_calibrating::ViveConfig &config, uint32_t level) {
     /**
      * Dynamic reconfigure callback for changing parameters during runtime
      */
    
    // Update controller and/or trackers if their IDs are valid
    if (ValidateDeviceID(config.controller) ) {
        controller_frame = config.controller;
        joy_sub_ = nh_.subscribe("/vive_node/joy/" + controller_frame, 1, &CalibratingNode::JoyCb, this);
    }
    if (ValidateDeviceID(config.tracker1) ) {
        tracker1_frame = config.tracker1;
    }
    if (ValidateDeviceID(config.tracker2) ) {
        tracker1_frame = config.tracker2;
    }
}

CalibratingNode::CalibratingNode(int frequency)
    : loop_rate_(frequency),
      tf_listener_(new tf2_ros::TransformListener(tf_buffer_) ),
      move_group_(moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP) ),
      rng(random_seed() ),
      r_dist(0.4, 0.8),
      theta_dist(4.5 * M_PI_4, 5.5 * M_PI_4),
      phi_dist(1.5 * M_PI_4, 2.5 * M_PI_4)
{
    // Subscribers
    devices_sub_ = nh_.subscribe("/vive_node/tracked_devices", 1, &CalibratingNode::DevicesCb, this);

    // Set dynamic reconfigure callback function
    callback_type_ = boost::bind(&CalibratingNode::ReconfCallback, this, _1, _2);
    reconf_server_.setCallback(callback_type_);

    // Define dynamic reconfigure message for calibrating frames
    srv_reconf_req_.config.doubles.resize(12);
    srv_reconf_req_.config.doubles[0].name = "vr_x_offset";
    srv_reconf_req_.config.doubles[1].name = "vr_y_offset";
    srv_reconf_req_.config.doubles[2].name = "vr_z_offset";
    srv_reconf_req_.config.doubles[3].name = "vr_yaw_offset";
    srv_reconf_req_.config.doubles[4].name = "vr_pitch_offset";
    srv_reconf_req_.config.doubles[5].name = "vr_roll_offset";
    srv_reconf_req_.config.doubles[6].name = "robot_x_offset";
    srv_reconf_req_.config.doubles[7].name = "robot_y_offset";
    srv_reconf_req_.config.doubles[8].name = "robot_z_offset";
    srv_reconf_req_.config.doubles[9].name = "robot_yaw_offset";
    srv_reconf_req_.config.doubles[10].name = "robot_pitch_offset";
    srv_reconf_req_.config.doubles[11].name = "robot_roll_offset";

    // Set planning parameters for the manipulator move group (MoveIt!)
    move_group_.setPoseReferenceFrame("base");
    move_group_.setMaxVelocityScalingFactor(0.25);
}
CalibratingNode::~CalibratingNode() {
}

void CalibratingNode::CalibrateWorld() {
     /**
      * Calibrates the world_vr frame based on current VIVE Tracker pose.
      * Offset parameters are calculated by thinking of the VIVE Tracker frame as the new world frame.
      */
    
    // Lookup transformation from VIVE Tracker to world_vr
    tf_msg_ = tf_buffer_.lookupTransform(tracker1_frame, "world_vr", ros::Time(0) );
    tf2::fromMsg(tf_msg_.transform, tf_tracker1_[0]);

    // Set new offset parameters based on transformation
    srv_reconf_req_.config.doubles[0].value = tf_tracker1_[0].getOrigin().getX();
    srv_reconf_req_.config.doubles[1].value = tf_tracker1_[0].getOrigin().getY();
    srv_reconf_req_.config.doubles[2].value = tf_tracker1_[0].getOrigin().getZ();
    tf_tracker1_[0].getBasis().getRPY(roll_offset, pitch_offset, yaw_offset);
    srv_reconf_req_.config.doubles[3].value = yaw_offset;
    srv_reconf_req_.config.doubles[4].value = pitch_offset;
    srv_reconf_req_.config.doubles[5].value = roll_offset;
    // Send request to the dynamic reconfigure service in vive_node
    ros::service::call("/vive_node/set_parameters", srv_reconf_req_, srv_reconf_resp_);
}

void CalibratingNode::CalibrateRobotRough() {
     /**
      * Calibrates the world_vr frame based on the desired VIVE Tracker frame.
      * The desired frame is defined relative to the robot in the launch file for this node.
      */
    
    // Lookup transformation from VIVE Tracker to world_vr
    tf_msg_ = tf_buffer_.lookupTransform(tracker1_frame, "world_vr", ros::Time(0) );
    tf2::fromMsg(tf_msg_.transform, tf_tracker1_[0]);
    
    // Lookup transformation from world to desired VIVE Tracker
    tf_msg_ = tf_buffer_.lookupTransform("world", "tracker1_desired", ros::Time(0) );
    tf2::Transform tf_tracker_desired_;
    tf2::fromMsg(tf_msg_.transform, tf_tracker_desired_);

    // Compute transformation from world to world_vr based on desired VIVE Tracker frame
    tf2::Transform tf_world_vr_ = tf_tracker_desired_ * tf_tracker1_[0];

    // Set new offset parameters based on transformation
    srv_reconf_req_.config.doubles[0].value = tf_world_vr_.getOrigin().getX();
    srv_reconf_req_.config.doubles[1].value = tf_world_vr_.getOrigin().getY();
    srv_reconf_req_.config.doubles[2].value = tf_world_vr_.getOrigin().getZ();
    tf_world_vr_.getBasis().getRPY(roll_offset, pitch_offset, yaw_offset);
    srv_reconf_req_.config.doubles[3].value = yaw_offset;
    srv_reconf_req_.config.doubles[4].value = pitch_offset;
    srv_reconf_req_.config.doubles[5].value = roll_offset;
    // Send request to the dynamic reconfigure service in vive_node
    ros::service::call("/vive_node/set_parameters", srv_reconf_req_, srv_reconf_resp_);
}

void CalibratingNode::JoyCb(const sensor_msgs::Joy& msg_) {
      /**
     * Handle VIVE Controller inputs
     */

    // Menu button - Calibrates world frame = VIVE Tracker frame
    if (msg_.buttons[0]) {
        CalibrateWorld();
    }

    // Grip button - Runs a calibration routine with measurements from the robot
    // if (msg_.buttons[1]) {
    //     MeasureRobot();
    // }

    // Trackpad button - Calibrates the world frame to hard coded default values
    if (msg_.buttons[2]) {
        srv_reconf_req_.config.doubles[0].value = 1.1245388116;
        srv_reconf_req_.config.doubles[1].value = -0.608462828586;
        srv_reconf_req_.config.doubles[2].value = 1.49679019946;
        srv_reconf_req_.config.doubles[3].value = 1.02410968817;
        srv_reconf_req_.config.doubles[4].value = -0.0623919286899;
        srv_reconf_req_.config.doubles[5].value = 1.63425725418;

        ros::service::call("/vive_node/set_parameters", srv_reconf_req_, srv_reconf_resp_);
    }

    // Trigger button - Calibrates world frame based on the desired VIVE Tracker frame
    if (msg_.buttons[3]) {
        CalibrateRobotRough();
    }
}

void CalibratingNode::DevicesCb(const vive_bridge::TrackedDevicesStamped& msg_) {
      /**
     * Update controller and tracker frames of tracked devices.
     */

    for (int i = 0; i < msg_.device_count; i++) {
        if (msg_.device_classes[i] == msg_.CONTROLLER) {
            controller_frame = msg_.device_frames[i];
        }
        if (msg_.device_classes[i] == msg_.TRACKER) {
            if (tracker1_frame.empty() ) {
                tracker1_frame = msg_.device_frames[i];
            } else {
                tracker2_frame = msg_.device_frames[i];
            }
        }
    }
}

bool CalibratingNode::ValidateDeviceID(std::string device_id) {
     /**
      * Validate device ID by checking string format and character length.
      * The serial number of tracked devices is 12 characters.
      */
    
    if (boost::algorithm::contains(device_id, "hmd_") ) {
        if (strlen(device_id.c_str() ) == 16) {
            return true;
        }
    }
    if (boost::algorithm::contains(device_id, "controller_") ) {
        if (strlen(device_id.c_str() ) == 23) {
            return true;
        }
    }
    if (boost::algorithm::contains(device_id, "tracker_") ) {
        if (strlen(device_id.c_str() ) == 20) {
            return true;
        }
    }
    if (boost::algorithm::contains(device_id, "lighthouse_") ) {
        if (strlen(device_id.c_str() ) == 23) {
            return true;
        }
    }

    return false;
}

void CalibratingNode::MeasureRobot() {
     /**
      * Runs a calibration routine on the robot by moving the robot to random poses.
      * The poses are generated by computing the normal vector of a sphere with random radius and angles.
      * 
      * Measurements for each pose are saved to a bag file in the home folder and consists of tf messages:
      * - camera_house
      * - Tracker1
      * - Tracker2
      * - (N - 1) transformations between poses
      * Each pose is w.r.t. the base frame
      * 
      * The transformation from camera_house to tracker is then calculated based on the measurements.
      */
    
    rosbag::Bag bag_;
    std::string bag_filename = "cal_data_" + boost::posix_time::to_iso_string(ros::Time::now().toBoost() ) + ".bag";
    ROS_INFO_STREAM("Writing measurements to " + bag_filename);
    bag_.open(bag_filename, rosbag::bagmode::Write);

    // geometry_msgs::TransformStamped tf_tracker1_desired_, tf_tracker2_desired_;
    // geometry_msgs::TransformStamped tf_tracker1_error_, tf_tracker2_error_;

    geometry_msgs::TransformStamped tf_msg_tool0_[2], tf_msg_tracker1_[2], tf_msg_tracker2_[2];
    std::vector<geometry_msgs::TransformStamped> tf_msgs_A_, tf_msgs_B1_, tf_msgs_B2_;
    geometry_msgs::TransformStamped tf_msg_A_, tf_msg_B1_, tf_msg_B2_;

    geometry_msgs::PoseStamped pose_msg_;
    pose_msg_.header.frame_id = "base";

    pose_msg_.pose = GenerateRandomPose();
    pose_msg_.header.stamp = ros::Time::now();

    int N = 50;

    bag_.write("desired_pose", ros::Time::now(), pose_msg_);
    move_group_.setPoseTarget(pose_msg_);
    if (move_group_.move() ) {
        ROS_INFO_STREAM("0/" << N << ": Trajectory execution succeeded");

        move_group_.stop();
        // ros::Duration(0.25).sleep();
    } else {
        ROS_WARN_STREAM("0/" << N << ": Trajectory execution failed with pose");
        ROS_WARN_STREAM(pose_msg_.pose);
    }

    std::string pError;
    if (tf_buffer_.canTransform("base", "camera_house", ros::Time(0), &pError) &&
        tf_buffer_.canTransform("base", tracker1_frame, ros::Time(0), &pError) &&
        tf_buffer_.canTransform("base", tracker2_frame, ros::Time(0), &pError) )
    {
        // Lookup and convert necessary transforms
        tf_msg_tool0_[0] = tf_buffer_.lookupTransform("base", "camera_house", ros::Time(0) );
        // tf_msg_tracker1_[0] = tf_buffer_.lookupTransform("base", "tracker1_desired", ros::Time(0) );
        // tf_msg_tracker2_[0] = tf_buffer_.lookupTransform("base", "tracker2_desired", ros::Time(0) );
        tf_msg_tracker1_[0] = tf_buffer_.lookupTransform("base", tracker1_frame, ros::Time(0) );
        tf_msg_tracker2_[0] = tf_buffer_.lookupTransform("base", tracker2_frame, ros::Time(0) );

        bag_.write("camera_house", ros::Time::now(), tf_msg_tool0_[0]);
        bag_.write("tracker1", ros::Time::now(), tf_msg_tracker1_[0]);
        bag_.write("tracker2", ros::Time::now(), tf_msg_tracker2_[0]);

        for (int i = 1; i <= N; i++) {
            pose_msg_.pose = GenerateRandomPose();

            // Visualize desired pose as transform in RViz
            tf2::Transform tf_pose_;
            geometry_msgs::TransformStamped tf_msg_pose_;
            tf2::convert(pose_msg_.pose, tf_pose_);
            tf2::convert(tf_pose_, tf_msg_pose_.transform);
            tf_msg_pose_.header.frame_id = "base";
            tf_msg_pose_.child_frame_id = "desired_pose";
            tf_msg_pose_.header.stamp = ros::Time::now();
            static_tf_broadcaster_.sendTransform(tf_msg_pose_);

            pose_msg_.header.stamp = ros::Time::now();
            bag_.write("desired_pose", ros::Time::now(), pose_msg_);

            move_group_.setPoseTarget(pose_msg_);
            if (move_group_.move() ) {
                ROS_INFO_STREAM(i << "/" << N << ": Trajectory execution succeeded");

                move_group_.stop();
                ros::Duration(3.00).sleep();

                if (tf_buffer_.canTransform("base", "camera_house", ros::Time(0), &pError) &&
                tf_buffer_.canTransform("base", tracker1_frame, ros::Time(0), &pError) &&
                tf_buffer_.canTransform("base", tracker2_frame, ros::Time(0), &pError) )
                {
                    // Lookup and convert necessary transforms
                    tf_msg_tool0_[1] = tf_buffer_.lookupTransform("base", "camera_house", ros::Time(0) );
                    // tf_msg_tracker1_[1] = tf_buffer_.lookupTransform("base", "tracker1_desired", ros::Time(0) );
                    // tf_msg_tracker2_[1] = tf_buffer_.lookupTransform("base", "tracker2_desired", ros::Time(0) );
                    tf_msg_tracker1_[1] = tf_buffer_.lookupTransform("base", tracker1_frame, ros::Time(0) );
                    tf_msg_tracker2_[1] = tf_buffer_.lookupTransform("base", tracker2_frame, ros::Time(0) );

                    tf_msg_A_ = ComputeTransformFromPoses(tf_msg_tool0_[0], tf_msg_tool0_[1]);
                    tf_msg_B1_ = ComputeTransformFromPoses(tf_msg_tracker1_[0], tf_msg_tracker1_[1]);
                    tf_msg_B2_ = ComputeTransformFromPoses(tf_msg_tracker2_[0], tf_msg_tracker2_[1]);

                    tf_msgs_A_.push_back(tf_msg_A_);
                    tf_msgs_B1_.push_back(tf_msg_B1_);
                    tf_msgs_B2_.push_back(tf_msg_B2_);

                    // Set reference for next pose
                    tf_msg_tool0_[0] = tf_msg_tool0_[1];
                    tf_msg_tracker1_[0] = tf_msg_tracker1_[1];
                    tf_msg_tracker2_[0] = tf_msg_tracker2_[1];

                    bag_.write("tool0", ros::Time::now(), tf_msg_tool0_[0]);
                    bag_.write("tracker1", ros::Time::now(), tf_msg_tracker1_[0]);
                    bag_.write("tracker2", ros::Time::now(), tf_msg_tracker2_[0]);
                    bag_.write("A", ros::Time::now(), tf_msg_A_);
                    bag_.write("B1", ros::Time::now(), tf_msg_B1_);
                    bag_.write("B2", ros::Time::now(), tf_msg_B2_);

                    geometry_msgs::TransformStamped tf_msg_X1_ = CalibrateRobot(tf_msgs_A_.data(), tf_msgs_B1_.data(), tf_msgs_A_.size() );
                    geometry_msgs::TransformStamped tf_msg_X2_ = CalibrateRobot(tf_msgs_A_.data(), tf_msgs_B2_.data(), tf_msgs_A_.size() );
                    bag_.write("X1", ros::Time::now(), tf_msg_X1_);
                    bag_.write("X2", ros::Time::now(), tf_msg_X2_);
                } else {
                    ROS_WARN_STREAM(pError);
                }
            } else {
                ROS_WARN_STREAM(i << "/" << N << ": Trajectory execution failed with pose");
                ROS_WARN_STREAM(pose_msg_.pose);
            }
        }
    } else {
        ROS_WARN_STREAM("0/" << N << ": " << pError);
    }

    bag_.close();

    // Test code for calibrating the robot w.r.t. to the world_vr frame

    // geometry_msgs::TransformStamped tf_msg_X1_ = CalibrateRobot(tf_msgs_A_.data(), tf_msgs_B1_.data(), tf_msgs_A_.size() );
    // geometry_msgs::TransformStamped tf_msg_X2_ = CalibrateRobot(tf_msgs_A_.data(), tf_msgs_B2_.data(), tf_msgs_A_.size() );

    // geometry_msgs::TransformStamped tf_msg_base_ = tf_buffer_.lookupTransform("camera_house", "base", ros::Time(0) );
    // geometry_msgs::TransformStamped tf_msg_tracker_ = tf_buffer_.lookupTransform(tracker1_frame, "world_vr", ros::Time(0) );

    // tf2::Transform tf_X_, tf_base_, tf_tracker_;
    // tf2::convert(tf_msg_X_.transform, tf_X_);
    // tf2::convert(tf_msg_base_.transform, tf_base_);
    // tf2::convert(tf_msg_base_.transform, tf_tracker_);

    // tf2::Transform tf_robot_ = tf_tracker_ * tf_X_.inverse() * tf_base_.inverse();

    // srv_reconf_req_.config.doubles[6].value = tf_robot_.getOrigin().getX();
    // srv_reconf_req_.config.doubles[7].value = tf_robot_.getOrigin().getY();
    // srv_reconf_req_.config.doubles[8].value = tf_robot_.getOrigin().getZ();
    // tf_robot_.getBasis().getRPY(roll_offset, pitch_offset, yaw_offset);
    // srv_reconf_req_.config.doubles[9].value = yaw_offset;
    // srv_reconf_req_.config.doubles[10].value = pitch_offset;
    // srv_reconf_req_.config.doubles[11].value = roll_offset;

    // ros::service::call("/vive_node/set_parameters", srv_reconf_req_, srv_reconf_resp_);
}

bool CalibratingNode::Init() {
      /**
     * Initialize the node and check if the necessary transforms are available
     */

    // Test code for solving AX = BX with example from report:
    // Robot sensor calibration: solving AX=XB on the Euclidean group
    // https://ieeexplore.ieee.org/document/326576/

    // tf2::Transform tf_A1_(tf2::Matrix3x3(-0.989992, -0.141120, 0., 0.141120, -0.989992, 0., 0., 0., 1.), tf2::Vector3(0., 0., 0.) );
    // tf2::Transform tf_A2_(tf2::Matrix3x3(0.070737, 0., 0.997495, 0., 1., 0., -0.997495, 0., 0.070737), tf2::Vector3(-400., 0., 400.) );
    // tf2::Transform tf_B1_(tf2::Matrix3x3(-0.989992, -0.138307, 0.028036,  0.138307, -0.911449,  0.387470, -0.028036,  0.387470, 0.921456), tf2::Vector3(- 26.9559, -96.1332,  19.4872) );
    // tf2::Transform tf_B2_(tf2::Matrix3x3( 0.070737,  0.198172, 0.997612, -0.198172,  0.963323, -0.180936, -0.977612, -0.180936, 0.107415), tf2::Vector3(-309.5430,  59.0244, 291.1770) );

    // geometry_msgs::TransformStamped tf_msg_A_[2];
    // tf2::convert(tf_A1_, tf_msg_A_[0].transform);
    // tf2::convert(tf_A2_, tf_msg_A_[1].transform);
    // geometry_msgs::TransformStamped tf_msg_B_[2];
    // tf2::convert(tf_B1_, tf_msg_B_[0].transform);
    // tf2::convert(tf_B2_, tf_msg_B_[1].transform);

    // geometry_msgs::TransformStamped tf_msg_X_ = CalibrateRobot(tf_msg_A_, tf_msg_B_, 2);
    // tf_msg_X_.header.frame_id = "camera_house";
    // tf_msg_X_.child_frame_id = tracker1_frame;
    // tf_msg_X_.header.stamp = ros::Time::now();

    if (!InitParams() ) {
        ROS_WARN("Failed to get parameters from the parameter server.");
    }

    if (controller_frame.empty() && tracker1_frame.empty() ) {
        while (controller_frame.empty() && tracker1_frame.empty() ) {
            ROS_INFO("Waiting for controller and trackers...");
            
            ros::spinOnce();
            ros::Duration(5.0).sleep();
        }

        conf_.controller = controller_frame;
        conf_.tracker1 = tracker1_frame;
        conf_.tracker2 = tracker2_frame;
        reconf_server_.updateConfig(conf_);
    }
    ROS_INFO_STREAM("Using " + controller_frame + " and " + tracker1_frame + ", " + tracker2_frame + " for calibration");

    joy_sub_ = nh_.subscribe("/vive_node/joy/" + controller_frame, 1, &CalibratingNode::JoyCb, this);

    std::string pError;
    if (!tf_buffer_.canTransform(tracker1_frame,
                                 "world", ros::Time(0),
                                 ros::Duration(5.0), &pError) )
    {
        ROS_ERROR_STREAM("Can't transform from world to " + tracker1_frame + ": " + pError);

        return false;
    }

    MeasureRobot();
    return false;

    // if (!tf_buffer_.canTransform(tracker2_frame,
    //                              "world", ros::Time(0),
    //                              ros::Duration(5.0), &pError) )
    // {
    //     ROS_ERROR_STREAM("Can't transform from world to " + tracker2_frame + ": " + pError);

    //     return false;
    // }
    
    return true;
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

geometry_msgs::Pose CalibratingNode::SphereNormalPose(double r, double theta, double phi) {
      /**
     * Find pose based on normal vector of sphere with radius r, polar angle theta and azimuthal angle phi
     */

    tf2::Quaternion tf_quat_ = tf2::Quaternion();
    tf_quat_.setRPY(0., 0., -M_PI_4);
    tf2::Transform tf_rot_ = tf2::Transform(tf_quat_, tf2::Vector3(0., 0., 0.) );

    tf2::Matrix3x3 tf_rotmat_;
    tf_rotmat_.setEulerYPR(theta, phi, 0.);
    tf2::Transform tf_pose_(tf_rotmat_, r * (tf_rotmat_ * tf2::Vector3(0., 0., 1.) ) );
    tf_pose_ *= tf_rot_;

    geometry_msgs::Pose pose_;
    tf2::toMsg(tf_pose_, pose_);

    return pose_;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vive_robot_calibrating_node");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CalibratingNode node_(120);

    if (!node_.Init() ) {
        return 0;
    }

    while (ros::ok() ) {
        node_.Loop();
    }

    node_.Shutdown();
    return 0;
}