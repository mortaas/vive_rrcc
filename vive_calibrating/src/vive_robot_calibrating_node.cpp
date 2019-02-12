#include "vive_robot_calibrating_node.h"

// Handle signal [ctrl + c]
bool sigint_flag = true;

void IntHandler(int signal) {
    sigint_flag = false;
}


CalibratingNode::CalibratingNode(int frequency)
    : loop_rate_(frequency),
      tf_listener_(new tf2_ros::TransformListener(tf_buffer_) ),
      move_group_(moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP) ),
      rng1(random_seed1() ),
      rng2(random_seed2() )
{
    // Publishers
    traj_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/floor/position_trajectory_controller/command", 2, false);

    // Subscribers
    device_sub_ = nh_.subscribe("/vive_node/tracked_devices", 1, &CalibratingNode::DevicesCb, this);

    // Services
    sample_client =  nh_.serviceClient<vive_calibrating::AddSample>("/vive_calibration/add_sample");
    compute_client = nh_.serviceClient<vive_calibrating::ComputeCalibration>("/vive_calibration/compute_calibration");

    // Define dynamic reconfigure message for calibrating frames
    srv_reconf_req_.config.doubles.resize(6);
    srv_reconf_req_.config.doubles[0].name = "vr_x_offset";
    srv_reconf_req_.config.doubles[1].name = "vr_y_offset";
    srv_reconf_req_.config.doubles[2].name = "vr_z_offset";
    srv_reconf_req_.config.doubles[3].name = "vr_yaw_offset";
    srv_reconf_req_.config.doubles[4].name = "vr_pitch_offset";
    srv_reconf_req_.config.doubles[5].name = "vr_roll_offset";
    
    // Set planning parameters of the MoveIt! move group
    move_group_.setPoseReferenceFrame(base_frame);
    // move_group_.setMaxVelocityScalingFactor(0.5);

    // MoveIt! constraints
    move_group_.clearPathConstraints();

    moveit_msgs::Constraints constraints_msg_;

    constraints_msg_.joint_constraints.push_back(moveit_msgs::JointConstraint() );
    constraints_msg_.joint_constraints.back().joint_name = "floor_joint_a3";
    constraints_msg_.joint_constraints.back().position = M_PI_2;
    constraints_msg_.joint_constraints.back().tolerance_above = M_PI_2;
    constraints_msg_.joint_constraints.back().tolerance_below = M_PI_2;
    constraints_msg_.joint_constraints.back().weight = 1.;

    constraints_msg_.joint_constraints.push_back(moveit_msgs::JointConstraint() );
    constraints_msg_.joint_constraints.back().joint_name = "floor_joint_a5";
    constraints_msg_.joint_constraints.back().position = 0.;
    constraints_msg_.joint_constraints.back().tolerance_above = M_PI_2;
    constraints_msg_.joint_constraints.back().tolerance_below = M_PI_2;
    constraints_msg_.joint_constraints.back().weight = 1.;

    constraints_msg_.joint_constraints.push_back(moveit_msgs::JointConstraint() );
    constraints_msg_.joint_constraints.back().joint_name = "floor_joint_a6";
    constraints_msg_.joint_constraints.back().position = 0.;
    constraints_msg_.joint_constraints.back().tolerance_above = M_PI_2;
    constraints_msg_.joint_constraints.back().tolerance_below = M_PI_2;
    constraints_msg_.joint_constraints.back().weight = 1.;

    move_group_.setPathConstraints(constraints_msg_);
}
CalibratingNode::~CalibratingNode() {
}

bool CalibratingNode::InitParams() {
     /**
      * Initialize parameters from the parameter server.
      * Returns true if the parameters were retrieved from the server, false otherwise.
      */

    if (nh_.param<std::string>("/vive_node/vr_frame", vr_frame, "world_vr") &&
        nh_.param<std::string>("/vive_node/world_frame", world_frame, "root") &&
        nh_.param<std::string>("/vive_calibrating_node/controller", controller_frame, "") &&
        nh_.param<std::string>("/vive_calibrating_node/base_frame", base_frame, "floor_base") &&
        nh_.param<std::string>("/vive_calibrating_node/tool_frame", tool_frame, "floor_tool0") &&
        nh_.param<std::string>("/vive_calibrating_node/test_frame", test_frame, "controller_test") )
    {
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

    if (!InitParams() ) {
        while (controller_frame.empty() && sigint_flag) {
            ROS_INFO("Waiting for available VIVE controller...");
            
            ros::spinOnce();
            ros::Duration(2.5).sleep();
        }

        if (controller_frame.empty() ) {
            return false;
        }
    }
    ROS_INFO_STREAM("Using " + controller_frame + " for calibration");

    std::string pError;
    if (!tf_buffer_.canTransform(vr_frame, controller_frame, ros::Time(0),
                                 ros::Duration(5.0), &pError) )
    {
        ROS_ERROR_STREAM("Can't transform from " + vr_frame + " to " + controller_frame + ": " + pError);

        return false;
    }

    // Parameter distributions for random poses
    r_dist = std::uniform_real_distribution<double>(1.2, 1.4);
    theta_dist1 = std::uniform_real_distribution<double>(5.5 * M_PI_4, 6.5 * M_PI_4);
    phi_dist1 = std::uniform_real_distribution<double>(0.5 * M_PI_4, 1.5 * M_PI_4);
    theta_dist2 = std::uniform_real_distribution<double>(4.5 * M_PI_4, 7.5 * M_PI_4);
    phi_dist2 = std::uniform_real_distribution<double>(1. * M_PI_4, 3. * M_PI_4);

    // Initialize transform headers
    tf_msg_pose_.header.frame_id = base_frame;
    tf_msg_pose_.child_frame_id = "tool0_desired";
    
    pose_msg_.header.frame_id = base_frame;

    if (!tf_buffer_.canTransform(test_frame, tool_frame, ros::Time(0),
                                 ros::Duration(5.0), &pError) )
    {
        ROS_ERROR_STREAM("Can't transform from " + tool_frame + " to " + controller_frame + ": " + pError);

        return false;
    } else {
        tf_msg_X_ = tf_buffer_.lookupTransform(test_frame, tool_frame, ros::Time(0) );
        tf_msg_X_inv_ = tf_buffer_.lookupTransform(tool_frame, test_frame, ros::Time(0) );

        tf2::convert(tf_msg_X_.transform, tf_X_);
        tf2::convert(tf_msg_X_inv_.transform, tf_X_inv_);
    }

    ROS_INFO_STREAM(tf_msg_X_.transform);
    ROS_INFO_STREAM(tf_msg_X_inv_.transform);

    joint_folded = {1.5707893454778887, -2.5900040327772613, 2.3999184786133068, -2.6179938779914945e-05, 0.799936756066561, 8.377580409572782e-05};

    std::vector<geometry_msgs::PoseStamped> test_poses_;

    // front left
    FillTestPlanePoses(test_poses_, base_frame, 0.25, 3, 0.7, -0.7, 1.1);
    FillTestPlanePoses(test_poses_, base_frame, 0.25, 3, 0.7, -0.7, 1.2);
    FillTestPlanePoses(test_poses_, base_frame, 0.25, 3, 0.7, -0.7, 1.3);

    // front
    FillTestPlanePoses(test_poses_, base_frame, 0.25, 3, 0., -0.7, 1.1);
    FillTestPlanePoses(test_poses_, base_frame, 0.25, 3, 0., -0.7, 1.2);
    FillTestPlanePoses(test_poses_, base_frame, 0.25, 3, 0., -0.7, 1.3);

    // front right
    FillTestPlanePoses(test_poses_, base_frame, 0.25, 3, -0.7, -0.7, 1.1);
    FillTestPlanePoses(test_poses_, base_frame, 0.25, 3, -0.7, -0.7, 1.2);
    FillTestPlanePoses(test_poses_, base_frame, 0.25, 3, -0.7, -0.7, 1.3);

    // // right
    // FillTestPlanePoses(test_poses_, base_frame, 0.25, 3, -0.7, 0., 1.1);
    // FillTestPlanePoses(test_poses_, base_frame, 0.25, 3, -0.7, 0., 1.2);
    // FillTestPlanePoses(test_poses_, base_frame, 0.25, 3, -0.7, 0., 1.3);

    // // back right
    // FillTestPlanePoses(test_poses_, base_frame, 0.3, 3, -0.7, 0.7, 1.1);
    // FillTestPlanePoses(test_poses_, base_frame, 0.3, 3, -0.7, 0.7, 1.2);
    // FillTestPlanePoses(test_poses_, base_frame, 0.3, 3, -0.7, 0.7, 1.3);

    // // back
    // FillTestPlanePoses(test_poses_, base_frame, 0.3, 3, 0., 0.7, 1.1);
    // FillTestPlanePoses(test_poses_, base_frame, 0.3, 3, 0., 0.7, 1.2);
    // FillTestPlanePoses(test_poses_, base_frame, 0.3, 3, 0., 0.7, 1.3);

    // // back left
    // FillTestPlanePoses(test_poses_, base_frame, 0.25, 3, 0.7, 0.7, 1.1);
    // FillTestPlanePoses(test_poses_, base_frame, 0.25, 3, 0.7, 0.7, 1.2);
    // FillTestPlanePoses(test_poses_, base_frame, 0.25, 3, 0.7, 0.7, 1.3);

    // left
    // FillTestPlanePoses(test_poses_, base_frame, 0.25, 3, 0.7, 0., 1.1);
    // FillTestPlanePoses(test_poses_, base_frame, 0.25, 3, 0.7, 0., 1.2);
    // FillTestPlanePoses(test_poses_, base_frame, 0.25, 3, 0.7, 0., 1.3);

    ExecuteTestPoses(test_poses_);

    // MeasureRobot(100);
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

void CalibratingNode::FillTestPlanePoses(std::vector<geometry_msgs::PoseStamped> &poses_, std::string frame_id, 
                                         double L, int n, double x_offset, double y_offset, double z_offset)
{
    const double h = L/n;
    const double L_2 = L/2;

    const std::size_t poses_sz = poses_.size();
    poses_.resize(poses_sz + n*n + n + 1);

    geometry_msgs::PoseStamped pose_;
    pose_.header.stamp = ros::Time::now();
    pose_.header.frame_id = frame_id;

    for (int i = 0; i <= n; i++) {
        for (int j = 0; j <= n; j++) {
            pose_.pose.position.x = x_offset + h*i - L_2;
            pose_.pose.position.y = y_offset + h*j - L_2;
            pose_.pose.position.z = z_offset;

            double offset_angle = atan2(pose_.pose.position.y, pose_.pose.position.x);

            pose_.pose.orientation.x = 1.;
            pose_.pose.orientation.y = 0.;
            pose_.pose.orientation.z = 0.;
            pose_.pose.orientation.w = 0.;

            tf2::fromMsg(pose_.pose, tf_pose_);
            tf2::toMsg(tf_pose_.inverseTimes(tf_X_), pose_.pose);

            tf2::Quaternion pose_orientation_, pose_offset_;
            pose_offset_.setRPY(0., 0., -offset_angle);
            tf2::fromMsg(pose_.pose.orientation, pose_orientation_);
            tf2::convert(pose_offset_ * pose_orientation_, pose_.pose.orientation);

            poses_[poses_sz + i*n + j] = pose_;
        }
    }
}

void CalibratingNode::ExecuteTestPoses(std::vector<geometry_msgs::PoseStamped> &poses_) {
    // Open a bag file for recording poses (named with iso-date)
    bag_.open("test_data_" + boost::posix_time::to_iso_string(ros::Time::now().toBoost() ) + ".bag",
              rosbag::bagmode::Write);
    
    move_group_.setJointValueTarget(joint_folded);

    ROS_INFO_STREAM("Moving robot to folded state");
    if (move_group_.move() ) {
        ROS_INFO_STREAM("Trajectory execution succeeded");

        move_group_.stop();
        ros::Duration(0.5).sleep();
    } else {
        ROS_WARN_STREAM("Trajectory execution failed");
    }

    bag_.write("X", ros::Time::now(), tf_msg_X_);

    for (std::vector<geometry_msgs::PoseStamped>::iterator it_ = poses_.begin(); it_ != poses_.end(); ++it_) {
        MoveRobot(*it_);

        std::string pError;
        if (tf_buffer_.canTransform(test_frame, controller_frame, ros::Time(0), &pError) &&
            tf_buffer_.canTransform(test_frame, base_frame, ros::Time(0), &pError) )
        {
            tf_msg_sensor_ = tf_buffer_.lookupTransform(test_frame, ros::Time(0), base_frame, ros::Time(0), base_frame);
            // tf_msg_diff_ = tf_buffer_.lookupTransform(controller_frame, test_frame, ros::Time(0) );
            SampleSensor(controller_frame, test_frame, 120, 30, tf_msg_diff_);

            bag_.write("FK_diff", ros::Time::now(), tf_msg_diff_);
            bag_.write("FK_sensor", ros::Time::now(), tf_msg_sensor_);
        } else {
            ROS_WARN_STREAM(pError);
        }
    }

    move_group_.setJointValueTarget(joint_folded);

    ROS_INFO_STREAM("Moving robot to folded state");
    if (move_group_.move() ) {
        ROS_INFO_STREAM("Trajectory execution succeeded");

        move_group_.stop();
        ros::Duration(0.5).sleep();
    } else {
        ROS_WARN_STREAM("Trajectory execution failed");
    }

    bag_.close();
}

geometry_msgs::Pose CalibratingNode::SphereNormalPose(double r, double theta, double phi, geometry_msgs::Pose &pose_) {
      /**
     * Find pose based on normal vector of sphere with radius r, polar angle theta and azimuthal angle phi
     */

    // tf2::Transform tf_rot_ = tf2::Transform(tf2::Quaternion(0., 0., std::sin(-M_PI/8), std::cos(-M_PI/8) ),
    //                                         tf2::Vector3(0., 0., 0.) );

    tf2::Matrix3x3 tf_rotmat_;
    tf_rotmat_.setEulerYPR(theta, phi, 0.);

    tf2::Transform tf_pose_(tf_rotmat_, r * (tf_rotmat_ * tf2::Vector3(0., 0., 1.) ) );
    // tf_pose_ *= tf_rot_;

    tf2::toMsg(tf_pose_, pose_);

    return pose_;
}

geometry_msgs::Pose CalibratingNode::GenerateRandomPose(geometry_msgs::Pose &pose_) {
     /**
      * Generate random pose from pre-defined uniform distributions r_dist, theta_dist, phi_dist
      * Two random poses are generated, and position is used from the first pose and orientation is used from the second pose
      */

    geometry_msgs::Pose pose_pos_;
    SphereNormalPose(r_dist(rng1), theta_dist1(rng1), phi_dist1(rng1), pose_pos_ );
    SphereNormalPose(r_dist(rng2), theta_dist2(rng2), phi_dist2(rng2), pose_ );
    
    // Set position from the first random pose
    pose_.position = pose_pos_.position;

    return pose_;
}

bool CalibratingNode::MoveRobot(const geometry_msgs::PoseStamped &pose_) {
     /**
      * Move robot to provided pose, stop, and wait for the dynamics to settle down.
      * Returns true if trajectory execution succeeded.
      */
    
    // Visualize desired pose as a transform in RViz
    tf2::convert(pose_.pose, tf_pose_);
    tf2::convert(tf_pose_, tf_msg_pose_.transform);
    tf_msg_pose_.header.stamp = ros::Time::now();
    static_tf_broadcaster_.sendTransform(tf_msg_pose_);


    move_group_.setPoseTarget(pose_);

    ROS_INFO_STREAM("Moving to pose:" << std::endl << pose_);
    if (move_group_.move() ) {
        ROS_INFO_STREAM("Trajectory execution succeeded");

        move_group_.stop();
        ros::Duration(5.).sleep();
        return true;
    } else {
        ROS_WARN_STREAM("Trajectory execution failed");

        return false;
    }
}

void CalibratingNode::SampleSensor(const std::string &target_frame, const std::string &source_frame,
                                   const int &N, const int &F, geometry_msgs::TransformStamped &tf_msg_avg_)
{
    ros::Rate sample_rate_(F);

    eigen_translations_.resize(N);
    eigen_rotations_.resize(N);

    for (int i = 0; i < N; i++) {
        tf_msg_sensor_ = tf_buffer_.lookupTransform(target_frame, ros::Time(0), source_frame, ros::Time(0), target_frame);

        tf2::fromMsg(tf_msg_sensor_.transform.translation, eigen_translations_[i]);
        tf2::fromMsg(tf_msg_sensor_.transform.rotation, eigen_rotations_[i]);

        sample_rate_.sleep();
    }

    geometry_msgs::Point tf_translation_ = tf2::toMsg(dr::averagePositions<double>(eigen_translations_) );
    tf_msg_avg_.transform.translation.x = tf_translation_.x;
    tf_msg_avg_.transform.translation.y = tf_translation_.y;
    tf_msg_avg_.transform.translation.z = tf_translation_.z;
    tf_msg_avg_.transform.rotation = tf2::toMsg(dr::averageQuaternions<double>(eigen_rotations_) );
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

    GenerateRandomPose(pose_msg_.pose);
    pose_msg_.header.stamp = ros::Time::now();
    bag_.write("tool0_desired", ros::Time::now(), pose_msg_);

    ROS_INFO_STREAM("0/" << N << std::endl);
    MoveRobot(pose_msg_);

    std::string pError;
    if (tf_buffer_.canTransform(tool_frame, base_frame, ros::Time(0), &pError) &&
        tf_buffer_.canTransform(controller_frame, vr_frame, ros::Time(0), &pError) )
    {
        // Lookup and convert necessary transforms from msgs
        tf_msg_tool0_ = tf_buffer_.lookupTransform(base_frame, ros::Time(0), tool_frame, ros::Time(0), base_frame);
        // tf_msg_sensor_ = tf_buffer_.lookupTransform(vr_frame, ros::Time(0), controller_frame, ros::Time(0), vr_frame);
        SampleSensor(vr_frame, controller_frame, 120, 30, tf_msg_sensor_);
        bag_.write("tool0", ros::Time::now(), tf_msg_tool0_);
        bag_.write("sensor", ros::Time::now(), tf_msg_sensor_);

        tf2::convert(tf_msg_tool0_.transform, tf_tool0_[0]);
        tf2::convert(tf_msg_sensor_.transform, tf_sensor_[0]);

        for (int i = 1; i <= N; i++) {
            GenerateRandomPose(pose_msg_.pose);
            pose_msg_.header.stamp = ros::Time::now();
            bag_.write("tool0_desired", ros::Time::now(), pose_msg_);

            ROS_INFO_STREAM(i << "/" << N << std::endl);
            if (MoveRobot(pose_msg_) ) {
                if (tf_buffer_.canTransform(tool_frame, base_frame, ros::Time(0), &pError) &&
                    tf_buffer_.canTransform(controller_frame, vr_frame, ros::Time(0), &pError) )
                {
                    // Lookup and convert necessary transforms
                    tf_msg_tool0_ = tf_buffer_.lookupTransform(base_frame, ros::Time(0), tool_frame, ros::Time(0), base_frame);
                    // tf_msg_sensor_ = tf_buffer_.lookupTransform(vr_frame, ros::Time(0), controller_frame, ros::Time(0), vr_frame);
                    SampleSensor(vr_frame, controller_frame, 120, 30, tf_msg_sensor_);
                    bag_.write("tool0", ros::Time::now(), tf_msg_tool0_);
                    bag_.write("sensor", ros::Time::now(), tf_msg_sensor_);

                    tf2::convert(tf_msg_tool0_.transform, tf_tool0_[1]);
                    tf2::convert(tf_msg_sensor_.transform, tf_sensor_[1]);

                    // Compute transforms between poses
                    tf2::convert(tf_tool0_[0].inverseTimes(tf_tool0_[1]), tf_msg_A_.transform);
                    tf2::convert(tf_sensor_[0].inverseTimes(tf_sensor_[1]), tf_msg_B_.transform);
                    bag_.write("A", ros::Time::now(), tf_msg_A_);
                    bag_.write("B", ros::Time::now(), tf_msg_B_);

                    // Sample transformation pair (A, B)
                    vive_calibrating::AddSample sample_srv;
                    sample_srv.request.A = tf_msg_A_;
                    sample_srv.request.B = tf_msg_B_;
                    if (sample_client.call(sample_srv) ) {
                        ROS_INFO_STREAM("Sampled " << sample_srv.response.n << " pairs (A, B)");
                    }

                    // Set the current pose as a reference for next pose
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

    if (CalibrateViveNode() ) {

    } else {
        ROS_ERROR("Failed solving the provided AX=XB problem");
    }

    bag_.close();
}

bool CalibratingNode::CalibrateViveNode() {
      /**
     * Calibrate VIVE node by calling the ParkMartin compute service,
     * and update the VIVE node parameters using dynamic reconfigure
     */

    vive_calibrating::ComputeCalibration compute_srv;

    if (compute_client.call(compute_srv) ) {
        if (compute_srv.response.success) {
            geometry_msgs::TransformStamped tf_msg_Tx_ = compute_srv.response.X;
            tf_msg_Tx_.header.frame_id = tool_frame;
            tf_msg_Tx_.child_frame_id = controller_frame;
            tf_msg_Tx_.header.stamp = ros::Time::now();
            bag_.write("X", ros::Time::now(), tf_msg_Tx_);
            ROS_INFO_STREAM("T_X =" << std::endl << tf_msg_Tx_);

            tf2::fromMsg(tf_msg_Tx_.transform, tf_X_);

            // Lookup transformation from VIVE Tracker to world_vr
            tf_msg_ = tf_buffer_.lookupTransform(world_frame, ros::Time(0), tool_frame, ros::Time(0), world_frame);
            tf2::fromMsg(tf_msg_.transform, tf_tool0_[1]);

            tf_msg_ = tf_buffer_.lookupTransform(controller_frame, vr_frame, ros::Time(0) );
            ROS_INFO_STREAM(tf_msg_);
            SampleSensor(controller_frame, vr_frame, 120, 30, tf_msg_);
            ROS_INFO_STREAM(tf_msg_);
            
            tf2::fromMsg(tf_msg_.transform, tf_controller_);

            tf_controller_ = tf_tool0_[1]*tf_X_*tf_controller_;

            // Set new offset parameters based on transformation
            srv_reconf_req_.config.doubles[0].value = tf_controller_.getOrigin().getX();
            srv_reconf_req_.config.doubles[1].value = tf_controller_.getOrigin().getY();
            srv_reconf_req_.config.doubles[2].value = tf_controller_.getOrigin().getZ();
            tf_controller_.getBasis().getRPY(roll_offset, pitch_offset, yaw_offset);
            srv_reconf_req_.config.doubles[3].value = yaw_offset;
            srv_reconf_req_.config.doubles[4].value = pitch_offset;
            srv_reconf_req_.config.doubles[5].value = roll_offset;
            // Send request to the dynamic reconfigure service in vive_node
            ros::service::call("/vive_node/set_parameters", srv_reconf_req_, srv_reconf_resp_);

            return true;
        } else {
            return false;
        }

        return false;
    }
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

    vive_calibrating::AddSample sample_srv;
    sample_srv.request.A.transform = tf_msg_A_[0];
    sample_srv.request.B.transform = tf_msg_B_[0];
    if (sample_client.call(sample_srv) ) {
        ROS_INFO_STREAM("Sampled " << sample_srv.response.n << " pairs (A, B)");
    }
    sample_srv.request.A.transform = tf_msg_A_[1];
    sample_srv.request.B.transform = tf_msg_B_[1];
    if (sample_client.call(sample_srv) ) {
        ROS_INFO_STREAM("Sampled " << sample_srv.response.n << " pairs (A, B)");
    }

    vive_calibrating::ComputeCalibration compute_srv;
    if (compute_client.call(compute_srv) ) {
        if (compute_srv.response.success) {
            geometry_msgs::TransformStamped tf_msg_Tx_ = compute_srv.response.X;
            ROS_INFO_STREAM(tf_msg_Tx_);
        } else {
            ROS_ERROR("Failed solving the provided AX=XB problem");
        }
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "vive_robot_calibrating_node");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    CalibratingNode node_(120);

    // Handle signal [ctrl + c]
    signal(SIGINT, IntHandler);

    if (!node_.Init() ) {
        node_.Shutdown();
        exit(EXIT_FAILURE);
    }

    node_.Shutdown();
    exit(EXIT_SUCCESS);
}