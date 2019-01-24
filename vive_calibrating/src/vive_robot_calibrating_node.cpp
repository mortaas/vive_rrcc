#include "vive_robot_calibrating_node.h"
#include "moveit_msgs/Constraints.h"

CalibratingNode::CalibratingNode(int frequency)
    : loop_rate_(frequency),
      tf_listener_(new tf2_ros::TransformListener(tf_buffer_) ),
      // MoveIt!
      move_group_(moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP) ),
    //   robot_model_loader_(robot_model_loader::RobotModelLoader("/robot_description") ),
    //   kinematic_model_(robot_model_loader_.getModel() ),
    //   kinematic_state_(new robot_state::RobotState(kinematic_model_) ),
    //   joint_model_group_(kinematic_model_->getJointModelGroup("floor_manipulator") ),
    //   action_client_(new actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>
    //                 ("/floor/joint_trajectory_action", true) ),
      // RNG
      rng1(random_seed1() ),
      rng2(random_seed2() ),
      r_dist(1.2, 1.4),
      theta_dist1(5.5 * M_PI_4, 6.5 * M_PI_4),
      phi_dist1(0.5 * M_PI_4, 1.5 * M_PI_4),
      theta_dist2(5.5 * M_PI_4, 6.5 * M_PI_4),
      phi_dist2(1.5 * M_PI_4, 2.5 * M_PI_4)
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
    move_group_.setPoseReferenceFrame("floor_base");
    // move_group_.setMaxVelocityScalingFactor(0.5);

    move_group_.clearPathConstraints();

    moveit_msgs::Constraints cm_;

    cm_.joint_constraints.push_back(moveit_msgs::JointConstraint() );
    cm_.joint_constraints[0].joint_name = "floor_joint_a1";
    cm_.joint_constraints[0].position = M_PI_2;
    cm_.joint_constraints[0].tolerance_above = M_PI_2;
    cm_.joint_constraints[0].tolerance_below = M_PI_2;
    cm_.joint_constraints[0].weight = 1.;

    // cm_.joint_constraints.push_back(moveit_msgs::JointConstraint() );
    // cm_.joint_constraints[1].joint_name = "floor_joint_a3";
    // cm_.joint_constraints[1].position = M_PI_2;
    // cm_.joint_constraints[1].tolerance_above = M_PI_2;
    // cm_.joint_constraints[1].tolerance_below = M_PI_2;
    // cm_.joint_constraints[1].weight = 1.;

    cm_.joint_constraints.push_back(moveit_msgs::JointConstraint() );
    cm_.joint_constraints[1].joint_name = "floor_joint_a6";
    cm_.joint_constraints[1].position = 0.;
    cm_.joint_constraints[1].tolerance_above = M_PI_2;
    cm_.joint_constraints[1].tolerance_below = M_PI_2;
    cm_.joint_constraints[1].weight = 1.;

    move_group_.setPathConstraints(cm_);
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

    if (!InitParams() ) {
        while (controller_frame.empty() ) {
            ROS_INFO("Waiting for available VIVE controller..");
            
            ros::spinOnce();
            ros::Duration(5.0).sleep();
        }
    }
    ROS_INFO_STREAM("Using " + controller_frame + " for calibration");

    // joy_sub_ = nh_.subscribe("/vive_node/joy/" + controller_frame, 1, &CalibratingNode::JoyCb, this);

    std::string pError;
    if (!tf_buffer_.canTransform("world_vr", controller_frame, ros::Time(0),
                                 ros::Duration(5.0), &pError) )
    {
        ROS_ERROR_STREAM("Can't transform from world_vr to " + controller_frame + ": " + pError);

        return false;
    }

    // Wait for action server
    // ROS_INFO("Waiting for FollowJointTrajectory action server...");
    // action_client_->waitForServer();
    // ROS_INFO("FollowJointTrajectory action server ready!");

    // Initialize test transform
    tf_X_.setOrigin(tf2::Vector3(2., -1., 0.5) );
    tf_X_.setRotation(tf2::Quaternion(0., 0., std::sin(M_PI_4), std::cos(M_PI_4) ) );

    // Initialize transform frame ids
    tf_msg_pose_.header.frame_id = "floor_base";
    tf_msg_pose_.child_frame_id = "desired_pose";
    pose_msg_.header.frame_id = "floor_base";

    // Initialize FollowJointTrajectoryGoal message
    // const std::vector<std::string>& joint_names = joint_model_group_->getVariableNames();
    // traj_goal_msg_.trajectory.joint_names = joint_names;
    // traj_goal_msg_.trajectory.points.push_back(trajectory_msgs::JointTrajectoryPoint() );
    // traj_goal_msg_.trajectory.points[0].velocities = {0., 0., 0., 0., 0., 0.};
    // traj_goal_msg_.trajectory.points[0].accelerations = {0., 0., 0., 0., 0., 0.};
    // traj_goal_msg_.trajectory.points[0].time_from_start = ros::Duration(10.);

    joint_folded = {1.5707893454778887, -2.5900040327772613, 2.3999184786133068, -2.6179938779914945e-05, 0.799936756066561, 8.377580409572782e-05};

    MeasureRobot(100);
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
    SphereNormalPose(r_dist(rng1), theta_dist1(rng1), phi_dist1(rng1), pose_pos_ );
    SphereNormalPose(r_dist(rng2), theta_dist2(rng2), phi_dist2(rng2), pose_ );
    
    // Set position from the first random pose
    pose_.position = pose_pos_.position;

    return pose_;
}


// void GenerateJointTrajectorySine(trajectory_msgs::JointTrajectory traj_msg_, std::vector<double> positions0,
//                                  int joint, int sampling_frequency, double amplitude, double frequency, int periods)
// {
//     traj_msg_.points.resize(int(sampling_frequency/frequency*periods) );

//     for (int i = 0; i < sampling_frequency/frequency*periods; i++) {
//         traj_msg_.points[i].positions = positions0;
//         traj_msg_.points[i].positions[joint] += std::sin(frequency/10*i);

//         traj_msg_.points[i].velocities = {0., 0., 0., 0., 0., 0.};
//         traj_msg_.points[i].velocities[joint] += frequency*std::cos(frequency/10*i);

//         traj_msg_.points[i].accelerations = {0., 0., 0., 0., 0., 0.};
//         traj_msg_.points[i].accelerations[joint] += -frequency*frequency*std::sin(frequency/10*i);
//     }
// }

// bool CalibratingNode::GetJointPositionsFromIK(const geometry_msgs::PoseStamped &pose_, std::vector<double> joint_values) {
//     geometry_msgs::TransformStamped tf_root_ = tf_buffer_.lookupTransform("root", pose_.header.frame_id, ros::Time(0) );
//     geometry_msgs::PoseStamped pose_root_;
//     tf2::doTransform(pose_, pose_root_, tf_root_);

    
// }


bool CalibratingNode::MoveRobot(const geometry_msgs::PoseStamped &pose_) {
     /**
      * Move robot to provided pose, stop, and wait for the dynamics to settle down.
      * Returns true if trajectory execution succeeded.
      */
    
    move_group_.setJointValueTarget(joint_folded);

    // move_group_.setPoseTarget(pose_);

    if (move_group_.move() ) {
        ROS_INFO_STREAM("Trajectory execution succeeded");

        move_group_.stop();
        ros::Duration(0.5).sleep();
        // return true;
    } else {
        ROS_WARN_STREAM("Trajectory execution failed with pose:" << std::endl
                                                                << pose_.pose);

        return false;
    }

    // traj_goal_msg_.trajectory.points[0].positions = joint_folded;
    // action_client_->sendGoalAndWait(traj_goal_msg_);

    // geometry_msgs::TransformStamped tf_root_ = tf_buffer_.lookupTransform("root", pose_.header.frame_id, ros::Time(0) );
    // geometry_msgs::PoseStamped pose_root_;
    // tf2::doTransform(pose_, pose_root_, tf_root_);
    
    // if (kinematic_state_->setFromIK(joint_model_group_, pose_root_.pose, 10, 0.1) ) {
    //     kinematic_state_->copyJointGroupPositions(joint_model_group_, joint_values);

    //     traj_goal_msg_.trajectory.header.frame_id = pose_.header.frame_id;
    //     traj_goal_msg_.trajectory.points[0].positions = joint_values;

    //     action_client_->sendGoalAndWait(traj_goal_msg_);

    //     // traj_pub_.publish(traj_goal_msg_.trajectory);
    //     // ros::Duration(5.).sleep();
    // } else {
    //     ROS_INFO("Unable to find a IK solution");
    // }

    move_group_.setPoseTarget(pose_);

    if (move_group_.move() ) {
        ROS_INFO_STREAM("Trajectory execution succeeded");

        move_group_.stop();
        ros::Duration(10.).sleep();
        return true;
    } else {
        ROS_WARN_STREAM("Trajectory execution failed with pose:" << std::endl
                                                                << pose_.pose);

        return false;
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
        tf2::convert(tf_msg_sensor_.transform, tf_sensor_[0]);
        // tf_sensor_[0] = tf_tool0_[0]*tf_X_;

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
                if (tf_buffer_.canTransform("floor_tool0", "floor_base", ros::Time(0), &pError) &&
                    tf_buffer_.canTransform(controller_frame, "world_vr", ros::Time(0), &pError) )
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

                    // Sample pair (A, B)
                    vive_calibrating::AddSample sample_srv;
                    sample_srv.request.A = tf_msg_A_;
                    sample_srv.request.B = tf_msg_B_;
                    if (sample_client.call(sample_srv) ) {
                        ROS_INFO_STREAM("Sampled " << sample_srv.response.n << " pairs (A, B)");
                    }

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
            tf_msg_Tx_.header.frame_id = "floor_tool0";
            tf_msg_Tx_.child_frame_id = controller_frame;
            tf_msg_Tx_.header.stamp = ros::Time::now();
            bag_.write("X", ros::Time::now(), tf_msg_Tx_);
            ROS_INFO_STREAM(tf_msg_Tx_);

            tf2::fromMsg(tf_msg_Tx_.transform, tf_X_);

            // Lookup transformation from VIVE Tracker to world_vr
            tf_msg_ = tf_buffer_.lookupTransform("root", "floor_tool0", ros::Time(0) );
            tf2::fromMsg(tf_msg_.transform, tf_tool0_[1]);
            tf_msg_ = tf_buffer_.lookupTransform(controller_frame, "world_vr", ros::Time(0) );
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

    if (!node_.Init() ) {
        exit(EXIT_FAILURE);
    }

    // while (ros::ok() ) {
    //     node_.Loop();
    // }

    node_.Shutdown();
    exit(EXIT_SUCCESS);
}