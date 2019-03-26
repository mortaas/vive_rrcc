#include "robot_interface.h"

RobotInterface::RobotInterface(const std::string &PLANNING_GROUP,
                               const std::string &PLANNING_FRAME) :
    move_group_(moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP) ),
    kinematic_model_(move_group_.getRobotModel() ),
    kinematic_state_(move_group_.getCurrentState() )
{
    move_group_.setStartStateToCurrentState();

    // Set planning parameters of the MoveIt! move group
    move_group_.setPoseReferenceFrame(PLANNING_FRAME);
    // Transform from model frame to planning frame
    // eigen_model_planning_ = kinematic_state_->getFrameTransform(PLANNING_FRAME);
    eigen_model_planning_.setIdentity();
    eigen_model_planning_.rotate(Eigen::Quaterniond(0.5, -0.5, 0.5, -0.5) );

    // Initialize transform headers
    tf_msg_.header.frame_id = kinematic_model_->getModelFrame();
    tf_msg_.child_frame_id = "tool0_desired";


    // Get joint names of move group
    joint_names = move_group_.getJointNames();


    // MoveIt! constraints
    move_group_.clearPathConstraints();

    move_group_.setMaxAccelerationScalingFactor(0.1);
    move_group_.setMaxVelocityScalingFactor(0.1);

    // moveit_msgs::Constraints constraints_msg_;

    // constraints_msg_.joint_constraints.push_back(moveit_msgs::JointConstraint() );
    // constraints_msg_.joint_constraints.back().joint_name = "shoulder_lift_joint";
    // constraints_msg_.joint_constraints.back().position = -2/3*M_PI_2;
    // constraints_msg_.joint_constraints.back().tolerance_above = M_PI_2/3;
    // constraints_msg_.joint_constraints.back().tolerance_below = M_PI_2*2/3;
    // constraints_msg_.joint_constraints.back().weight = 1.;

    // constraints_msg_.joint_constraints.push_back(moveit_msgs::JointConstraint() );
    // constraints_msg_.joint_constraints.back().joint_name = "elbow_joint";
    // constraints_msg_.joint_constraints.back().position = 2/3*M_PI_2;
    // constraints_msg_.joint_constraints.back().tolerance_above = M_PI_2*2/3;
    // constraints_msg_.joint_constraints.back().tolerance_below = M_PI_2*2/3;
    // constraints_msg_.joint_constraints.back().weight = 1.;

    // constraints_msg_.joint_constraints.push_back(moveit_msgs::JointConstraint() );
    // constraints_msg_.joint_constraints.back().joint_name = "floor_joint_a5";
    // constraints_msg_.joint_constraints.back().position = 0.;
    // constraints_msg_.joint_constraints.back().tolerance_above = M_PI_2;
    // constraints_msg_.joint_constraints.back().tolerance_below = M_PI_2;
    // constraints_msg_.joint_constraints.back().weight = 1.;

    // constraints_msg_.joint_constraints.push_back(moveit_msgs::JointConstraint() );
    // constraints_msg_.joint_constraints.back().joint_name = "floor_joint_a6";
    // constraints_msg_.joint_constraints.back().position = 0.;
    // constraints_msg_.joint_constraints.back().tolerance_above = M_PI_2;
    // constraints_msg_.joint_constraints.back().tolerance_below = M_PI_2;
    // constraints_msg_.joint_constraints.back().weight = 1.;

    // move_group_.setPathConstraints(constraints_msg_);
}

void RobotInterface::SetOrientationConstraint(const geometry_msgs::Quaternion &quat_) {
    moveit_msgs::Constraints constraints_msg_;

    constraints_msg_.orientation_constraints.push_back(moveit_msgs::OrientationConstraint() );
    constraints_msg_.orientation_constraints.back().link_name = "tool0";
    constraints_msg_.orientation_constraints.back().orientation = quat_;
    constraints_msg_.orientation_constraints.back().absolute_x_axis_tolerance = 0.01;
    constraints_msg_.orientation_constraints.back().absolute_y_axis_tolerance = 0.01;
    constraints_msg_.orientation_constraints.back().absolute_z_axis_tolerance = 0.01;
    constraints_msg_.orientation_constraints.back().weight = 1.;
}

void RobotInterface::SetPoseTarget(const geometry_msgs::PoseStamped &pose_) {
    move_group_.setPoseTarget(pose_);

    // Convert pose to transform
    tf2::convert(pose_.pose, tf_pose_);
    tf2::convert(tf_pose_, tf_msg_.transform);

    std::string frame_id = tf_msg_.header.frame_id;
    tf_msg_.header.frame_id = pose_.header.frame_id;
    static_tf_broadcaster_.sendTransform(tf_msg_);
    tf_msg_.header.frame_id = frame_id;
}

void RobotInterface::SetJointValueTarget(const std::vector<double> &joint_state) {
    move_group_.setJointValueTarget(joint_state);

    // // Get joint state target as transform from FK
    // kinematic_state_->setVariablePositions(joint_names, joint_state);
    // eigen_FK_ = kinematic_state_->getGlobalLinkTransform(move_group_.getEndEffectorLink() );
    // geometry_msgs::TransformStamped tf_FK_ = tf2::eigenToTransform(eigen_FK_);
    // tf_msg_.transform = tf_FK_.transform;

    // static_tf_broadcaster_.sendTransform(tf_msg_);
}

void RobotInterface::GetCurrentPose(geometry_msgs::PoseStamped &pose_) {
    pose_ = move_group_.getCurrentPose();
}

void RobotInterface::SetStartStateToCurrentState() {
    move_group_.setStartStateToCurrentState();
    kinematic_state_->setVariablePositions(joint_names, move_group_.getCurrentJointValues() );
}

bool RobotInterface::GetPlan(moveit::planning_interface::MoveGroupInterface::Plan &plan_) {
     /**
      * Gets a MoveIt! plan from the provided joint state or pose goal.
      * These goals are set with the setPoseTarget(*) or setJointTarget(*) functions.
      */

    if (move_group_.plan(plan_) ) {
        // Set goal state for this plan as the start state for the next plan
        std::vector<double> position = plan_.trajectory_.joint_trajectory.points.back().positions;
        kinematic_state_->setVariablePositions(joint_names, position);
        move_group_.setStartState(*kinematic_state_);

        return true;
    } else {
        return false;
    }
}

bool RobotInterface::ExecutePlan(const moveit::planning_interface::MoveGroupInterface::Plan &plan_) {
     /**
      * Executes the provided plan, stop, and wait for the dynamics to settle down.
      * Returns true if trajectory execution succeeded.
      */

    // Get plan goal as pose
    std::vector<double> position = plan_.trajectory_.joint_trajectory.points.back().positions;
    kinematic_state_->setVariablePositions(joint_names, position);
    eigen_FK_ = kinematic_state_->getGlobalLinkTransform(move_group_.getEndEffectorLink() );
    // Visualize desired pose as a transform in RViz
    geometry_msgs::TransformStamped tf_FK_ = tf2::eigenToTransform(eigen_FK_*eigen_model_planning_);
    tf_msg_.transform = tf_FK_.transform;
    tf_msg_.header.stamp = ros::Time::now();
    static_tf_broadcaster_.sendTransform(tf_msg_);


    ROS_INFO_STREAM("Moving to pose:" << std::endl << tf_msg_);
    if (move_group_.execute(plan_) ) {
        ROS_INFO_STREAM("Trajectory execution succeeded");

        move_group_.stop();
        return true;
    } else {
        ROS_WARN_STREAM("Trajectory execution failed");

        return false;
    }
}

bool RobotInterface::MoveIt() {
     /**
      * Move robot, stop, and wait for the dynamics to settle down.
      * Returns true if trajectory execution succeeded.
      */

    move_group_.setStartStateToCurrentState();

    moveit::planning_interface::MoveGroupInterface::Plan plan_;
    ROS_INFO_STREAM("Moving to pose:" << std::endl << tf_msg_);
    if (GetPlan(plan_) ) {
        if (ExecutePlan(plan_) ) {
            ROS_INFO_STREAM("Trajectory execution succeeded");
            move_group_.setStartStateToCurrentState();

            return true;
        }
    } else {
        ROS_WARN_STREAM("Trajectory execution failed");

        return false;
    }

    // // Visualize desired pose as a transform in RViz
    // tf_msg_.header.stamp = ros::Time::now();
    // static_tf_broadcaster_.sendTransform(tf_msg_);

    // ROS_INFO_STREAM("Moving to pose:" << std::endl << tf_msg_);
    // if (move_group_.move() ) {
    //     ROS_INFO_STREAM("Trajectory execution succeeded");
    //     move_group_.setStartStateToCurrentState();

    //     move_group_.stop();
    //     return true;
    // } else {
    //     ROS_WARN_STREAM("Trajectory execution failed");

    //     return false;
    // }
}