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
    eigen_model_planning_ = kinematic_state_->getFrameTransform(PLANNING_FRAME);

    // Initialize transform headers
    tf_msg_.header.frame_id = kinematic_model_->getModelFrame();
    tf_msg_.child_frame_id = "tool0_desired";

    // Get joint names of move group
    joint_names = move_group_.getJointNames();
}

void RobotInterface::SetPoseTarget(const geometry_msgs::PoseStamped &pose_) {
    move_group_.setPoseTarget(pose_);

    // // Convert pose to transform
    // tf2::convert(pose_.pose, tf_pose_);
    // tf2::convert(tf_pose_, tf_msg_.transform);
}

void RobotInterface::SetJointValueTarget(const std::vector<double> &joint_state) {
    move_group_.setJointValueTarget(joint_state);

    // // Get joint state target as transform from FK
    // kinematic_state_->setVariablePositions(joint_names, joint_state);
    // eigen_FK_ = kinematic_state_->getGlobalLinkTransform(move_group_.getEndEffectorLink() );
    // geometry_msgs::TransformStamped tf_FK_ = tf2::eigenToTransform(eigen_FK_);
    // tf_msg_.transform = tf_FK_.transform;
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
    geometry_msgs::TransformStamped tf_FK_ = tf2::eigenToTransform(eigen_FK_);
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

    moveit::planning_interface::MoveGroupInterface::Plan plan_;
    ROS_INFO_STREAM("Moving to pose:" << std::endl << tf_msg_);
    if (GetPlan(plan_) ) {
        if (ExecutePlan(plan_) ) {
            ROS_INFO_STREAM("Trajectory execution succeeded");

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

    //     move_group_.stop();
    //     return true;
    // } else {
    //     ROS_WARN_STREAM("Trajectory execution failed");

    //     return false;
    // }
}