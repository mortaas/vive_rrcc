#include <signal.h>

// ROS
#include <ros/ros.h>
// ROS msgs
#include <vive_bridge/TrackedDevicesStamped.h>
#include <sensor_msgs/JoyFeedback.h>

// MoveIt!
// #include <moveit_msgs/ApplyPlanningScene.h>
// #include <moveit_msgs/PlanningScene.h>

// tf2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// Eigen
#include "tf2_eigen/tf2_eigen.h"
#include <Eigen/Dense>

// RViz
#include <rviz_visual_tools/rviz_visual_tools.h>

// SDFormat
#include <sdf/sdf.hh>

// Ceres NLS solver solver
#include <ceres/ceres.h>
#include "ceres_cost_functors.h"

// Sophus - C++ implementation of Lie Groups using Eigen
#include <sophus/se3.hpp>

#include "sophus_ros_conversions/eigen.hpp"
#include "sophus_ros_conversions/geometry.hpp"


// Handle signal [ctrl + c]
bool sigint_flag = true;

void IntHandler(int signal) {
    sigint_flag = false;
}


enum E_DefineStates {
    STATE_DEFINE_POINT = 0,
    STATE_DEFINE_LINE,
    STATE_DEFINE_PLANE,
    STATE_DEFINE_BOX,
    STATE_DEFINE_BOX_POINT,
    STATE_DEFINE_BOX_LINE,
    STATE_DEFINE_BOX_PLANE,
    STATE_DEFINE_BOX_HEIGHT,
    STATE_DEFINE_SPHERE,
    STATE_DEFINE_CYLINDER,
    STATE_DEFINE_CYLINDER_BOTTOM,
    STATE_DEFINE_CYLINDER_TOP,
    STATE_DEFINE_CONE,
    STATE_DEFINE_CONE_HEIGHT
};

class SceneNode {
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;

    // Publishers
    ros::Publisher joy_feedback_pub_;

    // Subscribers
    ros::Subscriber devices_sub_;
    ros::Subscriber joy_sub_;

    void DevicesCb(const vive_bridge::TrackedDevicesStamped& msg_);
    void JoyCb(const sensor_msgs::Joy& msg_);

    // ROS msgs
    geometry_msgs::TransformStamped tf_msg_;
    sensor_msgs::JoyFeedback joy_feedback_msg_;

    // ros::ServiceClient planning_scene_client_;
    // moveit_msgs::ApplyPlanningScene planning_scene_srv_;
    // moveit_msgs::PlanningScene planning_scene_;

    // RViz
    rviz_visual_tools::RvizVisualToolsPtr rviz_tools_, rviz_infinite_tools_;

    // SDFormat
    sdf::SDFPtr sdf_;

    // tf2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener *tf_listener_;

    tf2::Transform tf_tracker_;
    std::string controller_frame, tool_frame, tracker_frame, world_frame;
    unsigned int controller_id, tracker_id;

    // Eigen
    Eigen::Affine3d eigen_msg_, eigen_pose_, eigen_controller_offset_;

    std::vector<Eigen::Vector3d> points_;
    Eigen::Vector3d vecs_[3], basis_[3], projected_point_;
    Eigen::Vector3d eigen_a_, eigen_b_, eigen_point_;
    double angle, height, radius, scale;

    unsigned int state;

    public:
        SceneNode(int frequency);
        ~SceneNode();

        bool Init();
        void Loop();
        void Shutdown();
};

void SceneNode::DevicesCb(const vive_bridge::TrackedDevicesStamped& msg_) {
    /**
     * Update information about the currently tracked devices.
     */

    for (int i = 0; i < msg_.device_count; i++) {
        if (msg_.device_classes[i] == msg_.CONTROLLER) {
            controller_frame = msg_.device_frames[i];
            controller_id = i;
        }
        if (msg_.device_classes[i] == msg_.TRACKER) {
            tracker_frame = msg_.device_frames[i];
            tracker_id = i;
        }
    }
}

SceneNode::SceneNode(int frequency)
    : loop_rate_(frequency),
      tf_listener_(new tf2_ros::TransformListener(tf_buffer_) ),
      sdf_(new sdf::SDF() )
{
    // Publishers
    joy_feedback_pub_ = nh_.advertise<sensor_msgs::JoyFeedback>("/vive_node/joy/haptic_feedback", 10, true);
    // Subscribers
    devices_sub_ = nh_.subscribe("/vive_node/tracked_devices", 1, &SceneNode::DevicesCb, this);

    // planning_scene_client_ = nh_.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    // planning_scene_client_.waitForExistence();

    // RViz
    rviz_tools_.reset(new rviz_visual_tools::RvizVisualTools(world_frame, "/rviz_visual_markers") );
    rviz_infinite_tools_.reset(new rviz_visual_tools::RvizVisualTools(world_frame, "/rviz_infinite_visual_markers") );

    // SDFormat
    sdf::init(sdf_);

    sdf::ElementPtr sdf_root_ = sdf_->Root();

    sdf::ElementPtr sdf_model_ = sdf_root_->AddElement("model");
    sdf_model_->GetAttribute("name")->Set("box");

    sdf::ElementPtr sdf_pose_ = sdf_model_->AddElement("pose");
    // sdf_pose_->GetAttribute("frame")->Set("box_frame");

    sdf::ElementPtr sdf_link_ = sdf_model_->AddElement("link");
    sdf_link_->GetAttribute("name")->Set("link");

    sdf::ElementPtr sdf_collision_ = sdf_link_->AddElement("collision");
    sdf_collision_->GetAttribute("name")->Set("collision");

    sdf::ElementPtr sdf_visual_ = sdf_link_->AddElement("visual");
    sdf_visual_->GetAttribute("name")->Set("visual");

    sdf::ElementPtr sdf_collision_geometry_ = sdf_collision_->GetElement("geometry");
    sdf::ElementPtr sdf_collision_box_ = sdf_collision_geometry_->AddElement("box");

    sdf::ElementPtr sdf_visual_geometry_ = sdf_visual_->GetElement("geometry");
    sdf::ElementPtr sdf_visual_box_ = sdf_visual_geometry_->AddElement("box");

    sdf_->Write("test.sdf");

    // Define joy feedback message
    joy_feedback_msg_.type = joy_feedback_msg_.TYPE_RUMBLE;
    joy_feedback_msg_.intensity = .1;

    // Controller offset transform (Eigen)
    double angle_offset = -std::atan((0.027553 - 0.00985)/(0.025907 + 0.002273) );
    eigen_controller_offset_.setIdentity();
    eigen_controller_offset_.translate(Eigen::Vector3d(0., -0.025907, -0.027553) );
    eigen_controller_offset_.rotate(Eigen::Quaterniond(0., 0., 1., 0.) *
                                    Eigen::Quaterniond(std::sin(angle_offset/2), 0., 0., std::cos(angle_offset/2) ) );

    // Set default start state
    state = STATE_DEFINE_BOX_POINT;
}
SceneNode::~SceneNode() {
}

void SceneNode::JoyCb(const sensor_msgs::Joy& msg_) {
      /**
     * Handle VIVE Controller inputs
     */

    if (msg_.axes[0] + msg_.axes[1] + msg_.axes[2] == 0.) {
        if (msg_.buttons[1]) { // Grip button
            points_.push_back(eigen_point_);
            ROS_INFO_STREAM("Point " << points_.size() << ": \n" << eigen_point_.matrix() );

            // Trigger controller haptic feedback
            joy_feedback_pub_.publish(joy_feedback_msg_);

            switch(state) {
                // BOX
                case STATE_DEFINE_BOX_HEIGHT: {
                    rviz_infinite_tools_->publishCuboid(eigen_pose_, vecs_[0].norm(), vecs_[1].norm(), std::abs(height), rviz_visual_tools::BLUE);
                    rviz_infinite_tools_->trigger();
                    
                    state = STATE_DEFINE_BOX_POINT;
                    points_.clear();
                    break;
                }
                case STATE_DEFINE_BOX_PLANE: {
                    state = STATE_DEFINE_BOX_HEIGHT;
                    break;
                }
                case STATE_DEFINE_BOX_LINE: {
                    vecs_[0] = points_[1] - points_[0];
                    basis_[0] = vecs_[0] / vecs_[0].norm();

                    state = STATE_DEFINE_BOX_PLANE;
                    break;
                }
                case STATE_DEFINE_BOX_POINT: {
                    state = STATE_DEFINE_BOX_LINE;
                    break;
                }
                // CYLINDER
                case STATE_DEFINE_CYLINDER_TOP: {
                    eigen_b_ = vecs_[2];

                    rviz_infinite_tools_->publishCylinder(eigen_a_, eigen_b_, rviz_visual_tools::BLUE, 2*radius);
                    rviz_infinite_tools_->trigger();
                    points_.clear();

                    state = STATE_DEFINE_CYLINDER;
                    break;
                }
                case STATE_DEFINE_CYLINDER_BOTTOM: {
                    eigen_a_ = vecs_[1];

                    state = STATE_DEFINE_CYLINDER_TOP;
                    break;
                }
                // CONE
                case STATE_DEFINE_CONE_HEIGHT: {
                    rviz_infinite_tools_->publishCone(eigen_pose_, angle, rviz_visual_tools::BLUE, scale);
                    rviz_infinite_tools_->trigger();
                    points_.clear();

                    state = STATE_DEFINE_CONE;
                    break;
                }
            }
        }
    }

    if (msg_.buttons[3]) { // Trigger button
        joy_feedback_pub_.publish(joy_feedback_msg_);

        // points_.push_back(eigen_point_);
        // ROS_INFO_STREAM("Point " << points_.size() << ": \n" << eigen_point_.matrix() );
    }

    if (msg_.buttons[2]) { // Touchpad button
        // CONE
        if (msg_.buttons[5]) { // Touchpad down
            // Trigger controller haptic feedback
            joy_feedback_pub_.publish(joy_feedback_msg_);

            if (state != STATE_DEFINE_CONE &&
                state != STATE_DEFINE_CONE_HEIGHT) {
                state = STATE_DEFINE_CONE;
                points_.clear();
            } else {
                if (points_.size() >= 6) {
                    // Ceres NLS solver
                    ceres::Problem ceres_problem;
                    ceres::Solver::Options ceres_options;
                    ceres::Solver::Summary ceres_summary;

                    // Compute centroid of point set
                    eigen_a_ = Eigen::Vector3d::Zero();
                    eigen_b_ = Eigen::Vector3d::Zero();
                    for (std::vector<Eigen::Vector3d>::iterator it_ = points_.begin(); it_ != points_.end(); ++it_) {
                        eigen_b_ += *it_;
                    }
                    eigen_b_ /= points_.size();

                    ROS_INFO_STREAM(eigen_b_);

                    // Solve line NLS problem
                    {
                        ceres::Problem ceres_problem;
                        ceres::Solver::Options ceres_options;
                        ceres::Solver::Summary ceres_summary;

                        for (std::vector<Eigen::Vector3d>::iterator it_ = points_.begin(); it_ != points_.end(); ++it_) {
                            ceres::CostFunction* cost_function =
                                new ceres::AutoDiffCostFunction<LineCostFunctor, 1, 3>
                                                            (new LineCostFunctor(eigen_b_, *it_) );
                            ceres_problem.AddResidualBlock(cost_function, NULL, eigen_a_.data() );
                        }

                        // Set solver options
                        ceres_options.linear_solver_type = ceres::DENSE_SCHUR;
                        ceres_options.minimizer_progress_to_stdout = true;

                        // Solve NLS problem
                        Solve(ceres_options, &ceres_problem, &ceres_summary);
                        ROS_INFO_STREAM(ceres_summary.BriefReport() << std::endl);
                    }

                    eigen_b_ += (eigen_b_ - eigen_a_);
                    ROS_INFO_STREAM(eigen_a_);
                    rviz_infinite_tools_->publishLine(eigen_a_, eigen_b_, rviz_visual_tools::RED, rviz_visual_tools::XLARGE);
                    rviz_infinite_tools_->trigger();

                    // Solver seed
                    angle = M_PI_4;

                    // Solve cone NLS problem
                    {
                        ceres::Problem ceres_problem;
                        ceres::Solver::Options ceres_options;
                        ceres::Solver::Summary ceres_summary;

                        for (std::vector<Eigen::Vector3d>::iterator it_ = points_.begin(); it_ != points_.end(); ++it_) {
                            ceres::CostFunction* cost_function =
                                new ceres::AutoDiffCostFunction<ConeCostFunctor, 1, 3, 3, 1>
                                                            (new ConeCostFunctor(*it_) );
                            ceres_problem.AddResidualBlock(cost_function, NULL, eigen_a_.data(), eigen_b_.data(), &angle);
                        }

                        // Set solver options
                        ceres_options.linear_solver_type = ceres::DENSE_SCHUR;
                        ceres_options.minimizer_progress_to_stdout = true;

                        // Solve NLS problem
                        Solve(ceres_options, &ceres_problem, &ceres_summary);
                        ROS_INFO_STREAM(ceres_summary.BriefReport() << std::endl);
                    }

                    vecs_[0] = eigen_a_ - eigen_b_;
                    basis_[0] = vecs_[0] / vecs_[0].norm();

                    state = STATE_DEFINE_CONE_HEIGHT;
                } // if (points_.size() >= 6)
            } // if (state != STATE_DEFINE_CONE)
        }

        // CYLINDER
        if (msg_.buttons[7]) { // Touchpad left
            // Trigger controller haptic feedback
            joy_feedback_pub_.publish(joy_feedback_msg_);

            if (state != STATE_DEFINE_CYLINDER &&
                state != STATE_DEFINE_CYLINDER_BOTTOM &&
                state != STATE_DEFINE_CYLINDER_TOP) {
                state = STATE_DEFINE_CYLINDER;
                points_.clear();
            } else {
                if (points_.size() >= 6) {
                    // Ceres NLS solver
                    ceres::Problem ceres_problem;
                    ceres::Solver::Options ceres_options;
                    ceres::Solver::Summary ceres_summary;

                    // Compute centroid of point set
                    eigen_a_ = Eigen::Vector3d::Zero();
                    eigen_b_ = Eigen::Vector3d::Zero();
                    for (std::vector<Eigen::Vector3d>::iterator it_ = points_.begin(); it_ != points_.end(); ++it_) {
                        eigen_b_ += *it_;
                    }
                    eigen_b_ /= points_.size();

                    ROS_INFO_STREAM(eigen_b_);

                    // Solve line NLS problem
                    {
                        ceres::Problem ceres_problem;
                        ceres::Solver::Options ceres_options;
                        ceres::Solver::Summary ceres_summary;

                        for (std::vector<Eigen::Vector3d>::iterator it_ = points_.begin(); it_ != points_.end(); ++it_) {
                            ceres::CostFunction* cost_function =
                                new ceres::AutoDiffCostFunction<LineCostFunctor, 1, 3>
                                                            (new LineCostFunctor(eigen_b_, *it_) );
                            ceres_problem.AddResidualBlock(cost_function, NULL, eigen_a_.data() );
                        }

                        // Set solver options
                        ceres_options.linear_solver_type = ceres::DENSE_SCHUR;
                        ceres_options.minimizer_progress_to_stdout = true;

                        // Solve NLS problem
                        Solve(ceres_options, &ceres_problem, &ceres_summary);
                        ROS_INFO_STREAM(ceres_summary.BriefReport() << std::endl);
                    }

                    eigen_b_ += (eigen_b_ - eigen_a_);
                    ROS_INFO_STREAM(eigen_a_);
                    rviz_infinite_tools_->publishLine(eigen_a_, eigen_b_, rviz_visual_tools::RED, rviz_visual_tools::XLARGE);
                    rviz_infinite_tools_->trigger();

                    // Solver seed
                    radius = 0.5;

                    // Residual blocks
                    for (std::vector<Eigen::Vector3d>::iterator it_ = points_.begin(); it_ != points_.end(); ++it_) {
                        ceres::CostFunction* cost_function =
                            new ceres::AutoDiffCostFunction<CylinderCostFunctor, 1, 3, 3, 1>
                                                        (new CylinderCostFunctor(*it_) );
                        ceres_problem.AddResidualBlock(cost_function, NULL, eigen_a_.data(), eigen_b_.data(), &radius);
                    }

                    // Set solver options
                    ceres_options.linear_solver_type = ceres::DENSE_SCHUR;
                    ceres_options.minimizer_progress_to_stdout = true;

                    // Solve NLS problem
                    Solve(ceres_options, &ceres_problem, &ceres_summary);
                    ROS_INFO_STREAM(ceres_summary.FullReport() << std::endl);

                    vecs_[0] = eigen_b_ - eigen_a_;
                    basis_[0] = vecs_[0] / vecs_[0].norm();

                    state = STATE_DEFINE_CYLINDER_BOTTOM;
                } // if (points_.size() >= 6)
            } // if (state != STATE_DEFINE_CYLINDER)
        } // if (msg_.buttons[7])

        // BOX
        if (msg_.buttons[9]) { // Touchpad right
            // Trigger controller haptic feedback
            joy_feedback_pub_.publish(joy_feedback_msg_);

            state = STATE_DEFINE_BOX_POINT;
            points_.clear();
        }
        if (msg_.buttons[11]) { // Touchpad up
            // Trigger controller haptic feedback
            joy_feedback_pub_.publish(joy_feedback_msg_);

            if (state != STATE_DEFINE_SPHERE) {
                state = STATE_DEFINE_SPHERE;
                points_.clear();
            } else {
                if (points_.size() >= 4) {
                    // Solver seed
                    Eigen::Matrix4d eigen_A_;
                    eigen_A_ << points_[0].transpose(), 1,
                                points_[1].transpose(), 1,
                                points_[2].transpose(), 1,
                                points_[3].transpose(), 1;

                    Eigen::Vector4d eigen_b_;
                    eigen_b_ << points_[0].squaredNorm(),
                                points_[1].squaredNorm(),
                                points_[2].squaredNorm(),
                                points_[3].squaredNorm();
                    
                    Eigen::Vector4d eigen_x_ = eigen_A_.fullPivHouseholderQr().solve(eigen_b_);
                    Eigen::Vector3d eigen_c_ = -0.5*eigen_x_.head<3>();
                    radius = 0.5*std::sqrt(eigen_c_.squaredNorm() - 4*eigen_x_(3) );

                    // Ceres NLS solver
                    ceres::Problem ceres_problem;
                    ceres::Solver::Options ceres_options;
                    ceres::Solver::Summary ceres_summary;

                    // Residual blocks
                    for (std::vector<Eigen::Vector3d>::iterator it_ = points_.begin(); it_ != points_.end(); ++it_) {
                        ceres::CostFunction* cost_function =
                            new ceres::AutoDiffCostFunction<SphereCostFunctor, 1, 3, 1>
                                                           (new SphereCostFunctor(*it_) );
                        ceres_problem.AddResidualBlock(cost_function, NULL, eigen_c_.data(), &radius);
                    }

                    // Set solver options
                    ceres_options.linear_solver_type = ceres::DENSE_SCHUR;
                    ceres_options.minimizer_progress_to_stdout = true;

                    // Solve NLS problem
                    Solve(ceres_options, &ceres_problem, &ceres_summary);
                    ROS_INFO_STREAM(ceres_summary.FullReport() << std::endl);

                    rviz_infinite_tools_->publishSphere(eigen_c_, rviz_visual_tools::BLUE, 2*radius);
                    rviz_infinite_tools_->trigger();

                    state = STATE_DEFINE_SPHERE;
                    points_.clear();
                } // if (points_.size() >= 4)
            } // if (state != STATE_DEFINE_SPHERE)
        } // if (msg_.buttons[11])
    } // if (msg_.buttons[2])
}

bool SceneNode::Init() {
      /**
     * Check if the necessary transforms are available and initialize the node
     */

    nh_.param<std::string>("/vive_node/world_frame", world_frame, "root");

    // Get available controller
    while (controller_frame.empty() && sigint_flag) {
        ROS_INFO("Waiting for controller...");
        
        ros::spinOnce();
        ros::Duration(3.0).sleep();
    }

    // Handle sigint
    if (!sigint_flag) {
        return false;
    }

    ROS_INFO_STREAM("Using " + controller_frame + " for calibration");

    // Subscribe to joy topic
    joy_sub_ = nh_.subscribe("/vive_node/joy/" + controller_frame, 1, &SceneNode::JoyCb, this);

    // Check if transforms are available
    std::string pError;
    if (!tf_buffer_.canTransform(controller_frame,
                                 world_frame, ros::Time(0),
                                 ros::Duration(10.0), &pError) )
    {
        ROS_ERROR_STREAM("Can't transform from " + world_frame + " to " + controller_frame + ": " + pError);

        return false;
    } else {
        tool_frame = controller_frame;
    }
    if (!tracker_frame.empty() ) {
        if (!tf_buffer_.canTransform(tracker_frame + "_tool0",
                                     world_frame, ros::Time(0),
                                     ros::Duration(0.), &pError) )
        {
            ROS_WARN_STREAM("Can't transform from " + world_frame + " to " + tracker_frame + "_tool0 : " + pError);
        } else {
            tool_frame = tracker_frame + "_tool0";
        }
    }

    joy_feedback_msg_.id = controller_id;

    // planning_scene_.is_diff = true;

    // Limited marker lifetime
    rviz_tools_->setLifetime(3./60.);
    rviz_tools_->setBaseFrame(world_frame);
    // Infinite marker lifetime
    rviz_infinite_tools_->setLifetime(0.);
    rviz_infinite_tools_->setBaseFrame(world_frame);
    
    return true;
}

void SceneNode::Loop() {
    // Get tracked device location from tf server
    tf_msg_ = tf_buffer_.lookupTransform(world_frame, tool_frame, ros::Time(0) );

    if (tool_frame == controller_frame) {
        eigen_msg_ = tf2::transformToEigen(tf_msg_) * eigen_controller_offset_;
    } else {
        eigen_msg_ = tf2::transformToEigen(tf_msg_);
    }
    eigen_point_ = eigen_msg_.translation();

    rviz_tools_->resetMarkerCounts();

    // Publish spheres at registered points
    for (std::vector<Eigen::Vector3d>::iterator it_ = points_.begin(); it_ != points_.end(); ++it_) {
        rviz_tools_->publishSphere(*it_, rviz_visual_tools::BLUE, rviz_visual_tools::XLARGE);
    }
    // Red sphere at current point
    rviz_tools_->publishSphere(eigen_point_, rviz_visual_tools::RED, rviz_visual_tools::XLARGE);

    // Visualize geometry in RViz
    switch(state) {
        // BOX
        case STATE_DEFINE_BOX_LINE: {
            rviz_tools_->publishLine(points_[0], eigen_point_, rviz_visual_tools::RED, rviz_visual_tools::XLARGE);

            break;
        }
        case STATE_DEFINE_BOX_PLANE: {
            // Project tracked device position to a line defined by the last two points
            projected_point_ = points_[0] + (eigen_point_ - points_[0]).dot(basis_[0]) * basis_[0];
            vecs_[1] = eigen_point_ - projected_point_;
            basis_[1] = vecs_[1] / vecs_[1].norm();

            vecs_[2] = basis_[0].cross(basis_[1]);
            basis_[2] = vecs_[2] / vecs_[2].norm();

            eigen_pose_.matrix().block<3, 1>(0, 0) = basis_[0];
            eigen_pose_.matrix().block<3, 1>(0, 1) = basis_[1];
            eigen_pose_.matrix().block<3, 1>(0, 2) = basis_[2];
            eigen_pose_.matrix().block<3, 1>(0, 3) = points_[0] + 0.5*(vecs_[0] + vecs_[1]);

            rviz_tools_->publishLine(points_[0],   points_[1], rviz_visual_tools::BLUE, rviz_visual_tools::XLARGE);
            rviz_tools_->publishLine(points_[0], eigen_point_, rviz_visual_tools::BLUE, rviz_visual_tools::XLARGE);
            rviz_tools_->publishLine(points_[0], points_[0] + vecs_[1], rviz_visual_tools::RED , rviz_visual_tools::XLARGE);
            
            rviz_tools_->publishAxis(eigen_pose_, rviz_visual_tools::XLARGE);
            rviz_tools_->publishCuboid(eigen_pose_, vecs_[0].norm(), vecs_[1].norm(), 0.01, rviz_visual_tools::BLUE);

            break;
        }
        case STATE_DEFINE_BOX_HEIGHT: {
            height = (eigen_point_ - points_[0]).dot(basis_[2]);
            eigen_pose_.matrix().block<3, 1>(0, 3) = points_[0] + 0.5*(vecs_[0] + vecs_[1] + height*basis_[2]);

            rviz_tools_->publishLine(points_[0], points_[1], rviz_visual_tools::BLUE, rviz_visual_tools::XLARGE);
            rviz_tools_->publishLine(points_[0], points_[0] + vecs_[1], rviz_visual_tools::BLUE, rviz_visual_tools::XLARGE);

            rviz_tools_->publishLine(points_[0], points_[0] + height*basis_[2], rviz_visual_tools::RED, rviz_visual_tools::XLARGE);
            rviz_tools_->publishCuboid(eigen_pose_, vecs_[0].norm(), vecs_[1].norm(), std::abs(height), rviz_visual_tools::BLUE);

            break;
        }

        // CYLINDER
        case STATE_DEFINE_CYLINDER_BOTTOM: {
            vecs_[1] = eigen_a_ + (eigen_point_ - eigen_a_).dot(basis_[0]) * basis_[0];

            rviz_tools_->publishCylinder(vecs_[1], eigen_b_, rviz_visual_tools::BLUE, 2*radius);
            break;
        }
        case STATE_DEFINE_CYLINDER_TOP: {
            vecs_[2] = eigen_b_ + (eigen_point_ - eigen_b_).dot(basis_[0]) * basis_[0];

            rviz_tools_->publishCylinder(eigen_a_, vecs_[2], rviz_visual_tools::BLUE, 2*radius);
            break;
        }

        // CONE
        case STATE_DEFINE_CONE_HEIGHT: {
            scale = (eigen_point_ - eigen_b_).dot(basis_[0]);
            vecs_[1] = eigen_b_ + scale * basis_[0];
            vecs_[2] = eigen_point_ - vecs_[1];
            basis_[1] = vecs_[2] / vecs_[2].norm();
            basis_[2] = basis_[0].cross(basis_[1]);

            eigen_pose_.matrix().block<3, 1>(0, 0) = basis_[0] / basis_[0].norm();
            eigen_pose_.matrix().block<3, 1>(0, 1) = basis_[1] / basis_[1].norm();
            eigen_pose_.matrix().block<3, 1>(0, 2) = basis_[2] / basis_[2].norm();
            eigen_pose_.matrix().block<3, 1>(0, 3) = eigen_b_; // - 0.5*scale * basis_[0];

            rviz_tools_->publishCone(eigen_pose_, angle, rviz_visual_tools::BLUE, scale);
            break;
        }
    }

    rviz_tools_->trigger();

    ros::spinOnce();
    loop_rate_.sleep();
}
void SceneNode::Shutdown() {
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "vive_scene_node");

    // Handle signal [ctrl + c]
    signal(SIGINT, IntHandler);

    SceneNode node_(60);

    if (!node_.Init() ) {
        node_.Shutdown();

        // Handle sigint
        if (sigint_flag) {
            exit(EXIT_SUCCESS);
        } else {
            exit(EXIT_FAILURE);
        }
    }

    while (ros::ok() && sigint_flag) {
        node_.Loop();
    }

    node_.Shutdown();
    exit(EXIT_SUCCESS);
}