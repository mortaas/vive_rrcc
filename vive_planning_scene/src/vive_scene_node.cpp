#include <signal.h>

// ROS
#include <ros/ros.h>
// ROS msgs
#include <vive_bridge/TrackedDevicesStamped.h>

// MoveIt!
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/PlanningScene.h>

// tf2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// Eigen
#include "tf2_eigen/tf2_eigen.h"
#include <Eigen/Dense>

// RViz
#include <rviz_visual_tools/rviz_visual_tools.h>

// SDFormat
// #include <sdf/sdf.hh>


// Handle signal [ctrl + c]
bool sigint_flag = true;

void IntHandler(int signal) {
    sigint_flag = false;
}


class SceneNode {
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;

    // Subscribers
    ros::Subscriber devices_sub_;
    ros::Subscriber joy_sub_;

    void DevicesCb(const vive_bridge::TrackedDevicesStamped& msg_);
    void JoyCb(const sensor_msgs::Joy& msg_);

    // ROS msgs
    geometry_msgs::TransformStamped tf_msg_;

    // ros::ServiceClient planning_scene_client_;
    // moveit_msgs::ApplyPlanningScene planning_scene_srv_;
    // moveit_msgs::PlanningScene planning_scene_;

    rviz_visual_tools::RvizVisualToolsPtr rviz_tools_;

    // SDFormat
    // sdf::SDFPtr sdf_;

    // tf2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener *tf_listener_;

    tf2::Transform tf_tracker_;
    std::string controller_frame, world_frame;

    // Eigen
    Eigen::Vector3d points_[8], basis_[4];
    Eigen::Affine3d eigen_pose_, eigen_controller_offset_; //, eigen_eurobox_center_;
    
    int state;
    double height;

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
        }
        // if (msg_.device_classes[i] == msg_.TRACKER) {
        //     tracker_frame = msg_.device_frames[i];
        // }
    }
}

SceneNode::SceneNode(int frequency)
    : loop_rate_(frequency),
    tf_listener_(new tf2_ros::TransformListener(tf_buffer_) )
{
    // Subscribers
    devices_sub_ = nh_.subscribe("/vive_node/tracked_devices", 1, &SceneNode::DevicesCb, this);

    // planning_scene_client_ = nh_.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    // planning_scene_client_.waitForExistence();

    rviz_tools_.reset(new rviz_visual_tools::RvizVisualTools("world", "/rviz_visual_markers") );

    // SDFormat
    // sdf_root_ = sdf_->Root();

    double angle_offset = -std::atan((0.027553 - 0.00985)/(0.025907 + 0.002273) );

    eigen_controller_offset_.rotate(Eigen::Quaterniond(0., 0., 1., 0.) *
                                    Eigen::Quaterniond(std::sin(angle_offset/2), 0., 0., std::cos(angle_offset/2) ) );
    eigen_controller_offset_.translate(Eigen::Vector3d(0., -0.036594, 0.00955) );

    // Eigen
    // point1.setZero();
    // point2.setZero();
    // point3.setZero();

    // eigen_eurobox_center_.matrix().setIdentity();
    // eigen_eurobox_center_.matrix().block<3, 1>(0, 3) = Eigen::Vector3d(-0.189224, -0.139224, -0.074999);

    state = 0;
}
SceneNode::~SceneNode() {
}

void SceneNode::JoyCb(const sensor_msgs::Joy& msg_) {
      /**
     * Handle VIVE Controller inputs
     */

    if (msg_.buttons[1]) {
        switch (state) {
            case 0:
                ROS_INFO("Point 1!");
                ROS_INFO_STREAM(points_[0].matrix() );
                state = 1;
                break;
            case 1:
                ROS_INFO("Point 2!");
                ROS_INFO_STREAM(points_[1].matrix() );
                state = 2;

                basis_[0] = points_[1] - points_[0];
                break;
            case 2:
                ROS_INFO("Point 3!");
                ROS_INFO_STREAM(points_[2].matrix() );
                state = 3;
                break;
            case 3:
                ROS_INFO("Point 4!");
                ROS_INFO_STREAM(points_[3].matrix() );
                state = 4;
                break;
            case 4:
                ROS_INFO("Publish!");
                state = 5;
                break;
            case 5:
                state = 0;
                break;

                // // Compute basis vectors from points_
                // basis_[0] = (points_[0] - points_[1]);
                // basis_[0] = basis_[0] / basis_[0].norm();
                
                // basis_[1] = (points_[0] - points_[2]);
                // basis_[1] = basis_[1] / basis_[1].norm();

                // basis_[2] = basis_[0].cross(basis_[1]);

                // // Create pose from basis vectors (rotation) and first point (translation)
                // eigen_pose_.matrix().block<3, 1>(0, 0) = basis_[0];
                // eigen_pose_.matrix().block<3, 1>(0, 1) = basis_[1];
                // eigen_pose_.matrix().block<3, 1>(0, 2) = basis_[2];
                // eigen_pose_.matrix().block<3, 1>(0, 3) = point1;
                // ROS_INFO_STREAM(eigen_pose_.matrix() );

                // rviz_tools_->publishMesh(eigen_pose_ * eigen_eurobox_center_,
                //                            "package://clean_grasps/workcell_support/urdf/eurobox/meshes/eurobox.STL",
                //                            rviz_visual_tools::GREY,
                //                            1.0,
                //                            "Eurobox");
                // rviz_tools_->trigger();

                // Reset to first point
        }
    }
}

bool SceneNode::Init() {
      /**
     * Check if the necessary transforms are available and initialize the node
     */

    nh_.param<std::string>("/vive_node/world_frame", world_frame, "root");

    while (controller_frame.empty() ) {
        ROS_INFO("Waiting for controller...");
        
        ros::spinOnce();
        ros::Duration(5.0).sleep();
    }
    ROS_INFO_STREAM("Using " + controller_frame + " for calibration");

    joy_sub_ = nh_.subscribe("/vive_node/joy/" + controller_frame, 1, &SceneNode::JoyCb, this);

    std::string pError;
    if (!tf_buffer_.canTransform(controller_frame,
                                 world_frame, ros::Time(0),
                                 ros::Duration(10.0), &pError) )
    {
        ROS_ERROR_STREAM("Can't transform from " + world_frame + " to " + controller_frame + ": " + pError);

        return false;
    }

    // planning_scene_.is_diff = true;

    rviz_tools_->setLifetime(0.1);
    rviz_tools_->setBaseFrame(world_frame);
    
    return true;
}

void SceneNode::Loop() {
    tf_msg_ = tf_buffer_.lookupTransform(world_frame, controller_frame, ros::Time(0) );

    switch (state) {
        case 0:
            points_[0](0, 0) = tf_msg_.transform.translation.x;
            points_[0](1, 0) = tf_msg_.transform.translation.y;
            points_[0](2, 0) = tf_msg_.transform.translation.z;

            rviz_tools_->resetMarkerCounts();
            rviz_tools_->publishSphere(points_[0], rviz_visual_tools::BLUE, rviz_visual_tools::MEDIUM);
            rviz_tools_->trigger();
            break;
        case 1:
            points_[1](0, 0) = tf_msg_.transform.translation.x;
            points_[1](1, 0) = tf_msg_.transform.translation.y;
            points_[1](2, 0) = tf_msg_.transform.translation.z;

            rviz_tools_->resetMarkerCounts();
            rviz_tools_->publishSphere(points_[0], rviz_visual_tools::BLUE, rviz_visual_tools::MEDIUM);
            rviz_tools_->publishSphere(points_[1], rviz_visual_tools::BLUE, rviz_visual_tools::MEDIUM);
            rviz_tools_->publishLine(points_[0], points_[1], rviz_visual_tools::BLUE, rviz_visual_tools::MEDIUM);
            rviz_tools_->trigger();
            break;
        case 2:
            points_[2](0, 0) = tf_msg_.transform.translation.x;
            points_[2](1, 0) = tf_msg_.transform.translation.y;
            points_[2](2, 0) = tf_msg_.transform.translation.z;

            basis_[1] = points_[2] - points_[0];
            basis_[2] = basis_[0].cross(basis_[1]);

            eigen_pose_.matrix().block<3, 1>(0, 0) = basis_[0] / basis_[0].norm();
            eigen_pose_.matrix().block<3, 1>(0, 1) = basis_[1] / basis_[1].norm();
            eigen_pose_.matrix().block<3, 1>(0, 2) = basis_[2] / basis_[2].norm();
            eigen_pose_.matrix().block<3, 1>(0, 3) = points_[0] + 0.5*(basis_[0] + basis_[1]);

            rviz_tools_->resetMarkerCounts();
            rviz_tools_->publishSphere(points_[0], rviz_visual_tools::BLUE, rviz_visual_tools::MEDIUM);
            rviz_tools_->publishSphere(points_[1], rviz_visual_tools::BLUE, rviz_visual_tools::MEDIUM);
            rviz_tools_->publishSphere(points_[2], rviz_visual_tools::BLUE, rviz_visual_tools::MEDIUM);
            rviz_tools_->publishCuboid(eigen_pose_, basis_[0].norm(), basis_[1].norm(), 0.01);
            rviz_tools_->trigger();
        case 3:
            points_[3](0, 0) = tf_msg_.transform.translation.x;
            points_[3](1, 0) = tf_msg_.transform.translation.y;
            points_[3](2, 0) = tf_msg_.transform.translation.z;

            height = (points_[3] - points_[0]).dot(basis_[2] / basis_[2].norm() );
            eigen_pose_.matrix().block<3, 1>(0, 3) = points_[0] + 0.5*(basis_[0] + basis_[1] + basis_[2] / basis_[2].norm() * height);

            rviz_tools_->resetMarkerCounts();
            rviz_tools_->publishSphere(points_[0], rviz_visual_tools::BLUE, rviz_visual_tools::MEDIUM);
            rviz_tools_->publishSphere(points_[1], rviz_visual_tools::BLUE, rviz_visual_tools::MEDIUM);
            rviz_tools_->publishSphere(points_[2], rviz_visual_tools::BLUE, rviz_visual_tools::MEDIUM);
            rviz_tools_->publishSphere(points_[3], rviz_visual_tools::BLUE, rviz_visual_tools::MEDIUM);
            rviz_tools_->publishCuboid(eigen_pose_, basis_[0].norm(), basis_[1].norm(), std::abs(height) );
            rviz_tools_->trigger();
            break;
        case 4:
            rviz_tools_->resetMarkerCounts();
            rviz_tools_->publishSphere(points_[0], rviz_visual_tools::BLUE, rviz_visual_tools::MEDIUM);
            rviz_tools_->publishSphere(points_[1], rviz_visual_tools::BLUE, rviz_visual_tools::MEDIUM);
            rviz_tools_->publishSphere(points_[2], rviz_visual_tools::BLUE, rviz_visual_tools::MEDIUM);
            rviz_tools_->publishSphere(points_[3], rviz_visual_tools::BLUE, rviz_visual_tools::MEDIUM);
            rviz_tools_->publishCuboid(eigen_pose_, basis_[0].norm(), basis_[1].norm(), std::abs(height) );
            rviz_tools_->trigger();
            break;
    }
    
    // Realtime visualisation of box based on two points_
    // if (state > 0) {
    //     rviz_tools_->resetMarkerCounts();
    //     rviz_tools_->publishCuboid(point1, point2);
    //     rviz_tools_->trigger();
    // }

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
        return 0;
    }

    while (ros::ok() && sigint_flag) {
        node_.Loop();
    }

    node_.Shutdown();
    return 0;
}