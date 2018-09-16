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

    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

    // SDFormat
    // sdf::SDFPtr sdf_;

    // tf2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener *tf_listener_;

    tf2::Transform tf_tracker_;
    std::string controller_frame, tracker_frame;

    // Eigen
    Eigen::Vector3d point1, point2, point3;
    Eigen::Vector3d x_basis_, y_basis_, z_basis_;
    Eigen::Affine3d eigen_pose_, eigen_eurobox_center_;
    
    int state;

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
        if (msg_.device_classes[i] == msg_.TRACKER) {
            tracker_frame = msg_.device_frames[i];
        }
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

    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world", "/rviz_visual_markers") );

    // SDFormat
    // sdf_root_ = sdf_->Root();

    // Eigen
    point1.setZero();
    point2.setZero();
    point3.setZero();

    eigen_eurobox_center_.matrix().setIdentity();
    eigen_eurobox_center_.matrix().block<3, 1>(0, 3) = Eigen::Vector3d(-0.189224, -0.139224, -0.074999);

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
                ROS_INFO_STREAM(point1.matrix() );
                state = 1;
                break;
            case 1:
                ROS_INFO("Point 2!");
                ROS_INFO_STREAM(point2.matrix() );
                state = 2;
                break;
            case 2:
                ROS_INFO("Point 3!");
                ROS_INFO_STREAM(point3.matrix() );
                state = 3;
                break;
            case 3:
                ROS_INFO("Publish eurobox!");

                // Compute basis vectors from points
                x_basis_ = (point1 - point2);
                x_basis_ = x_basis_ / x_basis_.norm();
                
                y_basis_ = (point1 - point3);
                y_basis_ = y_basis_ / y_basis_.norm();

                z_basis_ = x_basis_.cross(y_basis_);

                // Create pose from basis vectors (rotation) and first point (translation)
                eigen_pose_.matrix().block<3, 1>(0, 0) = x_basis_;
                eigen_pose_.matrix().block<3, 1>(0, 1) = y_basis_;
                eigen_pose_.matrix().block<3, 1>(0, 2) = z_basis_;
                eigen_pose_.matrix().block<3, 1>(0, 3) = point1;
                ROS_INFO_STREAM(eigen_pose_.matrix() );

                visual_tools_->publishMesh(eigen_pose_ * eigen_eurobox_center_,
                                           "package://clean_grasps/workcell_support/urdf/eurobox/meshes/eurobox.STL",
                                           rviz_visual_tools::GREY,
                                           1.0,
                                           "Eurobox");
                visual_tools_->trigger();

                // Reset to first point
                state = 0;
                break;
        }
    }
}

bool SceneNode::Init() {
      /**
     * Check if the necessary transforms are available and initialize the node
     */

    while (controller_frame.empty() && tracker_frame.empty() ) {
        ROS_INFO("Waiting for controller and tracker...");
        
        ros::spinOnce();
        ros::Duration(5.0).sleep();
    }
    ROS_INFO_STREAM("Using " + controller_frame + " and " + tracker_frame + " for calibration");

    joy_sub_ = nh_.subscribe("/vive_node/joy/" + controller_frame, 1, &SceneNode::JoyCb, this);

    std::string pError;
    if (!tf_buffer_.canTransform(tracker_frame,
                                 "world", ros::Time(0),
                                 ros::Duration(10.0), &pError) )
    {
        ROS_ERROR_STREAM("Can't transform from world to " + tracker_frame + ": " + pError);

        return false;
    }

    // planning_scene_.is_diff = true;

    // visual_tools_->setLifetime(0.01);
    visual_tools_->setBaseFrame("world");
    
    return true;
}

void SceneNode::Loop() {
    tf_msg_ = tf_buffer_.lookupTransform("world", tracker_frame, ros::Time(0) );

    switch (state) {
        case 0:
            point1(0, 0) = tf_msg_.transform.translation.x;
            point1(1, 0) = tf_msg_.transform.translation.y;
            point1(2, 0) = tf_msg_.transform.translation.z;
            break;
        case 1:
            point2(0, 0) = tf_msg_.transform.translation.x;
            point2(1, 0) = tf_msg_.transform.translation.y;
            point2(2, 0) = tf_msg_.transform.translation.z;
            break;
        case 2:
            point3(0, 0) = tf_msg_.transform.translation.x;
            point3(1, 0) = tf_msg_.transform.translation.y;
            point3(2, 0) = tf_msg_.transform.translation.z;
            break;
    }
    
    // Realtime visualisation of box based on two points
    // if (state > 0) {
    //     visual_tools_->resetMarkerCounts();
    //     visual_tools_->publishCuboid(point1, point2);
    //     visual_tools_->trigger();
    // }

    ros::spinOnce();
    loop_rate_.sleep();
}
void SceneNode::Shutdown() {
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vive_scene_node");

    SceneNode node_(60);

    if (!node_.Init() ) {
        return 0;
    }

    while (ros::ok() ) {
        node_.Loop();
    }

    node_.Shutdown();
    return 0;
}