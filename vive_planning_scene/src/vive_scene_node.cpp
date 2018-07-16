// ROS
#include <ros/ros.h>
// ROS msgs
#include <vive_bridge/TrackedDevicesStamped.h>

// MoveIt!
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/PlanningScene.h>

// #include <moveit_visual_tools/moveit_visual_tools.h>

// tf2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// RViz
#include <rviz_visual_tools/rviz_visual_tools.h>

#include <Eigen/Dense>
#include "tf2_eigen/tf2_eigen.h"

class SceneNode {
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;

    // Subscribers
    ros::Subscriber devices_sub_;
    ros::Subscriber joy_sub_;

    // ROS msgs
    geometry_msgs::TransformStamped tf_msg_;

    ros::ServiceClient planning_scene_client_;
    moveit_msgs::ApplyPlanningScene planning_scene_srv_;
    moveit_msgs::PlanningScene planning_scene_;

    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

    // tf2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener *tf_listener_;

    tf2::Transform tf_tracker_;

    void JoyCb(const sensor_msgs::Joy& msg_);
    void DevicesCb(const vive_bridge::TrackedDevicesStamped& msg_);

    std::string controller_frame, tracker_frame;

    Eigen::Vector3d point1, point2;
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

    point1.setZero();
    point2.setZero();

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
                state = 1;
                break;
            case 1:
                ROS_INFO("Point 2!");
                state = 2;
                break;
            case 2: 
                ROS_INFO("Publish cuboid");
                // point1.setZero();
                // point2.setZero();

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

    // tf_msg_ = tf_buffer_.lookupTransform("world", tracker_frame, ros::Time(0) );
    // point1(0, 0) = tf_msg_.transform.translation.x;
    // point1(1, 0) = tf_msg_.transform.translation.y;
    // point1(2, 0) = tf_msg_.transform.translation.z;
    // point2(0, 0) = tf_msg_.transform.translation.x;
    // point2(1, 0) = tf_msg_.transform.translation.y;
    // point2(2, 0) = tf_msg_.transform.translation.z;

    visual_tools_->setLifetime(0.01);
    visual_tools_->setBaseFrame("world");

    planning_scene_.is_diff = true;
    
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
    }
    
    if (state > 0) {
        visual_tools_->resetMarkerCounts();
        visual_tools_->publishCuboid(point1, point2);
        visual_tools_->trigger();
    }

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