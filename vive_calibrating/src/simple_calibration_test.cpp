// ROS
#include <ros/ros.h>

// ROS msgs
#include <geometry_msgs/PoseStamped.h>

// tf2
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class CalibrationTestNode
{
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;

    // Publisher
    ros::Publisher pose_pub_;

    // tf2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener *tf_listener_;

    // Coordinate frames
    std::string controller_frame, test_frame;

    tf2::Transform tf_diff_;
    geometry_msgs::TransformStamped tf_msg_diff_;
    geometry_msgs::PoseStamped pose_diff_;

    public:
        CalibrationTestNode(int frequency);
        ~CalibrationTestNode();

        bool InitParams();

        bool Init();
        void Loop();
        void Shutdown();
};

CalibrationTestNode::CalibrationTestNode(int frequency):
    loop_rate_(frequency),
    tf_listener_(new tf2_ros::TransformListener(tf_buffer_) )
{   
    // Publisher
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("pose_diff", 10, true);
}
CalibrationTestNode::~CalibrationTestNode() {
}

bool CalibrationTestNode::InitParams() {
     /**
      * Initialize parameters from the parameter server.
      * Returns true if the parameters were retrieved from the server, false otherwise.
      */

    if (nh_.param<std::string>("/vive_robot_calibrating_node/controller_frame", controller_frame, "") &&
        nh_.param<std::string>("/vive_robot_calibrating_node/test_frame",       test_frame,       "controller_test") )
    {
        return true;
    } else {
        ROS_WARN("Failed to get parameters from the parameter server.");

        return false;
    }
}

bool CalibrationTestNode::Init() {
      /**
     * Initialize the node
     */

    InitParams();

    pose_diff_.header.frame_id = test_frame;

    while (!tf_buffer_.canTransform(test_frame, controller_frame, ros::Time(0) ) ) {
        ROS_INFO("Waiting for available VIVE controller...");
        
        ros::spinOnce();
        ros::Duration(1.0).sleep();
    }

    ROS_INFO_STREAM("Using " + controller_frame + " for test");

    return true;
}

void CalibrationTestNode::Loop() {
      /**
     * Main loop of the node
     */

    if (tf_buffer_.canTransform(test_frame, controller_frame, ros::Time(0) ) ) {
        tf_msg_diff_ = tf_buffer_.lookupTransform(controller_frame, test_frame, ros::Time(0) );
        tf2::fromMsg(tf_msg_diff_.transform, tf_diff_);
        tf2::toMsg(tf_diff_, pose_diff_.pose);

        pose_diff_.header.stamp = ros::Time::now();
        pose_pub_.publish(pose_diff_);
    }

    ros::spinOnce();
    loop_rate_.sleep();
}

void CalibrationTestNode::Shutdown() {
      /**
     * Runs before shutting down the node
     */

}


int main(int argc, char** argv) {
    ros::init(argc, argv, "cal_test");

    CalibrationTestNode node_(120);

    if (!node_.Init() ) {
        exit(EXIT_FAILURE);
    }

    while (ros::ok() ) {
        node_.Loop();
    }

    node_.Shutdown();
    exit(EXIT_SUCCESS);
}