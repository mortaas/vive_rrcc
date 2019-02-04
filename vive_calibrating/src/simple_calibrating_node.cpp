// ROS
#include <ros/ros.h>

// ROS msgs
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>

#include "vive_bridge/TrackedDevicesStamped.h"

// Dynamic reconfigure
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/Reconfigure.h>

#include "vive_calibrating/ViveConfig.h"

// tf2
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Boost
#include <boost/algorithm/string/predicate.hpp>


class CalibratingNode {
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;

    // Subscribers
    ros::Subscriber joy_sub_;
    ros::Subscriber devices_sub_;
    // Callback functions
    void JoyCb(const sensor_msgs::Joy& msg_);
    void DevicesCb(const vive_bridge::TrackedDevicesStamped& msg_);

    bool ValidateDeviceID(std::string device_id);

    // msgs
    geometry_msgs::TransformStamped tf_msg_;

    // Parameters
    double yaw_offset, pitch_offset, roll_offset;
    bool InitParams();

    void CalibrateWorld();

    // Reconfigure request for changing frame offset, e.g. changing world_vr frame w.r.t. world
    dynamic_reconfigure::ReconfigureRequest srv_reconf_req_;
    dynamic_reconfigure::ReconfigureResponse srv_reconf_resp_;

    // tf2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener *tf_listener_;

    std::string controller_frame;
    tf2::Transform tf_controller_, tf_controller_offset_;


    public:
        CalibratingNode(int frequency);
        ~CalibratingNode();

        bool Init();
        void Loop();
        void Shutdown();
};

CalibratingNode::CalibratingNode(int frequency)
    : loop_rate_(frequency),
      tf_listener_(new tf2_ros::TransformListener(tf_buffer_) )
{
    // Subscribers
    devices_sub_ = nh_.subscribe("/vive_node/tracked_devices", 1, &CalibratingNode::DevicesCb, this);

    // Define dynamic reconfigure message for calibrating frames
    srv_reconf_req_.config.doubles.resize(6);
    srv_reconf_req_.config.doubles[0].name = "vr_x_offset";
    srv_reconf_req_.config.doubles[1].name = "vr_y_offset";
    srv_reconf_req_.config.doubles[2].name = "vr_z_offset";
    srv_reconf_req_.config.doubles[3].name = "vr_yaw_offset";
    srv_reconf_req_.config.doubles[4].name = "vr_pitch_offset";
    srv_reconf_req_.config.doubles[5].name = "vr_roll_offset";

    // Pivot point
    // y  0.002273
    // z -0.00985
    // Median point
    // y -0.025907
    // z -0.027553
    // Median point in rotated frame
    // y -0.036594
    // z -0.00955

    double angle_offset = -atan((0.027553 - 0.00985)/(0.025907 + 0.002273) );

    tf_controller_offset_.setOrigin(tf2::Vector3(0., -0.036594, 0.00955) );
    tf_controller_offset_.setRotation(tf2::Quaternion(0., 0., 1., 0.) * 
                                      tf2::Quaternion(std::sin(angle_offset/2), 0., 0.,
                                                      std::cos(angle_offset/2) ) );
}
CalibratingNode::~CalibratingNode() {
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

bool CalibratingNode::InitParams() {
     /**
      * Initialize parameters from the parameter server.
      * Return true if the parameters were retrieved from the server, false otherwise.
      */
    
    std::string controller;

    bool init_success = nh_.param<std::string>("/vive_calibrating_node/controller", controller, "");
    
    // Use controller from parameter server
    if (ValidateDeviceID(controller) ) {
        controller_frame = controller;
        joy_sub_ = nh_.subscribe("/vive_node/joy/" + controller_frame, 1, &CalibratingNode::JoyCb, this);
    }

    return init_success;
}

bool CalibratingNode::Init() {
      /**
     * Initialize the node and check if the necessary transforms are available
     */

    if (!InitParams() ) {
        ROS_WARN("Failed getting parameters from the parameter server.");
    }

    if (controller_frame.empty() ) {
        while (controller_frame.empty() ) {
            ROS_INFO("Waiting for controller...");
            
            ros::spinOnce();
            ros::Duration(1.0).sleep();
        }
    }
    ROS_INFO_STREAM("Using " + controller_frame + " for calibration");

    joy_sub_ = nh_.subscribe("/vive_node/joy/" + controller_frame, 1, &CalibratingNode::JoyCb, this);

    std::string pError;
    if (!tf_buffer_.canTransform(controller_frame,
                                 "root", ros::Time(0),
                                 ros::Duration(5.0), &pError) )
    {
        ROS_ERROR_STREAM("Can't transform from root to " + controller_frame + ": " + pError);

        return false;
    }

    return true;
}

void CalibratingNode::Loop() {
      /**
     * Main loop of the node
     */
    
    ros::spinOnce();
    loop_rate_.sleep();
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

    // Trigger button
    if (msg_.buttons[1]) {
        CalibrateWorld();
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
    }
}

void CalibratingNode::CalibrateWorld() {
     /**
      * Calibrates the world_vr frame based on current VIVE Tracker pose.
      * Offset parameters are calculated by thinking of the VIVE Tracker frame as the new world frame.
      */
    
    // Lookup transformation from VIVE Tracker to world_vr
    tf_msg_ = tf_buffer_.lookupTransform(controller_frame, "world_vr", ros::Time(0) );
    tf2::fromMsg(tf_msg_.transform, tf_controller_);

    tf_controller_ = tf_controller_offset_*tf_controller_;

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
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "vive_calibrating_node");

    CalibratingNode node_(120);

    if (!node_.Init() ) {
        exit(EXIT_FAILURE);
    }

    while (ros::ok() ) {
        node_.Loop();
    }

    node_.Shutdown();
    exit(EXIT_SUCCESS);
}