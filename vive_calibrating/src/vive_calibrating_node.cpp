// ROS
#include <ros/ros.h>
// ROS msgs
#include <sensor_msgs/Joy.h>
#include <vive_bridge/TrackedDevicesStamped.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include "vive_calibrating/ViveConfig.h"

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

// tf2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <boost/algorithm/string/predicate.hpp>

class CalibratingNode {
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;

    // Parameters
    bool InitParams();

    // Subscribers
    ros::Subscriber joy_sub_;
    ros::Subscriber devices_sub_;

    // ROS msgs
    geometry_msgs::TransformStamped tf_msg_;

    // Dynamic reconfigure
    dynamic_reconfigure::Server<vive_calibrating::ViveConfig> reconf_server_;
    dynamic_reconfigure::Server<vive_calibrating::ViveConfig>::CallbackType callback_type_;
    void ReconfCallback(vive_calibrating::ViveConfig &config, uint32_t level);

    dynamic_reconfigure::ReconfigureRequest srv_reconf_req_;
    dynamic_reconfigure::ReconfigureResponse srv_reconf_resp_;

    vive_calibrating::ViveConfig conf_;

    // tf2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener *tf_listener_;

    std::string controller_frame, tracker_frame;
    tf2::Transform tf_tracker_;

    double yaw_offset, pitch_offset, roll_offset;

    bool ValidateDeviceName(std::string device_name);

    void JoyCb(const sensor_msgs::Joy& msg_);
    void DevicesCb(const vive_bridge::TrackedDevicesStamped& msg_);

    public:
        CalibratingNode(int frequency);
        ~CalibratingNode();

        bool Init();
        void Loop();
        void Shutdown();
};

bool CalibratingNode::ValidateDeviceName(std::string device_name) {
     /**
      * Validate device name by checking string format and character length.
      * The serial number of tracked devices is 12 characters.
      */
    
    if (boost::algorithm::contains(device_name, "hmd_") ) {
        if (strlen(device_name.c_str() ) == 16) {
            return true;
        }
    }
    if (boost::algorithm::contains(device_name, "controller_") ) {
        if (strlen(device_name.c_str() ) == 23) {
            return true;
        }
    }
    if (boost::algorithm::contains(device_name, "tracker_") ) {
        if (strlen(device_name.c_str() ) == 20) {
            return true;
        }
    }
    if (boost::algorithm::contains(device_name, "lighthouse_") ) {
        if (strlen(device_name.c_str() ) == 23) {
            return true;
        }
    }

    return false;
}

bool CalibratingNode::InitParams() {
     /**
      * Initialize parameters from the parameter server.
      * Return true if the parameters was retrieved from the server, false otherwise.
      */
    
    std::string controller, tracker;

    bool init_success (nh_.param<std::string>("/vive_calibrating_node/controller", controller, "") &&
                       nh_.param<std::string>("/vive_calibrating_node/tracker",    tracker,    "") );
    
    if (ValidateDeviceName(controller) ) {
        controller_frame = controller;
    }
    if (ValidateDeviceName(tracker) ) {
        tracker_frame = tracker;
        joy_sub_ = nh_.subscribe("/vive_node/joy/" + controller_frame, 1, &CalibratingNode::JoyCb, this);
    }

    return init_success;
}
void CalibratingNode::ReconfCallback(vive_calibrating::ViveConfig &config, uint32_t level) {
     /**
      * Dynamic reconfigure callback for changing parameters during runtime
      */
    
    if (ValidateDeviceName(config.controller) ) {
        controller_frame = config.controller;
    }

    if (ValidateDeviceName(config.tracker) ) {
        tracker_frame = config.tracker;
        joy_sub_ = nh_.subscribe("/vive_node/joy/" + controller_frame, 1, &CalibratingNode::JoyCb, this);
    }
}

CalibratingNode::CalibratingNode(int frequency)
    : loop_rate_(frequency),
      tf_listener_(new tf2_ros::TransformListener(tf_buffer_) )
{
    // Set dynamic reconfigure callback function
    callback_type_ = boost::bind(&CalibratingNode::ReconfCallback, this, _1, _2);
    reconf_server_.setCallback(callback_type_);

    devices_sub_ = nh_.subscribe("/vive_node/tracked_devices", 1, &CalibratingNode::DevicesCb, this);

    srv_reconf_req_.config.doubles.resize(6);
    srv_reconf_req_.config.doubles[0].name = "x_offset";
    srv_reconf_req_.config.doubles[1].name = "y_offset";
    srv_reconf_req_.config.doubles[2].name = "z_offset";
    srv_reconf_req_.config.doubles[3].name = "yaw_offset";
    srv_reconf_req_.config.doubles[4].name = "pitch_offset";
    srv_reconf_req_.config.doubles[5].name = "roll_offset";
}
CalibratingNode::~CalibratingNode() {
}

void CalibratingNode::JoyCb(const sensor_msgs::Joy& msg_) {
      /**
     * Handle VIVE Controller inputs
     */

    // Set tracker frame as new world frame when pressing the menu button
    if (msg_.buttons[0]) {
        tf_msg_ = tf_buffer_.lookupTransform(tracker_frame, "world_vr", ros::Time(0) );
        tf2::convert(tf_msg_.transform, tf_tracker_);

        tf_tracker_.getBasis().getRPY(roll_offset, pitch_offset, yaw_offset);

        srv_reconf_req_.config.doubles[0].value = tf_tracker_.getOrigin().getX();
        srv_reconf_req_.config.doubles[1].value = tf_tracker_.getOrigin().getY();
        srv_reconf_req_.config.doubles[2].value = tf_tracker_.getOrigin().getZ();
        srv_reconf_req_.config.doubles[3].value = yaw_offset;
        srv_reconf_req_.config.doubles[4].value = pitch_offset;
        srv_reconf_req_.config.doubles[5].value = roll_offset;

        ros::service::call("/vive_node/set_parameters", srv_reconf_req_, srv_reconf_resp_);
    }
    // Set default configuration when pressing the grip button
    if (msg_.buttons[1]) {
        srv_reconf_req_.config.doubles[0].value = -0.708375580986;
        srv_reconf_req_.config.doubles[1].value = 1.25077580883;
        srv_reconf_req_.config.doubles[2].value = 1.9955853131;
        srv_reconf_req_.config.doubles[3].value = -2.7223704589;
        srv_reconf_req_.config.doubles[4].value = 0.0174707076117;
        srv_reconf_req_.config.doubles[5].value = 1.57467366024;

        ros::service::call("/vive_node/set_parameters", srv_reconf_req_, srv_reconf_resp_);
    }
}

void CalibratingNode::DevicesCb(const vive_bridge::TrackedDevicesStamped& msg_) {
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

bool CalibratingNode::Init() {
      /**
     * Initialize the node and check if the necessary transforms are available
     */

    if (!InitParams() ) {
        ROS_WARN("Failed to get parameters from the parameter server.");
    }

    if (controller_frame.empty() && tracker_frame.empty() ) {
        while (controller_frame.empty() && tracker_frame.empty() ) {
            ROS_INFO("Waiting for controller and tracker...");
            
            ros::spinOnce();
            ros::Duration(5.0).sleep();
        }

        conf_.controller = controller_frame;
        conf_.tracker = tracker_frame;
        reconf_server_.updateConfig(conf_);
    }
    ROS_INFO_STREAM("Using " + controller_frame + " and " + tracker_frame + " for calibration");

    joy_sub_ = nh_.subscribe("/vive_node/joy/" + controller_frame, 1, &CalibratingNode::JoyCb, this);

    std::string pError;
    if (!tf_buffer_.canTransform(tracker_frame,
                                 "world", ros::Time(0),
                                 ros::Duration(5.0), &pError) )
    {
        ROS_ERROR_STREAM("Can't transform from world to " + tracker_frame + ": " + pError);

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

int main(int argc, char** argv) {
    ros::init(argc, argv, "vive_calibrating_node");

    CalibratingNode node_(120);

    if (!node_.Init() ) {
        return 0;
    }

    while (ros::ok() ) {
        node_.Loop();
    }

    node_.Shutdown();
    return 0;
}