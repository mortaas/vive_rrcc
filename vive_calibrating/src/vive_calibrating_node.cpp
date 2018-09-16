// ROS
#include <ros/ros.h>

// ROS msgs
#include <vive_bridge/TrackedDevicesStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>

// Dynamic reconfigure
// #include <dynamic_reconfigure/server.h>
#include "vive_calibrating/ViveConfig.h"

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

// tf2
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "tf2_eigen/tf2_eigen.h"

// RViz
#include <rviz_visual_tools/rviz_visual_tools.h>

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

    // ROS msgs
    geometry_msgs::TransformStamped tf_msg_;

    // Parameters
    bool InitParams();

    // Dynamic reconfigure
    // dynamic_reconfigure::Server<vive_calibrating::ViveConfig> reconf_server_;
    // dynamic_reconfigure::Server<vive_calibrating::ViveConfig>::CallbackType callback_type_;
    // void ReconfCallback(vive_calibrating::ViveConfig &config, uint32_t level);

    // Reconfigure request for changing frame offset, e.g. changing world_vr frame w.r.t. world
    dynamic_reconfigure::ReconfigureRequest srv_reconf_req_;
    dynamic_reconfigure::ReconfigureResponse srv_reconf_resp_;
    vive_calibrating::ViveConfig conf_;

    // RViz
    rviz_visual_tools::RvizVisualToolsPtr visual_tools_;

    // tf2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener *tf_listener_;

    std::string controller_frame;
    tf2::Transform tf_controller_, tf_controller_offset_;

    Eigen::Affine3d eigen_pose_;
    Eigen::Vector3d eigen_points_[3], eigen_basis_[3];
    double yaw_offset, pitch_offset, roll_offset;

    int state;
    void CalibrateWorld();

    public:
        CalibratingNode(int frequency);
        ~CalibratingNode();

        bool Init();
        void Loop();
        void Shutdown();
};

bool CalibratingNode::InitParams() {
     /**
      * Initialize parameters from the parameter server.
      * Return true if the parameters were retrieved from the server, false otherwise.
      */
    
    std::string controller;

    bool init_success (nh_.param<std::string>("/vive_calibrating_node/controller", controller, "") );
    
    // Use controller from parameter server
    if (ValidateDeviceID(controller) ) {
        controller_frame = controller;
        joy_sub_ = nh_.subscribe("/vive_node/joy/" + controller_frame, 1, &CalibratingNode::JoyCb, this);
    }

    return init_success;
}

// void CalibratingNode::ReconfCallback(vive_calibrating::ViveConfig &config, uint32_t level) {
//      /**
//       * Dynamic reconfigure callback for changing parameters during runtime
//       */
    
//     // Update controller
//     if (ValidateDeviceID(config.controller) ) {
//         controller_frame = config.controller;
//         joy_sub_ = nh_.subscribe("/vive_node/joy/" + controller_frame, 1, &CalibratingNode::JoyCb, this);
//     }
// }

CalibratingNode::CalibratingNode(int frequency)
    : loop_rate_(frequency),
      tf_listener_(new tf2_ros::TransformListener(tf_buffer_) )
{
    // Subscribers
    devices_sub_ = nh_.subscribe("/vive_node/tracked_devices", 1, &CalibratingNode::DevicesCb, this);

    // Set dynamic reconfigure callback function
    // callback_type_ = boost::bind(&CalibratingNode::ReconfCallback, this, _1, _2);
    // reconf_server_.setCallback(callback_type_);

    // Define dynamic reconfigure message for calibrating frames
    srv_reconf_req_.config.doubles.resize(12);
    srv_reconf_req_.config.doubles[0].name = "vr_x_offset";
    srv_reconf_req_.config.doubles[1].name = "vr_y_offset";
    srv_reconf_req_.config.doubles[2].name = "vr_z_offset";
    srv_reconf_req_.config.doubles[3].name = "vr_yaw_offset";
    srv_reconf_req_.config.doubles[4].name = "vr_pitch_offset";
    srv_reconf_req_.config.doubles[5].name = "vr_roll_offset";
    srv_reconf_req_.config.doubles[6].name = "robot_x_offset";
    srv_reconf_req_.config.doubles[7].name = "robot_y_offset";
    srv_reconf_req_.config.doubles[8].name = "robot_z_offset";
    srv_reconf_req_.config.doubles[9].name = "robot_yaw_offset";
    srv_reconf_req_.config.doubles[10].name = "robot_pitch_offset";
    srv_reconf_req_.config.doubles[11].name = "robot_roll_offset";

    tf_controller_offset_.setIdentity();
    tf_controller_offset_.setOrigin(tf2::Vector3(0.001394, -0.077208, -0.038291) );

    state = 0;
}
CalibratingNode::~CalibratingNode() {
}

void CalibratingNode::CalibrateWorld() {
     /**
      * Calibrates the world_vr frame based on current VIVE Tracker pose.
      * Offset parameters are calculated by thinking of the VIVE Tracker frame as the new world frame.
      */
    
    // Lookup transformation from VIVE Tracker to world_vr
    tf_msg_ = tf_buffer_.lookupTransform(controller_frame, "world_vr", ros::Time(0) );
    tf2::fromMsg(tf_msg_.transform, tf_controller_);

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

void CalibratingNode::JoyCb(const sensor_msgs::Joy& msg_) {
      /**
     * Handle VIVE Controller inputs
     */

    // Trigger button
    if (msg_.buttons[0]) {
        switch (state) {
            case 0:
                ROS_INFO("Point 1!");
                ROS_INFO_STREAM(eigen_points_[0].matrix() );
                state = 1;
                break;
            case 1:
                ROS_INFO("Point 2!");
                ROS_INFO_STREAM(eigen_points_[1].matrix() );

                eigen_basis_[0] = eigen_points_[1] - eigen_points_[0];
                eigen_basis_[0] = eigen_basis_[0] / eigen_basis_[0].norm();

                state = 2;
                break;
            case 2:
                ROS_INFO("Point 3!");
                ROS_INFO_STREAM(eigen_points_[2].matrix() );

                eigen_basis_[1] = eigen_points_[2] - eigen_points_[0];
                eigen_basis_[1] = eigen_basis_[1] / eigen_basis_[1].norm();
                eigen_basis_[2] = eigen_basis_[0].cross(eigen_basis_[2]);

                eigen_pose_.matrix().block<3, 1>(0, 0) = eigen_basis_[0];
                eigen_pose_.matrix().block<3, 1>(0, 1) = eigen_basis_[1];
                eigen_pose_.matrix().block<3, 1>(0, 2) = eigen_basis_[2];
                eigen_pose_.matrix().block<3, 1>(0, 3) = eigen_points_[0];
                ROS_INFO_STREAM(eigen_pose_.matrix() );

                visual_tools_->setLifetime(0);
                visual_tools_->publishAxisLabeled(eigen_pose_, "World", rviz_visual_tools::LARGE);
                visual_tools_->trigger();
                visual_tools_->setLifetime(0.01);

                state = 3;
                break;
            case 3:
                ROS_INFO("Calibrate world");

                // Reset to first point
                state = 0;
                break;
        }
    }

    // Trackpad button - Calibrates the world frame to hard coded default values
    if (msg_.buttons[2]) {
        srv_reconf_req_.config.doubles[0].value = 1.1245388116;
        srv_reconf_req_.config.doubles[1].value = -0.608462828586;
        srv_reconf_req_.config.doubles[2].value = 1.49679019946;
        srv_reconf_req_.config.doubles[3].value = 1.02410968817;
        srv_reconf_req_.config.doubles[4].value = -0.0623919286899;
        srv_reconf_req_.config.doubles[5].value = 1.63425725418;

        ros::service::call("/vive_node/set_parameters", srv_reconf_req_, srv_reconf_resp_);
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

        // conf_.controller = controller_frame;
        // reconf_server_.updateConfig(conf_);
    }
    ROS_INFO_STREAM("Using " + controller_frame + " for calibration");

    joy_sub_ = nh_.subscribe("/vive_node/joy/" + controller_frame, 1, &CalibratingNode::JoyCb, this);

    std::string pError;
    if (!tf_buffer_.canTransform(controller_frame,
                                 "world", ros::Time(0),
                                 ros::Duration(5.0), &pError) )
    {
        ROS_ERROR_STREAM("Can't transform from world to " + controller_frame + ": " + pError);

        return false;
    }

    visual_tools_.reset(new rviz_visual_tools::RvizVisualTools("world", "/rviz_visual_markers") );
    visual_tools_->setLifetime(0.01);
    visual_tools_->setBaseFrame("world");
    
    return true;
}

void CalibratingNode::Loop() {
      /**
     * Main loop of the node
     */

    tf_msg_ = tf_buffer_.lookupTransform("world", controller_frame, ros::Time(0) );
    visual_tools_->resetMarkerCounts();

    switch (state) {
        case 0:
            eigen_points_[0] << tf_msg_.transform.translation.x,
                                tf_msg_.transform.translation.y,
                                tf_msg_.transform.translation.z;

            // visual_tools_->publishSphere(eigen_points_[0]);
            // visual_tools_->trigger();
            break;
        case 1:
            eigen_points_[1] << tf_msg_.transform.translation.x,
                                tf_msg_.transform.translation.y,
                                tf_msg_.transform.translation.z;
            
            visual_tools_->publishLine(eigen_points_[0],
                                       eigen_points_[1],
                                       rviz_visual_tools::RED);
            visual_tools_->trigger();
            break;
        case 2:
            eigen_points_[2] << tf_msg_.transform.translation.x,
                                tf_msg_.transform.translation.y,
                                tf_msg_.transform.translation.z;
            visual_tools_->publishLine(eigen_points_[0],
                                       eigen_points_[2],
                                       rviz_visual_tools::GREEN);
            visual_tools_->trigger();
            break;
    }

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
        exit(EXIT_FAILURE);
    }

    while (ros::ok() ) {
        node_.Loop();
    }

    node_.Shutdown();
    exit(EXIT_SUCCESS);
}