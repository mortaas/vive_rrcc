// ROS
#include <ros/ros.h>
#include <rosbag/bag.h>

// ROS msgs
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Joy.h>

#include "vive_bridge/TrackedDevicesStamped.h"
// Service msgs
#include <vive_calibrating/AddSample.h>
#include <vive_calibrating/ComputeCalibration.h>

// Dynamic reconfigure
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/Reconfigure.h>

#include "vive_calibrating/ViveConfig.h"

// tf2
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <tf2_eigen/tf2_eigen.h>

// Eigen
#include <Eigen/Geometry>
#include <Eigen/SVD>

// Boost
#include <boost/algorithm/string/predicate.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

class CalibratingNode {
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;

    rosbag::Bag bag_;

    // Reconfigure request for changing frame offset, e.g. changing world_vr frame w.r.t. world
    dynamic_reconfigure::ReconfigureRequest srv_reconf_req_;
    dynamic_reconfigure::ReconfigureResponse srv_reconf_resp_;

    // Subscribers
    ros::Subscriber joy_sub_;
    ros::Subscriber devices_sub_;
    // Callback functions
    void JoyCb(const sensor_msgs::Joy& msg_);
    void DevicesCb(const vive_bridge::TrackedDevicesStamped& msg_);

    bool ValidateDeviceID(std::string device_id);

    // Service
    ros::ServiceClient sample_client, compute_client;

    // msgs
    geometry_msgs::TransformStamped tf_msg_;

    // tf2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener *tf_listener_;

    std::string controller_frame, controller_input;
    tf2::Transform tf_controller_, tf_controller_offset_;

    geometry_msgs::TransformStamped tf_msg_tool0_, tf_msg_sensor_;
    geometry_msgs::TransformStamped tf_msg_A_, tf_msg_B_;
    std::vector<geometry_msgs::Transform> tf_Avec_, tf_Bvec_;

    tf2::Transform tf_tool0_[2], tf_sensor_[2];
    tf2::Transform tf_X_;

    // Eigen
    Eigen::Affine3d eigen_Ta_, eigen_Tb_;
    Eigen::Matrix3d eigen_Rx_, eigen_M_;
    Eigen::Vector3d eigen_tx_;

    bool Measure();
    int n_samples;

    // Parameters
    double yaw_offset, pitch_offset, roll_offset;
    bool InitParams();

    public:
        CalibratingNode(int frequency);
        ~CalibratingNode();

        bool Init();
        void Loop();
        void Shutdown();
};

bool CalibratingNode::Measure() {
    std::string pError;

    if (tf_buffer_.canTransform("floor_tool0", "floor_base", ros::Time(0), &pError) &&
        tf_buffer_.canTransform(controller_frame, "world_vr", ros::Time(0), &pError) )
    {
        // Lookup and convert necessary transforms
        tf_msg_tool0_ = tf_buffer_.lookupTransform("floor_base", ros::Time(0), "floor_tool0", ros::Time(0), "floor_base");
        tf_msg_sensor_ = tf_buffer_.lookupTransform("world_vr", ros::Time(0), controller_frame, ros::Time(0), "world_vr");
        bag_.write("tool0", ros::Time::now(), tf_msg_tool0_);
        bag_.write("sensor", ros::Time::now(), tf_msg_sensor_);

        if (n_samples > 0) {
            tf2::convert(tf_msg_tool0_.transform, tf_tool0_[1]);
            tf2::convert(tf_msg_sensor_.transform, tf_sensor_[1]);

            tf2::convert(tf_tool0_[0].inverseTimes(tf_tool0_[1]), tf_msg_A_.transform);
            tf2::convert(tf_sensor_[0].inverseTimes(tf_sensor_[1]), tf_msg_B_.transform);
            tf_msg_A_.header.frame_id = "floor_tool0";
            tf_msg_B_.header.frame_id = controller_frame;
            tf_msg_A_.header.stamp = tf_msg_tool0_.header.stamp;
            tf_msg_B_.header.stamp = tf_msg_sensor_.header.stamp;

            bag_.write("A", ros::Time::now(), tf_msg_A_);
            bag_.write("B", ros::Time::now(), tf_msg_B_);

            // Sample pair (A, B)
            vive_calibrating::AddSample sample_srv;
            sample_srv.request.A = tf_msg_A_;
            sample_srv.request.B = tf_msg_B_;
            if (sample_client.call(sample_srv) ) {
                ROS_INFO_STREAM("Sampled " << sample_srv.response.n << " pairs (A, B)");
            }

            // Set current pose as reference for next pose
            tf_tool0_[0]  = tf_tool0_[1];
            tf_sensor_[0] = tf_sensor_[1];
        } else {
            tf2::convert(tf_msg_tool0_.transform, tf_tool0_[0]);
            tf2::convert(tf_msg_sensor_.transform, tf_sensor_[0]);
        }

        return true;
    } else {
        return false;
    }
}

CalibratingNode::CalibratingNode(int frequency)
    : loop_rate_(frequency),
      tf_listener_(new tf2_ros::TransformListener(tf_buffer_) )
{
    // Subscribers
    devices_sub_ = nh_.subscribe("/vive_node/tracked_devices", 1, &CalibratingNode::DevicesCb, this);

    // Services
    sample_client =  nh_.serviceClient<vive_calibrating::AddSample>("/vive_calibration/add_sample");
    compute_client = nh_.serviceClient<vive_calibrating::ComputeCalibration>("/vive_calibration/compute_calibration");

    // Define dynamic reconfigure message for calibrating frames
    srv_reconf_req_.config.doubles.resize(6);
    srv_reconf_req_.config.doubles[0].name = "vr_x_offset";
    srv_reconf_req_.config.doubles[1].name = "vr_y_offset";
    srv_reconf_req_.config.doubles[2].name = "vr_z_offset";
    srv_reconf_req_.config.doubles[3].name = "vr_yaw_offset";
    srv_reconf_req_.config.doubles[4].name = "vr_pitch_offset";
    srv_reconf_req_.config.doubles[5].name = "vr_roll_offset";

    n_samples = 0;
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
    }

    return init_success;
}

bool CalibratingNode::Init() {
      /**
     * Initialize the node and check if the necessary transforms are available
     */

    bag_.open("calib_data_" + boost::posix_time::to_iso_string(ros::Time::now().toBoost() ) + ".bag",
              rosbag::bagmode::Write);

    if (!InitParams() ) {
        ROS_WARN("Failed getting parameters from the parameter server.");
    }

    if (controller_frame.empty() || controller_input.empty() ) {
        while (controller_frame.empty() || controller_input.empty() ) {
            ROS_INFO("Waiting for controller...");
            
            ros::spinOnce();
            ros::Duration(1.0).sleep();
        }
    }
    ROS_INFO_STREAM("Using " + controller_frame + " for calibration, and " + controller_input + " for input");

    joy_sub_ = nh_.subscribe("/vive_node/joy/" + controller_input, 1, &CalibratingNode::JoyCb, this);

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

    bag_.close();
}

void CalibratingNode::JoyCb(const sensor_msgs::Joy& msg_) {
      /**
     * Handle VIVE Controller inputs
     */

    // Grip button
    if (msg_.buttons[1]) {
        if (Measure() ) {
            n_samples++;
        }
    }
    // Menu button
    if (msg_.buttons[0]) {
        if (n_samples > 2) {
            ROS_INFO_STREAM("Calibrate with " << n_samples << " measurements!");

            vive_calibrating::ComputeCalibration compute_srv;
            if (compute_client.call(compute_srv) ) {
                if (compute_srv.response.success) {
                    geometry_msgs::TransformStamped tf_msg_Tx_ = compute_srv.response.X;
                    tf_msg_Tx_.header.frame_id = "floor_tool0";
                    tf_msg_Tx_.child_frame_id = controller_frame;
                    tf_msg_Tx_.header.stamp = ros::Time::now();
                    bag_.write("X", ros::Time::now(), tf_msg_Tx_);
                    ROS_INFO_STREAM(tf_msg_Tx_);

                    tf2::fromMsg(tf_msg_Tx_.transform, tf_X_);

                    // Lookup transformation from VIVE Tracker to world_vr
                    tf_msg_ = tf_buffer_.lookupTransform("root", "floor_tool0", ros::Time(0) );
                    tf2::fromMsg(tf_msg_.transform, tf_tool0_[1]);
                    tf_msg_ = tf_buffer_.lookupTransform(controller_frame, "world_vr", ros::Time(0) );
                    tf2::fromMsg(tf_msg_.transform, tf_controller_);

                    tf_controller_ = tf_tool0_[1]*tf_X_*tf_controller_;

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

                    n_samples = 0;
                } else {
                    ROS_ERROR("Failed solving the provided AX=XB problem");
                }
            }
        }
    }
}

void CalibratingNode::DevicesCb(const vive_bridge::TrackedDevicesStamped& msg_) {
      /**
     * Update controller and tracker frames of tracked devices.
     */

    for (int i = 0; i < msg_.device_count; i++) {
        if (msg_.device_classes[i] == msg_.CONTROLLER) {
            if (controller_frame.empty() ) {
                controller_frame = msg_.device_frames[i];
            } else {
                if (controller_input.empty() && msg_.device_frames[i] != controller_frame) {
                    controller_input = msg_.device_frames[i];
                    joy_sub_ = nh_.subscribe("/vive_node/joy/" + controller_input, 1, &CalibratingNode::JoyCb, this);
                }
            }
        }
    }
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