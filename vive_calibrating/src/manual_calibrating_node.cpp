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

#include <tf2_eigen/tf2_eigen.h>

// Eigen
#include <Eigen/Geometry>
#include <Eigen/SVD>

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

    Eigen::Vector3d RotationMatrixLogarithm(const Eigen::Matrix3d &rotmat_);

    geometry_msgs::TransformStamped ParkMartin(const geometry_msgs::Transform tf_Ta_[],
                                               const geometry_msgs::Transform tf_Tb_[],
                                               const int &size);
    
    int n;
    bool Measure();

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

        if (n > 0) {
            tf2::convert(tf_msg_tool0_.transform, tf_tool0_[1]);
            tf2::convert(tf_msg_sensor_.transform, tf_sensor_[1]);

            tf2::convert(tf_tool0_[0].inverseTimes(tf_tool0_[1]), tf_msg_A_.transform);
            tf2::convert(tf_sensor_[0].inverseTimes(tf_sensor_[1]), tf_msg_B_.transform);
            tf_Avec_.push_back(tf_msg_A_.transform);
            tf_Bvec_.push_back(tf_msg_B_.transform);

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

    // Define dynamic reconfigure message for calibrating frames
    srv_reconf_req_.config.doubles.resize(6);
    srv_reconf_req_.config.doubles[0].name = "vr_x_offset";
    srv_reconf_req_.config.doubles[1].name = "vr_y_offset";
    srv_reconf_req_.config.doubles[2].name = "vr_z_offset";
    srv_reconf_req_.config.doubles[3].name = "vr_yaw_offset";
    srv_reconf_req_.config.doubles[4].name = "vr_pitch_offset";
    srv_reconf_req_.config.doubles[5].name = "vr_roll_offset";

    n = 0;

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


}

void CalibratingNode::JoyCb(const sensor_msgs::Joy& msg_) {
      /**
     * Handle VIVE Controller inputs
     */

    // Trigger button
    if (msg_.buttons[1]) {
        // CalibrateWorld();
        if (Measure() ) {
            ROS_INFO("Measurement succeeded");
            n++;
        } else {
            ROS_INFO("Measurement failed");
        }
    }
    if (msg_.buttons[0]) {
        if (n > 2) {
            ROS_INFO("Calibrate!");
            geometry_msgs::TransformStamped tf_msg_Tx_ = ParkMartin(tf_Avec_.data(),
                                                                    tf_Bvec_.data(),
                                                                    tf_Avec_.size() );
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
                if (controller_input.empty() ) {
                    controller_input = msg_.device_frames[i];
                    joy_sub_ = nh_.subscribe("/vive_node/joy/" + controller_input, 1, &CalibratingNode::JoyCb, this);
                }
            }
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

geometry_msgs::TransformStamped CalibratingNode::ParkMartin(const geometry_msgs::Transform tf_Ta_[],
                                                            const geometry_msgs::Transform tf_Tb_[],
                                                            const int &size)
{
     /**
      * Solves Ta * Tx = Tx * Tb based on a closed-form least squares solution from:
      * Robot sensor calibration: solving AX=XB on the Euclidean group
      * https://ieeexplore.ieee.org/document/326576/
      * 
      * "The equation AX = XB on the Euclidean group, where A and
      *  B are known and X is unknown, is of fundamental importance in the
      *  problem of calibrating wrist-mounted robotic sensors."
      * 
      * A helpful illustration of this problem is found at:
      * https://torsteinmyhre.name/snippets/robcam_calibration.html
      * 
      * Input:
      * Ta - TF message array of tool0 poses
      * Tb - TF message array of sensor poses
      * size - Size of both arrays (should be the same)
      * 
      * Output:
      * Tx - Transformation between end effector and sensor (solution)
      */

    Eigen::Matrix3d eigen_Ra_[size], eigen_Rb_[size];
    Eigen::Vector3d eigen_ta_[size], eigen_tb_[size];

    Eigen::MatrixXd eigen_C_(3 * size, 3);
    Eigen::VectorXd eigen_d_(3 * size, 1);
    eigen_M_.setZero();
    eigen_C_.setZero();
    eigen_d_.setZero();

    for (int i = 0; i < size; i++) {
        // Convert tf msgs to Eigen's 4x4 transform matrices (OpenGL)
        eigen_Ta_ = tf2::transformToEigen(tf_Ta_[i]);
        eigen_Tb_ = tf2::transformToEigen(tf_Tb_[i]);
        // Get rotation matrices from transforms
        eigen_Ra_[i] = eigen_Ta_.matrix().block<3, 3>(0, 0);
        eigen_Rb_[i] = eigen_Tb_.matrix().block<3, 3>(0, 0);
        // Get translation vectors from transforms
        eigen_ta_[i] = eigen_Ta_.matrix().col(3).head<3>();
        eigen_tb_[i] = eigen_Tb_.matrix().col(3).head<3>();

        eigen_M_ += RotationMatrixLogarithm(eigen_Rb_[i]) *
                    RotationMatrixLogarithm(eigen_Ra_[i]).transpose();
    }

    if (size == 2) {
        eigen_M_ += RotationMatrixLogarithm(eigen_Rb_[1]).cross(RotationMatrixLogarithm(eigen_Rb_[0]) ) *
                    RotationMatrixLogarithm(eigen_Ra_[1]).cross(RotationMatrixLogarithm(eigen_Ra_[0]) ).transpose();
    }

    // Compute the optimal rotation matrix R_x = (M^T * M)^(-1/2) * M^T with SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd_(eigen_M_.transpose() * eigen_M_,
                                           Eigen::ComputeFullU | Eigen::ComputeFullV);
    eigen_Rx_ = svd_.matrixU() *
                ((((svd_.singularValues() ).cwiseSqrt() ).cwiseInverse() ).asDiagonal() ).toDenseMatrix() *
                svd_.matrixV().transpose() *
                eigen_M_.transpose();

    for (int i = 0; i < size; i++) {
        eigen_C_.block<3, 3>(3*i, 0) = Eigen::Matrix3d::Identity() - eigen_Ra_[i];
        eigen_d_.segment(3*i, 3) = eigen_ta_[i] - eigen_Rx_ * eigen_tb_[i];
    }
    // Compute the optimal translation vector t_x = C^T / (C^T * C) * d
    eigen_tx_ = (eigen_C_.transpose() * eigen_C_).inverse() * eigen_C_.transpose() * eigen_d_;

    // Create and return the resulting 4x4 transform as geometry_msgs::TransformStamped
    Eigen::Affine3d eigen_Tx_;
    eigen_Tx_.matrix().block<3, 3>(0, 0) = eigen_Rx_;
    eigen_Tx_.matrix().col(3).head<3>()  = eigen_tx_;

    ROS_INFO_STREAM(std::endl << eigen_Tx_.matrix() );
    return tf2::eigenToTransform(eigen_Tx_);
}

Eigen::Vector3d CalibratingNode::RotationMatrixLogarithm(const Eigen::Matrix3d &rotmat_) {
     /**
      * Compute the logarithm of a 3x3 rotation matrix
      */

    // Check if logarithm is uniquely defined
    if (rotmat_.trace() != -1.) {
        // Axis-angle magnitude
        const double theta = std::acos((rotmat_.trace() - 1.) / 2.);
        return theta / (2. * sin(theta) ) * Eigen::Vector3d(rotmat_(2,1) - rotmat_(1,2),
                                                            rotmat_(0,2) - rotmat_(2,0),
                                                            rotmat_(1,0) - rotmat_(0,1) );
    } else {
        ROS_ERROR_STREAM("Error occurred when computing logarithm of rotation matrix:" << std::endl <<
                         rotmat_.matrix() << std::endl <<
                         "This rotation matrix has a trace equal to: " << rotmat_.trace() <<
                         ", and therefore results in a logarithm that is not uniquely defined.");

        return Eigen::Vector3d::Constant(0.);
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