// ROS
#include <ros/ros.h>
// ROS msgs
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedback.h>
// tf2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "openvr_ros/vr_interface.h"

class VRNode {
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

    // ROS publisher maps
    std::map<std::string, ros::Publisher> twist_pubs_map_;
    std::map<std::string, ros::Publisher> joy_pubs_map_;

    VRInterface vr_;

    tf2::Vector3 ConvertVector(float v[3]);
    tf2::Transform ConvertTransform(float m[3][4]);

    public:
    VRNode(int frequency);
    ~VRNode();

    bool Init();
    void Loop();
    void Shutdown();
    
};

VRNode::VRNode(int frequency)
    : loop_rate_(frequency),
      static_tf_broadcaster_(),
      tf_broadcaster_(),
      vr_()
{
    // Send corrective transform
    tf2::Transform tf_world_;
    tf_world_.setOrigin(tf2::Vector3(0, 0, 2.0) );
    tf_world_.setRotation(tf2::Quaternion(0.0, M_PI/2.0, 0.0) );

    geometry_msgs::TransformStamped tf_world_msg_;
    tf_world_msg_.header.stamp = ros::Time::now();
    tf_world_msg_.header.frame_id = "world";
    tf_world_msg_.child_frame_id = "world_vr";
    tf2::convert(tf_world_, tf_world_msg_.transform);
    static_tf_broadcaster_.sendTransform(tf_world_msg_);
}
VRNode::~VRNode() {
}

bool VRNode::Init() {
    return vr_.Init();
}
void VRNode::Shutdown() {
    vr_.Shutdown();
}

// OpenVR to tf2 conversions
tf2::Transform VRNode::ConvertTransform(float m[3][4]) {
    tf2::Matrix3x3 m_basis = tf2::Matrix3x3(m[0][0], m[0][1], m[0][2],
                                            m[1][0], m[1][1], m[1][2],
                                            m[2][0], m[2][1], m[2][2]);
    tf2::Vector3 m_origin = tf2::Vector3(m[0][3], m[1][3], m[2][3]);

    return tf2::Transform(m_basis, m_origin);
}
tf2::Vector3 VRNode::ConvertVector(float v[3]) {
    return tf2::Vector3(v[0], v[1], v[2]);
}

void VRNode::Loop() {
    // Calculate updated poses for all tracked devices
    vr_.Update();

    for (int i = 0; i < 7; i++) {
        // Get current device class
        int device_class = vr_.GetDeviceClass(i);

        if (device_class != vr::TrackedDeviceClass_Invalid) {
            if (vr_.PoseIsValid(i) ) {
                // Get serial number of current device
                std::string device_sn = vr_.GetDeviceSN(i);
                std::replace(device_sn.begin(), device_sn.end(), '-', '_');

                // Get pose of current device
                float current_pose[3][4];
                vr_.GetDevicePose(i, current_pose);
                // Convert OpenVR-pose to tf2-transform
                tf2::Transform current_transform_ = ConvertTransform(current_pose);

                // Instantiate, populate and send transform message
                geometry_msgs::TransformStamped transform_msg_;

                transform_msg_.header.stamp = ros::Time::now();
                transform_msg_.header.frame_id = "world_vr";
                std::string device_frame;
                switch(device_class) {
                    case vr::TrackedDeviceClass_HMD:               device_frame = "hmd_" + device_sn;        break;
                    case vr::TrackedDeviceClass_Controller:        device_frame = "controller_" + device_sn; break;
                    case vr::TrackedDeviceClass_GenericTracker:    device_frame = "tracker_" + device_sn;    break;
                    case vr::TrackedDeviceClass_TrackingReference: device_frame = "lighthouse_" + device_sn; break;
                }
                transform_msg_.child_frame_id = device_frame;

                tf2::convert(current_transform_, transform_msg_.transform);
                tf_broadcaster_.sendTransform(transform_msg_);

                // Check if current device is not a lighthouse
                if (device_class != vr::TrackedDeviceClass_TrackingReference) {
                    // Get linear and angular velocity of current device
                    float linear_velocity[3];
                    float angular_velocity[3];
                    vr_.GetDeviceVelocity(i, linear_velocity, angular_velocity);

                    // Instantiate and populate twist message
                    geometry_msgs::TwistStamped twist_msg_;
                    twist_msg_.header.stamp = ros::Time::now();
                    twist_msg_.header.frame_id = device_frame;
                    tf2::convert(ConvertVector(linear_velocity), twist_msg_.twist.linear);
                    tf2::convert(ConvertVector(angular_velocity), twist_msg_.twist.angular);

                    // Advertise twist topic if the device is new
                    if (twist_pubs_map_.count(device_sn) == 0) {
                        switch(device_class) {
                            case vr::TrackedDeviceClass_HMD:            twist_pubs_map_[device_sn] = nh_.advertise<geometry_msgs::TwistStamped>("/vr/twist/hmd_" + device_sn, 10);        break;
                            case vr::TrackedDeviceClass_Controller:     twist_pubs_map_[device_sn] = nh_.advertise<geometry_msgs::TwistStamped>("/vr/twist/controller_" + device_sn, 10); break;
                            case vr::TrackedDeviceClass_GenericTracker: twist_pubs_map_[device_sn] = nh_.advertise<geometry_msgs::TwistStamped>("/vr/twist/tracker_" + device_sn, 10);    break;
                        }
                    }
                    twist_pubs_map_[device_sn].publish(twist_msg_);
                }

                if (device_class == vr::TrackedDeviceClass_Controller) {
                    sensor_msgs::Joy joy_msg_;
                    joy_msg_.header.stamp = ros::Time::now();
                    joy_msg_.header.frame_id = device_frame;
                    joy_msg_.axes.assign(3, 0.0);
                    joy_msg_.buttons.assign(4, 0);
                    vr_.GetControllerState(i, joy_msg_.axes, joy_msg_.buttons);

                    // Advertise joy topic if the device is new
                    if (joy_pubs_map_.count(device_sn) == 0) {
                        joy_pubs_map_[device_sn] = nh_.advertise<sensor_msgs::Joy>("/vr/joy/controller_" + device_sn, 10); break;
                    }
                    joy_pubs_map_[device_sn].publish(joy_msg_);
                }
            }
        } else {
            // Contine to next device if current device class is invalid
            continue;
        }
    }

    ros::spinOnce();
    loop_rate_.sleep();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vr_node");

    VRNode node_(120);

    if (!node_.Init() ) {
        node_.Shutdown();
        return 0;
    }

    ROS_INFO("OpenVR API initialization succeeded");

    while (ros::ok() ) {
        node_.Loop();
    }

    node_.Shutdown();
    return 0;
}