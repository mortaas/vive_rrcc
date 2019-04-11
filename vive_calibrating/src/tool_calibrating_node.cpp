#include <signal.h>

// ROS
#include <ros/ros.h>
// ROS msgs
#include <vive_bridge/TrackedDevicesStamped.h>
#include <sensor_msgs/JoyFeedback.h>

// tf2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// RViz
#include <rviz_visual_tools/rviz_visual_tools.h>

// Eigen
#include "tf2_eigen/tf2_eigen.h"
#include <Eigen/Dense>

// Sophus - C++ implementation of Lie Groups using Eigen
#include <sophus/se3.hpp>

#include "sophus_ros_conversions/eigen.hpp"
#include "sophus_ros_conversions/geometry.hpp"

#include "test/ceres/local_parameterization_se3.hpp"

// Ceres NLS solver solver
#include <ceres/ceres.h>


// Handle signal [ctrl + c]
bool sigint_flag = true;

void IntHandler(int signal) {
    sigint_flag = false;
}


enum E_CalibrationStates {
    STATE_SPHERE_POINTS = 0,
    STATE_CHECKERBOARD_POINTS
};

namespace Eigen {
    namespace internal {
        template <class T, int N, typename NewType>
            struct cast_impl<ceres::Jet<T, N>, NewType> {
            EIGEN_DEVICE_FUNC
            
            static inline NewType run(ceres::Jet<T, N> const& x) {
                return static_cast<NewType>(x.a);
            }
        };
    }  // namespace internal
}  // namespace Eigen

struct SphereCostFunctor {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    SphereCostFunctor(Eigen::Vector3d SurfacePoint) : SurfacePoint(SurfacePoint) {}

    template <class T>
    bool operator()(T const* const sCenterPoint,
                    T const* const sRadius,
                    T* sResiduals) const
    {
        using Vector3T = Eigen::Matrix<T, 3, 1>;
        Eigen::Map<Vector3T const> const CenterPoint(sCenterPoint);

        sResiduals[0] = (SurfacePoint.cast<T>() - CenterPoint).squaredNorm() - T(sRadius[0])*T(sRadius[0]);

        return true;
    }

    Eigen::Vector3d SurfacePoint;
};

struct CheckerboardCostFunctor {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CheckerboardCostFunctor(Sophus::SE3d T_s, Sophus::SE3d::Point p_t) : T_s(T_s), p_t(p_t) {}

    template <class T>
    bool operator()(T const* const sT_x, T* sResiduals) const
    {
        Eigen::Map<Sophus::SE3<T> const> const T_x(sT_x);
        Eigen::Map<Eigen::Matrix<T, 3, 1> > residuals(sResiduals);

        // residuals = ((T_a.cast<T>()*T_x).inverse()*(T_x*T_b.cast<T>() ) ).log();
        residuals = T_s.cast<T>() * T_x * p_t.cast<T>() - T_x.inverse() * T_s.translation().cast<T>();

        return true;
    }

    Sophus::SE3d T_s;
    Sophus::SE3d::Point p_t;
};


class ToolCalibratingNode {
    ros::NodeHandle nh_;
    ros::Rate loop_rate_;

    // Publishers
    ros::Publisher joy_feedback_pub_;

    // Subscribers
    ros::Subscriber devices_sub_;
    ros::Subscriber joy_sub_;

    void DevicesCb(const vive_bridge::TrackedDevicesStamped& msg_);
    void JoyCb(const sensor_msgs::Joy& msg_);

    // ROS msgs
    geometry_msgs::TransformStamped tf_msg_, tf_msg_tool_;
    sensor_msgs::JoyFeedback joy_feedback_msg_;

    // RViz
    rviz_visual_tools::RvizVisualToolsPtr rviz_tools_, rviz_mesh_tools_;

    // tf2
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener *tf_listener_;
    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

    std::string controller_frame, tool_frame, tracker_frame, world_frame;
    unsigned int controller_id, tracker_id;

    // Eigen
    Eigen::Affine3d eigen_msg_, eigen_tool_;

    std::vector<Eigen::Vector3d> points_;
    Eigen::Vector3d eigen_point_, eigen_c_, vecs_[3], basis_[3];
    double radius;

    Sophus::SE3d T_x;
    std::vector<Sophus::SE3d> tracker_poses_;
    std::vector<Sophus::SE3d::Point> checkerboard_points_;

    unsigned int state;

    public:
        ToolCalibratingNode(int frequency);
        ~ToolCalibratingNode();

        bool Init();
        void Loop();
        void Shutdown();
};

void ToolCalibratingNode::DevicesCb(const vive_bridge::TrackedDevicesStamped& msg_) {
    /**
     * Update information about the currently tracked devices.
     */

    for (int i = 0; i < msg_.device_count; i++) {
        if (msg_.device_classes[i] == msg_.CONTROLLER) {
            controller_frame = msg_.device_frames[i];
            controller_id = i;
        }
        if (msg_.device_classes[i] == msg_.TRACKER) {
            tracker_frame = msg_.device_frames[i];
            tracker_id = i;
        }
    }
}

ToolCalibratingNode::ToolCalibratingNode(int frequency)
    : loop_rate_(frequency),
      static_tf_broadcaster_(),
      tf_listener_(new tf2_ros::TransformListener(tf_buffer_) )
{
    // Publishers
    joy_feedback_pub_ = nh_.advertise<sensor_msgs::JoyFeedback>("/vive_node/joy/haptic_feedback", 10, true);
    // Subscribers
    devices_sub_ = nh_.subscribe("/vive_node/tracked_devices", 1, &ToolCalibratingNode::DevicesCb, this);

    // RViz
    rviz_tools_.reset(new rviz_visual_tools::RvizVisualTools(world_frame, "/rviz_visual_markers") );
    rviz_mesh_tools_.reset(new rviz_visual_tools::RvizVisualTools(world_frame, "/vive_node/rviz_mesh_markers") );

    // Define joy feedback message
    joy_feedback_msg_.type = joy_feedback_msg_.TYPE_RUMBLE;
    joy_feedback_msg_.intensity = .1;

    state = STATE_SPHERE_POINTS;

    eigen_tool_.setIdentity();

    const double W = 0.02;
    const double L = 0.02;
    const int m = 3;
    const int n = 9;

    checkerboard_points_.resize(m*n);
    for (int j = 0; j < m; j++) {
        for (int i = 0; i < n; i++) {
            checkerboard_points_[j*n + i] << L*i, W*j, 0;
        }
    }
}
ToolCalibratingNode::~ToolCalibratingNode() {
}

void ToolCalibratingNode::JoyCb(const sensor_msgs::Joy& msg_) {
      /**
     * Handle VIVE Controller inputs
     */

    if (msg_.axes[0] + msg_.axes[1] + msg_.axes[2] == 0.) {
        if (msg_.buttons[1]) { // Grip button
            std::string pError;
            if (tf_buffer_.canTransform(world_frame, tracker_frame, ros::Time(0), &pError) ) {
                // Get tracked device location from tf server
                tf_msg_ = tf_buffer_.lookupTransform(world_frame, tracker_frame, ros::Time(0) );
                eigen_msg_ = tf2::transformToEigen(tf_msg_);
                eigen_point_ = eigen_msg_.translation();

                // Trigger controller haptic feedback
                joy_feedback_pub_.publish(joy_feedback_msg_);

                // Publish sphere at sampled point
                rviz_tools_->publishSphere(eigen_point_, rviz_visual_tools::BLUE, rviz_visual_tools::MEDIUM);
                rviz_tools_->trigger();

                if (state == STATE_SPHERE_POINTS) {
                    points_.push_back(eigen_point_);
                    int n_points = points_.size();
                    ROS_INFO_STREAM("Point " << n_points << ": \n" << eigen_point_.matrix() );

                    if (n_points >= 4) {
                        // Compute solver seed from linear solution
                        if (n_points == 4) {
                            Eigen::Matrix4d eigen_A_;
                            eigen_A_ << points_[0].transpose(), 1,
                                        points_[1].transpose(), 1,
                                        points_[2].transpose(), 1,
                                        points_[3].transpose(), 1;

                            Eigen::Vector4d eigen_b_;
                            eigen_b_ << points_[0].squaredNorm(),
                                        points_[1].squaredNorm(),
                                        points_[2].squaredNorm(),
                                        points_[3].squaredNorm();
                            
                            Eigen::Vector4d eigen_x_ = eigen_A_.fullPivHouseholderQr().solve(eigen_b_);
                            eigen_c_ = -0.5*eigen_x_.head<3>();
                            radius = 0.5*std::sqrt(eigen_c_.squaredNorm() - 4*eigen_x_(3) );
                        }

                        // Ceres NLS solver
                        ceres::Problem ceres_problem;
                        ceres::Solver::Options ceres_options;
                        ceres::Solver::Summary ceres_summary;

                        // Residual blocks
                        for (std::vector<Eigen::Vector3d>::iterator it_ = points_.begin(); it_ != points_.end(); ++it_) {
                            ceres::CostFunction* cost_function =
                                new ceres::AutoDiffCostFunction<SphereCostFunctor, 1, 3, 1>
                                                                (new SphereCostFunctor(*it_) );
                            ceres_problem.AddResidualBlock(cost_function, NULL, eigen_c_.data(), &radius);
                        }

                        // Set solver options
                        ceres_options.linear_solver_type = ceres::DENSE_SCHUR;

                        // Solve NLS problem
                        Solve(ceres_options, &ceres_problem, &ceres_summary);
                        ROS_INFO_STREAM(ceres_summary.FullReport() << std::endl);

                        // for (std::vector<Eigen::Vector3d>::iterator it_ = points_.begin(); it_ != points_.end(); ++it_) {
                        //     eigen_tool_.translation() += eigen_msg_.rotation().inverse()*(eigen_c_ - *it_);
                        // }
                        // eigen_tool_.translation() /= n_points;

                        if (n_points > 4) {
                            eigen_tool_.translation() += (eigen_msg_.rotation().inverse()*(eigen_c_ - eigen_point_) - 
                                                          eigen_tool_.translation() ) / n_points;
                        } else {
                            eigen_tool_.translation() = eigen_msg_.rotation().inverse()*(eigen_c_ - eigen_point_);
                        }
                        
                        // eigen_tool_.translation() = Eigen::Vector3d(0., 0., std::abs(radius) );
                        tf_msg_tool_.transform = tf2::eigenToTransform(eigen_tool_).transform;
                        static_tf_broadcaster_.sendTransform(tf_msg_tool_);

                        T_x = sophus_ros_conversions::transformMsgToSophus(tf_msg_tool_.transform).cast<double>();

                        ROS_INFO_STREAM("Sphere center point (relative to tracker): \n" << eigen_tool_.translation().matrix() );
                        ROS_INFO_STREAM("Sphere radius: " << eigen_tool_.translation().norm() );
                        // ROS_INFO_STREAM("Sphere radius: " << std::abs(radius) );

                        rviz_tools_->deleteAllMarkers();
                        rviz_tools_->publishSphere(eigen_c_, rviz_visual_tools::BLUE, 2*radius);
                        rviz_tools_->trigger();

                        if (n_points == 4) {
                            // Publish tool mesh
                            rviz_mesh_tools_->publishMesh(Eigen::Affine3d::Identity(),
                                                        "package://vive_calibrating/meshes/spike.dae",
                                                        rviz_visual_tools::BLACK,
                                                        1,
                                                        tool_frame);
                            rviz_mesh_tools_->trigger();
                        }
                    }
                }

                if (state == STATE_CHECKERBOARD_POINTS) {
                    tracker_poses_.push_back(sophus_ros_conversions::transformMsgToSophus(tf_msg_.transform).cast<double>() );
                    ROS_INFO_STREAM("Pose " << tracker_poses_.size() << "/" << checkerboard_points_.size() << ":");

                    rviz_tools_->publishSphere((eigen_msg_ * eigen_tool_).translation(), rviz_visual_tools::BLUE, rviz_visual_tools::XLARGE);

                    if (tracker_poses_.size() == checkerboard_points_.size() ) {
                        // Ceres NLS solver
                        ceres::Problem ceres_problem;
                        ceres::Solver::Options ceres_options;
                        ceres::Solver::Summary ceres_summary;

                        ceres_problem.AddParameterBlock(T_x.data(), Sophus::SE3d::num_parameters,
                                                        new Sophus::test::LocalParameterizationSE3);

                        // Residual blocks
                        for (int i = 0; i < tracker_poses_.size(); i++) {
                            ceres::CostFunction* cost_function =
                                new ceres::AutoDiffCostFunction<CheckerboardCostFunctor, 3, Sophus::SE3d::num_parameters>
                                                                (new CheckerboardCostFunctor(tracker_poses_[i],
                                                                                             checkerboard_points_[i]) );
                            ceres_problem.AddResidualBlock(cost_function, NULL, T_x.data() );
                        }

                        // Set solver options
                        ceres_options.linear_solver_type = ceres::DENSE_SCHUR;

                        // Solve NLS problem
                        Solve(ceres_options, &ceres_problem, &ceres_summary);
                        ROS_INFO_STREAM(ceres_summary.FullReport() << std::endl);

                        tf_msg_tool_.transform = sophus_ros_conversions::sophusToTransformMsg(T_x.cast<float>() );
                        tf_msg_tool_.header.stamp = ros::Time::now();

                        static_tf_broadcaster_.sendTransform(tf_msg_tool_);
                        ROS_INFO_STREAM(tf_msg_tool_);
                    }
                }
            } else {
                ROS_WARN_STREAM("Can't transform from " + world_frame + " to " + tracker_frame + ": " + pError);
            }
        }
    }

    if (msg_.buttons[0]) { // Menu button
        if (state == STATE_SPHERE_POINTS) {
            if (points_.size() >= 4) {
                ROS_INFO_STREAM("Define checkerboard points");
                state = STATE_CHECKERBOARD_POINTS;
            }
        }
    }

    // if (msg_.buttons[3]) { // Trigger button
    //     joy_feedback_pub_.publish(joy_feedback_msg_);

    //     points_.push_back(eigen_point_);
    //     ROS_INFO_STREAM("Point " << n_points << ": \n" << eigen_point_.matrix() );
    // }
}

bool ToolCalibratingNode::Init() {
      /**
     * Check if the necessary transforms are available and initialize the node
     */

    nh_.param<std::string>("/vive_node/world_frame", world_frame, "root");

    // Get available controller
    while ((controller_frame.empty() || tracker_frame.empty() ) && sigint_flag) {
        ROS_INFO("Waiting for controller and tracker...");
        
        ros::spinOnce();
        ros::Duration(3.0).sleep();
    }

    // Handle sigint
    if (!sigint_flag) {
        return false;
    }

    ROS_INFO_STREAM("Using " + controller_frame + " and " + tracker_frame + " for calibration");

    // Subscribe to joy topic
    joy_sub_ = nh_.subscribe("/vive_node/joy/" + controller_frame, 1, &ToolCalibratingNode::JoyCb, this);

    // Check if transforms are available
    std::string pError;
    if (!tf_buffer_.canTransform(controller_frame,
                                 world_frame, ros::Time(0),
                                 ros::Duration(10.0), &pError) )
    {
        ROS_ERROR_STREAM("Can't transform from " + world_frame + " to " + controller_frame + ": " + pError);

        return false;
    }
    if (!tf_buffer_.canTransform(tracker_frame,
                                    world_frame, ros::Time(0),
                                    ros::Duration(0.), &pError) )
    {
        ROS_WARN_STREAM("Can't transform from " + world_frame + " to " + tracker_frame + ": " + pError);

        return false;
    }

    joy_feedback_msg_.id = controller_id;

    tool_frame = tracker_frame + "_tool0";
    tf_msg_tool_.header.frame_id = tracker_frame;
    tf_msg_tool_.child_frame_id = tool_frame;

    // Limited marker lifetime
    rviz_tools_->setAlpha(0.5);
    rviz_tools_->setLifetime(0.);

    rviz_tools_->setBaseFrame(tracker_frame);
    rviz_tools_->enableFrameLocking();

    // Publish red sphere at tracker's position
    rviz_tools_->publishSphere(Eigen::Vector3d(0., 0., 0.), rviz_visual_tools::RED, rviz_visual_tools::XLARGE);
    rviz_tools_->trigger();

    rviz_tools_->setBaseFrame(world_frame);
    rviz_tools_->enableFrameLocking(false);

    rviz_mesh_tools_->loadMarkerPub(false, true);
    rviz_mesh_tools_->enableFrameLocking();
    rviz_mesh_tools_->setBaseFrame(tool_frame);
    rviz_mesh_tools_->setLifetime(0.);
    
    return true;
}

void ToolCalibratingNode::Loop() {
    ros::spinOnce();
    loop_rate_.sleep();
}
void ToolCalibratingNode::Shutdown() {
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "tool_calibration_node");

    // Handle signal [ctrl + c]
    signal(SIGINT, IntHandler);

    ToolCalibratingNode node_(60);

    if (!node_.Init() ) {
        node_.Shutdown();

        // Handle sigint
        if (sigint_flag) {
            exit(EXIT_SUCCESS);
        } else {
            exit(EXIT_FAILURE);
        }
    }

    while (ros::ok() && sigint_flag) {
        node_.Loop();
    }

    node_.Shutdown();
    exit(EXIT_SUCCESS);
}