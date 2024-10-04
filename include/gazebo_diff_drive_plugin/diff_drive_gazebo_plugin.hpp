#ifndef DIFF_DRIVE_GAZEBO_PLUGIN_HPP_
#define DIFF_DRIVE_GAZEBO_PLUGIN_HPP_

#include <gazebo/common/Plugin.hh>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

namespace gazebo
{
    class DiffDriveGazeboPlugin : public ModelPlugin
    {
    public:
        DiffDriveGazeboPlugin();
        virtual ~DiffDriveGazeboPlugin();

        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
        void OnUpdate();

    private:
        void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

        // Robot parameters
        double wheel_radius_;
        double wheel_separation_;

        // Wheel joint references
        physics::JointPtr left_wheel_joint_;
        physics::JointPtr right_wheel_joint_;

        // ROS node and components
        gazebo_ros::Node::SharedPtr ros_node_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        // Gazebo update connection
        event::ConnectionPtr update_connection_;

        // State variables
        double left_wheel_velocity_;
        double right_wheel_velocity_;
        double x_pos_, y_pos_, theta_;
        double prev_time_;
    };
}

#endif // DIFF_DRIVE_GAZEBO_PLUGIN_HPP_
