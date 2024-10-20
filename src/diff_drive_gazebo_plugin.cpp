#include "gazebo_diff_drive_plugin/diff_drive_gazebo_plugin.hpp"
#include <gazebo_ros/node.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace gazebo
{
    DiffDriveGazeboPlugin::DiffDriveGazeboPlugin() : ModelPlugin(),
        left_wheel_velocity_(0.0), right_wheel_velocity_(0.0),
        x_pos_(0.0), y_pos_(0.0), theta_(0.0), prev_time_(0.0),
        clock(RCL_ROS_TIME)  {}
        

    DiffDriveGazeboPlugin::~DiffDriveGazeboPlugin() {}

    void DiffDriveGazeboPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        // Initialize ROS node
        ros_node_ = gazebo_ros::Node::Get(_sdf);


        // Load parameters from URDF/SDF
        if (_sdf->HasElement("wheel_radius"))
            wheel_radius_ = _sdf->Get<double>("wheel_radius");
        else
            wheel_radius_ = 0.165;  // Default value

        if (_sdf->HasElement("wheel_separation"))
            wheel_separation_ = _sdf->Get<double>("wheel_separation");
        else
            wheel_separation_ = 0.6;  // Default value

        // Get joint pointers from Gazebo model
        left_wheel_joint_ = _model->GetJoint("left_wheel_joint");
        right_wheel_joint_ = _model->GetJoint("right_wheel_joint");

        // Subscribe to cmd_vel topic
        cmd_vel_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&DiffDriveGazeboPlugin::cmdVelCallback, this, std::placeholders::_1));

        // Publisher for odometry
        odom_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        joint_state_pub_ = ros_node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        // TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node_);

        // Connect the update event to Gazebo's update cycle
        update_connection_ = event::Events::ConnectWorldUpdateBegin(
            std::bind(&DiffDriveGazeboPlugin::OnUpdate, this));

        prev_time_ = ros_node_->get_clock()->now().seconds();

    }

    void DiffDriveGazeboPlugin::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Calculate wheel velocities from cmd_vel
        double linear_velocity = msg->linear.x;
        double angular_velocity = msg->angular.z;

        left_wheel_velocity_ = (2 * linear_velocity - angular_velocity * wheel_separation_) / (2 * wheel_radius_);
        right_wheel_velocity_ = (2 * linear_velocity + angular_velocity * wheel_separation_) / (2 * wheel_radius_);
    }

    void DiffDriveGazeboPlugin::OnUpdate()
    {
        // Apply wheel velocities in Gazebo
        left_wheel_joint_->SetVelocity(0, left_wheel_velocity_);
        right_wheel_joint_->SetVelocity(0, right_wheel_velocity_);

        // Update robot's odometry
        now_ = ros_node_->get_clock()->now();
        double current_time = now_.seconds();
        double delta_time = current_time - prev_time_;
        prev_time_ = current_time;

        // Update pose and orientation
        double delta_x = (left_wheel_velocity_ + right_wheel_velocity_) / 2.0 * delta_time;
        double delta_theta = (right_wheel_velocity_ - left_wheel_velocity_) / wheel_separation_ * delta_time;

        x_pos_ += delta_x * cos(theta_ + delta_theta / 2.0);
        y_pos_ += delta_x * sin(theta_ + delta_theta / 2.0);
        theta_ += delta_theta;

        // Publish odometry
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now_;
        odom_msg.header.frame_id = "odom";
        odom_msg.pose.pose.position.x = x_pos_;
        odom_msg.pose.pose.position.y = y_pos_;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom_msg.pose.pose.orientation.x = q.x();
        odom_msg.pose.pose.orientation.y = q.y();
        odom_msg.pose.pose.orientation.z = q.z();
        odom_msg.pose.pose.orientation.w = q.w();
        odom_pub_->publish(odom_msg);

        // TF Transform
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = now_;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = x_pos_;
        transform.transform.translation.y = y_pos_;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        // Publish TF
        tf_broadcaster_->sendTransform(transform);

        // Joint State message
        sensor_msgs::msg::JointState joint_state_msg;
        joint_state_msg.header.stamp = now_;
        joint_state_msg.name = {"left_wheel_joint", "right_wheel_joint"};

        // Calculate wheel positions
        left_wheel_position_ += left_wheel_velocity_ * delta_time;
        right_wheel_position_ += right_wheel_velocity_ * delta_time;

        // Add position and velocity information
        joint_state_msg.position = {left_wheel_position_, right_wheel_position_};
        joint_state_msg.velocity = {left_wheel_velocity_, right_wheel_velocity_};

        // Publish joint state
        joint_state_pub_->publish(joint_state_msg);
    }
}

// Register plugin with Gazebo
GZ_REGISTER_MODEL_PLUGIN(gazebo::DiffDriveGazeboPlugin)
