#define _USE_MATH_DEFINES

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <scanner_msgs/msg/laserscan.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "tbot3_nav_monitor/metrico_collector.hpp"

#include <chrono>
#include <vector>
#include <string>
#include <memory>
#include <cmath>


namespace tbot3_nav_monitor
{
    /**
    @brief This class will implement:
        - Path execution time from goal assignment to completion
        - Navigation accuracy (distance from target position)
        - Obstacle avoidance efficiency (path length vs optimal path)
        - Recovery behavior frequency (how often robot gets stuck)
        - Battery consumption simulation (fictional metric based on distance traveled)
    */ 


    MetricCollector::MetricCollector(const std::string & node_name) :
                                    rclcpp_lifecycle::LifecycleNode(node_name)
    {
        RCLCPP_INFO(get_logger(), "Node %s create: ", node_name_.c_str());

        // Parameters declaration - runtime parameters
        declare_parameter("publish_rate",       1.0); // Node will publish on the topic 1 time per second
        declare_parameter("battery_drain_rate", 0.01); // Battery consuption battery_level_ -= battery_drain_rate_ * distance_;
        declare_parameter("target_x",           0.85); // Target x turtle position
        declare_parameter("target_y",           0.70); // Target y turtle position
        declare_parameter("theta",              0.75); // Target angle theta
        declare_parameter("distance_tolerance", 0.05); // Tolerance distance for reaching the target position
        declare_parameter("obstacle_distance_tolerance", 0.05); // Tolerance distance from an obstacle
        declare_parameter("angle_tolerance",    0.05); // Same tolerance but for the angle
        declare_parameter("max_linear_vel",     0.5); // Maximum linear velocity
        declare_parameter("max_angular_vel",    3.0); // Maximum angular velocity
        declare_parameter("linear_gain",        0.5) // Linear gain for KP control loop
        declare_parameter("angular_gain",       2.0) // Angular gain for KP control loop
    
        // Getter for the declared parameters
        target_.x = get_parameter("target_x").as_double();
        target_.y = get_parameter("target_y").as_double();
        target_.theta = get_parameter("theta").as_double;
       
        distance_tolerance_          = get_parameter("distance_tolerance").as_double();
        obstacle_distance_tolerance_ = get_parameter("obstacle_distance_tolerance").as_double();
        angle_tolerance_             = get_parameter("angle_tolerance").as_double();
        max_linear_vel_              = get_parameter("max_linear_vel").as_double();
        max_angular_vel_             = get_parameter("max_angular_vel").as_double();
        linear_gain_                 = get_parameter("linear_gain").as_double();
        angular_gain_                = get_parameter("angular_gain").as_double();

        RCLCPP_INFO(this->get_logger(),
        "MetricCollector initialized | Target: (%.2f, %.2f)", target_.x, target_.y);

    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
    {
        LifecycleNode::on_configure(state);

        // Publisher and subscribere configuration, these will publish and subscribe in the on active config
        // Initialize publisher lifecycle and subscribers
        metric_pub_ = create_publisher<tbot3_nav_monitor::msg::NavigationMetrics>("/navigation_metrics", 10);
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
                    std::bind(&MetricCollector::odom_callback, this, std::placeholders::_1));
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/", 10,
                    std::bind(&MetricCollector::scanner_callback, this, std::placeholders::_1));
        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
                    std::bind(&MetricCollector::cmdvel_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(get_logger(), "on_configure() is called, Node is still INACTIVE");
        return CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
    {
        LifecycleNode::on_activate(state);

        // Now activate the publisher (the publisher must be activated explicitally)
        metric_pub_->on_activate();

        // Create the timer for publishing
        auto rate = get_parameter("publish_rate").as_double();
        timer_ = create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / rate)), 
                std::bind(MetricCollector::publish_metrics, this)); // The timer will publish 1 times per second

        RCLCPP_INFO(get_logger(), "on_activate() is called, Node is ACTIVE");
        return CallbackReturn::SUCCESS;

    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
    {
        LifecycleNode::on_deactivate(state);

        // Stop the timer and remove the output
        metric_pub_->on_deactivate(); // Stop publishing
        timer_->cancel(); // Stop the timer
        RCLCPP_INFO(get_logger(), "on_deactivate() is called");
        return CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override
    {
        LifecycleNode::on_cleanup(state);

        // Clean up the resources
        timer_->reset();
        metric_pub_->reset();
        odom_sub_->reset();
        scan_sub_->reset();
        cmd_vel_sub_->reset();
        distance_traveled_ = 0.0;

        RCLCPP_INFO(get_logger(), "on_cleanup() is called, everything has been resetted the node is UNCONFIGURED");
        return CallbackReturn::SUCCESS
    }

    // ── Callback methods ──
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr & msg)
    {
        // /odom [nav_msgs/Odometry]
        // - pose.pose.position.x/y  → calcolo distanza percorsa (differenza tra pose successive)
        // - twist.twist.linear.x    → velocità attuale
        
        // Updates Pose
        current_.x = msg->pose.pose.position.x;
        current_.y = msg->pose.pose.position.y;

        if(current_.x == prev_odom_x_ && current_.y == prev_odom_y_)
        {
            // The position has not changed - the robot is stucked
            odom_position_unchanged_ = true;
        }

        // Save previous position
        prev_odom_x_ = current_.x;
        prev_odom_y_ = current_.y;

        tf2::Quaternion q
        (
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        current_.theta = yaw;

        odom_received_ = true; // Set to true once received the odom msg
    }   

    double MetricCollector::MetricCollector normalize_angle(double angle)
    {
        while(angle > M_PI) angle -= 2 * M_PI; // M_PI = 180°
        while(angle < M_PI) angle += 2 * M_PI;
        return angle;
    }

    void scanner_callback(const sensor_msgs::msgs::LaserScan::SharedPtr & msg)
    {
        // - usato per: min_obstacle_distance, obstacle avoidance efficiency

        sensor_ranges_ = msg->ranges; // Laser distances for every angle
        sensor_range_min_ = msg->range_min;
        sensor_range max_ = msg->range_max;// values < min > max should be discarder
        sensor_received_ = true; // Set to true once received the laser msg
    }

    void cmdvel_callback(const geometry_msgs::msgs::Twist::SharedPtr & msg)
    {
        // - usato per: conferma movimento, motor load simulation
        cmd_linear_vel_  = {msg->linear.x,  msg->linear.y,  msg->linear.z};
        cmd_angular_vel_ = {msg->angular.x, msg->angular.y, msg->angular.z};
        cmd_vel_received_ = true;
    }

    void collision_detection()
    { 

        int left_range = static_cast<int>(sensor_ranges_.size() / 4); // Left obstacle detection
        int right_range = static_cast<int>(sensor_ranges_.size() * 3 / 4); // Right obstacle detection

        // Minimum distance on left frontal sector
        double left_min = *std::min_element( 
            sensor_ranges_.begin(),
            sensor_ranges_.begin() + left_range
        );

        // Minimum distance on right frontal sector
        double right_min = *std::min_element(
            sensor_ranges_.begin() + right_range,
            sensor_ranges_.end()
        );

        // Global frontal minimum distance
        min_distance_obstacle_ = std::min(left_min, right_min);

        geometry_msgs::msg::Twist twist;

        if (min_distance_obstacle_ < obstacle_distance_tolerance_)
        {
            // COLLISION
            twist.linear.x = 0.0;
            twist.angular.z = cmd_angular_vel_.z;

            RCLCPP_INFO(get_logger(), "Obstacle detected! Stop the robot!");
        }
        else
        {
            // No obstacle
            twist.linear = cmd_linear_vel_;
            twist.angular = cmd_angular_vel_;
        }
    }

    void publish_metrics()
    {
        if((!odom_received_ && !sensor_received_ && !cmd_vel_received_) || !goal_rached_)
        {
            // If everything is true stop the loop
            return;
        }   
        
        // - usato per: distance_traveled, battery_level, recovery detection
        // Angle and linear distance from the target
        const double dx = target_.x - current_.x;
        const double dy = target_.y - current_.y;
        const double angle_to_target = std::atan2(dy, dx) // The angle between the current and the target position
        const double angle_difference = normalize_angle(angle_to_target); // Normalized angle
        const double distance = std::sqrt(std::pow(dx, 2), std::pow(dy, 2))// Euclidean distance sqrt(dx²+dy²)

        geometry_msgs::msg::Twist cmd_vel; // Initially zero since the robot is not moving
        cmd_vel.linear = cmd_linear_vel_;
        cmd_vel.angular = cmd_angular_vel_;

        for(const auto & vel : cmd_vel)
        {
            // If cmd_vel != 0 and odom does not change → robot is stucked → recovery_count++
            if((vel.linear != 0.0 || vel.angular != 0.0) && !odom_position_unchanged_)
            {
                recovery_count_++; // Robot is stucked
            }
        }

        if(distance <= distance_tolerance_)
        {
            // GOAL REACHED !!!
            goal_rached_ = true;
            RCLCPP_INFO(this->get_logger(),
                "Goal reached! Final position: (%.3f, %.3f)", current_.x, current_.y);
            return;
        }
        else if() // It target not reached: rotation towards the target
        {

        }
        else // If target not reached: linear motion towards the target
        {

        }
        metric_pub_->publish(cmd_vel) // It publish the velocity

    }

} // namespace tbot3_nav_monitor