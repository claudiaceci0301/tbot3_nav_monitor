#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <chrono>
#include <vector>
#include <string>
#include <memory>

#include "tbot3_nav_monitor/metrico_collector.hpp"

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

        // Parameters declaration 
        declare_parameter("publish_rate", 1.0); // Node will publish on the topic 1 time per second
        declare_parameter("battery_drain_rate", 0.01); // Battery consuption battery_level_ -= battery_drain_rate_ * distance_;
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
        // - usato per: distance_traveled, battery_level, recovery detection
    }   

    void scanner_callback(const sensor_msgs::msgs::LaserScan::SharedPtr & msg)
    {
        // /scan [sensor_msgs/LaserScan]
        // - ranges[]                → distanze laser per ogni angolo
        // - range_min, range_max    → soglie per filtrare valori non validi
        // - usato per: min_obstacle_distance, obstacle avoidance efficiency
    }

    void cmdvel_callback(const geometry_msgs::msgs::Twist::SharedPtr & msg)
    {
        // /cmd_vel [geometry_msgs/Twist]
        // - linear.x, angular.z    → comandi inviati da Nav2 al robot
        // - usato per: conferma movimento, motor load simulation
        // - se cmd_vel != 0 ma odom non cambia → robot bloccato → recovery_count++
    }

} // namespace tbot3_nav_monitor