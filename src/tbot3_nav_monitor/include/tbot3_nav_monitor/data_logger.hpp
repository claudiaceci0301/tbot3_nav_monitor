#ifndef DATA_LOGGER_HPP
#define DATA_LOGGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "tbot3_nav_monitor/msg/navigation_metrics.hpp"

#include <vector>
#include <string>
#include <memory>

namespace tbot3_nav_monitor
{

/**
    * @brief rclcpp Node that collects data and save them into csv files with timestamp
    *       - CSV logging of performance data with timestamps (time,planner_ok,controller_ok,tf_delay_ms,odom_rate,latency_ms)
    *       - RViz2 plugin or custom panel displaying performance metrics and system status
    *       - Alternative: Foxglove Studio dashboard for real-time monitoring
    *       - Alert system for performance degradation (console warnings or visual indicators)
    *       - Bonus: Custom web-based dashboard accessible outside the containerg
    *
*/

class DataLogger : public rclccpp::Node
{
public:
    /// @brief Constructor 
    /// @param node_name Name of the ROS2 node
    /// @param options If I pass an option it will take it, otherwise it will take the default NodeOptions
    explicit DataLogger(const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    /// @brief Destructor
    ~DataLogger() = default;

private:
    // ── Runtime parameters (loaded in constructor) ───────────────────────────

 

    // ── Sensor / command cache ───────────────────────────────────────────────



    // ── Status flags ────────────────────────────────────────────────────────



    // ── Subscriber / publisher / timer ──────────────────────────────────────

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr    odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr  cmd_vel_sub_;

    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<tbot3_nav_monitor::msg::NavigationMetrics>> metrics_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // ── Private methods ──────────────────────────────────────────────────────


};

}  // namespace tbot3_nav_monitor

#endif  // DATA_LOGGER_HPP
