#ifndef METRIC_COLLECTOR_HPP
#define METRIC_COLLECTOR_HPP

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

/// @brief Struct to hold a 2D pose (x, y, theta)
struct Pose2D
{
    double x     = 0.0;
    double y     = 0.0;
    double theta = 0.0;
};

/**
    * @brief LifecycleNode that collects and publishes navigation metrics:
    *        - Distance travelled
    *        - Battery consumption (simulated)
    *        - Minimum obstacle distance
    *        - Recovery behavior count
    *        - Goal reached flag
    *
    * Lifecycle transitions:
    *   unconfigured → configured → active → deactivated → finalized
*/

class MetricCollector : public rclcpp_lifecycle::LifecycleNode
{
public:
    /// @brief Constructor 
    /// @param node_name Name of the ROS2 node
    explicit MetricCollector(const std::string & node_name);

    /// @brief Destructor
    ~MetricCollector() = default;

protected:
    // ── Lifecycle callbacks ──────────────────────────────────────────────────

    /// @brief unconfigured → configured: create pub/sub, declare parameters
    /// @param state Current state of the node
    /// @return SUCCESS or FAILURE
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State & state) override;

    /// @brief configured → active: activate publisher, start timer
    /// @param state Current state of the node
    /// @return SUCCESS or FAILURE
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & state) override;

    /// @brief active → deactivated: deactivate publisher, cancel timer
    /// @param state Current state of the node
    /// @return SUCCESS or FAILURE
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & state) override;

    /// @brief any → unconfigured: release all resources
    /// @param state Current state of the node
    /// @return SUCCESS or FAILURE
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State & state) override;

private:
    // ── Runtime parameters (loaded in constructor) ───────────────────────────

    double publish_rate_;                   ///< Timer frequency [Hz]
    double battery_drain_rate_;             ///< Battery drain per metre travelled
    double distance_tolerance_;             ///< Linear goal tolerance [m]
    double obstacle_distance_tolerance_;    ///< Minimum safe obstacle distance [m]
    double angle_tolerance_;                ///< Angular goal tolerance [rad]
    double max_linear_vel_;                 ///< Maximum linear velocity [m/s]
    double max_angular_vel_;                ///< Maximum angular velocity [rad/s]
    double linear_gain_;                    ///< Proportional gain – linear
    double angular_gain_;                   ///< Proportional gain – angular

    // ── Navigation state ────────────────────────────────────────────────────

    Pose2D current_;                        ///< Current robot pose
    Pose2D target_;                         ///< Target pose
    Pose2D start_;                          ///< Initial robot pose (used to compute optimal_path_)

    double distance_traveled_  = 0.0;       ///< Cumulative distance travelled [m]
    double optimal_path_ = 0.0;             ///< Euclidean distance start → goal [m]
    double battery_level_      = 100.0;     ///< Remaining battery [%]
    double battery_consumption_ = 0.0;      ///< Consumed battery [%]
    double min_distance_obstacle_ = std::numeric_limits<double>::max(); ///< Min obstacle distance [m]

    int  recovery_count_ = 0;               ///< Number of recovery events

    double prev_odom_x_ = 0.0;             ///< Previous odometry x [m]
    double prev_odom_y_ = 0.0;             ///< Previous odometry y [m]

    // ── Sensor / command cache ───────────────────────────────────────────────

    std::vector<float> sensor_ranges_;      ///< Raw laser ranges (float, as in LaserScan)
    float sensor_range_min_ = 0.0f;         ///< Laser minimum valid range [m]
    float sensor_range_max_ = 0.0f;         ///< Laser maximum valid range [m]

    geometry_msgs::msg::Twist last_cmd_vel_; ///< Last received cmd_vel

    // ── Status flags ────────────────────────────────────────────────────────

    bool goal_reached_           = false;
    bool odom_received_          = false;
    bool sensor_received_        = false;
    bool cmd_vel_received_       = false;
    bool odom_position_unchanged_ = false;

    // ── Subscriber / publisher / timer ──────────────────────────────────────

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr    odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr  cmd_vel_sub_;

    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<tbot3_nav_monitor::msg::NavigationMetrics>> metrics_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // ── Private methods ──────────────────────────────────────────────────────

    /// @brief Odometry callback: updates current pose, accumulates distance
    /// @param msg Odometry Message
    void odom_callback(const std::shared_ptr<const nav_msgs::msg::Odometry> & msg);

    /// @brief LaserScan callback: caches ranges and valid-range bounds
    /// @param msg LaserScan Message
    void scanner_callback(const std::shared_ptr<const sensor_msgs::msg::LaserScan> & msg);

    /// @brief cmd_vel callback: caches the last commanded velocity
    /// @param msg Twist Message
    void cmdvel_callback(const std::shared_ptr<const geometry_msgs::msg::Twist> & msg);

    /// @brief Timer callback: runs one control iteration and publishes metrics
    void control_loop();

    /// @brief Checks for imminent collision using cached laser ranges.
    /// @return true if an obstacle is closer than obstacle_distance_tolerance_ (robot in collision)
    bool collision_detection();

    /// @brief Normalises an angle to [-π, π]
    /// @param angle Input angle [rad]
    /// @return Angle in [-π, π]
    static double normalize_angle(double angle);
};

}  // namespace tbot3_nav_monitor

#endif  // METRIC_COLLECTOR_HPP
