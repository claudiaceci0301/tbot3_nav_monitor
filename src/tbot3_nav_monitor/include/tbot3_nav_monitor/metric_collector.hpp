#ifndef METRIC_COLLECTOR_HPP
#define METRIC_COLLECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "tbot3_nav_monitor/msg/navigation_metrics.hpp"
#include "tbot3_nav_monitor/nav2_status.hpp"

#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <mutex> // for thread safe (avoid data race)
#include <vector>
#include <memory>
#include <cmath> // Per std::abs

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
    /// @param options If I pass an option it will take it, otherwise it will take the default NodeOptions
    explicit MetricCollector(const std::string & node_name,
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    
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
    double alpha_;                          ///< Alpha tuning parameter for exponential moving average

    // ── Navigation state ────────────────────────────────────────────────────
    Pose2D current_;                        ///< Current robot pose
    Pose2D target_;                         ///< Target pose
    Pose2D start_;                          ///< Initial robot pose (used to compute optimal_path_)

    // ── Private params ────────────────────────────────────────────────────
    double distance_traveled_  = 0.0;                                   ///< Cumulative distance travelled [m]
    double last_step_ = 0.0;                                            ///< Last step of the distance travelled [m]
    double optimal_path_ = 0.0;                                         ///< Euclidean distance start → goal [m]
    double battery_level_      = 100.0;                                 ///< Remaining battery [%]
    double battery_consumption_ = 0.0;                                  ///< Consumed battery [%]
    int    recovery_count_ = 0;                                         ///< Number of recovery events
    double prev_odom_x_ = 0.0;                                          ///< Previous odometry x [m]
    double prev_odom_y_ = 0.0;                                          ///< Previous odometry y [m]
    Nav2State nav2_state_{Nav2State::UNKNOWN};                          ///< Nav2 state param
    double min_distance_obstacle_ = std::numeric_limits<double>::max(); ///< Min obstacle distance [m]
    double smooth_distance_ = 0.0;                                      ///< Smooth distance for exponential moving average - Used for debug/visualization
    
    // ── Sensor / command cache ───────────────────────────────────────────────
    std::vector<float> sensor_ranges_;      ///< Raw laser ranges (float, as in LaserScan)
    float sensor_range_min_ = 0.0f;         ///< Laser minimum valid range [m]
    float sensor_range_max_ = 0.0f;         ///< Laser maximum valid range [m]

    geometry_msgs::msg::Twist last_cmd_vel_; ///< Last received cmd_vel

    // ── Status flags ────────────────────────────────────────────────────────
    bool goal_reached_            = false; ///< Bool to check if goal is achieved 
    bool odom_received_           = false; ///< Bool to check if the odom data has been received 
    bool sensor_received_         = false; ///< Bool to check if the sensor data has been received 
    bool cmd_vel_received_        = false; ///< Bool to check if the cmd_vel data has been received 
    bool odom_position_unchanged_ = false; ///< Bool to check if the odom position has changed

    // ── Subscriber / publisher / timer ──────────────────────────────────────
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr goal_status_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp_lifecycle::LifecyclePublisher<tbot3_nav_monitor::msg::NavigationMetrics>::SharedPtr metrics_pub_;
    rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    // ── Service ─────────────────────────────────────────────────────────────
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;

    // ── Mutex Thread-Safe ───────────────────────────────────────────────────
    mutable std::mutex state_mutex_; ///< Thread-Safe access variable

    // ── Buffer params ───────────────────────────────────────────────────────
    std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;              // Container that memorize all the transforms
    std::shared_ptr<tf2_ros::TransformListener> tf2_listener_; // Object that listen tf topics and fullfill the buffer

    // ── Private methods ─────────────────────────────────────────────────────

    /// @brief Normalizes angle to [-pi, pi]
    /// @param angle Angle to normalize [rad]
    /// @return Normalized angle [rad]
    double normalize_angle(double angle);

    /// @brief Odometry callback: updates current pose, accumulates distance
    /// @param msg Odometry Message
    void odom_callback(const std::shared_ptr<const nav_msgs::msg::Odometry> & msg);

    /// @brief LaserScan callback: caches ranges and valid-range bounds
    /// @param msg LaserScan Message
    void scanner_callback(const std::shared_ptr<const sensor_msgs::msg::LaserScan> & msg);

    /// @brief cmd_vel callback: caches the last commanded velocity
    /// @param msg Twist Message
    void cmdvel_callback(const std::shared_ptr<const geometry_msgs::msg::Twist> & msg);
 
    /// @brief Callback for subscribing the sent goal to the goal_pose topic
    /// @param msg PoseStamped message
    void goal_send_callback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> & msg);

    /// @brief Service Callback: reset goal state for next nav2 navigation
    /// @param response Response for the trigger of the next nav2 navigation
    void reset_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    /// @brief Timer callback: runs one control iteration and publishes metrics
    void control_loop();

    /// @brief Checks for imminent collision using cached laser ranges.
    /// @return true if an obstacle is closer than obstacle_distance_tolerance_ (robot in collision)
    bool collision_detection();
};

} // namespace tbot3_nav_monitor

#endif // METRIC_COLLECTOR_HPP