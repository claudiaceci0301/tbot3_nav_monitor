#ifndef MY_LIFECYCLE_NODE_HPP
#define MY_LIFECYCLE_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include "tbot3_nav_monitor/msg/navigationmetrics.hpp"

namespace tbot3_nav_monitor
{

/// @brief This class will implement a LifecycleNode to handle data
/// @parameters: unconfigured → configured → active → deactivated → finalized

class MetricCollector : public rclcpp_lifecycle::LifecycleNode
{
    public:
    /// @brief Constructor
    /// @param node_name name of ROS2 node
    explicit MetricCollector(const std::string & node_name);

    /// @brief Distructor
    ~MetricCollector() = default;

    protected:
    // ── Lifecycle callbacks ──

    /// @brief From unconfigured to configured
    /// @param current_state Current state of the node
    /// @return SUCCESS or FAILURE
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State & current_state) override;

    /// @brief Chiamato nella transizione configured → active
    /// @param current_state Current state of the node
    /// @return SUCCESS o FAILURE
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State& current_state) override;

    /// @brief From active to deactivated
    /// @param current_state Current state of the node
    /// @return SUCCESS o FAILURE
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & current_state) override;

    /// @brief From something to unconfigured
    /// @param current_state Current state of the node
    /// @return SUCCESS o FAILURE
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State & current_state) override;

    /// @brief From something to finalized
    /// @param current_state Current state of the node
    /// @return SUCCESS o FAILURE
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State & current_state) override;


    private:
    // ── Private members ──

    /// @brief Node name
    std::string node_name_;

    /// @brief Initial distance travelled
    double distance_traveled_ = 0.0;
    
    /// @brief Total Battery 100%
    double battery_level_ = 100.0; 

    /// @brief Initial minimum distance from an obstacle
    double min_distance_obstacle_;

    /// @brief Initial recovery behavior (how often robot gets stuck)
    int recovery_count_ = 0;

    /// @brief Initial current x and y robot position
    double current_;
    double theta_;

    /// @brief Previus odom x and y position used for checking the recovery robot mode
    double prev_odom_x_ = 0.0;
    double prev_odom_y_ = 0.0;

    /// @brief Initial cmd linear and angular velocity
    std::vector<double> cmd_linear_vel_;
    std::vector<double> cmd_angular_vel_;

    /// @brief Initial sensor data
    std::vector<double> sensor_ranges_;
    double sensor_range_min_;
    double sensor_range_max_;

    /// @brief booleans parameters
    goal_rached_ = false;
    odom_received_ = false;
    sensor_received_ = false;
    cmd_vel_received_ = false;
    odom_position_unchanged_ = false;

    /// @brief Callback to updates internal state, save pose, accumulates distance etc
    /// @param msg Message of odom type
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    /// @brief Callback to updates internal state, save pose, accumulates distance etc
    /// @param msg Message of laser scanner type
    void scanner_callback(const sensor_msgs::msgs::LaserScan::SharedPtr msg);

    /// @brief Callback to updates internal state, save pose, accumulates distance etc
    /// @param msg Message of geometry twist type
    void cmdvel_callback(const geometry_msgs::msgs::Twist::SharedPtr msg);

    /// @brief Callback to publish only if it is active
    void publish_metrics();

    /// @brief Method to normalize the angle received
    /// @param angle Angle to normalize
    /// @return Normalized angle [-90, 90] rad (es. 270° -> -90°)
    static double normalized_angle(double angle);

    /// @brief Obstacle avoidance method
    bool collision_detection();

    /// @brief ROS2 declaration
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<tbot3_nav_monitor::msg::NavigationMetrics>> metrics_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace tbot3_nav_monitor

#endif  // MY_LIFECYCLE_NODE_HPP