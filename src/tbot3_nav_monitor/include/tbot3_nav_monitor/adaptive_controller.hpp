#ifndef ADAPTIVE_CONTROLLER_HPP
#define ADAPTIVE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "tbot3_nav_monitor/msg/navigation_metrics.hpp"

#include <memory>
#include <string>

namespace tbot3_nav_monitor
{

/**
 * @brief LifecycleNode that based on collected metrics implements dynamic parameter adjustments:
 *   - Reduce maximum velocity when frequent recovery behaviors occur
 *   - Increase goal tolerance when navigation accuracy is consistently poor
 *   - Switch to more conservative path planning when obstacle avoidance is inefficient
 *   - Adjust local costmap parameters based on environment complexity
 *
 *   Flow:
 *     /navigation_metrics → metrics_callback → AsyncParametersClient → Nav2 nodes
 *
 *   Lifecycle transitions:
 *      unconfigured → configured → active → deactivated → finalized
 */
class AdaptiveController : public rclcpp_lifecycle::LifecycleNode
{
public:
    /// @brief Constructor
    /// @param node_name Name of the ROS2 node
    /// @param options If I pass an option it will take it, otherwise it will take the default NodeOptions
    explicit AdaptiveController(const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    
    /// @brief Destructor
    ~AdaptiveController() = default;

protected:
    // ── Lifecycle callbacks ──────────────────────────────────────────────────

    /// @brief unconfigured → configured: create clients and subscriber
    /// @param state Current state of the node
    /// @return SUCCESS or FAILURE
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State & state) override;

    /// @brief configured → active: log only (no publisher or timer)
    /// @param state Current state of the node
    /// @return SUCCESS or FAILURE
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & state) override;

    /// @brief active → deactivated: log only
    /// @param state Current state of the node
    /// @return SUCCESS or FAILURE
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State & state) override;

    /// @brief any → unconfigured: release all resources and reset state
    /// @param state Current state of the node
    /// @return SUCCESS or FAILURE
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State & state) override;

private:
    // ── Thresholds ───────────────────────────────────────────────────────────
    int    recovery_threshold_;     ///< Max recovery count before reducing velocity
    double accuracy_threshold_;     ///< Min mean accuracy before relaxing goal tolerance
    double efficiency_threshold_;   ///< Min efficiency [0,1] before switching to conservative plan
    double obstacle_threshold_;     ///< Min mean obstacle proximity before adjusting costmap
    int    window_size_;            ///< Number of messages per evaluation window

    // ── Window state ─────────────────────────────────────────────────────────
    int    window_count_             = 0;   ///< Messages accumulated in current window
    double sum_accuracy_             = 0.0; ///< Accuracy accumulator for current window
    double sum_obstacle_proximity_   = 0.0; ///< Obstacle proximity accumulator for current window
    double mean_accuracy_            = 0.0; ///< Mean accuracy over last window
    double mean_obstacle_proximity_  = 0.0; ///< Mean obstacle proximity over last window

    // ── Efficiency ───────────────────────────────────────────────────────────
    double efficiency_               = 0.0; ///< optimal_path / distance_traveled → [0, 1]

    // ── Subscriber and Nav2 parameter clients ────────────────────────────────
    rclcpp::Subscription<tbot3_nav_monitor::msg::NavigationMetrics>::SharedPtr metrics_sub_;
    rclcpp::AsyncParametersClient::SharedPtr controller_client_;
    rclcpp::AsyncParametersClient::SharedPtr costmap_client_;
    rclcpp::AsyncParametersClient::SharedPtr planner_client_;

    // ── Private methods ──────────────────────────────────────────────────────

    /// @brief Reads NavigationMetrics and applies adaptive logic
    /// @param msg Incoming NavigationMetrics message
    void metrics_callback(
        const std::shared_ptr<const tbot3_nav_monitor::msg::NavigationMetrics> & msg);
};

}  // namespace tbot3_nav_monitor

#endif  // ADAPTIVE_CONTROLLER_HPP
