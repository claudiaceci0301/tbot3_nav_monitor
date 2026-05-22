#ifndef ADAPTIVE_CONTROLLER_HPP
#define ADAPTIVE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include "tbot3_nav_monitor/msg/navigation_metrics.hpp"

#include <vector>
#include <string>
#include <memory>

namespace tbot3_nav_monitor
{

/**
    * @brief LifecycleNode that based on collected metrics, implement dynamic parameter adjustments:
    *   - Reduce maximum velocity when frequent recovery behaviors occur
    *   - Increase goal tolerance when navigation accuracy is consistently poor
    *   - Switch to more conservative path planning when obstacle avoidance is inefficient
    *   - Adjust local costmap parameters based on environment complexity
    *   -> This node will read the metrics then adjusts them 
    *   -> And finally, it will send parameter modification commands to the Nav2 nodes
    *
    * Lifecycle transitions:
    *   unconfigured → configured → active → deactivated → finalized
*/

class AdaptiveController : public rclcpp_lifecycle::LifecycleNode
{
public:
    /// @brief Constructor
    /// @param node_name Name of the ROS2 node
    explicit AdaptiveController(const std::string & node_name);

    /// @brief Destructor
    ~AdaptiveController() = default;

protected:
    // ── Lifecycle callbacks ──────────────────────────────────────────────────

    /// @brief unconfigured → configured: create clients and subscriber
    /// @param state Current state of the node
    /// @return SUCCESS or FAILURE
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State & state) override;

    /// @brief configured → active: log info 
    /// @param state Current state of the node
    /// @return SUCCESS or FAILURE
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State & state) override;

    /// @brief active → deactivated: log info 
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
    // ── Private parameters ───────────────────────────────────────────────────
    int recovery_threshold_;            ///< Es. after 3 recovery →  Reduce maximum velocity
    double accuracy_threshold_;         ///< Es. accurancy low -> Increase goal tolerance
    double efficiency_threshold_;       ///< Switch to more conservative path planning when obstacle avoidance is inefficient
    int window_size_;                   ///< Es. after 10 msgs
    int window_count_           = 0;    ///< Window for a stream of msg
    double optimal_path_        = 0.0;  ///< Euclidean distance start → goal
    double efficiency_          = 0.0;  ///< Distance_traveled / optimal_path -> [0, 1]
    double mean_accuracy_       = 0.0;  ///< Mean Accurancy (depends on the window size)
    double sum_accuracy_        = 0.0;  ///< Sum Accurancy

    // ── Subscriber and Clients to Nav2 nodes ─────────────────────────────────
    rclcpp::Subscription<tbot3_nav_monitor::msg::NavigationMetrics>::SharedPtr metrics_sub_;
    rclcpp::AsyncParametersClient::SharedPtr controller_client_;
    rclcpp::AsyncParametersClient::SharedPtr costmap_client_;
    rclcpp::AsyncParametersClient::SharedPtr planner_client_;
   
    // ── Private methods ──────────────────────────────────────────────────────
    /// @brief Adaptive controller callback: reads metric parameters
    /// @param msg NavigationMetrics Message
    void metrics_callback(const std::shared_ptr<const tbot3_nav_monitor::msg::NavigationMetrics> & msg);
};

}  // namespace tbot3_nav_monitor

#endif  // ADAPTIVE_CONTROLLER_HPP
