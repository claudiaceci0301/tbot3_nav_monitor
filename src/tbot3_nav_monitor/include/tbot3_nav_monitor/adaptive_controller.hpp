#ifndef ADAPTIVE_CONTROLLER_HPP
#define ADAPTIVE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "tbot3_nav_monitor/msg/navigation_metrics.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <std_srvs/srv/trigger.hpp>

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
    // ── Prive Params ───────────────────────────────────────────────────────────
    int    recovery_threshold_;                  ///< Max recovery count before reducing velocity
    double accuracy_threshold_;                  ///< Min mean accuracy before relaxing goal tolerance
    double efficiency_threshold_;                ///< Min efficiency [0,1] before switching to conservative plan
    double obstacle_threshold_;                  ///< Min mean obstacle proximity before adjusting costmap
    int    window_size_;                         ///< Number of messages per evaluation window
    int    last_recovery_count_ = 0;             ///< Last read recovery count from MetricController
    bool   last_obstacle_too_close_ = false;     ///< Bool to get the last obstacle too close
    std::atomic<bool> navigation_active_{false}; ///< Atomic bool to activate the navigation
    int    window_count_             = 0;        ///< Messages accumulated in current window
    double sum_accuracy_             = 0.0;      ///< Accuracy accumulator for current window
    double sum_obstacle_proximity_   = 0.0;      ///< Obstacle proximity accumulator for current window
    double mean_accuracy_            = 0.0;      ///< Mean accuracy over last window
    double mean_obstacle_proximity_  = 0.0;      ///< Mean obstacle proximity over last window
    bool   window_ready_             = false;    ///< Bool for window ready, true when the window is full

    // ── Efficiency ───────────────────────────────────────────────────────────
    double efficiency_               = 0.0;   ///< optimal_path / distance_traveled → [0, 1]
    
    // ── Nav2 Params ──────────────────────────────────────────────────────────
    /**
        * @brief Nav2 Params - std::atomic 
            * - std::atomic ensures thread-safe access to the variables (no mutex needed for simple types)
            * - std::atomic prevents the data race
            * - .load() to read, .store() to write
            * - allows concurrent threads to read/write without waiting for each other
    */

    enum Nav2State : uint8_t
    {
        UNKNOWN = 0,
        SUCCEEDED = 1,
        ABORTED = 2,
        CANCELED = 3
    };

    std::atomic<Nav2State> nav2_state_{Nav2State::UNKNOWN}; // Default value

    // ── Default Nav2 parameter ───────────────────────────────────────────────
    struct Nav2Params
    {
        double max_vel_x;
        double max_vel_theta;
        double xy_goal_tolerance;
        double yaw_goal_tolerance;
        double inflation_radius;
        double gridbase_tolerance;
        double costmap_resolution;
        int    costmap_width;
        int    costmap_height;
    };
    
    // ── Private Time parameters : Diff and Rate Limit Check───────────────────
    rclcpp::Time last_controller_apply_time_{0,0,RCL_ROS_TIME}; ///< ROS2 time for the controller (seconds, nanoseconds, clocktype)
    rclcpp::Time last_costmap_apply_time_{0,0,RCL_ROS_TIME};    ///< ROS2 time for the costmap
    rclcpp::Time last_planner_apply_time_{0,0,RCL_ROS_TIME};    ///< ROS2 time for the planner
    Nav2Params last_controller_params_{};                       ///< Last param sent to Nav2 controller
    Nav2Params last_costmap_params_{};                          ///< Last param sent to Nav2 costmap
    Nav2Params last_planner_params_{};                          ///< Last param sent to Nav2 planner
    bool has_last_controller_params_ = false;                   ///< Bool for did i send something to Nav2 controller ?!
    bool has_last_costmap_params_ = false;                      ///< Bool for did i send something to Nav2 costmap ?!
    bool has_last_planner_params_ = false;                      ///< Bool for did i send something to Nav2 planner ?!
    double controller_apply_interval_ = 0.5;                    ///< Threshold time for nav2 controller (faster - 2Hz)
    double costmap_apply_interval_    = 1.0;                    ///< Threshold time for nav2 costmap (middle - 1Hz)
    double planner_apply_interval_    = 2.0;                    ///< Threshold time for nav2 planner (slower - 0.5Hz)

    // ── Subscriber, Srv Client and Nav2 Client ───────────────────────────────
    rclcpp::Subscription<tbot3_nav_monitor::msg::NavigationMetrics>::SharedPtr metrics_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

    rclcpp::AsyncParametersClient::SharedPtr controller_client_;
    rclcpp::AsyncParametersClient::SharedPtr costmap_client_;
    rclcpp::AsyncParametersClient::SharedPtr planner_client_;

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_client_;
    
    // Life cycle Publisher (to publish the Nav2 state SUCCEEDED)
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt8>::SharedPtr goal_status_pub_;    

    // ── Mutex Thread-Safe ───────────────────────────────────────────────────
    mutable std::mutex state_mutex_; ///< Thread-Safe access variable
    
    // ── Private methods and Callbacks ────────────────────────────────────────
    /// @brief Callback for reading NavigationMetrics and applies adaptive logic
    /// @param msg Incoming NavigationMetrics message
    void metrics_callback(const std::shared_ptr<const tbot3_nav_monitor::msg::NavigationMetrics> & msg);

    /// @brief Method to send a new Nav2 goal
    /// @param goal New goal to send
    void send_nav2_goal(const geometry_msgs::msg::PoseStamped & goal);
    
    /// @brief Method to reset nav2 state
    void reset_nav2_state();

    /// @brief Method to reset nav2 parameters
    Nav2Params reset_to_normal() const;

    /// @brief Compute the desired nav2 parameters (concerning the adaptive logic)
    Nav2Params compute_desired_params() const;

    /// @brief Apply the calculated parameters
    /// @param param Calculated parameter
    void apply_params(const Nav2Params & param);

    /// @brief Callback for Nav2 goal reached result
    /// @param result Nav2 State result
    void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result);

    /// @brief Callback for Nav2 goal response
    /// @param goal_received NavigateToPose msg goal received from nav2 service
    void goal_response_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_received);

    /// @brief Callback for Nav2 intermediate feedback
    /// @param feedback NavigateToPose msg feedback received from nav2 service
    void feedback_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);
};

}  // namespace tbot3_nav_monitor

#endif  // ADAPTIVE_CONTROLLER_HPP
