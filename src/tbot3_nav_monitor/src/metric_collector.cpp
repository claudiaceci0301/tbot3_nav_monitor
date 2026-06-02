#define _USE_MATH_DEFINES

#include "tbot3_nav_monitor/metric_collector.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <algorithm>    // std::min_element, std::clamp
#include <chrono>
#include <cmath>
#include <limits>


namespace tbot3_nav_monitor
{
// ── Constructor ──────────────────────────────────────────────────────────────
MetricCollector::MetricCollector(const std::string & node_name, const rclcpp::NodeOptions & options) :
                                    rclcpp_lifecycle::LifecycleNode(node_name, options)
{
    // ── Declare parameters with defaults ─────────────────────────────────────
    declare_parameter("publish_rate",                  1.0); 
    declare_parameter("battery_drain_rate",           0.01); 
    declare_parameter("target_x",                     0.85); 
    declare_parameter("target_y",                     0.70); 
    declare_parameter("target_theta",                 0.75); 
    declare_parameter("distance_tolerance",            0.25); 
    declare_parameter("obstacle_distance_tolerance",  0.15); 
    declare_parameter("angle_tolerance",               0.4); 
    declare_parameter("max_linear_vel",                0.3);
    declare_parameter("max_angular_vel",               1.0); 
    declare_parameter("linear_gain",                   0.5);
    declare_parameter("angular_gain",                  2.0); 
    
    // ── Get parameter values ────────────────────────────────────────────────
    publish_rate_                 = get_parameter("publish_rate").as_double();
    battery_drain_rate_           = get_parameter("battery_drain_rate").as_double();
    target_.x                     = get_parameter("target_x").as_double();
    target_.y                     = get_parameter("target_y").as_double();
    target_.theta                 = get_parameter("target_theta").as_double(); 
    distance_tolerance_           = get_parameter("distance_tolerance").as_double();
    obstacle_distance_tolerance_  = get_parameter("obstacle_distance_tolerance").as_double();
    angle_tolerance_              = get_parameter("angle_tolerance").as_double();
    max_linear_vel_               = get_parameter("max_linear_vel").as_double();
    max_angular_vel_              = get_parameter("max_angular_vel").as_double();
    linear_gain_                  = get_parameter("linear_gain").as_double();
    angular_gain_                 = get_parameter("angular_gain").as_double();

    RCLCPP_INFO(get_logger(), "Metric Collector node %s  has been created! ", node_name.c_str());

    RCLCPP_INFO(get_logger(),
            "MetricCollector initialized | Target: (%.2f, %.2f, %2.f)", 
            target_.x, target_.y, target_.theta);
}

// ── LifecycleNode configuration  ────────────────────────────────────────────

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MetricCollector::on_configure(const rclcpp_lifecycle::State & state)
{
    // Call parent implementation (lifecycle node required)
    rclcpp_lifecycle::LifecycleNode::on_configure(state);

    // Create lifecycle publisher  (inactive until on_activate)
    metrics_pub_ = create_publisher<tbot3_nav_monitor::msg::NavigationMetrics>("/navigation_metrics", 10);        
        
     // Create subscribers
    odom_sub_       = create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
                        std::bind(&MetricCollector::odom_callback, this, std::placeholders::_1));
    scan_sub_       = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10,
                        std::bind(&MetricCollector::scanner_callback, this, std::placeholders::_1));
    cmd_vel_sub_        = create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10,
                            std::bind(&MetricCollector::cmdvel_callback, this, std::placeholders::_1));
    goal_sub_       = create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10,
                        std::bind(&MetricCollector::goal_send_callback, this, std::placeholders::_1));
    goal_status_sub_ = create_subscription<std_msgs::msg::UInt8>("/nav2_goal_status", 10,
                            [this](const std::shared_ptr<const std_msgs::msg::UInt8> msg){
                                std::lock_guard<std::mutex> lock(state_mutex_);
                                if(msg->data == 1) // Received nav2stat SUCCEEDED
                                {
                                    goal_reached_ = true;
                                }
                        });

    // Create Service
    reset_srv_ = create_service<std_srvs::srv::Trigger>("/metric_collector/reset",
                std::bind(&MetricCollector::reset_callback, this, std::placeholders::_1, std::placeholders::_2));
    
    // Buffer params
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
    
    RCLCPP_INFO(get_logger(), "on_configure() called, Node is still INACTIVE");
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MetricCollector::on_activate(const rclcpp_lifecycle::State & state)
{
    rclcpp_lifecycle::LifecycleNode::on_activate(state);

    // Activate lifecycle publisher (explicit call)
    metrics_pub_->on_activate();

    // Start periodic control / publish timer
    const auto publish_in_ms = static_cast<int>(1000.0 / publish_rate_);
    timer_ = create_wall_timer(
        std::chrono::milliseconds(publish_in_ms),
        std::bind(&MetricCollector::control_loop, this));

    RCLCPP_INFO(get_logger(), "on_activate() called, Node is now ACTIVE");
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MetricCollector::on_deactivate(const rclcpp_lifecycle::State & state)
{
    rclcpp_lifecycle::LifecycleNode::on_deactivate(state);

    // Stop the timer 
    metrics_pub_->on_deactivate(); // Stop publishing
    timer_->cancel(); // Stop the timer
    RCLCPP_INFO(get_logger(), "on_deactivate() called, Node is again INACTIVE");
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
MetricCollector::on_cleanup(const rclcpp_lifecycle::State & state)
{
    rclcpp_lifecycle::LifecycleNode::on_cleanup(state);

    // Clean up the resources
    timer_.reset();
    metrics_pub_.reset();
    odom_sub_.reset();
    scan_sub_.reset();
    cmd_vel_sub_.reset();
    reset_srv_.reset();
    goal_sub_.reset();
    goal_status_sub_.reset();
    tf2_listener_.reset();
    tf2_buffer_.reset();

    // Reset state
    distance_traveled_    = 0.0;
    battery_consumption_  = 0.0;
    battery_level_        = 100.0;
    recovery_count_       = 0;
    last_step_            = 0;
    goal_reached_         = false;
    odom_received_        = false;
    sensor_received_      = false;
    cmd_vel_received_     = false;

    RCLCPP_INFO(get_logger(), "on_cleanup() is called, everything has been resetted the node is UNCONFIGURED");
    return CallbackReturn::SUCCESS;
}

// ── Subscriber callbacks  ──────────────────────────────────────────────

void MetricCollector::odom_callback(const std::shared_ptr<const nav_msgs::msg::Odometry> & msg)
{
    // Thread-Safe
    std::lock_guard<std::mutex> lock(state_mutex_);

    // DEBUG odom frame
    RCLCPP_DEBUG(get_logger(), "ODOM frame=[%s]", msg->header.frame_id.c_str());

    const double new_x = msg->pose.pose.position.x;
    const double new_y = msg->pose.pose.position.y;

    if (!odom_received_) 
    {
        // Save the start position 
        start_.x = new_x;
        start_.y = new_y;
    }
    else
    {
        const double dx   = new_x - prev_odom_x_;
        const double dy   = new_y - prev_odom_y_;
        const double step = std::sqrt(dx*dx + dy*dy);
        distance_traveled_ += step;
        last_step_ = step;

        // If one changed, the robot is in motion otherwhise the position has not changed the robot is stucked
        constexpr double EPS = 1e-4; // If lower than EPS than the values are equal
        odom_position_unchanged_ = std::abs(new_x - prev_odom_x_) < EPS &&
                                    std::abs(new_y - prev_odom_y_) < EPS;
    }

    prev_odom_x_ = new_x;
    prev_odom_y_ = new_y;
    current_.x   = new_x;
    current_.y   = new_y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    current_.theta = yaw;

    odom_received_ = true;  
}

void MetricCollector::scanner_callback(const std::shared_ptr<const sensor_msgs::msg::LaserScan> & msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    sensor_ranges_    = msg->ranges; // Laser distances for every angle
    sensor_range_min_ = msg->range_min;
    sensor_range_max_ = msg->range_max;// values < min > max should be discarder
    sensor_received_  = true; 
}

void MetricCollector::cmdvel_callback(const std::shared_ptr<const geometry_msgs::msg::Twist> & msg)
{
    std::lock_guard<std::mutex> lock(state_mutex_);

    last_cmd_vel_     = *msg; // Deference of the twist shared_ptr
    cmd_vel_received_ = true;
}

 void MetricCollector::goal_send_callback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> & msg)
 {
    // This method recieves the new goal and updates the params

    // DEBUG frame
    RCLCPP_INFO(get_logger(), "GOAL RECEIVED at frame=[%s] pos=(%.3f, %.3f)",
    msg->header.frame_id.c_str(), msg->pose.position.x, msg->pose.position.y);
    
    // Transformation between odom frame and map frame
    geometry_msgs::msg::PoseStamped goal_in_odom;
    try {
        // Tranform(input, output, target_frame)
        // Transform the pose in odom frame
        tf_buffer_->transform(*msg, goal_in_odom, "odom",
            tf2::durationFromSec(0.1));

        // Thread-Safe
        std::lock_guard<std::mutex> lock(state_mutex_);

        //Update target position with the sent goal
        target_.x = goal_in_odom.pose.position.x;
        target_.y = goal_in_odom.pose.position.y;

        // Updated the target theta from quaternion
        tf2::Quaternion q(
            goal_in_odom.pose.orientation.x,
            goal_in_odom.pose.orientation.y,
            goal_in_odom.pose.orientation.z,
            goal_in_odom.pose.orientation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        target_.theta = yaw;

        RCLCPP_INFO(get_logger(), "Target updated in odom frame: (%.3f, %.3f, %.3f)",
            target_.x, target_.y, target_.theta);

    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN(get_logger(), "[WARN] TF2 Transform failed: %s", ex.what());
        
        //fallback: goal without frame transform
        target_.x = msg->pose.position.x;
        target_.y = msg->pose.position.y;
        tf2::Quaternion q(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w
        );
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        target_.theta = yaw;

        RCLCPP_DEBUG(get_logger(), "Target in map frame: (%.3f, %.3f, %.3f)",
            target_.x, target_.y, target_.theta);
    }

    // Re-calculate optimal path with new target
    if (odom_received_)
    {
        const double dx = target_.x - start_.x;
        const double dy = target_.y - start_.y;
        optimal_path_ = std::sqrt(dx*dx + dy*dy);
    }
}

// ── Service Callback ────────────────────────────────────────────────

void MetricCollector::reset_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    // Thread-Safe access on these variable (concurrent variable btw callback and control loop)
    // lock_guard for automatic Lock/Unlock of the mutex (it’s exception-safe)
    std::lock_guard<std::mutex> lock(state_mutex_);

    // Reset tutto lo stato per la prossima navigazione
    distance_traveled_    = 0.0;
    battery_consumption_  = 0.0;
    recovery_count_       = 0;
    last_step_            = 0.0;
    goal_reached_         = false;
    odom_received_        = false;  
    sensor_received_      = false;
    cmd_vel_received_     = false;

    response->success = true;
    response->message = "MetricCollector reset for next goal";
    RCLCPP_INFO(get_logger(), "Reset triggered — ready for next navigation goal!");
}

// ── Helper methods  ────────────────────────────────────────────────

double MetricCollector::normalize_angle(double angle)
{
    while (angle >  M_PI) angle -= 2.0 * M_PI; // M_PI = 180°
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

bool MetricCollector::collision_detection()
{
    if (sensor_ranges_.empty()) return false;

    // Front-left sector  [0, n/4)
    // Front-right sector [3n/4, n)
    // Values outside [range_min, range_max] are invalid skip them
    auto valid = [&](float r) 
    {
        return std::isfinite(r) && r >= sensor_range_min_ && r <= sensor_range_max_;
    };

    double left_min  = std::numeric_limits<double>::max();
    double right_min = std::numeric_limits<double>::max();

    for (std::size_t i = 0; i < sensor_ranges_.size() / 4; ++i)
    {
        if (valid(sensor_ranges_[i])) 
        left_min = std::min(left_min, static_cast<double>(sensor_ranges_[i]));
    }

    for (std::size_t i = (3 * sensor_ranges_.size()) / 4; i < sensor_ranges_.size(); ++i)
    {
        if (valid(sensor_ranges_[i]))
        right_min = std::min(right_min, static_cast<double>(sensor_ranges_[i]));
    }
        
    // Global minimum distance
    min_distance_obstacle_ = std::min(left_min, right_min);
        
    // Check for collision
    if (min_distance_obstacle_ < obstacle_distance_tolerance_)
    {
        RCLCPP_WARN(get_logger(),
            "Obstacle detected at %.3f [m], stop the robot!", min_distance_obstacle_);
        return true; // Robot is in collision
    }
    return false;
}

// Control loop function (called by the timer)
// The timer calls this function repeatedly
void MetricCollector::control_loop()
{
    // Thread-Safe access on these variable
    std::lock_guard<std::mutex> lock(state_mutex_);

    // Wait until all sensor streams have been received at least once
    if (!odom_received_ || !sensor_received_ || !cmd_vel_received_) return;

    // Stop if goal already reached or battery flat
    if (goal_reached_ || battery_consumption_ >= battery_level_) return;

    // ── Collision check ──────────────────────────────────────────────────────
    if (collision_detection())
    {
        // Publish a zero-velocity command to stop the robot
        // Recovery: if cmd_vel was non-zero but robot did not move the robot is stucked
        constexpr double CMD_EPS = 1e-3;
        if((std::abs(last_cmd_vel_.linear.x) > CMD_EPS || 
            std::abs(last_cmd_vel_.angular.z) > CMD_EPS) 
            && odom_position_unchanged_)
        {
            recovery_count_++;
            RCLCPP_WARN(get_logger(), "Recovery event #%d detected, Robot is STUCKED!", recovery_count_);
        }
    }

    // ── Battery consumption (cumulative) ────────────────────────────────────
    battery_consumption_ += battery_drain_rate_ * last_step_;
    battery_consumption_  = std::min(battery_consumption_, battery_level_); // Avoid battery consumption > 100%
    
    // DEBUG before goal check to see if nav2 and metriccollector sees the same position
    RCLCPP_DEBUG(
    get_logger(),
    "CURRENT=(%.3f %.3f %.3f) TARGET=(%.3f %.3f %.3f)",
    current_.x,
    current_.y,
    current_.theta,
    target_.x,
    target_.y,
    target_.theta);

    // ── Goal check - Geometry check ───────────────────────────────────────────────────────────
    const double dx               = target_.x - current_.x;
    const double dy               = target_.y - current_.y;
    const double distance         = std::sqrt(dx*dx + dy*dy);  // Euclidean distance
    //const double angle_to_target  = std::atan2(dy, dx);
    //const double angle_difference = normalize_angle(angle_to_target - current_.theta); 
    const double angle_difference = normalize_angle(target_.theta - current_.theta);
    
    // DEBUG Check
    RCLCPP_DEBUG(get_logger(),
    "CHECK: dist=%.3f tol=%.3f angle_diff=%.3f angle_tol=%.3f",
    distance, distance_tolerance_, angle_difference, angle_tolerance_);

    // Geometry check - Backup check 
    if (distance <= distance_tolerance_ && std::abs(angle_difference) <= angle_tolerance_)
    {
        RCLCPP_INFO(get_logger(),
        "GOAL REACHED! Final position: (%.3f, %.3f, %.3f)",
        current_.x, current_.y, current_.theta);
    }
    
    // ── Build NavigationMetrics message and publish ──────────────────────────
    tbot3_nav_monitor::msg::NavigationMetrics metrics_msg;

    metrics_msg.distance_traveled           = distance_traveled_;
    metrics_msg.battery_consumption         = battery_consumption_;
    metrics_msg.min_obstacle_distance       = min_distance_obstacle_;
    metrics_msg.recovery_count              = recovery_count_;
    metrics_msg.goal_reached                = goal_reached_;
    metrics_msg.current_x                   = current_.x;
    metrics_msg.current_y                   = current_.y;
    metrics_msg.current_theta               = current_.theta;
    metrics_msg.distance_to_goal            = distance;
    metrics_msg.distance_tolerance          = distance_tolerance_;
    metrics_msg.angle_tolerance             = angle_tolerance_;
    metrics_msg.obstacle_distance_tolerance = obstacle_distance_tolerance_;
    metrics_msg.optimal_path                = optimal_path_;
    metrics_msg.header.stamp                = this->get_clock()->now();
    metrics_msg.header.frame_id             = "odom";

    if (metrics_pub_->is_activated()) // If the node is active publish the custom msg
    {
        metrics_pub_->publish(metrics_msg);
    }
    
    // ── Debug log ────────────────────────────────────────────────────────────
    RCLCPP_DEBUG(get_logger(),
        "Current Position: (%.2f, %.2f, %.2f) | Target: (%.2f, %.2f) | Distance: %.3f "
        "| AngleDiff: %.3f | Battery consumed: %.2f%% | Recoveries: %d",
        current_.x, current_.y, current_.theta,
        target_.x,  target_.y,
        distance, angle_difference,
        battery_consumption_, recovery_count_);
}

} // namespace tbot3_nav_monitor