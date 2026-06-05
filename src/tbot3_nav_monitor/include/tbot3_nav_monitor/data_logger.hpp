#ifndef DATA_LOGGER_HPP
#define DATA_LOGGER_HPP

#include <rclcpp/rclcpp.hpp>


#include "tbot3_nav_monitor/msg/navigation_metrics.hpp"
#include <nav_msgs/msg/odometry.hpp>

#include <vector>
#include <string>
#include <fstream>

namespace tbot3_nav_monitor
{

/**
    * @brief rclcpp Node that collects data and save them into csv files with timestamp
    *       - CSV logging of performance data with timestamps
*/

class DataLogger : public rclcpp::Node
{
public:
    /// @brief Constructor 
    /// @param node_name Name of the ROS2 node
    /// @param options If I pass an option it will take it, otherwise it will take the default NodeOptions
    explicit DataLogger(const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    /// @brief Destructor
    ~DataLogger();

private:

    // ── Private parameters  ──────────────────────────────────────────────────
    int64_t session_timestamp_;                        ///< Timestamp of the session start [ms]
    std::string log_directory_;                        ///< where to save the csv files
    std::string log_filename_;                         ///< Name file prefix
    bool enable_csv_;                                  ///< To disable/enable file logging
    bool odom_rate_initialized_ = false;               ///< To initilized the odom rate evaluation
    int stagnant_count_alert_thresholds_;              ///< Stagnang count alert 
    std::vector<double> battery_alert_thresholds_;     ///< Battery alert threshold [%]
    std::vector<double> odom_rate_alert_threshold_;    ///< Odom alert rate threshold [Hz]
    std::vector<double> latency_ms_alert_threshold_;   ///< Latency ms alert threshold [ms]
    int odom_msg_count_           = 0;                 ///< Odom count of msgs
    double odom_rate_             = 0.0;               ///< Odom Rate [s]
    double prev_distance_to_goal_ = 0.0;               ///< Previous Distance to goal [m]
    int stagnant_count_           = 0;                 ///< Initial stagnant count value
    rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};  ///< Last odom time (coherent time type)
    rclcpp::TimerBase::SharedPtr flush_timer_;         ///< TimerBase ptr to allor for flushing data
    
    // ── Subscriber / files  ──────────────────────────────────────────────────
    rclcpp::Subscription<tbot3_nav_monitor::msg::NavigationMetrics>::SharedPtr csv_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_rate_sub_;
    std::ofstream metric_collector_csv_file;

    // ── Private methods ──────────────────────────────────────────────────────
    /// @brief CSV callback: received data and save them with timestamp
    /// @param msg NavigationMetrics Message
    void csv_callback(const std::shared_ptr<const tbot3_nav_monitor::msg::NavigationMetrics> & msg);

    /// @brief Odom callback: check the rate of the received odom msg
    void odom_rate_callback(const std::shared_ptr<const nav_msgs::msg::Odometry> &);
};

}  // namespace tbot3_nav_monitor

#endif  // DATA_LOGGER_HPP
