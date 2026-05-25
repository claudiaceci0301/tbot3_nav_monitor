#ifndef DATA_LOGGER_HPP
#define DATA_LOGGER_HPP

#include <rclcpp/rclcpp.hpp>


#include "tbot3_nav_monitor/msg/navigation_metrics.hpp"

#include <vector>
#include <string>
#include <fstream>
#include <sstream>

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

    // ── Private parameters  ──────────────────────────────────────────────────
    std::string log_directory_;                ///< where to save the csv files
    std::string log_filename_;                 ///< Name file prefix
    bool enable_csv_;                          ///< To disable/enable file logging
    std::vector<double> alert_thresholds_;     ///< Threshold for the alert

    // ── Subscriber / files  ──────────────────────────────────────────────────
    rclcpp::Subscription<tbot3_nav_monitor::msg::NavigationMetrics>::SharedPtr  csv_sub_;
    std::ofstream metric_collector_csv_file;

    // ── Private methods ──────────────────────────────────────────────────────
    /// @brief CSV callback: received data and save them with timestamp
    /// @param msg NavigationMetrics Message
    void csv_callback(const std::shared_ptr<const tbot3_nav_monitor::msg::NavigationMetrics> & msg);


};

}  // namespace tbot3_nav_monitor

#endif  // DATA_LOGGER_HPP
