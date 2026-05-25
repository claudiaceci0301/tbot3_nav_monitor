#define _USE_MATH_DEFINES

#include "tbot3_nav_monitor/data_collector.hpp"

#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include <chrono>

namespace tbot3_nav_monitor
{
// ── Constructor  ────────────────────────────────────────────
DataLogger::DataLogger(const std::string & node_name, const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options)
{
    // ── Declare parameters with defaults ─────────────────────────────────────
    declare_parameter("enable_csv", false);
    declare_parameter("log_directory", "/root/logs");
    declare_parameter("log_filename", "metric_collector_data");
    declare_parameter("alert_thresholds", std::vector<0.0, 0.0, 0.0>);

    // ── Get parameter values ────────────────────────────────────────────────
    enable_csv_ = get_parameter("enable_csv").as_bool();
    log_directory_ = get_parameter("log_directory").as_string();
    log_filename_ = get_parameter("log_filename").as_string();
    alert_thresholds_ = get_parameter("alert_thresholds").as_double_vector();

    if (enable_csv_) {
        std::string full_path = log_directory_ + "/" + log_filename_ + ".csv";

        metric_collector_csv_file.open(full_path); //ofstream - write only

        if (!metric_collector_csv_file.is_open()) 
        {
            RCLCPP_ERROR(get_logger(), "Failed to open CSV file!");
        } 
        else 
        {
            RCLCPP_INFO(get_logger(), "CSV logging enabled!");
        }
    }

    csv_sub_ = create_subscription<tbot3_nav_monitor::msg::NavigationMetrics>("/navigation_metrics", 10,
            std::bind(&DataLogger::csv_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "Data Logger node %s  has been created! ", node_name.c_str());
}


// ── Subscriber callbacks  ──────────────────────────────────────────────

void DataLogger::csv_callback(const std::shared_ptr<const tbot3_nav_monitor::msg::NavigationMetrics> & msg)
{
    if (!enable_csv_ || !metric_collector_csv_file.is_open()) return;

    auto now = std::chrono::system_clock::now();
    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()).count(); //int64 in mms from 1/1/1970

    metric_collector_csv_file
        << msg->distance_traveled << ","
        << msg->battery_consumption << ","
        << msg->min_obstacle_distance << ","
        << msg->goal_reached << ","
        << msg->distance_to_goal << ","
        << timestamp
        << "\n";

    metric_collector_csv_file.flush(); // It forced the writing of the buffer on the disk immediatly (riduced loose of info)

    if()
    {
        enable_csv_ = true;
    }
    else
}


// ── Destructor  ────────────────────────────────────────────
DataLogger::~DataLogger()
{
    if (metric_collector_csv_file.is_open()) {
        metric_collector_csv_file.close();
    }
}

} // namespace tbot3_nav_monitor