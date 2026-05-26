#include "tbot3_nav_monitor/data_collector.hpp"

#include <rclcpp/rclcpp.hpp>
#include <chrono>

namespace tbot3_nav_monitor
{
// ── Constructor  ────────────────────────────────────────────
DataLogger::DataLogger(const std::string & node_name, const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options)
{
    // ── Declare parameters with defaults ─────────────────────────────────────
    declare_parameter("enable_csv",                      false);
    declare_parameter("log_directory",                   "/src/tbot3_nav_monitor/logs");
    declare_parameter("log_filename",                    "metric_collector_data");
    declare_parameter("battery_alert_thresholds",        std::vector<double>{0.5, 0.85, 0.95});  
    declare_parameter("stagnant_count_alert_thresholds", 5);
    declare_parameter("odom_rate_alert_threshold",       std::vector<double>{5.0, 10.0});
    declare_parameter("latency_ms_alert_threshold",      std::vector<double>{100.0, 500.0});

    // ── Get parameter values ─────────────────────────────────────────────────
    enable_csv_ =                      get_parameter("enable_csv").as_bool();
    log_directory_ =                   get_parameter("log_directory").as_string();
    log_filename_ =                    get_parameter("log_filename").as_string();
    battery_alert_thresholds_ =        get_parameter("battery_alert_thresholds").as_double_vector();
    stagnant_count_alert_thresholds_ = get_parameter("stagnant_count_alert_thresholds").as_int();
    odom_rate_alert_threshold_ =       get_parameter("odom_rate_alert_threshold").as_double_vector();
    latency_ms_alert_threshold_ =      get_parameter("latency_ms_alert_threshold").as_double_vector();

    // ── Timestamp ────────────────────────────────────────────────────────────
    auto now = this->get_clock()->now();              // get_clock() follows ROS2 clock
    session_timestamp_ = now.nanoseconds() / 1000000; //int64 in mms from 1/1/1970

    // ── Subscribers ──────────────────────────────────────────────────────────
    odom_rate_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
        std::bind(&DataLogger::odom_rate_callback, this, std::placeholders::_1));

    if (enable_csv_) 
    {
        std::string full_path = log_directory_ + "/" + log_filename_ + "_" + std::to_string(now.nanoseconds()) + ".csv";

        metric_collector_csv_file.open(full_path); //ofstream - write only

        if (!metric_collector_csv_file.is_open()) 
        {
            RCLCPP_ERROR(get_logger(), "Failed to open CSV file!");
        } 
        else 
        {
            RCLCPP_INFO(get_logger(), "CSV logging enabled!");

            metric_collector_csv_file
                << "distance_traveled [m],"
                << "battery_consumption [%},"
                << "min_obstacle_distance [m],"
                << "goal reached,"
                << "distance_to_goal [m],"
                << "session_timestamp [ms],"
                << "\n";
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

    metric_collector_csv_file
        << msg->distance_traveled << ","
        << msg->battery_consumption << ","
        << msg->min_obstacle_distance << ","
        << msg->goal_reached << ","
        << msg->distance_to_goal << ","
        << session_timestamp_
        << "\n";

    metric_collector_csv_file.flush(); // It forced the writing of the buffer on the disk immediatly (riduced loose of info)

    // ── Battery Alert  ─────────────────────────────────────────────────
    if (msg->battery_consumption > battery_alert_thresholds_.at(2))      // > 95% → ERROR
    {
        RCLCPP_ERROR(get_logger(), " [ERROR] Battery consumption almost 100%!");
    }
    else if (msg->battery_consumption > battery_alert_thresholds_.at(1)) // > 85% → WARN
    {
        RCLCPP_WARN(get_logger(), " [WARN] Battery consumption is now greater than 85%!");
        if(!msg->goal_reached)
        {
            RCLCPP_WARN(get_logger(), " [WARN] Battery consumption is greater than 85 and has not reached the goal 
                                        | The robot is consuming too much battery!")
        }
    }
    else if (msg->battery_consumption > battery_alert_thresholds_.at(0)) // > 50% → INFO
    {
        RCLCPP_INFO(get_logger(), " [INFO] Battery consumption is now greater than 50%!");
    }
   

    // ── Distance to Goal  ─────────────────────────────────────────────────
    if (msg->distance_to_goal >= prev_distance_to_goal_)
    {
        stagnant_count_++;
        if (stagnant_count_ >= stagnant_count_alert_thresholds_)  // after 5 consecutive ticks
            RCLCPP_WARN(get_logger(), "Distance from goal not decreasing for %d ticks!", stagnant_count_);
    }
    else
    {
        stagnant_count_ = 0;  // Reset if it improves
    }
    prev_distance_to_goal_ = msg->distance_to_goal;

    // ── Latency  ──────────────────────────────────────────────────────────
    // How much time has passed from when MetricCollector publishes its metrics and DataLogger receives them
    const auto received_time = this->get_clock()->now(); // rclcpp::Time
    const auto publish_time = rclcpp::Time(msg->header.stamp);
    double latency_ms = (received_time - publish_time).seconds() * 1000.0; // ms

    if(latency_ms > latency_ms_alert_threshold_.at(0))
    {
        RCLCPP_WARN(get_logger(), " [WARN] Latency ms is greater than 100ms!")
    }

    if(latency_ms > latency_ms_alert_threshold_.at(1))
    {
        RCLCPP_ERROR(get_logger(), " [ERROR] Latency ms is greater than 500ms!")
    }

}

void DataLogger::odom_rate_callback(const std::shared_ptr<const nav_msgs::msg::Odometry> &)
{
    // ── Odom Frequency check  ─────────────────────────────────────────────
    // Tbot3 in general send odom msg with a rate of 30 Hz (30 times at second)

    odom_msg_count_++;
    auto time_now = this->get_clock()->now();  // rclcpp::Time // It returns an std::chrono::time_point 
    double elapsed = (time_now - last_odom_time_).seconds();
    if (elapsed >= 1.0) // After 1 second
    {
        odom_rate_ = odom_msg_count_ / elapsed; // The accetable rate should be 30/s or similar
        odom_msg_count_ = 0; // Reset value
        last_odom_time_ = time_now;
        odom_rate_initialized_ = true;
    }

    if(!odom_rate_initialized_) return; // If not inizialised

    if(odom_rate_ < odom_rate_alert_threshold_.at(1))
    {
        RCLCPP_WARN(get_logger(), " [WARN] Odom rate is low - Not reliable data!");
    }

    if(odom_rate_ < odom_rate_alert_threshold_.at(0))
    {
        RCLCPP_ERROR(get_logger(), " [ERROR] Critical odom rate - Compromised navigation!");
    }
}

// ── Destructor  ───────────────────────────────────────────────────────────
DataLogger::~DataLogger()
{
    if (metric_collector_csv_file.is_open()) {
        metric_collector_csv_file.close();
    }
}

} // namespace tbot3_nav_monitor