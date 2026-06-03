#include "tbot3_nav_monitor/data_logger.hpp"

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <filesystem>

namespace tbot3_nav_monitor
{
// ── Constructor  ────────────────────────────────────────────
DataLogger::DataLogger(const std::string & node_name, const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options)
{
    // ── Declare parameters with defaults ─────────────────────────────────────
    declare_parameter("enable_csv",                      false);
    declare_parameter("log_directory",                   "/root/tbot3_nav_monitor/logs");
    declare_parameter("log_filename",                    "metric_collector_data");
    declare_parameter("battery_alert_thresholds",        std::vector<double>{0.5, 0.85, 0.95});  
    declare_parameter("stagnant_count_alert_thresholds", 5);
    declare_parameter("odom_rate_alert_threshold",       std::vector<double>{5.0, 10.0});
    declare_parameter("latency_ms_alert_threshold",      std::vector<double>{100.0, 500.0});

    // ── Get parameter values ─────────────────────────────────────────────────
    enable_csv_ =                      get_parameter("enable_csv").as_bool();
    log_directory_ =                   get_parameter("log_directory").as_string();
    log_filename_ =                    get_parameter("log_filename").as_string();
    battery_alert_thresholds_ =        get_parameter("battery_alert_thresholds").as_double_array();
    stagnant_count_alert_thresholds_ = get_parameter("stagnant_count_alert_thresholds").as_int();
    odom_rate_alert_threshold_ =       get_parameter("odom_rate_alert_threshold").as_double_array();
    latency_ms_alert_threshold_ =      get_parameter("latency_ms_alert_threshold").as_double_array();

    // ── Timestamp ────────────────────────────────────────────────────────────
    auto now = this->get_clock()->now(); // get_clock() follows ROS2 clock
    session_timestamp_ = now.nanoseconds() / 1000000; 

    last_odom_time_ = this->get_clock()->now(); 

    // ── Subscribers ──────────────────────────────────────────────────────────
    odom_rate_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
        std::bind(&DataLogger::odom_rate_callback, this, std::placeholders::_1));

    if (enable_csv_) 
    {
        std::filesystem::create_directories(log_directory_);

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
                << "real_time_ms [ms],"        
                << "sim_time_ms [ms],"    
                << "timestamp_ms [ms],"
                << "publish_time [ms],"
                << "distance_traveled [m],"
                << "battery_consumption [%],"   
                << "min_obstacle_distance [m],"
                << "goal_reached,"
                << "distance_to_goal [m]\n";    
        }
    }

    csv_sub_ = create_subscription<tbot3_nav_monitor::msg::NavigationMetrics>("/navigation_metrics", 10,
            std::bind(&DataLogger::csv_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(get_logger(), "Data Logger node %s  has been created! ", node_name.c_str());
}


// ── Subscriber callbacks  ──────────────────────────────────────────────

void DataLogger::csv_callback(
    const std::shared_ptr<const tbot3_nav_monitor::msg::NavigationMetrics> & msg)
{
    if (!enable_csv_ || !metric_collector_csv_file.is_open())
        return;

    // Real Time (system clock - wall time) 
    auto real_time_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch()
        ).count();

     // Sim Time 
    auto sim_time_ms = this->get_clock()->now().nanoseconds() / 1000000; // rclcpp::Time

    // Publish time rclcpp::Time (from message header) -> to force RCL_ROS_TIME 
    const auto publish_time = rclcpp::Time(msg->header.stamp,
                            this->get_clock()->get_clock_type());

    // int64_t ms -> CSV
    const auto publish_time_ms = publish_time.nanoseconds() / 1000000;

    // Latency -> btw rclcpp::Time
    const auto received_time = this->get_clock()->now(); // rclcpp::Time
    double latency_ms = (received_time - publish_time).seconds() * 1000.0;

    // Write CSV file
    metric_collector_csv_file
        << real_time_ms << ","        // wall clock
        << sim_time_ms << ","         // sim clock
        << publish_time_ms << ","     // publisher timestamp
        << latency_ms << ","          // transport delay
        << msg->distance_traveled << ","
        << msg->battery_consumption << ","
        << msg->min_obstacle_distance << ","
        << msg->goal_reached << ","
        << msg->distance_to_goal
        << "\n";

    metric_collector_csv_file.flush();

    // ── Battery Alert  ─────────────────────────────────────────────────

    if (msg->battery_consumption > battery_alert_thresholds_.at(2))
    {
        RCLCPP_ERROR(get_logger(), "[ERROR] Battery almost empty!");
    }
    else if (msg->battery_consumption > battery_alert_thresholds_.at(1))
    {
        RCLCPP_WARN(get_logger(), "[WARN] Battery high consumption");

        if (!msg->goal_reached)
        {
            RCLCPP_WARN(get_logger(),
                "[WARN] High battery usage without reaching goal");
        }
    }
    else if (msg->battery_consumption > battery_alert_thresholds_.at(0))
    {
        RCLCPP_INFO(get_logger(), "[INFO] Battery > 50");
    }

    // ── Distance to Goal  ─────────────────────────────────────────────────

    if (msg->distance_to_goal >= prev_distance_to_goal_)
    {
        stagnant_count_++;

        if (stagnant_count_ >= stagnant_count_alert_thresholds_)
        {
            RCLCPP_WARN(get_logger(),
                "Distance not decreasing for %d ticks",
                stagnant_count_);
        }
    }
    else
    {
        stagnant_count_ = 0;
    }

    prev_distance_to_goal_ = msg->distance_to_goal;

    // ── Latency  ──────────────────────────────────────────────────────────

    if (latency_ms > latency_ms_alert_threshold_.at(0))
    {
        RCLCPP_WARN(get_logger(), "[WARN] Latency > 100ms");
    }

    if (latency_ms > latency_ms_alert_threshold_.at(1))
    {
        RCLCPP_ERROR(get_logger(), "[ERROR] Latency > 500ms");
    }
}

void DataLogger::odom_rate_callback(const std::shared_ptr<const nav_msgs::msg::Odometry> &)
{
    // ── Odom Frequency check  ─────────────────────────────────────────────
    // Tbot3 in general send odom msg with a rate of 30 Hz (30 times at second)

    odom_msg_count_++;
    const auto time_now = this->get_clock()->now();
    const double elapsed = (time_now - last_odom_time_).seconds();

    if (elapsed >= 1.0)
    {
        odom_rate_           = odom_msg_count_ / elapsed;
        odom_msg_count_      = 0;
        last_odom_time_      = time_now;
        odom_rate_initialized_ = true;
    }

    // If not initialized return
    if (!odom_rate_initialized_) return;

    if (odom_rate_ < odom_rate_alert_threshold_.at(0))
    {
        RCLCPP_ERROR(get_logger(), "[ERROR] Critical odom rate %.1f Hz — compromised navigation!", odom_rate_);
        return;
    }

    if (odom_rate_ < odom_rate_alert_threshold_.at(1))
    {
        RCLCPP_WARN(get_logger(), "[WARN] Odom rate low %.1f Hz — unreliable data!", odom_rate_);
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