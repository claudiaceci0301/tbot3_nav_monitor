#include "tbot3_nav_monitor/metric_collector.hpp"
#include "tbot3_nav_monitor/adaptive_behavior.hpp"  
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>  


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv); // Start ROS2

    auto metric_node = std::make_shared<tbot3_nav_monitor::MetricCollector>("metric_collector_node");
    auto adaptive_node = std::make_shared<tbot3_nav_monitor::MetricCollector>("adaptive_behavior_node");

    // Configure and activate the lifecycle node
    metric_node->configure();
    metric_node->activate();
    adaptive_node->configure();
    adaptive_node->activate();

    // Multithread spin more node together
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(metric_node->get_node_base_interface()); // Spin ROS2 lifecycle node
    executor.add_node(adaptive_node->get_node_base_interface());
    executor.spin();

    metric_node->deactivate();
    metric_node->cleanup();
    adaptive_node->deactivate();
    adaptive_node->cleanup();

    rclcpp::shutdown(); // ROS2 shutdown
    return 0;
}