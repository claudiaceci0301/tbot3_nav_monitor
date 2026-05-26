#include "tbot3_nav_monitor/metric_collector.hpp"
#include "tbot3_nav_monitor/adaptive_controller.hpp"
#include "tbot3_nav_monitor/data_logger.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <lifecycle_msgs/msg/state.hpp>

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    // Options for the time confgiuration
    // In Gazebo the time is not real but simulated - use_sim_time allows sincronization of time
    rclcpp::NodeOptions options;
    options.parameter_overrides({rclcpp::Parameter("use_sim_time", true)});

    // Nodes
    auto metric_node      = std::make_shared<tbot3_nav_monitor::MetricCollector>("metric_collector_node", options);
    auto adaptive_node    = std::make_shared<tbot3_nav_monitor::AdaptiveController>("adaptive_controller_node", options);
    auto data_logger_node = std::make_shared<tbot3_nav_monitor::DataLogger>("data_logger_node", options);

    // Configure MetricCollector 
    metric_node->configure();
    metric_node->activate();

    // Configure AdaptiveController — check because it has Nav2 client dependencies
    if (adaptive_node->configure().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "AdaptiveController on_configure failed!");
        rclcpp::shutdown();
        return 1;
    }
    adaptive_node->activate();

    // Spin the nodes together
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(metric_node->get_node_base_interface()); // rclccpp lifecycle nodes
    executor.add_node(adaptive_node->get_node_base_interface());
    executor.add_node(data_logger_node-); //rclcpp node
    executor.spin();

    // Clean shutdown
    metric_node->deactivate();
    metric_node->cleanup();
    adaptive_node->deactivate();
    adaptive_node->cleanup();

    rclcpp::shutdown();
    return 0;
}