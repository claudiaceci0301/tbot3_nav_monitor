int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto metric_node = std::make_shared<MetricCollector>("metric_collector_node");
  rclcpp::spin(metric_node);
  
  rclcpp::shutdown();
  return 0;
}