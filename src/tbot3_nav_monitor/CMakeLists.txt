cmake_minimum_required(VERSION 3.8)
project(tbot3_nav_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)        
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)                     
find_package(tf2_ros REQUIRED)                 
find_package(lifecycle_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED) 

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/NavigationMetrics.msg"
  DEPENDENCIES geometry_msgs
  DEPENDENCIES std_msgs
)

# Executable definition
add_executable(metric_collector
  src/metric_collector.cpp
  src/adaptive_controller.cpp
  src/data_logger.cpp
  src/main.cpp 
  
)

target_include_directories(metric_collector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(metric_collector
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  lifecycle_msgs
)

# Custom message
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
target_link_libraries(metric_collector ${cpp_typesupport_target})

install(TARGETS metric_collector
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()