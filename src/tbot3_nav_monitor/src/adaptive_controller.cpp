#define _USE_MATH_DEFINES

#include "tbot3_nav_monitor/metric_collector.hpp"

#include <algorithm>    // std::min_element, std::clamp
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>

/*
C — I nodi Nav2 in dettaglio
1 — planner_server
Calcola il path globale dal punto corrente al goal — il percorso completo sulla mappa. 
I parametri che ti interessano:
GridBased.tolerance          → distanza accettabile dal goal nel path planning
GridBased.use_astar          → usa A* invece di Dijkstra (tipo di algoritmo)
Quando dici "switch to more conservative path planning" puoi aumentare tolerance o cambiare il plugin di planning con uno più cauto.

2 — controller_server
Prende il path globale dal planner e genera i comandi di velocità (/cmd_vel) per seguirlo in tempo reale.
È il nodo che muove fisicamente il robot. Il plugin default si chiama FollowPath (DWB controller). 
I parametri chiave:
FollowPath.max_vel_x         → velocità lineare massima [m/s]
FollowPath.max_vel_theta     → velocità angolare massima [rad/s]
FollowPath.min_vel_x         → velocità minima (evita movimenti troppo lenti)
FollowPath.acc_lim_x         → accelerazione massima lineare
FollowPath.acc_lim_theta     → accelerazione massima angolare
Quando hai troppi recovery → riduci max_vel_x perché il robot va troppo veloce per reagire agli ostacoli.

3 — local_costmap
È una mappa locale centrata sul robot (es. 3x3 metri) che si aggiorna in tempo reale con i dati del laser. 
Rappresenta dove ci sono ostacoli nell'immediato intorno. 
I parametri chiave:
local_costmap.inflation_layer.inflation_radius   → raggio di "gonfiamento" degli ostacoli [m]
local_costmap.obstacle_layer.obstacle_range      → distanza massima a cui registra ostacoli
local_costmap.width                              → larghezza della mappa locale [m]
local_costmap.height                             → altezza della mappa locale [m]
local_costmap.resolution                         → risoluzione in metri per cella
inflation_radius è il più importante per te — aumentandolo il robot mantiene più distanza dagli ostacoli, rendendo la navigazione più conservativa ma più sicura.
Quando l'obstacle avoidance è inefficiente → aumenti questo valore.

4 — global_costmap
Simile alla local ma rappresenta tutta la mappa dell'ambiente. Viene aggiornata più lentamente.
Il planner la usa per calcolare il path globale. Per ora non ti serve modificarla a runtime.

5 — behavior_server
Gestisce i recovery behavior — cosa fa il robot quando è bloccato. 
I behavior default sono:
spin → ruota su se stesso per trovare una via libera
backup → fa retromarcia
wait → aspetta che l'ostacolo si sposti

Questo nodo è quello che incrementa il tuo recovery_count_ nel MetricCollector.

Schema riassuntivo
/scan ──────────────────────────────────────────→ local_costmap
                                                        ↓
/goal_pose ──→ planner_server (path globale) ──→ controller_server ──→ /cmd_vel ──→ robot
                                                        ↑
                                               behavior_server (recovery)
                                               
AdaptiveBehavior ──→ parameter service ──→ controller_server (velocità)
                                       ──→ local_costmap (inflation)
                                       ──→ planner_server (tolleranza

quindi alla fine avrò:
recovery alto        → controller_server  → riduci max_vel_x
accuracy bassa       → planner_server     → aumenta tolerance
avoidance inefficiente → local_costmap    → aumenta inflation_radius
*/


namespace tbot3_nav_monitor
{
    // ── Constructor ─────────────────────────────────────────────────────────────
    AdaptiveController::AdaptiveController(const std::string & node_name) :
                                    rclcpp_lifecycle::LifecycleNode(node_name)
    {
        RCLCPP_INFO(get_logger(), "Adaptive Controller node %s  has been created! ", node_name.c_str());

        // ── Declare parameters with defaults ─────────────────────────────────────
        declare_parameter("recovery_threshold",   3); 
        declare_parameter("accuracy_threshold",   0.7); 
        declare_parameter("efficiency_threshold", 0.75);
        declare_parameter("window_size",          10); 

        // Default values from yaml TurtleBot3
        declare_parameter("normal_max_vel_x",            0.3);
        declare_parameter("normal_max_vel_theta",        1.0);
        declare_parameter("normal_xy_goal_tolerance",    0.25);
        declare_parameter("normal_yaw_goal_tolerance",   0.25);

        // Increased or reduced values due to the adaptive logic (avoid hardcoded values)
        declare_parameter("reduced_max_vel_x",              0.15);
        declare_parameter("reduced_max_vel_theta",          0.5);
        declare_parameter("increaded_xy_goal_tolerance",    0.08);
        declare_parameter("increased_yaw_goal_tolerance",   0.4);
        declare_parameter("increased_inflation_radius",     0.4);
        
        // ── Get parameter values ─────────────────────────────────────────────────
        recovery_threshold_   = get_parameter("recovery_threshold").as_int();
        accuracy_threshold_   = get_parameter("accuracy_threshold").as_double();
        efficiency_threshold_ = get_parameter("efficiency_threshold").as_double();
        window_size_          = get_parameter("window_size").as_int();
    }

    // ── LifecycleNode configuration  ─────────────────────────────────────────────

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    AdaptiveController::on_configure(const rclcpp_lifecycle::State & state)
    {
        // Call parent implementation (lifecycle node required)
        rclcpp_lifecycle::LifecycleNode::on_configure(state);

        // Create Clients
        // AsyncParametersClient — asyncronous so it does not stop the node while waiting
        // It allows for parameter updates/changes runtime
        controller_client_  = std::make_shared<rclcpp::AsyncParametersClient>(this, "/controller_server"); // (node, remote_node_name)
        costmap_client_     = std::make_shared<rclcpp::AsyncParametersClient>(this, "/local_costmap");
        planner_client_     = std::make_shared<rclcpp::AsyncParametersClient>(this, "/planner_server");

        // Wait for services - (when nav2 will be ready i will change the warn with a failure)
        if (!controller_client_->wait_for_service(std::chrono::seconds(5)))
            RCLCPP_WARN(get_logger(), "controller_server not available");

        if (!costmap_client_->wait_for_service(std::chrono::seconds(5)))
            RCLCPP_WARN(get_logger(), "local_costmap not available");

        if (!planner_client_->wait_for_service(std::chrono::seconds(5)))
            RCLCPP_WARN(get_logger(), "planner_server not available");
        
        // Create Subscriber
        metrics_sub_ = create_subscription<tbot3_nav_monitor::msg::NavigationMetrics>("/navigation_metrics", 10,
            std::bind(&AdaptiveController::metrics_callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "on_configure() called, Node is still INACTIVE");
        return CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    AdaptiveController::on_activate(const rclcpp_lifecycle::State & state)
    {
        rclcpp_lifecycle::LifecycleNode::on_activate(state);
        RCLCPP_INFO(get_logger(), "on_activate() called, Node is now ACTIVE");
        return CallbackReturn::SUCCESS;

    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    AdaptiveController::on_deactivate(const rclcpp_lifecycle::State & state)
    {
        rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
        RCLCPP_INFO(get_logger(), "on_deactivate() called, Node is again INACTIVE");
        return CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    AdaptiveController::on_cleanup(const rclcpp_lifecycle::State & state)
    {
        rclcpp_lifecycle::LifecycleNode::on_cleanup(state);

        // Clean up the resources
        controller_client_.reset();
        costmap_client_.reset();
        planner_client_.reset();
        metrics_sub_.reset();

        // Reset state
        window_count_           = 0;
        optimal_path_           = 0.0;
        efficiency_             = 0.0;
        mean_accuracy_          = 0.0;
        sum_accuracy_           = 0.0;

        RCLCPP_INFO(get_logger(), "on_cleanup() is called, everything has been resetted the node is UNCONFIGURED");
        return CallbackReturn::SUCCESS;
    }

    // ── Subscriber callback  ────────────────────────────────────────────────────
    void AdaptiveController::metrics_callback(const std::shared_ptr<const tbot3_nav_monitor::msg::NavigationMetrics> & msg)
    {
        // LOGIC HERE
        const auto recovery_count  = msg->recovery_count; // Retrieve the recovery count from the metric collector
        if(recovery_count > recovery_threshold_)
        {
            //Reduce maximum velocity using Nav2 node
            controller_client_->set_parameters(
                {rclcpp::Parameter("FollowPath.max_vel_x", 
                            get_parameter("reduced_max_vel_x").as_double()),     // It was 0.3 before
                rclcpp::Parameter("FollowPath.max_vel_theta ", 
                            get_parameter("reduced_max_vel_theta").as_double())} // It was 1.0 before
            );

            RCLCPP_WARN(get_logger(), "Recovery count high (%d), Reduced max linear and angular velocity!", 
                        recovery_count);
        }

        // Restore normal values (from yaml file)
        if (recovery_count <= recovery_threshold_)
        {
            controller_client_->set_parameters({
            rclcpp::Parameter("FollowPath.max_vel_x",    get_parameter("normal_max_vel_x").as_double()),
            rclcpp::Parameter("FollowPath.max_vel_theta", get_parameter("normal_max_vel_theta").as_double())
            });
        }

        const auto distance_to_goal = msg->distance_to_goal 
        const auto distance_tol =  msg->distance_tolerance;
        if(msg->goal_reached == true || distance_to_goal < 1/2 * distance_tol) // Robot reached the goal or is really close to it
        {
            // Evaluate the moving average accurancy : 1 - relative error
            double relattive_err = std::clamp(std::abs((distance_to_goal - distance_tol) / distance_tol), 0.0, 1.0);
            sum_accuracy_ += 1 - relattive_err;
            window_count_++;
        }

        if(window_count_ >= window_size_)
        {
            mean_accuracy_ = sum_accuracy_ / static_cast<double>(window_size_);
        }

        if(mean_accuracy_ < accuracy_threshold_)
        {
            // Increase the goal tolerance
            controller_client_->set_parameters(
                {rclcpp::Parameter("goal_checker.xy_goal_tolerance", 
                        get_parameter("increaded_xy_goal_tolerance").as_double()),  // It was 0.25 before
                rclcpp::Parameter("goal_checker.yaw_goal_tolerance", 
                        get_parameter("increased_yaw_goal_tolerance").as_double())} // It was 0.25 before
            );

            RCLCPP_WARN(get_logger(), "Average accurancy is low (%.3f), Increase the goal tolerance!", 
                        mean_accuracy_);
        }

        // Restore normal values (from yaml file)
        if(mean_accuracy_ >= accuracy_threshold_)
        {
            controller_client_->set_parameters(
                {rclcpp::Parameter("goal_checker.xy_goal_tolerance", 
                        get_parameter("normal_xy_goal_tolerance").as_double()),  
                rclcpp::Parameter("goal_checker.yaw_goal_tolerance", 
                        get_parameter("normal_yaw_goal_tolerance").as_double())} 
            );
        }



    }
    
} // namespace tbot3_nav_monitor