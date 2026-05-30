#include "tbot3_nav_monitor/adaptive_controller.hpp"

#include <algorithm>  // std::clamp, std::max
#include <chrono>
#include <cmath>

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

//  ── Constructor ─────────────────────────────────────────────────────────────

AdaptiveController::AdaptiveController(const std::string & node_name, const rclcpp::NodeOptions & options) :
                                    rclcpp_lifecycle::LifecycleNode(node_name, options)
{
    // ── Declare thresholds ───────────────────────────────────────────────────
    declare_parameter("recovery_threshold",   3);    // >3 recovery → reduce velocity
    declare_parameter("accuracy_threshold",   0.7);  // <0.7 mean accuracy → relax goal tolerance
    declare_parameter("efficiency_threshold", 0.75); // <0.75 efficiency → conservative plan
    declare_parameter("obstacle_threshold",   0.4);  // <0.4 mean proximity → complex environment
    declare_parameter("window_size",          10);   // evaluate every 10 messages

    // ── Default values from params.yaml ─────────────────────────────────────
    declare_parameter("normal_max_vel_x",             0.3);
    declare_parameter("normal_max_vel_theta",          1.0);
    declare_parameter("normal_xy_goal_tolerance",      0.25);
    declare_parameter("normal_yaw_goal_tolerance",     0.25);
    declare_parameter("normal_inflation_radius",       0.5);
    declare_parameter("normal_gridbase_tolerance",     0.5);
    declare_parameter("normal_costmap_resolution",     0.05);
    declare_parameter("normal_costmap_width",          2);
    declare_parameter("normal_costmap_height",         2);

    // ── Adaptive values ──────────────────────────────────────────────────────
    declare_parameter("reduced_max_vel_x",             0.15);
    declare_parameter("reduced_max_vel_theta",          0.5);
    declare_parameter("increased_xy_goal_tolerance",   0.35);
    declare_parameter("increased_yaw_goal_tolerance",  0.35);
    declare_parameter("increased_inflation_radius",    0.6);
    declare_parameter("reduced_gridbase_tolerance",    0.25);
    declare_parameter("increased_costmap_resolution",  0.1);
    declare_parameter("increased_costmap_width",       3);
    declare_parameter("increased_costmap_height",      3);

    // ── Cache threshold values ───────────────────────────────────────────────
    recovery_threshold_   = get_parameter("recovery_threshold").as_int();
    accuracy_threshold_   = get_parameter("accuracy_threshold").as_double();
    efficiency_threshold_ = get_parameter("efficiency_threshold").as_double();
    obstacle_threshold_   = get_parameter("obstacle_threshold").as_double();
    window_size_          = get_parameter("window_size").as_int();

    RCLCPP_INFO(get_logger(), "AdaptiveController node %s created", node_name.c_str());
}

// ── Lifecycle callbacks ─────────────────────────────────────────────────────

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AdaptiveController::on_configure(const rclcpp_lifecycle::State & state)
{
    rclcpp_lifecycle::LifecycleNode::on_configure(state);

    // ── Create Nav2 parameter clients ───────────────────────────────────────
    // AsyncParametersClient: non-blocking, allows runtime parameter changes
    controller_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
        this, "controller_server");
    costmap_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
        this, "local_costmap");
    planner_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
        this, "planner_server");
    
    // Srv Client
    reset_client_ = create_client<std_srvs::srv::Trigger>("/metric_collector/reset");
    
    // Nav2 Action Client
    nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

    // Wait for services with timeout
    // (change WARN to FAILURE when Nav2 environment is ready)
    if (!controller_client_->wait_for_service(std::chrono::seconds(5)))
        RCLCPP_WARN(get_logger(), "Controller_server not available!");

    if (!costmap_client_->wait_for_service(std::chrono::seconds(5)))
        RCLCPP_WARN(get_logger(), "Local_costmap not available!");

    if (!planner_client_->wait_for_service(std::chrono::seconds(5)))
        RCLCPP_WARN(get_logger(), "Planner_server not available!");
    
    if(!reset_client_->wait_for_service(std::chrono::seconds(5)))
        RCLCPP_WARN(get_logger(), "Reset client not available!");

    // ── Create subscriber ────────────────────────────────────────────────────
    metrics_sub_ = create_subscription<tbot3_nav_monitor::msg::NavigationMetrics>("/navigation_metrics", 10,
        std::bind(&AdaptiveController::metrics_callback, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10,
        std::bind(&AdaptiveController::goal_send_callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "on_configure() called — node INACTIVE");
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AdaptiveController::on_activate(const rclcpp_lifecycle::State & state)
{
    rclcpp_lifecycle::LifecycleNode::on_activate(state);
    // No publisher or timer to activate in this node
    RCLCPP_INFO(get_logger(), "on_activate() called — node ACTIVE");
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AdaptiveController::on_deactivate(const rclcpp_lifecycle::State & state)
{
    rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
    RCLCPP_INFO(get_logger(), "on_deactivate() called — node INACTIVE");
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AdaptiveController::on_cleanup(const rclcpp_lifecycle::State & state)
{
    rclcpp_lifecycle::LifecycleNode::on_cleanup(state);

    // Release all resources
    controller_client_.reset();
    costmap_client_.reset();
    planner_client_.reset();
    metrics_sub_.reset();
    goal_sub_.reset();
    nav2_client_.reset();
    reset_client_.reset();

    // Reset window state
    window_count_            = 0;
    sum_accuracy_            = 0.0;
    sum_obstacle_proximity_  = 0.0;
    mean_accuracy_           = 0.0;
    mean_obstacle_proximity_ = 0.0;
    efficiency_              = 0.0;
    prev_goal_reached_       = false;
    nav2_state_.store(Nav2State::UNKNOWN);

    RCLCPP_INFO(get_logger(), "on_cleanup() called — node UNCONFIGURED");
    return CallbackReturn::SUCCESS;
}

//  ── Private method ─────────────────────
void AdaptiveController::send_nav2_goal(const geometry_msgs::msg::PoseStamped & goal)
{
    if(!nav2_client_->wait_for_action_server(std::chrono::seconds(5)))
        RCLCPP_WARN(get_logger(), "Nav2 action server not available!");

    nav2_msgs::action::NavigateToPose::Goal nav2_goal;
    nav2_goal.pose = goal;

    // All the callbacks here
    auto send_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_options.goal_response_callback = std::bind(&AdaptiveController::goal_response_callback, this, std::placeholders::_1);
    send_options.feedback_callback      = std::bind(&AdaptiveController::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_options.result_callback        = std::bind(&AdaptiveController::result_callback, this, std::placeholders::_1);
    
    // Reset nav2 state before sending the new goal
    reset_nav2_state();
    
    // Send a new Nav2 goal
    RCLCPP_INFO(get_logger(), "Sending new Nav2 goal to the Service!");
    nav2_client_->async_send_goal(nav2_goal, send_options); 

    // After sending a new goal the Srv client request the reset
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    // Send the request without waiting for the reply
    reset_client_->async_send_request(request,
        // SharedFuture is the future result
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
        {   // If the reply has arrived from the MetricCollector get it
            if(future.get()->success) // success srv boolean
            {
                RCLCPP_INFO(get_logger(), "MetricCollector reset confirmed!");
            }
            else
            {
                RCLCPP_WARN(get_logger(), "MetricCollector reset failed!");
            }
        }
    );
}

void AdaptiveController::reset_nav2_state()
{
    nav2_state_.store(Nav2State::UNKNOWN);
}

//  ── Subscriber callback — all adaptive logic lives here ─────────────────────
void AdaptiveController::goal_response_callback(const rclcpp_action::ClientGoalHandle<
        nav2_msgs::action::NavigateToPose>::SharedPtr goal_received)
{
    if(!goal_received)
    {
        RCLCPP_ERROR(get_logger(), "Goal rejected by server!");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result!");
    }
}

void AdaptiveController::feedback_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
{
    (void)feedback; //NOLINT
}

void AdaptiveController::result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
{
    // std::atomic used - Thread-safe state shared between callbacks and control loop
    using ResultCode = rclcpp_action::ResultCode;

    switch(result.code)
    {
        case ResultCode::SUCCEEDED:
            nav2_state_.store(Nav2State::SUCCEEDED);
            RCLCPP_INFO(get_logger(), "Nav2 SUCCEDED - GOAL REACHED!");
            break;
        case ResultCode::ABORTED:
            nav2_state_.store(Nav2State::ABORTED);
            RCLCPP_INFO(get_logger(), "Nav2 ABORTED!");
            break;
        case ResultCode::CANCELED:
            nav2_state_.store(Nav2State::CANCELED);
            RCLCPP_INFO(get_logger(), "Nav2 CANCELED!");
            break;
        default:
            RCLCPP_INFO(get_logger(), "Nav2 UNKNOWN!");
    }
}

// ── Callbacks ────────────────────────────────

void AdaptiveController::goal_send_callback(const std::shared_ptr<const geometry_msgs::msg::PoseStamped> & msg)
{
    send_nav2_goal(*msg);
}

void AdaptiveController::metrics_callback(
    const std::shared_ptr<const tbot3_nav_monitor::msg::NavigationMetrics> & msg)
{
    // ── Read all fields from the message once ────────────────────────────────
    const auto recovery_count        = msg->recovery_count;
    const auto distance_to_goal      = msg->distance_to_goal;
    const auto distance_tol          = msg->distance_tolerance;
    const auto min_obstacle_distance = msg->min_obstacle_distance;
    const auto obstacle_tol          = msg->obstacle_distance_tolerance;
    const auto distance_traveled     = msg->distance_traveled;
    const auto optimal_path          = msg->optimal_path;

    // ── Efficiency [0, 1]: 1.0 = perfect straight path ──────────────────────
    if (distance_traveled > 0.0)
    {
        efficiency_ = std::clamp(optimal_path / distance_traveled, 0.0, 1.0);
    }

    // ── Window: accumulate accuracy and obstacle proximity ───────────────────
    // Only meaningful when the robot is near the goal
    if (msg->goal_reached || distance_to_goal < 0.5 * distance_tol)
    {
        // Accuracy: 1 - relative error (clamped to [0, 1])
        const double relative_err = std::clamp(
            std::abs((distance_to_goal - distance_tol) / distance_tol), 0.0, 1.0);
        sum_accuracy_ += 1.0 - relative_err;

        // Obstacle proximity: relative error of obstacle distance vs tolerance
        // High value → robot is too close to obstacles
        const double obstacle_err = std::clamp(
            std::abs((min_obstacle_distance - obstacle_tol) / obstacle_tol), 0.0, 1.0);
        sum_obstacle_proximity_ += obstacle_err;

        window_count_++;
    }

    // When window is full, compute means and reset accumulators
    if (window_count_ >= window_size_)
    {
        mean_accuracy_           = sum_accuracy_           / static_cast<double>(window_size_);
        mean_obstacle_proximity_ = sum_obstacle_proximity_ / static_cast<double>(window_size_);

        RCLCPP_DEBUG(get_logger(),
            "Window complete | mean_accuracy: %.3f | mean_obstacle_proximity: %.3f | efficiency: %.3f",
            mean_accuracy_, mean_obstacle_proximity_, efficiency_);

        // Reset for next window
        sum_accuracy_           = 0.0;
        sum_obstacle_proximity_ = 0.0;
        window_count_           = 0;
    }

    //  ── Adaptive logic ──────────────────────────────────────
    // Goal reached if and only if Nav2 is SUCCEEDED and the geometry check is achieved
    const bool nav2_succeeded = (nav2_state_.load() == Nav2State::SUCCEEDED);
    const bool geometry_ok    = msg->goal_reached; // from MetricCollector

    if ((nav2_succeeded || geometry_ok) && !prev_goal_reached_) // If is ok and the state has changed
    {
        prev_goal_reached_ = true; 

        // Restore all default Nav2 parameters for next navigation
        controller_client_->set_parameters({
            rclcpp::Parameter("FollowPath.max_vel_x",
                get_parameter("normal_max_vel_x").as_double()),
            rclcpp::Parameter("FollowPath.max_vel_theta",
                get_parameter("normal_max_vel_theta").as_double()),
            rclcpp::Parameter("general_goal_checker.xy_goal_tolerance",
                get_parameter("normal_xy_goal_tolerance").as_double()),
            rclcpp::Parameter("general_goal_checker.yaw_goal_tolerance",
                get_parameter("normal_yaw_goal_tolerance").as_double())
        });

        costmap_client_->set_parameters({
            rclcpp::Parameter("inflation_layer.inflation_radius",
                get_parameter("normal_inflation_radius").as_double()),
            rclcpp::Parameter("resolution",
                get_parameter("normal_costmap_resolution").as_double()),
            rclcpp::Parameter("width",
                get_parameter("normal_costmap_width").as_int()),
            rclcpp::Parameter("height",
                get_parameter("normal_costmap_height").as_int())
        });

        planner_client_->set_parameters({
            rclcpp::Parameter("GridBased.tolerance",
                get_parameter("normal_gridbase_tolerance").as_double())
        });

        RCLCPP_INFO(get_logger(), "Goal reached — all Nav2 parameters restored to defaults");
        
        return;
    }

    // ── Block 1: Recovery count high ─────────────────────────────────────────
    // Too many recoveries → reduce velocity + increase inflation radius
    if (recovery_count > recovery_threshold_)
    {
        controller_client_->set_parameters({
            rclcpp::Parameter("FollowPath.max_vel_x",
                get_parameter("reduced_max_vel_x").as_double()),
            rclcpp::Parameter("FollowPath.max_vel_theta",
                get_parameter("reduced_max_vel_theta").as_double())
        });
        RCLCPP_WARN(get_logger(),
            "Recovery count high (%d) — velocity reduced", recovery_count);
    }
    else
    {
        controller_client_->set_parameters({
            rclcpp::Parameter("FollowPath.max_vel_x",
                get_parameter("normal_max_vel_x").as_double()),
            rclcpp::Parameter("FollowPath.max_vel_theta",
                get_parameter("normal_max_vel_theta").as_double())
        });
    }

    // ── Block 2: Accuracy poor ───────────────────────────────────────────────
    // Consistently low accuracy → relax goal tolerance
    if (mean_accuracy_ < accuracy_threshold_)
    {
        controller_client_->set_parameters({
            rclcpp::Parameter("general_goal_checker.xy_goal_tolerance",
                get_parameter("increased_xy_goal_tolerance").as_double()),
            rclcpp::Parameter("general_goal_checker.yaw_goal_tolerance",
                get_parameter("increased_yaw_goal_tolerance").as_double())
        });
        RCLCPP_WARN(get_logger(),
            "Mean accuracy low (%.3f) — goal tolerance relaxed", mean_accuracy_);
    }
    else
    {
        controller_client_->set_parameters({
            rclcpp::Parameter("general_goal_checker.xy_goal_tolerance",
                get_parameter("normal_xy_goal_tolerance").as_double()),
            rclcpp::Parameter("general_goal_checker.yaw_goal_tolerance",
                get_parameter("normal_yaw_goal_tolerance").as_double())
        });
    }

    // ── Block 3: Inflation radius — take the most conservative value ─────────
    // Combines signals from: recovery count, obstacle proximity, efficiency
    // std::max ensures the most protective value always wins when multiple
    // conditions are true simultaneously
    double inflation = get_parameter("normal_inflation_radius").as_double();

    if (recovery_count > recovery_threshold_)
        inflation = std::max(inflation,
            get_parameter("increased_inflation_radius").as_double());

    if (mean_obstacle_proximity_ < obstacle_threshold_)
        inflation = std::max(inflation,
            get_parameter("increased_inflation_radius").as_double());

    if (efficiency_ < efficiency_threshold_ ||
        recovery_count > 0 ||
        min_obstacle_distance < obstacle_tol)
        inflation = std::max(inflation,
            get_parameter("increased_inflation_radius").as_double());

    costmap_client_->set_parameters({
        rclcpp::Parameter("inflation_layer.inflation_radius", inflation)
    });

    // ── Block 4: Complex environment → adjust full costmap ───────────────────
    // Systematically close to obstacles → enlarge and refine costmap
    if (mean_obstacle_proximity_ < obstacle_threshold_)
    {
        costmap_client_->set_parameters({
            rclcpp::Parameter("resolution",
                get_parameter("increased_costmap_resolution").as_double()),
            rclcpp::Parameter("width",
                get_parameter("increased_costmap_width").as_int()),
            rclcpp::Parameter("height",
                get_parameter("increased_costmap_height").as_int())
        });
        RCLCPP_WARN(get_logger(),
            "Complex environment detected — costmap enlarged and refined");
    }
    else
    {
        costmap_client_->set_parameters({
            rclcpp::Parameter("resolution",
                get_parameter("normal_costmap_resolution").as_double()),
            rclcpp::Parameter("width",
                get_parameter("normal_costmap_width").as_int()),
            rclcpp::Parameter("height",
                get_parameter("normal_costmap_height").as_int())
        });
    }

    // ── Block 5: Obstacle avoidance inefficient → conservative plan ──────────
    // Path much longer than optimal, or recovery, or too close to obstacles
    if (efficiency_ < efficiency_threshold_ ||
        recovery_count > 0 ||
        min_obstacle_distance < obstacle_tol)
    {
        planner_client_->set_parameters({
            rclcpp::Parameter("GridBased.tolerance",
                get_parameter("reduced_gridbase_tolerance").as_double())
        });
        RCLCPP_WARN(get_logger(),
            "Obstacle avoidance inefficient (efficiency: %.3f) — conservative plan activated",
            efficiency_);
    }
    else
    {
        planner_client_->set_parameters({
            rclcpp::Parameter("GridBased.tolerance",
                get_parameter("normal_gridbase_tolerance").as_double())
        });
    }
}

}  // namespace tbot3_nav_monitor
