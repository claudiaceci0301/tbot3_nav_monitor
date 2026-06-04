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
    declare_parameter("normal_xy_goal_tolerance",      0.35);
    declare_parameter("normal_yaw_goal_tolerance",     0.35);
    declare_parameter("normal_inflation_radius",       0.5);
    declare_parameter("normal_gridbase_tolerance",     0.5);
    declare_parameter("normal_costmap_resolution",     0.05);
    declare_parameter("normal_costmap_width",          2);
    declare_parameter("normal_costmap_height",         2);

    // ── Adaptive values ──────────────────────────────────────────────────────
    declare_parameter("reduced_max_vel_x",             0.2);
    declare_parameter("reduced_max_vel_theta",          0.8);
    declare_parameter("increased_xy_goal_tolerance",   0.40);
    declare_parameter("increased_yaw_goal_tolerance",  0.40);
    declare_parameter("increased_inflation_radius",    0.6);
    declare_parameter("reduced_gridbase_tolerance",    0.35);
    declare_parameter("increased_costmap_resolution",  0.08);
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

    // ── Create subscriber ────────────────────────────────────────────────────
    metrics_sub_ = create_subscription<tbot3_nav_monitor::msg::NavigationMetrics>("/navigation_metrics", 10,
        std::bind(&AdaptiveController::metrics_callback, this, std::placeholders::_1));
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10,
        [this](const std::shared_ptr<const geometry_msgs::msg::PoseStamped> msg){
            std::lock_guard<std::mutex> lock(state_mutex_);
            send_nav2_goal(*msg);
        });
    
    // Create Publisher
    goal_status_pub_ = create_publisher<std_msgs::msg::UInt8>("/nav2_goal_status", 10);

    // Wait for services with timeout
    auto warn_if_missing = [this](auto & client, const char * name) {
        if (!client->wait_for_service(std::chrono::seconds(5)))
            RCLCPP_WARN(get_logger(), "%s not available at configure time", name);
    };
    warn_if_missing(controller_client_, "controller_server");
    warn_if_missing(costmap_client_,    "local_costmap");
    warn_if_missing(planner_client_,    "planner_server");
    warn_if_missing(reset_client_,      "metric_collector/reset");

    if(!nav2_client_->wait_for_action_server(std::chrono::seconds(10)))
    {     
        RCLCPP_ERROR(get_logger(), "Nav2 action server not available!");
        return CallbackReturn::FAILURE;
    }

    RCLCPP_INFO(get_logger(), "on_configure() called — node still INACTIVE");
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AdaptiveController::on_activate(const rclcpp_lifecycle::State & state)
{
    rclcpp_lifecycle::LifecycleNode::on_activate(state);
    
    // Activate lifecycle publisher (explicit call)
    goal_status_pub_->on_activate();

    RCLCPP_INFO(get_logger(), "on_activate() called — node ACTIVE");
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
AdaptiveController::on_deactivate(const rclcpp_lifecycle::State & state)
{
    rclcpp_lifecycle::LifecycleNode::on_deactivate(state);
    
    // Deactivate lifecycle publisher (explicit call)
    goal_status_pub_->on_deactivate();
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
    goal_status_pub_.reset();

    // Reset window state
    window_count_               = 0;
    sum_accuracy_               = 0.0;
    sum_obstacle_proximity_     = 0.0;
    mean_accuracy_              = 0.0;
    mean_obstacle_proximity_    = 0.0;
    efficiency_                 = 0.0;
    window_ready_               = false;
    last_recovery_count_        = 0;
    last_obstacle_too_close_    = false;
    navigation_active_.store(false);
    has_last_controller_params_ = false;
    has_last_costmap_params_    = false;
    has_last_planner_params_    = false;
    nav2_state_.store(Nav2State::UNKNOWN);

    RCLCPP_INFO(get_logger(), "on_cleanup() called — node UNCONFIGURED");
    return CallbackReturn::SUCCESS;
}

//  ── Private helpers method ─────────────────────
void AdaptiveController::send_nav2_goal(const geometry_msgs::msg::PoseStamped & goal)
{
    
    // Activate the robot navigation
    navigation_active_.store(true);
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

    // After sending a new goal the Srv client request MetricColletor to reset
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

//  ── Subscribers callback — all adaptive logic lives here ────────────────────
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

    // Publish the result msg
    auto status_msg = std_msgs::msg::UInt8{}; 

    switch(result.code)
    {
        case ResultCode::SUCCEEDED:
            nav2_state_.store(Nav2State::SUCCEEDED);
            status_msg.data = static_cast<uint8_t>(Nav2State::SUCCEEDED);
            RCLCPP_INFO(get_logger(), "Nav2 SUCCEEDED - GOAL REACHED!");
            break;
        case ResultCode::ABORTED:
            nav2_state_.store(Nav2State::ABORTED);
            status_msg.data = static_cast<uint8_t>(Nav2State::ABORTED);
            RCLCPP_INFO(get_logger(), "Nav2 ABORTED!");
            break;
        case ResultCode::CANCELED:
            nav2_state_.store(Nav2State::CANCELED);
            status_msg.data = static_cast<uint8_t>(Nav2State::CANCELED);
            RCLCPP_INFO(get_logger(), "Nav2 CANCELED!");
            break;
        default:
            RCLCPP_INFO(get_logger(), "Nav2 UNKNOWN!");
            break;
    }

    if(goal_status_pub_->is_activated())
    {
        goal_status_pub_->publish(status_msg);
    }
}

AdaptiveController::Nav2Params AdaptiveController::reset_to_normal() const
{
    // Restore all default Nav2 parameters for next navigation
    Nav2Params p;

    p.max_vel_x = get_parameter("normal_max_vel_x").as_double();
    p.max_vel_theta = get_parameter("normal_max_vel_theta").as_double();
    p.xy_goal_tolerance = get_parameter("normal_xy_goal_tolerance").as_double();
    p.yaw_goal_tolerance = get_parameter("normal_yaw_goal_tolerance").as_double();
    p.inflation_radius = get_parameter("normal_inflation_radius").as_double();
    p.gridbase_tolerance = get_parameter("normal_gridbase_tolerance").as_double();
    p.costmap_resolution = get_parameter("normal_costmap_resolution").as_double();
    p.costmap_width = get_parameter("normal_costmap_width").as_int();
    p.costmap_height = get_parameter("normal_costmap_height").as_int();

    return p; 
}

// Adaptive logic here - compute the parameters concerning the logic
AdaptiveController::Nav2Params AdaptiveController::compute_desired_params() const
{
    // Start from the normal (default) baseline
    Nav2Params nav2_params = reset_to_normal();

    // ── Condition 1: too many recoveries ─────────────────────────────────────
    // Robot is going too fast to react → reduce velocity + push inflation
    if (nav2_state_.load() != Nav2State::SUCCEEDED)
    {
        // We compare against the live recovery_count stored in last_metrics_
        // (see metrics_callback where it is cached before this call)
        if (last_recovery_count_ > recovery_threshold_)
        {
            nav2_params.max_vel_x     = get_parameter("reduced_max_vel_x").as_double();
            nav2_params.max_vel_theta = get_parameter("reduced_max_vel_theta").as_double();
            nav2_params.inflation_radius = std::max(nav2_params.inflation_radius,
                get_parameter("increased_inflation_radius").as_double());

            RCLCPP_WARN(get_logger(),
                "[Cond 1] Recovery count %d > threshold %d — velocity reduced",
                last_recovery_count_, recovery_threshold_);
        }
    }

    // ── Conditions 2–4 require at least one complete window ──────────────────
    // So if the window is not complete use the default params
    if (!window_ready_) 
        return nav2_params; 

    // ── ADAPTIVE LOGICE HERE  ────────────────────────────────────────────────
    
    // ── Condition 2: poor goal accuracy ──────────────────────────────────────
    // Robot consistently stops far from goal → relax goal checker tolerances
    if (mean_accuracy_ < accuracy_threshold_)
    {
        nav2_params.xy_goal_tolerance  = std::max(nav2_params.xy_goal_tolerance,
            get_parameter("increased_xy_goal_tolerance").as_double());
        nav2_params.yaw_goal_tolerance = std::max(nav2_params.yaw_goal_tolerance,
            get_parameter("increased_yaw_goal_tolerance").as_double());

        RCLCPP_WARN(get_logger(),
            "[Cond 2] Mean accuracy %.3f < %.3f — goal tolerance relaxed",
            mean_accuracy_, accuracy_threshold_);
    }

    // ── Condition 3: robot systematically close to obstacles ─────────────────
    // Dense environment → enlarge + refine local costmap
    if (mean_obstacle_proximity_ < obstacle_threshold_)
    {
        nav2_params.inflation_radius  = std::max(nav2_params.inflation_radius,
            get_parameter("increased_inflation_radius").as_double());
        //nav2_params.costmap_resolution = get_parameter("increased_costmap_resolution").as_double();
        //nav2_params.costmap_width      = get_parameter("increased_costmap_width").as_int();
        //nav2_params.costmap_height     = get_parameter("increased_costmap_height").as_int();

        // DEBUG
        RCLCPP_WARN(
        get_logger(),
        "Updating inflation radius to %.2f",
        nav2_params.inflation_radius);

        RCLCPP_WARN(get_logger(),
            "[Cond 3] Mean obstacle proximity %.3f < %.3f — costmap enlarged",
            mean_obstacle_proximity_, obstacle_threshold_);
    }

    // ── Condition 4: path efficiency low or immediate obstacle danger ─────────
    // Robot is taking long detours or getting too close → conservative planner
    if (efficiency_ < efficiency_threshold_ || last_obstacle_too_close_)
    {
        nav2_params.inflation_radius  = std::max(nav2_params.inflation_radius,
            get_parameter("increased_inflation_radius").as_double());
        nav2_params.gridbase_tolerance = get_parameter("reduced_gridbase_tolerance").as_double();

        RCLCPP_WARN(get_logger(),
            "[Cond 4] Efficiency %.3f or obstacle too close — conservative planner",
            efficiency_);
    }

    return nav2_params;
}


// Translates a Nav2Params into three async set_parameters calls, one per Nav2 server, once per decision cycle
// In apply_params: Diff and Rate limit check
void AdaptiveController::apply_params(const Nav2Params & param)
{
    const rclcpp::Time now = this->now(); // RCL_ROS_TIME colck type

    // ── Controller ───────────────────────────────────────────────────────────

    const bool ctrl_changed =
        !has_last_controller_params_ ||
        std::abs(param.max_vel_x - last_controller_params_.max_vel_x) > 0.02 ||          
        std::abs(param.xy_goal_tolerance - last_controller_params_.xy_goal_tolerance) > 0.02 ||
        std::abs(param.max_vel_theta - last_controller_params_.max_vel_theta) > 0.05 ||
        std::abs(param.yaw_goal_tolerance - last_controller_params_.yaw_goal_tolerance) > 0.05;

    if (ctrl_changed &&
        (now - last_controller_apply_time_).seconds() >= controller_apply_interval_)
    {
        controller_client_->set_parameters({
            rclcpp::Parameter("FollowPath.max_vel_x",                    param.max_vel_x),
            rclcpp::Parameter("FollowPath.max_vel_theta",                param.max_vel_theta),
            rclcpp::Parameter("general_goal_checker.xy_goal_tolerance",  param.xy_goal_tolerance),
            rclcpp::Parameter("general_goal_checker.yaw_goal_tolerance", param.yaw_goal_tolerance)
        });
        last_controller_params_      = param;
        last_controller_apply_time_  = now;
        has_last_controller_params_  = true;
    }

    // ── Costmap ───────────────────────────────────────────────────────────────
    const bool cmap_changed =
        !has_last_costmap_params_ ||
        std::abs(param.inflation_radius - last_costmap_params_.inflation_radius) > 0.05 ||
        std::abs(param.costmap_resolution - last_costmap_params_.costmap_resolution) > 0.01 ||
        param.costmap_width != last_costmap_params_.costmap_width ||
        param.costmap_height != last_costmap_params_.costmap_height;

    if (cmap_changed &&
        (now - last_costmap_apply_time_).seconds() >= costmap_apply_interval_)
    {
        RCLCPP_WARN(get_logger(),
        "Applying costmap params: inflation=%.3f res=%.3f w=%d h=%d",
        param.inflation_radius, param.costmap_resolution,
        param.costmap_width, param.costmap_height);

        costmap_client_->set_parameters({
            rclcpp::Parameter("inflation_layer.inflation_radius", param.inflation_radius),
            rclcpp::Parameter("resolution",                       param.costmap_resolution),
            rclcpp::Parameter("width",                            param.costmap_width),
            rclcpp::Parameter("height",                           param.costmap_height)
        });
        last_costmap_params_     = param;
        last_costmap_apply_time_ = now;
        has_last_costmap_params_ = true;
    }

    // ── Planner ───────────────────────────────────────────────────────────────
    const bool plan_changed =
        !has_last_planner_params_ ||
        std::abs(param.gridbase_tolerance - last_planner_params_.gridbase_tolerance) > 0.02;

    if (plan_changed &&
        (now - last_planner_apply_time_).seconds() >= planner_apply_interval_)
    {
        planner_client_->set_parameters({
            rclcpp::Parameter("GridBased.tolerance", param.gridbase_tolerance)
        });
        last_planner_params_     = param;
        last_planner_apply_time_ = now;
        has_last_planner_params_ = true;
    }
}

// ── Main Metric Callback Here ────────────────────────────────

void AdaptiveController::metrics_callback(
    const std::shared_ptr<const tbot3_nav_monitor::msg::NavigationMetrics> & msg)
{
    // Thread-Safe
    std::lock_guard<std::mutex> lock(state_mutex_);

    // Nav2 state guard (if navigation is not active)
    if(!navigation_active_.load()) return;

    // ── Read all fields from the message once ────────────────────────────────
    last_recovery_count_     = msg->recovery_count;
    last_obstacle_too_close_ = (msg->min_obstacle_distance < msg->obstacle_distance_tolerance);

    const auto distance_to_goal            = msg->distance_to_goal;
    const auto distance_tolerance          = msg->distance_tolerance;
    const auto min_obstacle_distance       = msg->min_obstacle_distance;
    const auto obstacle_distance_tolerance = msg->obstacle_distance_tolerance;
    const auto distance_traveled           = msg->distance_traveled;
    const auto optimal_path                = msg->optimal_path;

    // ── Efficiency [0, 1]: 1.0 = perfect straight path ──────────────────────
    if (distance_traveled > 0.0)
    {   
        // std::max to avoid inf or nan when distance_traveled is initially 0
        efficiency_ = std::clamp(optimal_path / std::max(distance_traveled, 1e-3), 0.0, 1.0);
    }

    // ── Window: accumulate accuracy and obstacle proximity ───────────────────
    // Only meaningful when the robot is near the goal
    if (msg->goal_reached || distance_to_goal < distance_tolerance)
    {
        // Accuracy: 1 - relative error (clamped to [0, 1])
        const double relative_err = std::clamp(
            std::abs((distance_to_goal - distance_tolerance) 
                    / distance_tolerance), 0.0, 1.0);
        sum_accuracy_ += 1.0 - relative_err;

        // Obstacle proximity: relative error of obstacle distance vs tolerance
        // High value → robot is too close to obstacles
        const double obstacle_err = std::clamp(
            std::abs((min_obstacle_distance - obstacle_distance_tolerance) 
                    / obstacle_distance_tolerance), 0.0, 1.0);
        sum_obstacle_proximity_ += obstacle_err;

        window_count_++;
    }

    // When window is full, compute means and reset accumulators
    if (window_count_ >= window_size_)
    {
        mean_accuracy_           = sum_accuracy_           / static_cast<double>(window_size_);
        mean_obstacle_proximity_ = sum_obstacle_proximity_ / static_cast<double>(window_size_);
        window_ready_ = true;

        RCLCPP_DEBUG(get_logger(),
            "Window complete | mean_accuracy: %.3f | mean_obstacle_proximity: %.3f | efficiency: %.3f",
            mean_accuracy_, mean_obstacle_proximity_, efficiency_);

        // Reset for next window
        sum_accuracy_           = 0.0;
        sum_obstacle_proximity_ = 0.0;
        window_count_           = 0;
    }

    // GOAL REACHED ?!
    if (msg->goal_reached)
    {
        if (navigation_active_.exchange(false))
        {
            apply_params(reset_to_normal());
            window_ready_ = false;
            window_count_ = 0;
            sum_accuracy_ = 0.0;
            sum_obstacle_proximity_ = 0.0;
            RCLCPP_INFO(get_logger(), "Goal reached — parameters restored to normal");
        }
        return; // Exit from loop
    }

    // If goal not reached:
    const Nav2Params desired = compute_desired_params();
    apply_params(desired);
}

}  // namespace tbot3_nav_monitor
