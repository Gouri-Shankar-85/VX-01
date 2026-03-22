#include "vx01_locomotion_control/hexapod_approach_node.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

namespace vx01_locomotion_control {

// Lifecycle transition IDs
static constexpr uint8_t TRANSITION_CONFIGURE   = lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE;
static constexpr uint8_t TRANSITION_ACTIVATE    = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
static constexpr uint8_t TRANSITION_DEACTIVATE  = lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE;
static constexpr uint8_t TRANSITION_CLEANUP     = lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP;

HexapodApproachNode::HexapodApproachNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("hexapod_approach_node", options),
  state_(State::IDLE), victim_x_(0.0), victim_y_(0.0), victim_valid_(false)
{
    declare_parameter("arrival_distance", 0.5);
    declare_parameter("turn_threshold",   0.15);

    arrival_distance_ = get_parameter("arrival_distance").as_double();
    turn_threshold_   = get_parameter("turn_threshold").as_double();

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Lifecycle service clients for walk node
    walk_change_state_ = create_client<lifecycle_msgs::srv::ChangeState>(
        "/hexapod_forward_walk_node/change_state");
    walk_get_state_ = create_client<lifecycle_msgs::srv::GetState>(
        "/hexapod_forward_walk_node/get_state");

    // Lifecycle service clients for turn node
    turn_change_state_ = create_client<lifecycle_msgs::srv::ChangeState>(
        "/hexapod_turn_node/change_state");
    turn_get_state_ = create_client<lifecycle_msgs::srv::GetState>(
        "/hexapod_turn_node/get_state");

    victim_sub_ = create_subscription<vx01_msgs::msg::VictimArray>(
        "/victim_detections", 10,
        std::bind(&HexapodApproachNode::cbVictims, this, std::placeholders::_1));

    // Wait for lifecycle services
    RCLCPP_INFO(get_logger(), "Waiting for lifecycle services...");
    walk_change_state_->wait_for_service(10s);
    walk_get_state_->wait_for_service(10s);
    turn_change_state_->wait_for_service(10s);
    turn_get_state_->wait_for_service(10s);

    // Configure both nodes at startup — they sit in inactive state
    RCLCPP_INFO(get_logger(), "Configuring walk and turn nodes...");
    changeState(walk_change_state_, TRANSITION_CONFIGURE);
    changeState(turn_change_state_, TRANSITION_CONFIGURE);
    RCLCPP_INFO(get_logger(), "Both nodes configured — approach node ready.");

    // Control loop at 2 Hz
    control_timer_ = create_wall_timer(500ms,
        std::bind(&HexapodApproachNode::controlLoop, this));
}

void HexapodApproachNode::cbVictims(const vx01_msgs::msg::VictimArray::SharedPtr msg)
{
    if (msg->victims.empty()) {
        victim_valid_ = false;
        return;
    }

    // Best detection by confidence
    const auto& best = *std::max_element(
        msg->victims.begin(), msg->victims.end(),
        [](const auto& a, const auto& b){ return a.confidence < b.confidence; });

    // Transform to base_link
    geometry_msgs::msg::PointStamped pt_in, pt_out;
    pt_in.header = best.position.header;
    pt_in.point  = best.position.point;

    try {
        auto transform = tf_buffer_->lookupTransform(
            "base_link", pt_in.header.frame_id,
            rclcpp::Time(0),
            rclcpp::Duration::from_seconds(0.1));
        tf2::doTransform(pt_in, pt_out, transform);
        victim_x_     = pt_out.point.x;
        victim_y_     = pt_out.point.y;
        victim_valid_ = true;
    } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(get_logger(), "TF failed: %s", ex.what());
        victim_valid_ = false;
    }
}

void HexapodApproachNode::controlLoop()
{
    if (state_ == State::ARRIVED) return;

    if (!victim_valid_) {
        if (state_ != State::IDLE) {
            RCLCPP_INFO(get_logger(), "Victim lost — stopping.");
            if (state_ == State::WALKING) deactivateWalk();
            if (state_ == State::TURNING) deactivateTurn();
            state_ = State::IDLE;
        }
        return;
    }

    const double distance = std::sqrt(victim_x_*victim_x_ + victim_y_*victim_y_);
    const double bearing  = std::atan2(victim_y_, victim_x_);

    RCLCPP_INFO(get_logger(),
        "State=%-8s  dist=%.2fm  bearing=%.1f°",
        state_ == State::IDLE    ? "IDLE"    :
        state_ == State::TURNING ? "TURNING" :
        state_ == State::WALKING ? "WALKING" : "ARRIVED",
        distance, bearing * 180.0 / M_PI);

    if (distance <= arrival_distance_) {
        RCLCPP_INFO(get_logger(), "Arrived at victim! dist=%.2fm — stopping.", distance);
        if (state_ == State::WALKING) deactivateWalk();
        if (state_ == State::TURNING) deactivateTurn();
        state_ = State::ARRIVED;
        return;
    }

    if (std::abs(bearing) > turn_threshold_) {
        // Need to turn
        if (state_ == State::WALKING) {
            RCLCPP_INFO(get_logger(), "Bearing off — switching walk → turn.");
            deactivateWalk();
            state_ = State::IDLE;
        }
        if (state_ != State::TURNING) {
            int direction = (bearing > 0.0) ? +1 : -1;
            RCLCPP_INFO(get_logger(), "Turning %s (bearing=%.1f°)",
                direction > 0 ? "LEFT (CCW)" : "RIGHT (CW)",
                bearing * 180.0 / M_PI);
            activateTurn(direction);
            state_ = State::TURNING;
        }
    } else {
        // Bearing aligned — walk forward
        if (state_ == State::TURNING) {
            RCLCPP_INFO(get_logger(), "Aligned — switching turn → walk.");
            deactivateTurn();
            state_ = State::IDLE;
        }
        if (state_ != State::WALKING) {
            RCLCPP_INFO(get_logger(), "Walking toward victim at dist=%.2fm", distance);
            activateWalk();
            state_ = State::WALKING;
        }
    }
}

void HexapodApproachNode::activateWalk()
{
    changeState(walk_change_state_, TRANSITION_ACTIVATE);
}

void HexapodApproachNode::deactivateWalk()
{
    changeState(walk_change_state_, TRANSITION_DEACTIVATE);
}

void HexapodApproachNode::activateTurn(int direction)
{
    // Set turn_direction parameter before activating
    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(
        this, "/hexapod_turn_node");
    if (param_client->wait_for_service(2s)) {
        param_client->set_parameters({
            rclcpp::Parameter("turn_direction", direction)
        });
    }
    changeState(turn_change_state_, TRANSITION_ACTIVATE);
}

void HexapodApproachNode::deactivateTurn()
{
    changeState(turn_change_state_, TRANSITION_DEACTIVATE);
}

void HexapodApproachNode::changeState(
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr client,
    uint8_t transition_id)
{
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition_id;

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(
            get_node_base_interface(), future, 10s)
        != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(get_logger(), "changeState service call failed (transition=%d)", transition_id);
        return;
    }
    if (!future.get()->success) {
        RCLCPP_ERROR(get_logger(), "changeState rejected (transition=%d)", transition_id);
    }
}

}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<vx01_locomotion_control::HexapodApproachNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}