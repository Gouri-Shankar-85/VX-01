#include "vx01_locomotion_control/hexapod_gait_node.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cmath>
#include <chrono>
#include <stdexcept>

using namespace std::chrono_literals;

namespace hexapod_gait
{

HexapodGaitNode::HexapodGaitNode(const rclcpp::NodeOptions & options)
: Node("hexapod_gait_node", options),
  cmd_vx_(0.0), cmd_vy_(0.0), cmd_omega_(0.0),
  gait_block_(0), gait_time_(0.0), block_period_(0.0), initialized_(false)
{
    loadParameters();
    block_period_ = step_period_ / 2.0;

    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    initLegGeometry();

    for (int i = 0; i < NUM_LEGS; ++i) {
        last_angles_[i] = {0.0, 0.0, 0.0};
    }

    for (int i = 0; i < NUM_LEGS; ++i) {
        action_clients_[i] = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/" + controller_names_[i] + "/follow_joint_trajectory");
    }

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10,
        std::bind(&HexapodGaitNode::cmdVelCallback, this, std::placeholders::_1));

    rclcpp::sleep_for(3s);
    waitForControllers();
    initHomeFootPositions();
    sendStandby();

    auto standby_ms = static_cast<int>((standby_duration_ + 0.5) * 1000.0);
    rclcpp::sleep_for(std::chrono::milliseconds(standby_ms));

    startGait();

    auto period_ms = static_cast<int>(1000.0 / update_rate_);
    gait_timer_ = create_wall_timer(
        std::chrono::milliseconds(period_ms),
        std::bind(&HexapodGaitNode::gaitTick, this));
}

void HexapodGaitNode::loadParameters()
{
    declare_parameter("L1", 60.55);
    declare_parameter("L2", 73.84);
    declare_parameter("L3", 112.16);
    declare_parameter("home_reach", 227.689);
    declare_parameter("home_z", -61.379);
    declare_parameter("step_length", 60.0);
    declare_parameter("step_height", 30.0);
    declare_parameter("step_period", 2.0);
    declare_parameter("standby_duration", 3.0);
    declare_parameter("update_rate", 50.0);
    declare_parameter("leg_controllers", std::vector<std::string>{
        "leg_0_controller", "leg_1_controller", "leg_2_controller",
        "leg_3_controller", "leg_4_controller", "leg_5_controller"});

    L1_               = get_parameter("L1").as_double();
    L2_               = get_parameter("L2").as_double();
    L3_               = get_parameter("L3").as_double();
    home_reach_       = get_parameter("home_reach").as_double();
    home_z_           = get_parameter("home_z").as_double();
    step_length_      = get_parameter("step_length").as_double();
    step_height_      = get_parameter("step_height").as_double();
    step_period_      = get_parameter("step_period").as_double();
    standby_duration_ = get_parameter("standby_duration").as_double();
    update_rate_      = get_parameter("update_rate").as_double();
    controller_names_ = get_parameter("leg_controllers").as_string_array();
}

void HexapodGaitNode::initLegGeometry()
{
    const std::array<std::string, NUM_LEGS> coxa_frames = {
        "coxa_link_0", "coxa_link_1", "coxa_link_2",
        "coxa_link_3", "coxa_link_4", "coxa_link_5"
    };

    for (int i = 0; i < NUM_LEGS; ++i) {
        leg_geometry_[i].coxa_frame  = coxa_frames[i];
        leg_geometry_[i].base_frame  = "base_link";
        leg_geometry_[i].joint_names = {
            "coxa_leg"  + std::to_string(i) + "_joint",
            "femur_leg" + std::to_string(i) + "_joint",
            "tibia_leg" + std::to_string(i) + "_joint"
        };
    }
}

void HexapodGaitNode::initHomeFootPositions()
{
    // Home foot position in each coxa_link frame:
    // x = home_reach (straight out along coxa X), y = 0, z = home_z
    // Transform to base_link using TF2 to store the base_link home position.

    rclcpp::sleep_for(500ms);

    for (int i = 0; i < NUM_LEGS; ++i) {
        geometry_msgs::msg::PointStamped pt_coxa, pt_base;
        pt_coxa.header.frame_id    = leg_geometry_[i].coxa_frame;
        pt_coxa.header.stamp       = rclcpp::Time(0);
        pt_coxa.point.x            = home_reach_ / 1000.0;
        pt_coxa.point.y            = 0.0;
        pt_coxa.point.z            = home_z_ / 1000.0;

        try {
            tf_buffer_->transform(pt_coxa, pt_base, "base_link",
                                  tf2::durationFromSec(2.0));
            foot_base_[i] = {
                pt_base.point.x * 1000.0,
                pt_base.point.y * 1000.0,
                pt_base.point.z * 1000.0
            };
            RCLCPP_INFO(get_logger(), "Leg %d home in base_link: (%.1f, %.1f, %.1f) mm",
                i, foot_base_[i][0], foot_base_[i][1], foot_base_[i][2]);
        } catch (const tf2::TransformException & ex) {
            RCLCPP_ERROR(get_logger(), "TF2 failed for leg %d home: %s", i, ex.what());
            // Fallback: keep zero
            foot_base_[i] = {0.0, 0.0, 0.0};
        }
    }
}

void HexapodGaitNode::waitForControllers()
{
    RCLCPP_INFO(get_logger(), "Waiting for all controllers...");
    for (int i = 0; i < NUM_LEGS; ++i) {
        while (!action_clients_[i]->wait_for_action_server(1s)) {
            RCLCPP_INFO(get_logger(), "Waiting for leg_%d_controller...", i);
        }
    }
    RCLCPP_INFO(get_logger(), "All controllers ready.");
}

bool HexapodGaitNode::allControllersReady() const
{
    for (int i = 0; i < NUM_LEGS; ++i) {
        if (!action_clients_[i]->action_server_is_ready()) return false;
    }
    return true;
}

void HexapodGaitNode::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    cmd_vx_    = msg->linear.x;
    cmd_vy_    = msg->linear.y;
    cmd_omega_ = msg->angular.z;
}

bool HexapodGaitNode::solveIK(double x, double y, double z,
                               double & t1, double & t2, double & t3) const
{
    // IK in coxa_link frame (mm).
    // Joint axes from URDF: all revolute around local Z with axis xyz="0 0 -1"
    // DH convention matches the library: coxa rotates in XY, femur/tibia in the
    // plane defined by coxa angle.

    t1 = std::atan2(y, x);

    const double c1 = std::cos(t1);
    if (std::abs(c1) < 1e-9) return false;

    const double r2  = x / c1 - L1_;
    if (r2 < 0.0) return false;

    const double r1  = std::sqrt(z * z + r2 * r2);
    const double max_reach = L2_ + L3_;
    const double min_reach = std::abs(L2_ - L3_);
    if (r1 > max_reach || r1 < min_reach) return false;

    const double phi2 = std::atan2(z, r2);
    const double cp1  = (L3_ * L3_ - L2_ * L2_ - r1 * r1) / (-2.0 * L2_ * r1);
    if (cp1 < -1.0 || cp1 > 1.0) return false;
    const double phi1 = std::acos(cp1);
    t2 = phi2 - phi1;

    const double cp3 = (r1 * r1 - L2_ * L2_ - L3_ * L3_) / (-2.0 * L2_ * L3_);
    if (cp3 < -1.0 || cp3 > 1.0) return false;
    t3 = M_PI - std::acos(cp3);

    return true;
}

bool HexapodGaitNode::footBaseToCoxa(int leg,
                                      const std::array<double, 3> & foot_base,
                                      std::array<double, 3> & foot_coxa) 
{
    geometry_msgs::msg::PointStamped pt_base, pt_coxa;
    pt_base.header.frame_id = "base_link";
    pt_base.header.stamp    = rclcpp::Time(0);
    pt_base.point.x         = foot_base[0] / 1000.0;
    pt_base.point.y         = foot_base[1] / 1000.0;
    pt_base.point.z         = foot_base[2] / 1000.0;

    try {
        tf_buffer_->transform(pt_base, pt_coxa,
                              leg_geometry_[leg].coxa_frame,
                              tf2::durationFromSec(0.05));
        foot_coxa = {
            pt_coxa.point.x * 1000.0,
            pt_coxa.point.y * 1000.0,
            pt_coxa.point.z * 1000.0
        };
        return true;
    } catch (const tf2::TransformException & ex) {
        auto clock = this->get_clock();

        RCLCPP_WARN_THROTTLE(
            this->get_logger(),
            *clock,
            1000,
            "message"
        );
        return false;
    }
}

std::array<double, 3> HexapodGaitNode::swingFootBase(int leg, double global_t) const
{
    // global_t in [0,1]: 0 = lift-off, 1 = touch-down
    // Foot sweeps from -step_length/2 to +step_length/2 in the body forward direction.
    // Body forward = +X_base (for vx), +Y_base (for vy), rotation for omega.
    //
    // With teleop: scale stride by velocity command magnitude.
    const double half = step_length_ / 2.0;

    // Stride vector in base_link (mm)
    // Linear velocity component
    const double stride_x = cmd_vx_  * half;
    const double stride_y = cmd_vy_  * half;

    // Rotational component: tangential to foot position
    // foot tangent = omega * (-foot_y, foot_x) normalised
    const double fx0 = foot_base_[leg][0];
    const double fy0 = foot_base_[leg][1];
    const double r   = std::sqrt(fx0 * fx0 + fy0 * fy0);
    double rot_x = 0.0, rot_y = 0.0;
    if (r > 1e-3) {
        rot_x = cmd_omega_ * (-fy0 / r) * half;
        rot_y = cmd_omega_ * ( fx0 / r) * half;
    }

    const double dx = stride_x + rot_x;
    const double dy = stride_y + rot_y;

    // Bezier quadratic arc: P0=start, P1=midpoint lifted, P2=end
    // start = home - (dx, dy), end = home + (dx, dy), mid = home at peak height
    const double t  = global_t;
    const double u  = 1.0 - t;

    const double p0x = foot_base_[leg][0] - dx;
    const double p0y = foot_base_[leg][1] - dy;
    const double p0z = foot_base_[leg][2];

    const double p1x = foot_base_[leg][0];
    const double p1y = foot_base_[leg][1];
    const double p1z = foot_base_[leg][2] + step_height_;

    const double p2x = foot_base_[leg][0] + dx;
    const double p2y = foot_base_[leg][1] + dy;
    const double p2z = foot_base_[leg][2];

    return {
        u * u * p0x + 2.0 * u * t * p1x + t * t * p2x,
        u * u * p0y + 2.0 * u * t * p1y + t * t * p2y,
        u * u * p0z + 2.0 * u * t * p1z + t * t * p2z
    };
}

std::array<double, 3> HexapodGaitNode::stanceFootBase(int leg) const
{
    // Stance foot stays at its current ground contact while the body moves over it.
    // The stance foot position is the touch-down point of the previous swing,
    // which is home + dx in the previous block's stride direction.
    // For simplicity and correctness: stance = foot_base_[leg] (home).
    // The body effectively moves because the swing legs step forward.
    return foot_base_[leg];
}

void HexapodGaitNode::sendStandby()
{
    RCLCPP_INFO(get_logger(), "Moving to standby pose.");

    for (int i = 0; i < NUM_LEGS; ++i) {
        std::array<double, 3> foot_coxa;
        if (!footBaseToCoxa(i, foot_base_[i], foot_coxa)) continue;

        double t1, t2, t3;
        if (!solveIK(foot_coxa[0], foot_coxa[1], foot_coxa[2], t1, t2, t3)) {
            RCLCPP_ERROR(get_logger(), "Standby IK failed for leg %d", i);
            continue;
        }

        last_angles_[i] = {t1, t2, t3};
        sendTrajectory(i, {{last_angles_[i]}}, {standby_duration_});
    }
}

void HexapodGaitNode::startGait()
{
    RCLCPP_INFO(get_logger(), "Starting tripod gait.");
    initialized_ = true;
    gait_block_  = 0;
    gait_time_   = 0.0;
}

void HexapodGaitNode::gaitTick()
{
    if (!initialized_) return;

    const double dt = 1.0 / update_rate_;
    gait_time_ += dt;

    if (gait_time_ >= block_period_) {
        gait_time_ -= block_period_;
        gait_block_ = (gait_block_ + 1) % 2;
        executeBlock();
    }
}

void HexapodGaitNode::executeBlock()
{
    if (!allControllersReady()) return;

    const bool group_a_swings = (gait_block_ == 0);
    const std::array<int, 3> swing_legs  = group_a_swings
        ? std::array<int,3>{0, 2, 4} : std::array<int,3>{1, 3, 5};
    const std::array<int, 3> stance_legs = group_a_swings
        ? std::array<int,3>{1, 3, 5} : std::array<int,3>{0, 2, 4};

    RCLCPP_INFO(get_logger(), "Block %d: swing [%d,%d,%d] stance [%d,%d,%d]",
        gait_block_,
        swing_legs[0], swing_legs[1], swing_legs[2],
        stance_legs[0], stance_legs[1], stance_legs[2]);

    const int    N              = 10;
    const double swing_duration = block_period_ * 0.95;
    const double stance_hold    = block_period_ * 0.95;

    // Swing legs: N waypoints along Bezier arc
    for (int leg : swing_legs) {
        std::vector<std::array<double, 3>> waypoints;
        std::vector<double>                times;

        // Anchor at t=0 from last known angles
        waypoints.push_back(last_angles_[leg]);
        times.push_back(0.0);

        for (int k = 1; k <= N; ++k) {
            const double global_t = static_cast<double>(k) / N;
            const auto   foot_b   = swingFootBase(leg, global_t);

            std::array<double, 3> foot_c;
            if (!footBaseToCoxa(leg, foot_b, foot_c)) continue;

            double t1, t2, t3;
            if (!solveIK(foot_c[0], foot_c[1], foot_c[2], t1, t2, t3)) {
                RCLCPP_WARN(get_logger(), "Swing IK failed leg %d t=%.2f", leg, global_t);
                continue;
            }

            last_angles_[leg] = {t1, t2, t3};
            waypoints.push_back({t1, t2, t3});
            times.push_back(swing_duration * global_t);
        }

        sendTrajectory(leg, waypoints, times);
    }

    // Stance legs: hold current position for block duration
    for (int leg : stance_legs) {
        const auto foot_b = stanceFootBase(leg);
        std::array<double, 3> foot_c;
        if (!footBaseToCoxa(leg, foot_b, foot_c)) continue;

        double t1, t2, t3;
        if (!solveIK(foot_c[0], foot_c[1], foot_c[2], t1, t2, t3)) {
            RCLCPP_WARN(get_logger(), "Stance IK failed leg %d", leg);
            continue;
        }

        last_angles_[leg] = {t1, t2, t3};

        // Two points: anchor at t=0 and hold at t=stance_hold
        sendTrajectory(leg,
            {last_angles_[leg], last_angles_[leg]},
            {0.0, stance_hold});
    }
}

void HexapodGaitNode::sendTrajectory(
    int leg,
    const std::vector<std::array<double, 3>> & waypoints,
    const std::vector<double> & times)
{
    if (waypoints.empty()) return;
    if (!action_clients_[leg]->action_server_is_ready()) return;

    auto goal = FollowJointTrajectory::Goal();
    goal.trajectory.joint_names = {
        leg_geometry_[leg].joint_names[0],
        leg_geometry_[leg].joint_names[1],
        leg_geometry_[leg].joint_names[2]
    };
    goal.trajectory.header.stamp = rclcpp::Time(0);

    for (size_t i = 0; i < waypoints.size(); ++i) {
        trajectory_msgs::msg::JointTrajectoryPoint pt;
        pt.positions       = {waypoints[i][0], waypoints[i][1], waypoints[i][2]};
        pt.time_from_start = rclcpp::Duration::from_seconds(times[i]);
        goal.trajectory.points.push_back(pt);
    }

    action_clients_[leg]->async_send_goal(goal);
}

} // namespace hexapod_gait

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<hexapod_gait::HexapodGaitNode>());
    rclcpp::shutdown();
    return 0;
}
