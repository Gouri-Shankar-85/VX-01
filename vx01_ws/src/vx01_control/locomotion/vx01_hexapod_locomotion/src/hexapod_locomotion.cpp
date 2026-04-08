#include "vx01_hexapod_locomotion/hexapod_locomotion.hpp"
#include "vx01_hexapod_locomotion/kinematics/inverse_kinematics.hpp"
#include <cmath>
#include <iostream>

namespace vx01_hexapod_locomotion {

    HexapodLocomotion::HexapodLocomotion(double L1, double L2, double L3,
                                     double body_radius, double beta_angle,
                                     double home_x, double home_y, double home_z,
                                     double step_length, double step_height, double step_period)
    : L1_(L1), L2_(L2), L3_(L3),
      body_radius_(body_radius), beta_angle_(beta_angle),
      state_(LocomotionState::STOPPED),
      velocity_x_(0.0), velocity_y_(0.0), velocity_omega_(0.0),
      gait_time_(0.0), step_period_(step_period),
      step_length_(step_length),
      step_height_(step_height),
      track_width_(home_x),
      home_x_(home_x), home_y_(home_y), home_z_(home_z),
      stance_femur_(0.0), stance_tibia_(0.0)
    {
        const double b = beta_angle;
        leg_angles_ = { 0.0, b, 2.0*b, M_PI, -2.0*b, -b };

        gait_pattern_ = std::make_shared<gait::GaitPattern>(
            home_x_, step_length_, step_height_, leg_angles_);

        initializeLegControllers();
        current_joint_angles_.resize(18, 0.0);
        validateWorkspace();
    }

    HexapodLocomotion::~HexapodLocomotion() { stop(); }

    void HexapodLocomotion::validateWorkspace()
    {
        const double max_reach = L2_ + L3_;

        struct CheckPoint { double x, z; const char* label; };
        CheckPoint pts[] = {
            { home_x_,  home_z_,                "home (local)"      },
            { home_x_,  home_z_ + step_height_, "lift_peak (local)" },
        };

        bool ok = true;
        for (auto& p : pts) {
            double r2   = p.x - L1_;
            double dist = std::sqrt(r2*r2 + p.z*p.z);
            if (dist > max_reach || r2 < 0.0) {
                std::cerr << "[HexapodLocomotion] WORKSPACE VIOLATION at '"
                          << p.label << "': dist=" << dist
                          << " max=" << max_reach << "\n";
                ok = false;
            }
        }
        if (ok) std::cout << "[HexapodLocomotion] Workspace check PASSED\n";
    }

    void HexapodLocomotion::initializeLegControllers()
    {
        leg_controllers_.clear();
        leg_controllers_.reserve(6);
        for (int i = 0; i < 6; ++i) {
            leg_controllers_.push_back(
                std::make_shared<control::LegController>(
                    i, leg_angles_[i], body_radius_, L1_, L2_, L3_));
        }
    }

    void HexapodLocomotion::rebuildGaitPattern()
    {
        gait_pattern_ = std::make_shared<gait::GaitPattern>(
            home_x_, step_length_, step_height_, leg_angles_);
    }

    void HexapodLocomotion::applyIK(int leg_index,
                                    double foot_x, double foot_y, double foot_z)
    {
        bool ok = leg_controllers_[leg_index]->setFootPosition(foot_x, foot_y, foot_z);
        if (!ok) {
            std::cerr << "[HexapodLocomotion] IK failed leg=" << leg_index
                      << " (" << foot_x << "," << foot_y << "," << foot_z << ")\n";
            return;
        }
        current_joint_angles_[leg_index*3+0] = leg_controllers_[leg_index]->getTheta1();
        current_joint_angles_[leg_index*3+1] = leg_controllers_[leg_index]->getTheta2();
        current_joint_angles_[leg_index*3+2] = leg_controllers_[leg_index]->getTheta3();
    }

    void HexapodLocomotion::stand()
    {
        state_ = LocomotionState::STANDING;
        velocity_x_ = velocity_y_ = velocity_omega_ = 0.0;
        gait_time_  = 0.0;
        gait_pattern_->reset();
        for (int i = 0; i < 6; ++i)
            applyIK(i, home_x_, home_y_, home_z_);
    }

    void HexapodLocomotion::walk()
    {
        state_     = LocomotionState::WALKING;
        gait_time_ = 0.0;
        gait_pattern_->reset();
    }

    void HexapodLocomotion::stop()
    {
        state_      = LocomotionState::STOPPED;
        velocity_x_ = velocity_y_ = velocity_omega_ = 0.0;
    }

    LocomotionState HexapodLocomotion::getState() const { return state_; }

    void HexapodLocomotion::setVelocity(double vx, double vy, double omega) {
        velocity_x_ = vx; velocity_y_ = vy; velocity_omega_ = omega;
    }
    void HexapodLocomotion::getVelocity(double& vx, double& vy, double& omega) const {
        vx = velocity_x_; vy = velocity_y_; omega = velocity_omega_;
    }

    void HexapodLocomotion::update(double dt)
    {
        if (state_ == LocomotionState::STOPPED ||
            state_ == LocomotionState::STANDING) return;

        const double block_period = step_period_ / 6.0;
        gait_time_ += dt;
        if (gait_time_ >= block_period) {
            gait_time_ -= block_period;
            gait_pattern_->nextBlock();
        }
        for (int i = 0; i < 6; ++i) updateLeg(i);
    }

    void HexapodLocomotion::updateLeg(int leg_index)
    {
        const double block_period = step_period_ / 6.0;
        double t = (block_period > 1e-9) ? (gait_time_ / block_period) : 0.0;
        t = std::max(0.0, std::min(1.0, t));

        double leg_x, leg_y, leg_z_delta;
        gait_pattern_->getFootPosition(leg_index, t, leg_x, leg_y, leg_z_delta);
        applyIK(leg_index, leg_x, leg_y, home_z_ + leg_z_delta);
    }

    std::vector<double> HexapodLocomotion::getJointAngles() const {
        return current_joint_angles_;
    }

    void HexapodLocomotion::getLegAngles(int leg_index,
                                         double& theta1, double& theta2, double& theta3) const
    {
        if (leg_index < 0 || leg_index >= 6) { theta1=theta2=theta3=0.0; return; }
        theta1 = current_joint_angles_[leg_index*3+0];
        theta2 = current_joint_angles_[leg_index*3+1];
        theta3 = current_joint_angles_[leg_index*3+2];
    }

    void HexapodLocomotion::sampleLegAnglesAt(int leg_index, double t,
                                               double& theta1, double& theta2, double& theta3)
    {
        t = std::max(0.0, std::min(1.0, t));

        double leg_x, leg_y, leg_z_delta;
        gait_pattern_->getFootPosition(leg_index, t, leg_x, leg_y, leg_z_delta);

        kinematics::InverseKinematics ik(L1_, L2_, L3_);
        double th1=0.0, th2=0.0, th3=0.0;
        bool ok = ik.compute(leg_x, leg_y, home_z_ + leg_z_delta, th1, th2, th3);
        if (!ok) {
            std::cerr << "[sampleLegAnglesAt] IK failed leg=" << leg_index << "\n";
            th1 = current_joint_angles_[leg_index*3+0];
            th2 = current_joint_angles_[leg_index*3+1];
            th3 = current_joint_angles_[leg_index*3+2];
        }
        theta1=th1; theta2=th2; theta3=th3;
    }

    void HexapodLocomotion::sampleSwingAtGlobalT(int leg_index, double global_t,
                                                  double& theta1, double& theta2, double& theta3)
    {
        global_t = std::max(0.0, std::min(1.0, global_t));

        if (leg_index < 0 || leg_index >= 6) {
            theta1 = theta2 = theta3 = 0.0;
            return;
        }

        const double half = step_length_ / 2.0;
        const double u    = 1.0 - global_t;

        // Swing trajectory in leg-local frame using Bézier curve
        // P0 = back, P1 = peak (lifted), P2 = forward
        const double foot_y_leg = u*u*(-half) + 2.0*u*global_t*0.0 + global_t*global_t*(half);
        const double foot_z_leg = u*u*home_z_ + 2.0*u*global_t*(home_z_ + 2.0*step_height_) + global_t*global_t*home_z_;
        const double foot_x_leg = home_x_;  // always at reach in leg-local X

        // Transform from leg-local frame to body frame using leg controller
        double foot_x_body, foot_y_body, foot_z_body;
        leg_controllers_[leg_index]->legToBodyFrame(
            foot_x_leg, foot_y_leg, foot_z_leg,
            foot_x_body, foot_y_body, foot_z_body);

        kinematics::InverseKinematics ik(L1_, L2_, L3_);
        double th1 = 0.0, th2 = 0.0, th3 = 0.0;
        bool ok = ik.compute(foot_x_body, foot_y_body, foot_z_body, th1, th2, th3);
        if (!ok) {
            std::cerr << "[sampleSwingAtGlobalT] IK failed leg=" << leg_index
                      << " t=" << global_t
                      << " foot_body=(" << foot_x_body << "," << foot_y_body << "," << foot_z_body << ")\n";
            th1 = current_joint_angles_[leg_index*3+0];
            th2 = current_joint_angles_[leg_index*3+1];
            th3 = current_joint_angles_[leg_index*3+2];
        }
        theta1 = th1;
        theta2 = th2;
        theta3 = th3;
    }

    void HexapodLocomotion::setStancePose(double femur, double tibia)
    {
        stance_femur_ = femur;
        stance_tibia_ = tibia;
    }

    void HexapodLocomotion::sampleDragAtGlobalT(int leg_index, double global_t,
                                                 double& theta1, double& theta2, double& theta3)
    {
        global_t = std::max(0.0, std::min(1.0, global_t));

        if (leg_index < 0 || leg_index >= 6) {
            theta1 = theta2 = theta3 = 0.0;
            return;
        }

        const double half = step_length_ / 2.0;
        // Drag (stance) phase: foot slides backward in leg-local frame
        // from +half (forward) to -half (backward) over the duration
        const double foot_y_leg = half - 2.0 * half * global_t;
        const double foot_x_leg = home_x_;  // always at reach in leg-local X
        const double foot_z_leg = home_z_;  // on ground

        // Transform from leg-local frame to body frame using leg controller
        double foot_x_body, foot_y_body, foot_z_body;
        leg_controllers_[leg_index]->legToBodyFrame(
            foot_x_leg, foot_y_leg, foot_z_leg,
            foot_x_body, foot_y_body, foot_z_body);
        
        kinematics::InverseKinematics ik(L1_, L2_, L3_);
        double th1=0.0, th2=0.0, th3=0.0;
        bool ok = ik.compute(foot_x_body, foot_y_body, foot_z_body, th1, th2, th3);
        if (!ok) {
            std::cerr << "[sampleDragAtGlobalT] IK failed leg=" << leg_index
                      << " t=" << global_t
                      << " foot_body=(" << foot_x_body << "," << foot_y_body << "," << foot_z_body << ")\n";
            th1 = current_joint_angles_[leg_index*3+0];
            th2 = current_joint_angles_[leg_index*3+1];
            th3 = current_joint_angles_[leg_index*3+2];
        }
        theta1=th1; theta2=th2; theta3=th3;
    }

    void HexapodLocomotion::setStepLength(double length) { step_length_ = length; rebuildGaitPattern(); }
    void HexapodLocomotion::setStepHeight(double height) { step_height_ = height; rebuildGaitPattern(); }
    void HexapodLocomotion::setStepPeriod(double period) { step_period_ = period; }

    double HexapodLocomotion::getStepLength() const { return step_length_; }
    double HexapodLocomotion::getStepHeight() const { return step_height_; }
    double HexapodLocomotion::getStepPeriod() const { return step_period_; }

    void HexapodLocomotion::setHomePosition(double x, double y, double z) {
        home_x_ = x; home_y_ = y; home_z_ = z;
        rebuildGaitPattern();
    }

    void HexapodLocomotion::getHomePosition(double& x, double& y, double& z) const {
        x = home_x_; y = home_y_; z = home_z_;
    }

    void HexapodLocomotion::setBodyRPY(double roll, double pitch, double yaw) {
        body_translation_.setRPY(roll, pitch, yaw);
    }

    void HexapodLocomotion::setBodyTranslation(double tx, double ty, double tz) {
        body_translation_.setTranslation(tx, ty, tz);
    }

    void HexapodLocomotion::setBodyPose(double tx, double ty, double tz,
                                        double roll, double pitch, double yaw) {
        body_translation_.setPose(tx, ty, tz, roll, pitch, yaw);
    }

}