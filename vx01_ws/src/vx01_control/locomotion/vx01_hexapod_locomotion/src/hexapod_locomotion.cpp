#include "vx01_hexapod_locomotion/hexapod_locomotion.hpp"
#include <cmath>
#include <iostream>

namespace vx01_hexapod_locomotion {

    // ─────────────────────────────────────────────────────────────────────────
    //  Constructor
    //
    //  FIX #1: home position updated for actual robot link lengths.
    //  FIX #3: leg_angles_ now derived from beta=62.91 deg (hexapod.JPG CAD).
    //          leg 0 at 0 deg, CCW spacing of beta between adjacent legs.
    // ─────────────────────────────────────────────────────────────────────────
    HexapodLocomotion::HexapodLocomotion(double L1, double L2, double L3,
                                         double body_radius, double beta_angle)
        : L1_(L1), L2_(L2), L3_(L3),
          body_radius_(body_radius), beta_angle_(beta_angle),
          state_(LocomotionState::STOPPED),
          velocity_x_(0.0), velocity_y_(0.0), velocity_omega_(0.0),
          gait_time_(0.0), step_period_(2.0),
          step_length_(110.0),
          step_height_(22.78),
          track_width_(170.0),
          home_x_(170.0), home_y_(0.0), home_z_(-80.0)
    {
        // Leg mounting angles from hexapod.JPG CAD drawing.
        // beta = 62.91 deg = 1.09792 rad
        // Legs numbered 0..5 CCW starting from right side (+X axis).
        const double b = beta_angle;

        leg_angles_ = {
             0.0,       // leg 0: right side          (  0 deg)
             b,         // leg 1: front-right         (+62.91 deg)
             2.0 * b,   // leg 2: front-left          (+125.82 deg)
             M_PI,      // leg 3: left side            (+180 deg)
            -2.0 * b,   // leg 4: rear-left           (-125.82 deg)
            -b          // leg 5: rear-right           (-62.91 deg)
        };

        gait_pattern_ = std::make_shared<gait::GaitPattern>(
            home_x_, step_length_, step_height_);

        initializeLegControllers();
        current_joint_angles_.resize(18, 0.0);
    }

    HexapodLocomotion::~HexapodLocomotion() { stop(); }

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
            home_x_, step_length_, step_height_);
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
        for (int i = 0; i < 6; ++i) {
            applyIK(i, home_x_, home_y_, home_z_);
        }
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

    // ─────────────────────────────────────────────────────────────────────────
    //  update
    //
    //  Advances the gait clock by dt. When a full block period has elapsed,
    //  nextBlock() is called so the gait table advances.
    // ─────────────────────────────────────────────────────────────────────────
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

    // ─────────────────────────────────────────────────────────────────────────
    //  updateLeg
    //
    //  Computes the desired foot position for this leg in leg-local frame
    //  and sends it to IK.
    //
    //  The gait pattern gives (gait_x, gait_y, gait_z) in leg-local frame:
    //    gait_x = S (reach depth, constant)
    //    gait_y = stride offset in [-T/2, +T/2]   (swing arc or drag line)
    //    gait_z = lift height [0, A]               (0 during drag)
    //
    //  Velocity scaling:
    //    The body-frame velocity (velocity_x_, velocity_y_) is projected onto
    //    the leg's Y axis (the stride direction) to get the leg-local stride
    //    speed. This scales gait_y so faster commands produce longer steps.
    //
    //  FIX: ik_z = home_z_ + gait_z (not home_z_ alone) so the foot actually
    //  lifts during swing. Previously gait_z was not added to home_z_.
    // ─────────────────────────────────────────────────────────────────────────
    void HexapodLocomotion::updateLeg(int leg_index)
    {
        const double block_period = step_period_ / 6.0;
        double t = (block_period > 1e-9) ? (gait_time_ / block_period) : 0.0;
        t = std::max(0.0, std::min(1.0, t));

        double gait_x, gait_y, gait_z;
        gait_pattern_->getFootPosition(leg_index, t, gait_x, gait_y, gait_z);

        // Project body velocity onto this leg's stride axis (leg-local Y)
        const double leg_rot = leg_controllers_[leg_index]->getRotationAngle();
        const double vy_leg  =
            -std::sin(leg_rot) * velocity_x_
            + std::cos(leg_rot) * velocity_y_
            + velocity_omega_ * body_radius_;

        // Scale gait_y by the ratio of actual vs nominal stride speed
        const double nominal_speed = step_length_ / (step_period_ * 0.5);
        const double scale = (nominal_speed > 1e-6) ? (vy_leg / nominal_speed) : 1.0;

        const double ik_x = home_x_;
        const double ik_y = gait_y * scale;
        const double ik_z = home_z_ + gait_z;   // FIX: add gait_z for foot lift

        applyIK(leg_index, ik_x, ik_y, ik_z);
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

    void HexapodLocomotion::setStepLength(double length) { step_length_ = length; rebuildGaitPattern(); }
    void HexapodLocomotion::setStepHeight(double height) { step_height_ = height; rebuildGaitPattern(); }
    void HexapodLocomotion::setStepPeriod(double period) { step_period_ = period; }

    double HexapodLocomotion::getStepLength() const { return step_length_; }
    double HexapodLocomotion::getStepHeight() const { return step_height_; }
    double HexapodLocomotion::getStepPeriod() const { return step_period_; }

    void HexapodLocomotion::setHomePosition(double x, double y, double z) {
        home_x_ = x; home_y_ = y; home_z_ = z;
        rebuildGaitPattern();   // rebuild so S = new home_x
    }

    void HexapodLocomotion::getHomePosition(double& x, double& y, double& z) const {
        x = home_x_; y = home_y_; z = home_z_;
    }

}  // namespace vx01_hexapod_locomotion