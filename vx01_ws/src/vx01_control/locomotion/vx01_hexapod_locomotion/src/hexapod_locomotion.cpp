#include "vx01_hexapod_locomotion/hexapod_locomotion.hpp"
#include <cmath>
#include <iostream>

namespace vx01_hexapod_locomotion {

    HexapodLocomotion::HexapodLocomotion(double L1, double L2, double L3,
                                         double body_radius, double beta_angle)
        : L1_(L1), L2_(L2), L3_(L3),
          body_radius_(body_radius), beta_angle_(beta_angle),
          state_(LocomotionState::STOPPED),
          velocity_x_(0.0), velocity_y_(0.0), velocity_omega_(0.0),
          gait_time_(0.0), step_period_(4.0),
          step_length_(80.0),
          step_height_(20.0),
          track_width_(108.67),
          home_x_(108.67), home_y_(0.0), home_z_(-80.0)

    {
        // Leg mounting angles MUST match the URDF coxa_legN_joint rpy-z values.
        // These angles define how body-frame velocity is projected onto each leg's
        // local stride axis in updateLeg(). Using wrong values here causes legs
        // to move in the wrong direction or not move at all.
        //
        // Source: vx01_description/urdf/hexapod/leg_N.xacro, coxa joint rpy-z:
        //   leg0: rpy="0 0 -1.5708"  →  -π/2  (right-side leg, points in -Y body)
        //   leg1: rpy="0 0 -0.7854"  →  -π/4  (front-right)
        //   leg2: rpy="0 0  0.7854"  →  +π/4  (front-left)
        //   leg3: rpy="0 0  1.5708"  →  +π/2  (left-side leg)
        //   leg4: rpy="0 0  2.392"   →  +2.392 (rear-left)
        //   leg5: rpy="0 0 -2.3562"  →  -2.3562 (rear-right)
        leg_angles_ = {
            -M_PI / 2.0,   // leg 0: right-side   (-90°)
            -M_PI / 4.0,   // leg 1: front-right  (-45°)
             M_PI / 4.0,   // leg 2: front-left   (+45°)
             M_PI / 2.0,   // leg 3: left-side    (+90°)
             2.392,        // leg 4: rear-left    (+137°)
            -2.3562        // leg 5: rear-right   (-135°)
        };

        gait_pattern_ = std::make_shared<gait::GaitPattern>(
            home_x_, step_length_, step_height_);

        initializeLegControllers();
        current_joint_angles_.resize(18, 0.0);
    }

    HexapodLocomotion::~HexapodLocomotion() { stop(); }

    void HexapodLocomotion::initializeLegControllers() {

        leg_controllers_.clear();
        leg_controllers_.reserve(6);

        for (int i = 0; i < 6; ++i) {
            
            leg_controllers_.push_back(std::make_shared<control::LegController>(
                i, leg_angles_[i], body_radius_, L1_, L2_, L3_));
        }
    }

    void HexapodLocomotion::rebuildGaitPattern() {

        gait_pattern_ = std::make_shared<gait::GaitPattern>(
            home_x_, step_length_, step_height_);
    }

    void HexapodLocomotion::applyIK(int leg_index,
                                    double foot_x, double foot_y, double foot_z) {

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

    void HexapodLocomotion::stand() {

        state_ = LocomotionState::STANDING;
        velocity_x_ = velocity_y_ = velocity_omega_ = 0.0;
        gait_time_ = 0.0;
        gait_pattern_->reset();
        for (int i = 0; i < 6; ++i) {
            applyIK(i, home_x_, home_y_, home_z_);
        }
    }

    void HexapodLocomotion::walk() {

        state_ = LocomotionState::WALKING;
        gait_time_ = 0.0;
        gait_pattern_->reset();
    }

    void HexapodLocomotion::stop() {

        state_ = LocomotionState::STOPPED;
        velocity_x_ = velocity_y_ = velocity_omega_ = 0.0;
    }

    LocomotionState HexapodLocomotion::getState() const { return state_; }

    void HexapodLocomotion::setVelocity(double vx, double vy, double omega) {
        velocity_x_ = vx; velocity_y_ = vy; velocity_omega_ = omega;
    }

    void HexapodLocomotion::getVelocity(double& vx, double& vy, double& omega) const {
        vx = velocity_x_; vy = velocity_y_; omega = velocity_omega_;
    }

    void HexapodLocomotion::update(double dt) {
        if (state_ == LocomotionState::STOPPED ||
            state_ == LocomotionState::STANDING) return;

        double block_period = step_period_ / 6.0;
        gait_time_ += dt;
        if (gait_time_ >= block_period) {
            gait_time_ -= block_period;
            gait_pattern_->nextBlock();
        }
        for (int i = 0; i < 6; ++i) updateLeg(i);
    }

    void HexapodLocomotion::updateLeg(int leg_index) {

        double block_period = step_period_ / 6.0;
        double t = (block_period > 1e-9) ? (gait_time_ / block_period) : 0.0;
        t = std::max(0.0, std::min(1.0, t));

        double gait_x, gait_y, gait_z;
        gait_pattern_->getFootPosition(leg_index, t, gait_x, gait_y, gait_z);

        double leg_rot = leg_controllers_[leg_index]->getRotationAngle();

        double vy_leg = -std::sin(leg_rot) * velocity_x_
                    + std::cos(leg_rot) * velocity_y_
                    + velocity_omega_ * body_radius_; 

        double nominal_speed = step_length_ / (step_period_ * 0.5);
        double scale = (nominal_speed > 1e-6) ? (vy_leg / nominal_speed) : 0.0;

        double ik_x = home_x_;               
        double ik_y = gait_y * scale;
        double ik_z = home_z_ + gait_z;

        applyIK(leg_index, ik_x, ik_y, ik_z);
    }

    std::vector<double> HexapodLocomotion::getJointAngles() const {
        return current_joint_angles_;
    }

    void HexapodLocomotion::getLegAngles(int leg_index,
                                         double& theta1, double& theta2, double& theta3) const {

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
    }

    void HexapodLocomotion::getHomePosition(double& x, double& y, double& z) const {
        x = home_x_; y = home_y_; z = home_z_;
    }

}