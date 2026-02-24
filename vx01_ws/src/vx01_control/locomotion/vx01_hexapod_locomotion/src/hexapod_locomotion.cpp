#include "vx01_hexapod_locomotion/hexapod_locomotion.hpp"
#include <cmath>
#include <iostream>

namespace vx01_hexapod_locomotion {

    // =========================================================================
    // Constructor
    // =========================================================================
    HexapodLocomotion::HexapodLocomotion(double L1, double L2, double L3,
                                         double body_radius, double beta_angle)
        : L1_(L1), L2_(L2), L3_(L3),
          body_radius_(body_radius), beta_angle_(beta_angle),
          state_(LocomotionState::STOPPED),
          velocity_x_(0.0), velocity_y_(0.0), velocity_omega_(0.0),
          gait_time_(0.0), step_period_(2.0),
          step_length_(108.67), step_height_(22.78), track_width_(110.0),
          home_x_(170.0), home_y_(0.0), home_z_(-100.0)
    {
        // ----------------------------------------------------------------
        // Leg mounting angles (body frame, measured from +X axis)
        //
        // From the hexapod drawing:
        //   Leg 1 (index 0) – right,        0°  → 0
        //   Leg 2 (index 1) – right-front, +62.91° → +beta
        //   Leg 3 (index 2) – left-front,  +125.82° → +2*beta   (approx.)
        //   Leg 4 (index 3) – left,         180°  → π
        //   Leg 5 (index 4) – left-rear,   -125.82° → -(π - 2*beta)
        //   Leg 6 (index 5) – right-rear,  -62.91°  → -beta
        //
        // beta_angle_ is the angular gap between adjacent legs on one side,
        // expressed in radians (e.g. 62.91° ≈ 1.098 rad).
        // ----------------------------------------------------------------
        leg_angles_ = {
             0.0,                      // Leg 0: right
             beta_angle_,              // Leg 1: right-front
             2.0 * beta_angle_,        // Leg 2: left-front
             M_PI,                     // Leg 3: left
            -(M_PI - 2.0*beta_angle_), // Leg 4: left-rear
            -beta_angle_               // Leg 5: right-rear
        };

        // ----------------------------------------------------------------
        // Gait pattern  –  constructor signature: GaitPattern(S, T, A)
        //   S = stride length, T = track width, A = step height
        // ----------------------------------------------------------------
        gait_pattern_ = std::make_shared<gait::GaitPattern>(
            step_length_, track_width_, step_height_);

        // ----------------------------------------------------------------
        // Leg controllers
        // ----------------------------------------------------------------
        initializeLegControllers();

        // ----------------------------------------------------------------
        // Joint-angle buffer
        // ----------------------------------------------------------------
        current_joint_angles_.resize(18, 0.0);

        std::cout << "HexapodLocomotion initialized" << std::endl;
        std::cout << "  Dimensions: L1=" << L1_ << "  L2=" << L2_
                  << "  L3=" << L3_ << std::endl;
        std::cout << "  Body radius: " << body_radius_ << std::endl;
        std::cout << "  Home position (leg-local): ("
                  << home_x_ << ", " << home_y_ << ", " << home_z_ << ")"
                  << std::endl;
    }

    // =========================================================================
    // Destructor
    // =========================================================================
    HexapodLocomotion::~HexapodLocomotion() {
        stop();
    }

    // =========================================================================
    // initializeLegControllers
    //
    // FIX: LegController constructor requires
    //        (int leg_id, double rotation_angle, double x_start,
    //         double L1,  double L2,             double L3)
    //      The old code passed only (L1, L2, L3) — three wrong arguments.
    //
    // x_start is the radial distance from the body centre to the coxa pivot,
    // i.e. body_radius_.
    // =========================================================================
    void HexapodLocomotion::initializeLegControllers() {
        leg_controllers_.clear();
        leg_controllers_.reserve(6);

        for (int i = 0; i < 6; ++i) {
            auto controller = std::make_shared<control::LegController>(
                i,               // leg_id
                leg_angles_[i],  // rotation_angle  – mounts the leg in the body frame
                body_radius_,    // x_start         – coxa pivot offset from body centre
                L1_, L2_, L3_    // link lengths
            );
            leg_controllers_.push_back(controller);
        }
    }

    // =========================================================================
    // applyIK  (private helper)
    //
    // FIX: The old code called leg_controllers_[i]->solveIK(...) which does not
    //      exist.  The correct method is setFootPosition(x, y, z) which returns
    //      bool and stores the result internally.  Angles are then retrieved via
    //      getTheta1/2/3().
    //
    // foot_x/y/z are expressed in the leg-local frame.
    // =========================================================================
    void HexapodLocomotion::applyIK(int leg_index,
                                    double foot_x, double foot_y, double foot_z)
    {
        bool ok = leg_controllers_[leg_index]->setFootPosition(foot_x, foot_y, foot_z);

        if (!ok) {
            // IK failed – keep the previous joint angles unchanged.
            std::cerr << "IK failed for leg " << leg_index
                      << "  target=(" << foot_x << ", " << foot_y
                      << ", " << foot_z << ")" << std::endl;
            return;
        }

        current_joint_angles_[leg_index * 3 + 0] = leg_controllers_[leg_index]->getTheta1();
        current_joint_angles_[leg_index * 3 + 1] = leg_controllers_[leg_index]->getTheta2();
        current_joint_angles_[leg_index * 3 + 2] = leg_controllers_[leg_index]->getTheta3();
    }

    // =========================================================================
    // stand
    //
    // FIX 1: The old code called the non-existent solveIK().  Now uses applyIK().
    // FIX 2: The old update() called stand() on every tick while STANDING, which
    //        reset gait_time_ and printed "Standing..." every frame.  That loop
    //        is fixed in update() – stand() is now a one-shot command only.
    // =========================================================================
    void HexapodLocomotion::stand() {
        std::cout << "Standing..." << std::endl;

        state_          = LocomotionState::STANDING;
        velocity_x_     = 0.0;
        velocity_y_     = 0.0;
        velocity_omega_ = 0.0;
        gait_time_      = 0.0;

        // Move all legs to the home position (expressed in leg-local frame).
        for (int i = 0; i < 6; ++i) {
            applyIK(i, home_x_, home_y_, home_z_);
        }
    }

    // =========================================================================
    // walk
    // =========================================================================
    void HexapodLocomotion::walk() {
        std::cout << "Walking – vx=" << velocity_x_
                  << "  vy="    << velocity_y_
                  << "  omega=" << velocity_omega_ << std::endl;

        state_     = LocomotionState::WALKING;
        gait_time_ = 0.0;
    }

    // =========================================================================
    // stop
    // =========================================================================
    void HexapodLocomotion::stop() {
        std::cout << "Stopping..." << std::endl;

        state_          = LocomotionState::STOPPED;
        velocity_x_     = 0.0;
        velocity_y_     = 0.0;
        velocity_omega_ = 0.0;
    }

    // =========================================================================
    // getState
    // =========================================================================
    LocomotionState HexapodLocomotion::getState() const {
        return state_;
    }

    // =========================================================================
    // setVelocity / getVelocity
    // =========================================================================
    void HexapodLocomotion::setVelocity(double vx, double vy, double omega) {
        velocity_x_     = vx;
        velocity_y_     = vy;
        velocity_omega_ = omega;
    }

    void HexapodLocomotion::getVelocity(double& vx, double& vy,
                                        double& omega) const {
        vx    = velocity_x_;
        vy    = velocity_y_;
        omega = velocity_omega_;
    }

    // =========================================================================
    // update
    //
    // FIX: The old code called stand() on every STANDING tick, which reset
    //      gait_time_ and flooded stdout.  Now we simply keep the stored joint
    //      angles – stand() already set them once.
    // =========================================================================
    void HexapodLocomotion::update(double dt) {
        if (state_ == LocomotionState::STOPPED) {
            return;
        }

        if (state_ == LocomotionState::STANDING) {
            // Nothing to do each tick – joint angles were set once by stand().
            return;
        }

        // ---- WALKING -------------------------------------------------------
        gait_time_ += dt;
        if (gait_time_ >= step_period_) {
            gait_time_ -= step_period_;
        }

        for (int i = 0; i < 6; ++i) {
            updateLeg(i);
        }
    }

    // =========================================================================
    // updateLeg
    //
    // FIX: replaced solveIK() with applyIK().
    // =========================================================================
    void HexapodLocomotion::updateLeg(int leg_index) {
        // Tripod gait:  legs 0, 2, 4  share phase offset 0.0
        //               legs 1, 3, 5  share phase offset 0.5
        double phase_offset = (leg_index % 2 == 0) ? 0.0 : 0.5;
        double phase = std::fmod(gait_time_ / step_period_ + phase_offset, 1.0);

        double target_x, target_y, target_z;
        calculateFootTarget(leg_index, phase, target_x, target_y, target_z);

        applyIK(leg_index, target_x, target_y, target_z);
    }

    // =========================================================================
    // calculateFootTarget
    //
    // FIX: The old implementation computed foot targets in the body frame and
    //      then added home_x_/y_/z_ directly.  Because LegController::setFootPosition
    //      expects coordinates in the leg-local frame, the home position must
    //      also be expressed in that frame.  home_x_/y_/z_ ARE already defined
    //      in the leg-local frame (forward along the coxa axis), so we only need
    //      to ensure the swing/stance deltas are also in that frame.
    //
    //      The velocity is transformed into the leg-local frame first via
    //      transformVelocity(), and all displacements are added in that frame
    //      before handing off to applyIK().
    // =========================================================================
    void HexapodLocomotion::calculateFootTarget(int leg_index, double phase,
                                                double& x, double& y, double& z)
    {
        // Velocity in leg-local frame
        double local_vx, local_vy;
        transformVelocity(leg_index, velocity_x_, velocity_y_,
                          local_vx, local_vy);

        // Maximum displacement from neutral during one half-cycle
        double step_x = (local_vx * step_period_) / 2.0;
        double step_y = (local_vy * step_period_) / 2.0;

        if (phase < 0.5) {
            // ------ SWING phase (foot in air) --------------------------------
            double t = phase * 2.0;   // remap to [0, 1]

            double t2  = t * t;
            double t3  = t2 * t;
            double mt  = 1.0 - t;
            double mt2 = mt * mt;
            double mt3 = mt2 * mt;

            // Cubic Bezier control points along the forward (x) axis
            double p0_x = -step_x,        p3_x = step_x;
            double p1_x = -step_x * 0.5,  p2_x = step_x * 0.5;

            // Height profile: rises to step_height_ and returns to 0
            double p0_z = 0.0, p1_z = step_height_,
                   p2_z = step_height_, p3_z = 0.0;

            x = mt3*p0_x + 3.0*mt2*t*p1_x + 3.0*mt*t2*p2_x + t3*p3_x;
            y = step_y * (2.0 * t - 1.0);   // sweeps from -step_y to +step_y
            z = mt3*p0_z + 3.0*mt2*t*p1_z + 3.0*mt*t2*p2_z + t3*p3_z;

        } else {
            // ------ STANCE phase (foot on ground, pushing back) --------------
            double t = (phase - 0.5) * 2.0;  // remap to [0, 1]

            x =  step_x - 2.0 * step_x * t;
            y =  step_y - 2.0 * step_y * t;
            z =  0.0;
        }

        // Add home (neutral) position – already in leg-local frame
        x += home_x_;
        y += home_y_;
        z += home_z_;
    }

    // =========================================================================
    // transformVelocity
    //
    // Rotates the body-frame translational velocity into leg i's local frame,
    // then adds the tangential velocity due to the yaw-rate command.
    // =========================================================================
    void HexapodLocomotion::transformVelocity(int leg_index,
                                              double vx, double vy,
                                              double& local_vx,
                                              double& local_vy)
    {
        double leg_angle = leg_angles_[leg_index];
        double cos_a = std::cos(leg_angle);
        double sin_a = std::sin(leg_angle);

        // Passive rotation: body frame → leg frame
        local_vx =  vx * cos_a + vy * sin_a;
        local_vy = -vx * sin_a + vy * cos_a;

        // Yaw contribution: v_tangential = omega × r
        double base_x, base_y;
        getLegBasePosition(leg_index, base_x, base_y);

        local_vx += -velocity_omega_ * base_y;
        local_vy +=  velocity_omega_ * base_x;
    }

    // =========================================================================
    // getLegBasePosition
    //
    // Returns the coxa pivot position in the body frame.
    // =========================================================================
    void HexapodLocomotion::getLegBasePosition(int leg_index,
                                               double& base_x,
                                               double& base_y)
    {
        double angle = leg_angles_[leg_index];
        base_x = body_radius_ * std::cos(angle);
        base_y = body_radius_ * std::sin(angle);
    }

    // =========================================================================
    // Joint-angle accessors
    // =========================================================================
    std::vector<double> HexapodLocomotion::getJointAngles() const {
        return current_joint_angles_;
    }

    void HexapodLocomotion::getLegAngles(int leg_index,
                                         double& theta1,
                                         double& theta2,
                                         double& theta3) const
    {
        if (leg_index < 0 || leg_index >= 6) {
            theta1 = theta2 = theta3 = 0.0;
            return;
        }
        theta1 = current_joint_angles_[leg_index * 3 + 0];
        theta2 = current_joint_angles_[leg_index * 3 + 1];
        theta3 = current_joint_angles_[leg_index * 3 + 2];
    }

    // =========================================================================
    // Gait parameter setters / getters
    //
    // FIX: The old setStepLength / setStepHeight called gait_pattern_->setStride()
    //      and gait_pattern_->setHeight() which are not declared in GaitPattern.
    //      GaitPattern has no runtime setters; parameters are set at construction.
    //      The correct approach is to rebuild the GaitPattern with the new values.
    // =========================================================================
    void HexapodLocomotion::setStepLength(double length) {
        step_length_ = length;
        // Rebuild gait pattern with the new stride length
        gait_pattern_ = std::make_shared<gait::GaitPattern>(
            step_length_, track_width_, step_height_);
    }

    void HexapodLocomotion::setStepHeight(double height) {
        step_height_ = height;
        // Rebuild gait pattern with the new step height
        gait_pattern_ = std::make_shared<gait::GaitPattern>(
            step_length_, track_width_, step_height_);
    }

    void HexapodLocomotion::setStepPeriod(double period) {
        step_period_ = period;
    }

    double HexapodLocomotion::getStepLength() const { return step_length_; }
    double HexapodLocomotion::getStepHeight() const { return step_height_; }
    double HexapodLocomotion::getStepPeriod() const { return step_period_; }

    // =========================================================================
    // Home-position setters / getters
    // =========================================================================
    void HexapodLocomotion::setHomePosition(double x, double y, double z) {
        home_x_ = x;
        home_y_ = y;
        home_z_ = z;
    }

    void HexapodLocomotion::getHomePosition(double& x, double& y,
                                            double& z) const {
        x = home_x_;
        y = home_y_;
        z = home_z_;
    }

} // namespace vx01_hexapod_locomotion