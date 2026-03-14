#ifndef VX01_HEXAPOD_LOCOMOTION_HEXAPOD_LOCOMOTION_HPP
#define VX01_HEXAPOD_LOCOMOTION_HEXAPOD_LOCOMOTION_HPP

#include "vx01_hexapod_locomotion/control/leg_controller.hpp"
#include "vx01_hexapod_locomotion/gait/gait_pattern.hpp"
#include "vx01_hexapod_locomotion/body/body_kinematics.hpp"
#include "vx01_hexapod_locomotion/body/body_translation.hpp"
#include <vector>
#include <memory>

namespace vx01_hexapod_locomotion {

    enum class LocomotionState {
        STANDING,
        WALKING,
        STOPPED
    };

    class HexapodLocomotion {
    private:
        std::vector<std::shared_ptr<control::LegController>> leg_controllers_;
        std::shared_ptr<gait::GaitPattern>                   gait_pattern_;

        // Body pose module (orientation + translation)
        body::BodyTranslation body_translation_;

        double L1_, L2_, L3_;
        double body_radius_;
        double beta_angle_;

        std::vector<double> leg_angles_;

        LocomotionState state_;

        double velocity_x_;
        double velocity_y_;
        double velocity_omega_;

        double gait_time_;
        double step_period_;

        double step_length_;
        double step_height_;
        double track_width_;

        // Home foot position in O-frame (mm)
        double home_x_;
        double home_y_;
        double home_z_;

        std::vector<double> current_joint_angles_;

    public:
        HexapodLocomotion(double L1, double L2, double L3,
                  double body_radius, double beta_angle,
                  double home_x, double home_y, double home_z,
                  double step_length, double step_height, double step_period);
        ~HexapodLocomotion();

        void stand();
        void walk();
        void stop();

        LocomotionState getState() const;

        void setVelocity(double vx, double vy, double omega);
        void getVelocity(double& vx, double& vy, double& omega) const;

        void update(double dt);

        std::vector<double> getJointAngles() const;
        void getLegAngles(int leg_index,
                          double& theta1, double& theta2, double& theta3) const;

        // Sample IK for one leg at sub-block fraction t ∈ [0,1] within the
        // current gait block. Does NOT advance gait state. Used by the node
        // to build multi-point trajectories that trace the full Bezier arc.
        void sampleLegAnglesAt(int leg_index, double t,
                               double& theta1, double& theta2, double& theta3);

        // Gait parameters
        void   setStepLength(double length);
        void   setStepHeight(double height);
        void   setStepPeriod(double period);
        double getStepLength() const;
        double getStepHeight() const;
        double getStepPeriod() const;

        double getBlockPeriod() const   { return step_period_ / 6.0; }
        int    getGaitBlock()   const   { return gait_pattern_->getCurrentBlock(); }
        void   setBlockPeriod(double bp) { step_period_ = bp * 6.0; }

        void setHomePosition(double x, double y, double z);
        void getHomePosition(double& x, double& y, double& z) const;

        void setBodyRPY(double roll, double pitch, double yaw);

        void setBodyTranslation(double tx, double ty, double tz);

        // Combined pose
        void setBodyPose(double tx, double ty, double tz,
                         double roll, double pitch, double yaw);

    private:
        void initializeLegControllers();
        void applyIK(int leg_index, double foot_x, double foot_y, double foot_z);
        void updateLeg(int leg_index);
        void rebuildGaitPattern();
    };

}

#endif