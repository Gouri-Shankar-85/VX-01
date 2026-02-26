#ifndef VX01_HEXAPOD_LOCOMOTION_HEXAPOD_LOCOMOTION_HPP
#define VX01_HEXAPOD_LOCOMOTION_HEXAPOD_LOCOMOTION_HPP

#include "vx01_hexapod_locomotion/control/leg_controller.hpp"
#include "vx01_hexapod_locomotion/gait/gait_pattern.hpp"
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

        // Home foot position in leg-local frame (mm)
        double home_x_;
        double home_y_;
        double home_z_;

        std::vector<double> current_joint_angles_;

    public:
        HexapodLocomotion(double L1, double L2, double L3,
                          double body_radius, double beta_angle);
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

        void   setStepLength(double length);
        void   setStepHeight(double height);
        void   setStepPeriod(double period);

        double getBlockPeriod() const   { return step_period_ / 6.0; }
        void   setBlockPeriod(double bp) { step_period_ = bp * 6.0; }

        double getStepLength() const;
        double getStepHeight() const;
        double getStepPeriod() const;

        void setHomePosition(double x, double y, double z);
        void getHomePosition(double& x, double& y, double& z) const;

    private:
        void initializeLegControllers();
        void applyIK(int leg_index, double foot_x, double foot_y, double foot_z);
        void updateLeg(int leg_index);
        void rebuildGaitPattern();
    };

}

#endif