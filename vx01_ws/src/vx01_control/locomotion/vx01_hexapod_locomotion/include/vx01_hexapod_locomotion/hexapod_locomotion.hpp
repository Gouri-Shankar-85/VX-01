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
                std::shared_ptr<gait::GaitPattern> gait_pattern_;

                // Robot dimensions (mm)
                double L1_;           // Coxa  length
                double L2_;           // Femur length
                double L3_;           // Tibia length
                double body_radius_;  // Coxa pivot distance from body centre
                double beta_angle_;   // Angular separation between adjacent legs (rad)

                // Leg mounting angles measured from body +X axis (rad)
                // From slide 10: i=γ, j=−β+γ, k=β+γ, l=π+γ, m=−β+γ, n=β+γ  (γ=0)
                std::vector<double> leg_angles_;

                LocomotionState state_;

                // Velocity commands
                double velocity_x_;      // Forward  (mm/s)
                double velocity_y_;      // Lateral  (mm/s)
                double velocity_omega_;  // Yaw-rate (rad/s)

                // Gait timing
                double gait_time_;    // Elapsed time in current gait block (s)
                double step_period_;  // Full gait cycle duration (s); block = step_period_/6

                // Step geometry
                double step_length_;  // Stride length T  (mm)
                double step_height_;  // Step height  A  (mm)
                double track_width_;  // Leg reach / track width S  (mm)

                // Home foot position in leg-local frame (mm)
                double home_x_;
                double home_y_;
                double home_z_;

                // Flat storage of all 18 joint angles [leg0_t1, leg0_t2, leg0_t3, ...]
                std::vector<double> current_joint_angles_;

            public:
                HexapodLocomotion(double L1, double L2, double L3,
                                  double body_radius, double beta_angle);
                ~HexapodLocomotion();

                // High-level commands
                void stand();
                void walk();
                void stop();

                LocomotionState getState() const;

                // Velocity interface (mm/s, mm/s, rad/s)
                void setVelocity(double vx, double vy, double omega);
                void getVelocity(double& vx, double& vy, double& omega) const;

                // Main update – call at fixed rate (e.g. 50 Hz)
                void update(double dt);

                // Joint-angle accessors
                std::vector<double> getJointAngles() const;
                void getLegAngles(int leg_index,
                                  double& theta1, double& theta2, double& theta3) const;

                // Gait parameter setters/getters
                void   setStepLength(double length);
                void   setStepHeight(double height);
                void   setBlockPeriod(double period);
                void   setStepPeriod(double period);

                double getStepLength() const;
                double getStepHeight() const;
                double getBlockPeriod() const;
                double getStepPeriod() const;

                // Home position (leg-local frame, mm)
                void setHomePosition(double x, double y, double z);
                void getHomePosition(double& x, double& y, double& z) const;

            private:
                void initializeLegControllers();

                // Apply IK for leg_index with foot position in leg-LOCAL frame
                void applyIK(int leg_index, double foot_x, double foot_y, double foot_z);

                // Drive one leg for the current gait state
                void updateLeg(int leg_index);

                // Rebuild the gait pattern object (called when parameters change)
                void rebuildGaitPattern();

                // Compute foot target at arbitrary gait phase [0,1)
                void calculateFootTarget(int leg_index, double phase,
                                         double& x, double& y, double& z);

                // Transform body-frame velocity into leg-local frame
                void transformVelocity(int leg_index, double vx, double vy,
                                       double& local_vx, double& local_vy);

                // Get coxa-pivot position in body frame (mm)
                void getLegBasePosition(int leg_index, double& base_x, double& base_y);
        };

} // namespace vx01_hexapod_locomotion

#endif