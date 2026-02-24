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
                // Leg controllers (one per leg)
                // LegController lives in the control sub-namespace
                std::vector<std::shared_ptr<control::LegController>> leg_controllers_;

                // Gait pattern generator lives in the gait sub-namespace
                std::shared_ptr<gait::GaitPattern> gait_pattern_;

                // Robot dimensions (mm)
                double L1_;          // Coxa  length
                double L2_;          // Femur length
                double L3_;          // Tibia length
                double body_radius_; // Distance from body centre to coxa pivot
                double beta_angle_;  // Angular separation between adjacent legs (rad)

                // Leg mounting angles measured from body +X axis (rad)
                std::vector<double> leg_angles_;

                // Current locomotion state
                LocomotionState state_;

                // Velocity commands
                double velocity_x_;      // Forward  velocity (mm/s)
                double velocity_y_;      // Lateral  velocity (mm/s)
                double velocity_omega_;  // Yaw-rate            (rad/s)

                // Gait timing
                double gait_time_;   // Elapsed time inside the current gait cycle (s)
                double step_period_; // Duration of one complete gait cycle        (s)

                // Step geometry
                double step_length_; // Stride length (mm)  – maps to S in GaitPattern
                double step_height_; // Lift  height  (mm)  – maps to A in GaitPattern
                double track_width_; // Lateral track width (mm) – maps to T in GaitPattern

                // Neutral / home foot position in the leg-local frame (mm)
                double home_x_;
                double home_y_;
                double home_z_;

                // Flat storage of all 18 joint angles  [leg0_t1, leg0_t2, leg0_t3, ...]
                std::vector<double> current_joint_angles_;

            public:
                // ------------------------------------------------------------
                // Construction / destruction
                // ------------------------------------------------------------
                HexapodLocomotion(double L1, double L2, double L3,
                                double body_radius, double beta_angle);
                ~HexapodLocomotion();

                // ------------------------------------------------------------
                // High-level commands
                // ------------------------------------------------------------
                void stand();  // Move all legs to home position and hold
                void walk();   // Begin walking with the currently set velocity
                void stop();   // Stop motion immediately

                // ------------------------------------------------------------
                // State query
                // ------------------------------------------------------------
                LocomotionState getState() const;

                // ------------------------------------------------------------
                // Velocity interface
                // ------------------------------------------------------------
                void setVelocity(double vx, double vy, double omega);
                void getVelocity(double& vx, double& vy, double& omega) const;

                // ------------------------------------------------------------
                // Main update – call at a fixed rate (e.g. 50 Hz)
                // ------------------------------------------------------------
                void update(double dt);

                // ------------------------------------------------------------
                // Joint-angle accessors
                // ------------------------------------------------------------
                std::vector<double> getJointAngles() const;
                void getLegAngles(int leg_index,
                                double& theta1, double& theta2, double& theta3) const;

                // ------------------------------------------------------------
                // Gait parameter setters / getters
                // ------------------------------------------------------------
                void   setStepLength(double length);
                void   setStepHeight(double height);
                void   setStepPeriod(double period);

                double getStepLength() const;
                double getStepHeight() const;
                double getStepPeriod() const;

                // ------------------------------------------------------------
                // Home-position setters / getters (leg-local frame, mm)
                // ------------------------------------------------------------
                void setHomePosition(double x, double y, double z);
                void getHomePosition(double& x, double& y, double& z) const;

            private:
                // ------------------------------------------------------------
                // Internal helpers
                // ------------------------------------------------------------

                // Build one LegController per leg with the correct arguments
                void initializeLegControllers();

                // Apply IK for a single leg and store the resulting joint angles.
                // foot_x/y/z are given in the leg-local frame.
                void applyIK(int leg_index, double foot_x, double foot_y, double foot_z);

                // Compute target foot position (leg-local frame) for the given
                // normalised gait phase [0, 1).
                void calculateFootTarget(int leg_index, double phase,
                                        double& x, double& y, double& z);

                // Drive one leg for the current gait_time_
                void updateLeg(int leg_index);

                // Rotate the body-frame velocity command into leg-i's local frame
                // and add the contribution from the yaw-rate command.
                void transformVelocity(int leg_index, double vx, double vy,
                                    double& local_vx, double& local_vy);

                // Return the (x, y) position of leg i's coxa pivot in the body frame
                void getLegBasePosition(int leg_index,
                                        double& base_x, double& base_y);
        };

    } 

#endif 