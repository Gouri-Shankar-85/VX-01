#include "vx01_hexapod_locomotion/kinematics/inverse_kinematics.hpp"
#include <cmath>

namespace vx01_hexapod_locomotion {

    namespace kinematics {

        InverseKinematics::InverseKinematics(double L1, double L2, double L3) 
            : L1_(L1), L2_(L2), L3_(L3) {}

        bool InverseKinematics::isReachable(double xp, double yp, double zp) {
            
            double max_reach = L1_ + L2_ + L3_;
            double min_reach = std::abs(L1_ - L2_ - L3_);

            double distance = std::sqrt(xp*xp + yp*yp + zp*zp);

            return (distance <= max_reach && distance >= min_reach);
        }

        // Implements the IK formulas from the slide exactly:
        //   theta1 = atan2(yp, xp)
        //   r2     = xp/cos(theta1) - L1
        //   r1     = sqrt(zp^2 + r2^2)
        //   phi2   = atan2(zp, r2)
        //   phi1   = acos( (L3^2 - L2^2 - r1^2) / (-2*L2*r1) )
        //   theta2 = phi1 + phi2
        //   phi3   = acos( (r1^2 - L2^2 - L3^2) / (-2*L2*L3) )
        //   theta3 = -(pi - phi3)
        // Joint limits (rad) — must match the YAML hardware_interface limits
        static constexpr double JOINT_LIMIT = 0.785398;  // ±45°

        bool InverseKinematics::compute(double xp, double yp, double zp,
                                        double& theta1, double& theta2, double& theta3) {
                                                        
            if (!isReachable(xp, yp, zp)) {
                return false;
            }

            // Step 1: theta1 from top-view projection
            theta1 = std::atan2(yp, xp);

            // Step 2: reduce to 2D problem in the sagittal plane
            double r2 = xp / std::cos(theta1) - L1_;
            double r1 = std::sqrt(zp*zp + r2*r2);

            // Step 3: phi2 is the angle of the target below/above the horizontal
            double phi2 = std::atan2(zp, r2);

            // Step 4: phi1 via cosine rule (L3 opposite side)
            double numerator   = L3_*L3_ - L2_*L2_ - r1*r1;
            double denominator = -2.0 * L2_ * r1;

            if (std::abs(denominator) < 1e-6) {
                return false;
            }

            double cos_phi1 = numerator / denominator;

            if (cos_phi1 < -1.0 || cos_phi1 > 1.0) {
                return false;
            }

            double phi1 = std::acos(cos_phi1);

            // ELBOW-UP solution: femur angles downward, tibia angles back up.
            // Elbow-down (phi1+phi2) causes theta3 to violate ±45° joint limits.
            // Elbow-up: theta2 = phi2 - phi1 (femur dips below horizontal)
            theta2 = phi2 - phi1;

            // Step 5: phi3 via cosine rule (r1 opposite side)
            numerator   = r1*r1 - L2_*L2_ - L3_*L3_;
            denominator = -2.0 * L2_ * L3_;

            if (std::abs(denominator) < 1e-6) {
                return false;
            }

            double cos_phi3 = numerator / denominator;

            if (cos_phi3 < -1.0 || cos_phi3 > 1.0) {
                return false;
            }

            double phi3 = std::acos(cos_phi3);

            // ELBOW-UP: tibia angles positively back toward body (within ±45°)
            theta3 = M_PI - phi3;

            // Enforce joint limits — reject solutions the hardware cannot reach.
            // isReachable() only checks Euclidean distance; this catches
            // configurations that are kinematically valid but mechanically impossible.
            if (std::abs(theta1) > JOINT_LIMIT ||
                std::abs(theta2) > JOINT_LIMIT ||
                std::abs(theta3) > JOINT_LIMIT) {
                return false;
            }

            return true;
        }
    }
}