#include "vx01_hexapod_locomotion/kinematics/inverse_kinematics.hpp"
#include <cmath>

namespace vx01_hexapod_locomotion {

    namespace kinematics {

        InverseKinematics::InverseKinematics(double L1, double L2, double L3) 
            :L1_(L1), L2_(L2), L3_(L3) {}

        bool InverseKinematics::isReachable(double xp, double yp, double zp) {
            
            double max_reach = L1_ + L2_ + L3_;
            double min_reach = std::abs(L1_ - L2_ - L3_);

            double distance = std::sqrt(xp*xp + yp*yp + zp*zp);

            return (distance <= max_reach && distance >= min_reach);
        }

        bool InverseKinematics::compute(double xp, double yp, double zp,
                                        double& theta1, double& theta2, double& theta3) {
                                                        
            if (!isReachable(xp, yp, zp)) {
                return false;
            }

            theta1 = std::atan2(yp, xp);

            double r2 = xp / std::cos(theta1) - L1_;
            double r1 = std::sqrt(zp*zp + r2*r2);

            double phi2 = std::atan2(zp, r2);

            double numerator = L3_*L3_ - L2_*L2_ - r1*r1;
            double denominator = -2.0 * L2_ * r1;

            if (std::abs(denominator) < 1e-6) {
                return false;
            }

            double cos_phi1 = numerator / denominator;

            if (cos_phi1 < -1.0 || cos_phi1 > 1.0) {
                return false;
            }

            double phi1 = std::acos(cos_phi1);

            theta2 = phi2 + phi1;

            numerator = r1 * r1 - L2_ * L2_ - L3_ * L3_;
            denominator = -2.0 * L2_ * L3_;

            if (std::abs(denominator) < 1e-6) {
                return false;
            }

            double cos_phi3 = numerator / denominator;

            if (cos_phi3 < -1.0 || cos_phi3 > 1.0) {
                return false;
            }

            double phi3 = std::acos(cos_phi3);

            theta3 = -(M_PI - phi3);

            return true;
        }
    }
}