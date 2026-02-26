#include "vx01_hexapod_locomotion/kinematics/inverse_kinematics.hpp"
#include <cmath>

namespace vx01_hexapod_locomotion {
    namespace kinematics {

        InverseKinematics::InverseKinematics(double L1, double L2, double L3)
            : L1_(L1), L2_(L2), L3_(L3) {}

        bool InverseKinematics::isReachable(double xp, double yp, double zp) {
            double dist = std::sqrt(xp*xp + yp*yp + zp*zp);
            return (dist <= L1_ + L2_ + L3_ &&
                    dist >= std::abs(L1_ - L2_ - L3_));
        }

        bool InverseKinematics::compute(double xp, double yp, double zp,
                                        double& theta1, double& theta2, double& theta3)
        {
            if (!isReachable(xp, yp, zp)) return false;

            theta1 = std::atan2(yp, xp);
            double ct1 = std::cos(theta1);
            if (std::abs(ct1) < 1e-9) return false;

            double r2 = xp / ct1 - L1_;
            double r1 = std::sqrt(zp*zp + r2*r2);

            double phi2 = std::atan2(zp, r2);

            // Cosine rule: triangle (L2, L3, r1), solve angle at L2-r1 vertex
            double cp1 = (L2_*L2_ + r1*r1 - L3_*L3_) / (2.0*L2_*r1);
            if (cp1 < -1.0 || cp1 > 1.0) return false;
            double phi1 = std::acos(cp1);

            // Elbow-up: femur tips downward
            theta2 = phi2 - phi1;

            // Angle at L2-L3 vertex
            double cp3 = (r1*r1 - L2_*L2_ - L3_*L3_) / (-2.0*L2_*L3_);
            if (cp3 < -1.0 || cp3 > 1.0) return false;
            theta3 = M_PI - std::acos(cp3);

            static constexpr double LIMIT = 0.785398;
            return (std::abs(theta1) <= LIMIT &&
                    std::abs(theta2) <= LIMIT &&
                    std::abs(theta3) <= LIMIT);
        }

    }
}