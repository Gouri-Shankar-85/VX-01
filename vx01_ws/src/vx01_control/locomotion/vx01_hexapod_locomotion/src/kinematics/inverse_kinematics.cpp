#include "vx01_hexapod_locomotion/kinematics/inverse_kinematics.hpp"
#include <cmath>

namespace vx01_hexapod_locomotion {
    
    namespace kinematics {

        InverseKinematics::InverseKinematics(double L1, double L2, double L3)
            : L1_(L1), L2_(L2), L3_(L3) {}

        bool InverseKinematics::isReachable(double xp, double yp, double zp)
        {
            // r_plane: horizontal distance from femur pivot to foot projection
            // = total horizontal reach from origin  MINUS  coxa length L1
            const double r_plane = std::sqrt(xp*xp + yp*yp) - L1_;
            if (r_plane < 0.0) return false;
            const double dist = std::sqrt(r_plane*r_plane + zp*zp);
            return (dist <= L2_ + L3_ &&
                    dist >= std::abs(L2_ - L3_));
        }

        bool InverseKinematics::compute(double xp, double yp, double zp,
                                        double& theta1, double& theta2, double& theta3)
        {
            if (!isReachable(xp, yp, zp)) return false;

            // theta1: coxa rotates in the horizontal (XY) plane
            theta1 = std::atan2(yp, xp);

            // r_plane: horizontal distance from femur pivot to foot
            // (total XY reach minus coxa length)
            const double r_plane = std::sqrt(xp*xp + yp*yp) - L1_;

            // r1: 3D straight-line distance from femur pivot to foot
            const double r1 = std::sqrt(zp*zp + r_plane*r_plane);

            // phi2: elevation angle from horizontal to the foot
            const double phi2 = std::atan2(zp, r_plane);

            // Law of cosines for femur angle
            const double cp1 = (L3_*L3_ - L2_*L2_ - r1*r1) / (-2.0 * L2_ * r1);
            if (cp1 < -1.0 || cp1 > 1.0) return false;
            const double phi1 = std::acos(cp1);

            theta2 = phi2 - phi1;

            // Law of cosines for knee (tibia) angle
            const double cp3 = (r1*r1 - L2_*L2_ - L3_*L3_) / (-2.0 * L2_ * L3_);
            if (cp3 < -1.0 || cp3 > 1.0) return false;
            const double phi3 = std::acos(cp3);

            theta3 = M_PI - phi3;

            return true;
        }

    }  
}  