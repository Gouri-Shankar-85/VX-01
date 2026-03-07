#include "vx01_hexapod_locomotion/kinematics/inverse_kinematics.hpp"
#include <cmath>

namespace vx01_hexapod_locomotion {
    
    namespace kinematics {

        InverseKinematics::InverseKinematics(double L1, double L2, double L3)
            : L1_(L1), L2_(L2), L3_(L3) {}

        bool InverseKinematics::isReachable(double xp, double yp, double zp)
        {
            double r_xy = std::sqrt(xp*xp + yp*yp) - L1_;
            if (r_xy < 0.0) return false;
            double dist = std::sqrt(r_xy*r_xy + zp*zp);
            return (dist <= L2_ + L3_ &&
                    dist >= std::abs(L2_ - L3_));
        }

        // ─────────────────────────────────────────────────────────────────
        //  Analytical IK — reference: slide Image 4
        //
        //  theta1 = atan2(Yp, Xp)
        //  r2     = Xp/cos(theta1) - L1
        //  r1     = sqrt(Zp^2 + r2^2)
        //  phi2   = atan2(Zp, r2)
        //
        //  FIX: phi1 numerator was (L2^2+r1^2-L3^2) — WRONG triangle angle.
        //  CORRECT: phi1 = acos( (L3^2 - L2^2 - r1^2) / (-2*L2*r1) )
        //
        //  FIX: theta2 was phi2 - phi1 — WRONG sign.
        //  CORRECT: theta2 = phi1 + phi2  (ref slide Image 4)
        //
        //  theta3 = -(pi - phi3)          (ref slide Image 4)
        // ─────────────────────────────────────────────────────────────────
        bool InverseKinematics::compute(double xp, double yp, double zp,
                                        double& theta1, double& theta2, double& theta3)
        {
            if (!isReachable(xp, yp, zp)) return false;

            // theta1
            theta1 = std::atan2(yp, xp);

            double ct1 = std::cos(theta1);
            if (std::abs(ct1) < 1e-9) return false;

            // r2 = Xp/cos(theta1) - L1
            double r2 = xp / ct1 - L1_;

            // r1 = sqrt(Zp^2 + r2^2)
            double r1 = std::sqrt(zp*zp + r2*r2);

            // phi2 = atan2(Zp, r2)
            double phi2 = std::atan2(zp, r2);

            // CORRECT phi1: (L3^2 - L2^2 - r1^2) / (-2*L2*r1)
            double cp1 = (L3_*L3_ - L2_*L2_ - r1*r1) / (-2.0 * L2_ * r1);
            if (cp1 < -1.0 || cp1 > 1.0) return false;
            double phi1 = std::acos(cp1);

            // CORRECT theta2 = phi1 + phi2
            theta2 = phi1 + phi2;

            // phi3 = acos( (r1^2 - L2^2 - L3^2) / (-2*L2*L3) )
            double cp3 = (r1*r1 - L2_*L2_ - L3_*L3_) / (-2.0 * L2_ * L3_);
            if (cp3 < -1.0 || cp3 > 1.0) return false;
            double phi3 = std::acos(cp3);

            // theta3 = -(pi - phi3)
            theta3 = -(M_PI - phi3);

            return true;
        }

    }  // namespace kinematics
}  // namespace vx01_hexapod_locomotion