#include "vx01_hexapod_locomotion/body/body_translation.hpp"

namespace vx01_hexapod_locomotion {
    namespace body {

        BodyTranslation::BodyTranslation()
            : tx_(0.0), ty_(0.0), tz_(0.0)
        {}

        void BodyTranslation::setTranslation(double tx, double ty, double tz)
        {
            tx_ = tx;
            ty_ = ty;
            tz_ = tz;
        }

        void BodyTranslation::setRPY(double roll, double pitch, double yaw)
        {
            body_kinematics_.setRPY(roll, pitch, yaw);
        }

        void BodyTranslation::setPose(double tx, double ty, double tz,
                                       double roll, double pitch, double yaw)
        {
            setTranslation(tx, ty, tz);
            setRPY(roll, pitch, yaw);
        }

        double BodyTranslation::getTx() const { return tx_; }
        double BodyTranslation::getTy() const { return ty_; }
        double BodyTranslation::getTz() const { return tz_; }

        void BodyTranslation::computeFootInOFrame(double hx, double hy, double hz,
                                                   double& ox, double& oy, double& oz) const
        {
            // Reference slide: Body Translation
            //
            // Full formula:
            //   [x,y,z]_leg = Orientation Term + Translation Term
            //
            // Orientation Term = ROT(leg_offset) * RPY * [x_start, y_start, z_start] + offset_O_to_leg
            // Translation Term = ROT(leg_offset)*[Po]+offset - ROT(leg_offset)*[Pc]-offset
            //                  = ROT(leg_offset) * ([Po] - [Pc])
            //                  = ROT(leg_offset) * (-[Pc])   since Po = 0
            //
            // So combined before bodyToLegFrame:
            //   O_frame_point = RPY * [hx, hy, hz] - [Pc]
            //
            // bodyToLegFrame is then called externally by HexapodLocomotion
            // to apply ROT(leg_offset) and add x_start offset.

            // Step 1: Apply RPY rotation to home position
            double rx, ry, rz;
            body_kinematics_.applyRPY(hx, hy, hz, rx, ry, rz);

            // Step 2: Subtract translation (Pc) — moving body forward
            // means feet push backward, hence minus sign per reference
            ox = rx - tx_;
            oy = ry - ty_;
            oz = rz - tz_;
        }

        const BodyKinematics& BodyTranslation::getBodyKinematics() const
        {
            return body_kinematics_;
        }

    }  // namespace body
}  // namespace vx01_hexapod_locomotion
