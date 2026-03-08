#ifndef VX01_HEXAPOD_LOCOMOTION_BODY_TRANSLATION_HPP
#define VX01_HEXAPOD_LOCOMOTION_BODY_TRANSLATION_HPP

#include "vx01_hexapod_locomotion/body/body_kinematics.hpp"
#include <array>

namespace vx01_hexapod_locomotion {
    namespace body {

        // BodyTranslation computes the full foot position in each leg's local frame
        // given a desired body translation (Pc) and orientation (RPY).
        //
        // Reference slide: Body Translation
        //
        // Full formula:
        //   [x,y,z]_leg = ROT(leg_offset) * RPY * [x_start] + offset_O_to_leg    <- Orientation Term
        //               + ROT(leg_offset) * [Po] + offset_O_to_leg                <- Origin Center Point (=0)
        //               - ROT(leg_offset) * [Pc] - offset_O_to_leg                <- New Translation Point
        //
        // Simplified (Po=0):
        //   [x,y,z]_leg = ROT(leg_offset) * (RPY * [x_start,y_start,z_start] - [Pc])
        //               + offset_O_to_leg
        //
        // Where offset_O_to_leg = [-C1, 0, 0] for leg i (side leg)
        //                        = [-C2, 0, 0] for diagonal legs j,k,m,n
        // (handled externally by LegController's x_start / bodyToLegFrame)

        class BodyTranslation {
        private:
            BodyKinematics body_kinematics_;

            // Desired body translation in O-frame (mm)
            double tx_;  // translation along Xo
            double ty_;  // translation along Yo (forward)
            double tz_;  // translation along Zo (height)

        public:
            BodyTranslation();

            // Set desired body translation in O-frame (mm)
            void setTranslation(double tx, double ty, double tz);

            // Set body orientation (roll, pitch, yaw in radians)
            void setRPY(double roll, double pitch, double yaw);

            // Set both at once
            void setPose(double tx, double ty, double tz,
                         double roll, double pitch, double yaw);

            double getTx() const;
            double getTy() const;
            double getTz() const;

            // Compute new foot target in O-frame given:
            //   home position [hx, hy, hz] in O-frame (x_start, y_start, z_start)
            //
            // Returns the displacement to apply in O-frame before bodyToLegFrame:
            //   out = RPY * [hx, hy, hz] - [Pc]
            //
            // Then caller passes this to bodyToLegFrame() for final leg-local coords.
            void computeFootInOFrame(double hx, double hy, double hz,
                                     double& ox, double& oy, double& oz) const;

            const BodyKinematics& getBodyKinematics() const;

        };

    }  // namespace body
}  // namespace vx01_hexapod_locomotion

#endif
