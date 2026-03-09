#ifndef VX01_HEXAPOD_LOCOMOTION_BODY_TRANSLATION_HPP
#define VX01_HEXAPOD_LOCOMOTION_BODY_TRANSLATION_HPP

#include "vx01_hexapod_locomotion/body/body_kinematics.hpp"
#include <array>

namespace vx01_hexapod_locomotion {
    namespace body {

        class BodyTranslation {
        private:
            BodyKinematics body_kinematics_;

            // Desired body translation in O-frame (mm)
            double tx_;  // translation along Xo
            double ty_;  // translation along Yo (forward)
            double tz_;  // translation along Zo (height)

        public:
            BodyTranslation();

            void setTranslation(double tx, double ty, double tz);

            void setRPY(double roll, double pitch, double yaw);

            void setPose(double tx, double ty, double tz,
                         double roll, double pitch, double yaw);

            double getTx() const;
            double getTy() const;
            double getTz() const;

            void computeFootInOFrame(double hx, double hy, double hz,
                                     double& ox, double& oy, double& oz) const;

            const BodyKinematics& getBodyKinematics() const;

        };

    } 
}  

#endif
