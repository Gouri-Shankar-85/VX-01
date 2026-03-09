#ifndef VX01_HEXAPOD_LOCOMOTION_BODY_KINEMATICS_HPP
#define VX01_HEXAPOD_LOCOMOTION_BODY_KINEMATICS_HPP

#include <array>

namespace vx01_hexapod_locomotion {
    namespace body {
        
        class BodyKinematics {
        private:
            double roll_;   // gamma (rad)
            double pitch_;  // psi   (rad)
            double yaw_;    // alpha (rad)

            // 3x3 combined RPY matrix: R(-gamma, -psi, -alpha)
            std::array<std::array<double, 3>, 3> rpy_matrix_;

        public:
            BodyKinematics();

            void setRPY(double roll, double pitch, double yaw);

            double getRoll()  const;
            double getPitch() const;
            double getYaw()   const;

            const std::array<std::array<double, 3>, 3>& getRPYMatrix() const;

            void applyRPY(double x, double y, double z,
                          double& rx, double& ry, double& rz) const;

        private:
            void computeRPYMatrix();
        };

    }  
}

#endif
