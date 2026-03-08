#ifndef VX01_HEXAPOD_LOCOMOTION_BODY_KINEMATICS_HPP
#define VX01_HEXAPOD_LOCOMOTION_BODY_KINEMATICS_HPP

#include <array>

namespace vx01_hexapod_locomotion {
    namespace body {

        // BodyKinematics computes the Reverse Euler Rotation Matrix
        // R(-gamma, -psi, -alpha) = R(-alpha) * R(-psi) * R(-gamma)
        //
        // Where:
        //   alpha = yaw   (rotation about Z axis)
        //   psi   = pitch (rotation about Y axis)
        //   gamma = roll  (rotation about X axis)
        //
        // Reference slide: Pure Body Rotation
        // Used in: XYZ_leg = ROT(leg_offset) * RPY_matrix * [x_start] + offset
        
        class BodyKinematics {
        private:
            double roll_;   // gamma (rad)
            double pitch_;  // psi   (rad)
            double yaw_;    // alpha (rad)

            // 3x3 combined RPY matrix: R(-gamma, -psi, -alpha)
            std::array<std::array<double, 3>, 3> rpy_matrix_;

        public:
            BodyKinematics();

            // Set roll (gamma), pitch (psi), yaw (alpha) in radians
            void setRPY(double roll, double pitch, double yaw);

            double getRoll()  const;
            double getPitch() const;
            double getYaw()   const;

            // Get the full 3x3 combined RPY matrix
            // R(-gamma,-psi,-alpha) = R(-alpha)*R(-psi)*R(-gamma)
            const std::array<std::array<double, 3>, 3>& getRPYMatrix() const;

            // Apply RPY rotation to a point [x,y,z]
            // result = RPY_matrix * [x, y, z]
            void applyRPY(double x, double y, double z,
                          double& rx, double& ry, double& rz) const;

        private:
            void computeRPYMatrix();
        };

    }  // namespace body
}  // namespace vx01_hexapod_locomotion

#endif
