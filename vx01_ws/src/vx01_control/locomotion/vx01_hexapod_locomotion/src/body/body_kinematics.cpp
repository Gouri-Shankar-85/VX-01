#include "vx01_hexapod_locomotion/body/body_kinematics.hpp"
#include <cmath>

namespace vx01_hexapod_locomotion {
    namespace body {

        BodyKinematics::BodyKinematics()
            : roll_(0.0), pitch_(0.0), yaw_(0.0)
        {
            computeRPYMatrix();
        }

        void BodyKinematics::setRPY(double roll, double pitch, double yaw)
        {
            roll_  = roll;
            pitch_ = pitch;
            yaw_   = yaw;
            computeRPYMatrix();
        }

        double BodyKinematics::getRoll()  const { return roll_;  }
        double BodyKinematics::getPitch() const { return pitch_; }
        double BodyKinematics::getYaw()   const { return yaw_;   }

        const std::array<std::array<double, 3>, 3>& BodyKinematics::getRPYMatrix() const
        {
            return rpy_matrix_;
        }

        void BodyKinematics::applyRPY(double x, double y, double z,
                                       double& rx, double& ry, double& rz) const
        {
            rx = rpy_matrix_[0][0]*x + rpy_matrix_[0][1]*y + rpy_matrix_[0][2]*z;
            ry = rpy_matrix_[1][0]*x + rpy_matrix_[1][1]*y + rpy_matrix_[1][2]*z;
            rz = rpy_matrix_[2][0]*x + rpy_matrix_[2][1]*y + rpy_matrix_[2][2]*z;
        }

        void BodyKinematics::computeRPYMatrix()
        {
            // Reference slide: Pure Body Rotation
            // R(-gamma,-psi,-alpha) = R(-alpha) * R(-psi) * R(-gamma)
            //
            // R(-alpha) Yaw:
            //   [cos(-a)  -sin(-a)  0]
            //   [sin(-a)   cos(-a)  0]
            //   [0         0        1]
            //
            // R(-psi) Pitch:
            //   [cos(-p)   0   sin(-p)]
            //   [0         1   0      ]
            //   [-sin(-p)  0   cos(-p)]
            //
            // R(-gamma) Roll:
            //   [1   0         0       ]
            //   [0   cos(-g)  -sin(-g) ]
            //   [0   sin(-g)   cos(-g) ]

            double ca = std::cos(-yaw_);
            double sa = std::sin(-yaw_);
            double cp = std::cos(-pitch_);
            double sp = std::sin(-pitch_);
            double cg = std::cos(-roll_);
            double sg = std::sin(-roll_);

            // Combined matrix R(-gamma,-psi,-alpha) per reference slide:
            // Row 0
            rpy_matrix_[0][0] =  ca * cp;
            rpy_matrix_[0][1] =  ca * sp * sg - sa * cg;
            rpy_matrix_[0][2] =  ca * sp * cg + sa * sg;

            // Row 1
            rpy_matrix_[1][0] =  sa * cp;
            rpy_matrix_[1][1] =  sa * sp * sg + ca * cg;
            rpy_matrix_[1][2] =  sa * sp * cg - ca * sg;

            // Row 2
            rpy_matrix_[2][0] = -sp;
            rpy_matrix_[2][1] =  cp * sg;
            rpy_matrix_[2][2] =  cp * cg;
        }

    }  // namespace body
}  // namespace vx01_hexapod_locomotion
