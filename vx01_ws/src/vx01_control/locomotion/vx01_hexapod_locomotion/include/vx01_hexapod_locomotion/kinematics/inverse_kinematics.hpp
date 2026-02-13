#ifndef VX01_HEXAPOD_LOCOMOTION_INVERSE_KINEMATICS_HPP
#define VX01_HEXAPOD_LOCOMOTION_INVERSE_KINEMATICS_HPP

namespace vx01_hexapod_locomotion {

    namespace kinematics {

        class InverseKinematics {

            private:
                double L1_;
                double L2_;
                double L3_;

            public:

                InverseKinematics(double L1, double L2, double L3);

                bool compute(double xp, double yp, double zp,
                             double& theta1, double& theta2, double& theta3);
                             
                bool isReachable(double xp, double yp, double zp);
        };
    }
}

#endif