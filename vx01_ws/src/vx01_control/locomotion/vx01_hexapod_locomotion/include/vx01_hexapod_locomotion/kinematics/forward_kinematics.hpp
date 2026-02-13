#ifndef VX01_HEXAPOD_LOCOMOTION_FORWARD_KINEMATICS_HPP
#define VX01_HEXAPOD_LOCOMOTION_FORWARD_KINEMATICS_HPP

#include "vx01_hexapod_locomotion/kinematics/transformation_matrix.hpp"
#include "vx01_hexapod_locomotion/kinematics/dh_parameters.hpp"

namespace vx01_hexapod_locomotion {

    namespace kinematics {

        class ForwardKinematics {
        
            private:
                double L1_;
                double L2_;
                double L3_;

            public:
                ForwardKinematics(double L1, double L2, double L3);

                void compute(double theta1, double theta2, double theta3, 
                             double& x, double& y, double& z);

                TransformationMatrix getT01(double theta1);
                TransformationMatrix getT12(double theta2);
                TransformationMatrix getT23(double theta3);

                TransformationMatrix getT03(double theta1, double theta2, double theta3);
        };

    }

}

#endif