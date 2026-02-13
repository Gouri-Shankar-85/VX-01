#include "vx01_hexapod_locomotion/kinematics/forward_kinematics.hpp"
#include <cmath>

namespace vx01_hexapod_locomotion {

    namespace kinematics {

        ForwardKinematics::ForwardKinematics(double L1, double L2, double L3) :
            L1_(L1), L2_(L2), L3_(L3) {}

        TransformationMatrix ForwardKinematics::getT01(double theta1) {

            DHParameters dh1(L1_, 0.0, M_PI_2, theta1);
            return TransformationMatrix(dh1);
        }

        TransformationMatrix ForwardKinematics::getT12(double theta2) {
            
            DHParameters dh2(L2_, 0.0, 0.0, theta2);
            return TransformationMatrix(dh2);
        }

        TransformationMatrix ForwardKinematics::getT23(double theta3) {
            
            DHParameters dh3(L3_, 0.0, 0.0, theta3);
            return TransformationMatrix(dh3);
        }

        TransformationMatrix ForwardKinematics::getT03(double theta1, double theta2, double theta3) {

            TransformationMatrix T01 = getT01(theta1);
            TransformationMatrix T12 = getT12(theta2);
            TransformationMatrix T23 = getT23(theta3);
            
            TransformationMatrix T02 = T01.multiply(T12);
            TransformationMatrix T03 = T02.multiply(T23);

            return T03;
        }

        void ForwardKinematics::compute(double theta1, double theta2, double theta3, 
                                        double& x, double& y, double& z) {
            
            TransformationMatrix T03 = getT03(theta1, theta2, theta3);
            x = T03.getX();
            y = T03.getY();
            z = T03.getZ();
        }
    }
}