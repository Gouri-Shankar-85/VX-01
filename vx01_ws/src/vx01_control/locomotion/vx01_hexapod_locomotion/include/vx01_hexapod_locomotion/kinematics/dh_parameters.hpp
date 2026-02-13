#ifndef VX01_HEXAPOD_LOCOMOTION_DH_PARAMETERS_HPP
#define VX01_HEXAPOD_LOCOMOTION_DH_PARAMETERS_HPP

#include <cmath>

namespace vx01_hexapod_locomotion {

    namespace kinematics {

        class DHParameters {
            private:
                double a_;      // Link length
                double d_;      // Link offset
                double alpha_;  // Link twist
                double theta_;  // Joint angle

            public:
                DHParameters(double a, double alpha, double d, double theta);

                double getA() const;
                double getAlpha() const;
                double getD() const;
                double getTheta() const;

                void setTheta(double theta);
                
        };
    }
}

#endif