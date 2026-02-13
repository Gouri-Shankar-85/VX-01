#include "vx01_hexapod_locomotion/kinematics/dh_parameters.hpp"

namespace vx01_hexapod_locomotion {

    namespace kinematics {

        DHParameters::DHParameters(double a, double alpha, double d, double theta)
            : a_(a), alpha_(alpha), d_(d), theta_(theta) {}

        double DHParameters::getA() const {
            return a_;
        }

        double DHParameters::getAlpha() const {
            return alpha_;
        }

        double DHParameters::getD() const {
            return d_;
        }

        double DHParameters::getTheta() const {
            return theta_;
        }

        void DHParameters::setTheta(double theta) {
            theta_ = theta;
        }
    }
}