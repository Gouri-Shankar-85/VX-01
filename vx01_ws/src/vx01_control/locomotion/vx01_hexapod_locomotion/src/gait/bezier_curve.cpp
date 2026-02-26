#include "vx01_hexapod_locomotion/gait/bezier_curve.hpp"

namespace vx01_hexapod_locomotion {
    namespace gait {

        BezierCurve::BezierCurve()
            : P1x_(0), P1y_(0), P1z_(0),
              P2x_(0), P2y_(0), P2z_(0),
              P3x_(0), P3y_(0), P3z_(0) {}

        BezierCurve::BezierCurve(double p1x, double p1y, double p1z,
                                 double p2x, double p2y, double p2z,
                                 double p3x, double p3y, double p3z)
            : P1x_(p1x), P1y_(p1y), P1z_(p1z),
              P2x_(p2x), P2y_(p2y), P2z_(p2z),
              P3x_(p3x), P3y_(p3y), P3z_(p3z) {}

        void BezierCurve::setControlPoints(double p1x, double p1y, double p1z,
                                           double p2x, double p2y, double p2z,
                                           double p3x, double p3y, double p3z) {
            P1x_=p1x; P1y_=p1y; P1z_=p1z;
            P2x_=p2x; P2y_=p2y; P2z_=p2z;
            P3x_=p3x; P3y_=p3y; P3z_=p3z;
        }

        void BezierCurve::getPoint(double t, double& x, double& y, double& z) const {
            double tc = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
            double u  = 1.0 - tc;
            double a  = u * u;
            double b  = 2.0 * u * tc;
            double c  = tc * tc;
            x = a*P1x_ + b*P2x_ + c*P3x_;
            y = a*P1y_ + b*P2y_ + c*P3y_;
            z = a*P1z_ + b*P2z_ + c*P3z_;
        }

        void BezierCurve::createSwingTrajectory(double T, double S, double A) {
            // X = reach (constant S)
            // Y = stride direction: starts rear (-T/2), apex (0), ends front (+T/2)
            // Z = height: ground (0), apex (A), ground (0)
            P1x_ =  S;  P1y_ = -T/2.0;  P1z_ = 0.0;
            P2x_ =  S;  P2y_ =  0.0;    P2z_ = A;
            P3x_ =  S;  P3y_ =  T/2.0;  P3z_ = 0.0;
        }

    }
}