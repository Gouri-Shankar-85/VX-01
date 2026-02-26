#include "vx01_hexapod_locomotion/gait/bezier_curve.hpp"

namespace vx01_hexapod_locomotion {

    namespace gait {

        BezierCurve::BezierCurve()
            : P1x_(0.0), P1y_(0.0), P1z_(0.0),
              P2x_(0.0), P2y_(0.0), P2z_(0.0),
              P3x_(0.0), P3y_(0.0), P3z_(0.0) {}

        BezierCurve::BezierCurve(double p1x, double p1y, double p1z,
                                 double p2x, double p2y, double p2z,
                                 double p3x, double p3y, double p3z)
            : P1x_(p1x), P1y_(p1y), P1z_(p1z),
              P2x_(p2x), P2y_(p2y), P2z_(p2z),
              P3x_(p3x), P3y_(p3y), P3z_(p3z) {}

        void BezierCurve::setControlPoints(double p1x, double p1y, double p1z,
                                           double p2x, double p2y, double p2z,
                                           double p3x, double p3y, double p3z) {
            P1x_ = p1x; P1y_ = p1y; P1z_ = p1z;
            P2x_ = p2x; P2y_ = p2y; P2z_ = p2z;
            P3x_ = p3x; P3y_ = p3y; P3z_ = p3z;
        }

        // Quadratic Bezier:  B(t) = (1-t)^2 * P1 + 2(1-t)t * P2 + t^2 * P3
        void BezierCurve::getPoint(double t, double& x, double& y, double& z) {

            double t_clamped  = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
            double one_minus_t = 1.0 - t_clamped;
            double term1 = one_minus_t * one_minus_t;
            double term2 = 2.0 * one_minus_t * t_clamped;
            double term3 = t_clamped * t_clamped;

            x = term1 * P1x_ + term2 * P2x_ + term3 * P3x_;
            y = term1 * P1y_ + term2 * P2y_ + term3 * P3y_;
            z = term1 * P1z_ + term2 * P2z_ + term3 * P3z_;
        }

        // Build swing-phase control points.
        //
        // Coordinate convention (leg-local O-frame used by GaitPattern):
        //   x = reach/depth along coxa axis  (constant = S during swing)
        //   y = stride forward/backward      (foot sweeps ±T/2 along y)
        //   z = vertical height              (foot lifts to +A at apex)
        //
        // Control points:
        //   P1 = (S,  -T/2,  0)   -- swing start: foot at rear of stride, on ground
        //   P2 = (S,   0,    A)   -- swing apex:  foot at mid-stride, lifted to height A
        //   P3 = (S,  +T/2,  0)   -- swing end:   foot at front of stride, on ground
        //
        // This correctly encodes: reach=S constant, stride sweeps in Y, height in Z.
        void BezierCurve::createSwingTrajectory(double T, double S, double A) {

            P1x_ =  S;
            P1y_ = -T / 2.0;
            P1z_ =  0.0;

            P2x_ =  S;
            P2y_ =  0.0;
            P2z_ =  A;       // height lift encoded in Z

            P3x_ =  S;
            P3y_ =  T / 2.0;
            P3z_ =  0.0;
        }
    }
}