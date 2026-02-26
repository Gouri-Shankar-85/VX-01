#ifndef VX01_HEXAPOD_LOCOMOTION_BEZIER_CURVE_HPP
#define VX01_HEXAPOD_LOCOMOTION_BEZIER_CURVE_HPP

namespace vx01_hexapod_locomotion {
    namespace gait {

        class BezierCurve {
        private:
            double P1x_, P1y_, P1z_;
            double P2x_, P2y_, P2z_;
            double P3x_, P3y_, P3z_;

        public:
            BezierCurve();
            BezierCurve(double p1x, double p1y, double p1z,
                        double p2x, double p2y, double p2z,
                        double p3x, double p3y, double p3z);

            void setControlPoints(double p1x, double p1y, double p1z,
                                  double p2x, double p2y, double p2z,
                                  double p3x, double p3y, double p3z);

            void getPoint(double t, double& x, double& y, double& z) const;

            // Swing arc: foot sweeps stride in Y, lifts to height A in Z, reach S in X.
            // P1=(S, -T/2, 0), P2=(S, 0, A), P3=(S, +T/2, 0)
            void createSwingTrajectory(double T, double S, double A);
        };

    }
}

#endif