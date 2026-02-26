#ifndef VX01_HEXAPOD_LOCOMOTION_BEZIER_CURVE_HPP
#define VX01_HEXAPOD_LOCOMOTION_BEZIER_CURVE_HPP

namespace vx01_hexapod_locomotion {

    namespace gait {

        // Quadratic Bezier curve used for the swing-phase foot trajectory.
        //
        // The curve lives in the leg's O-frame (body-stationary frame):
        //   - The Y axis points outward along the leg (depth/reach direction)
        //   - The Z axis points upward
        //   - The X axis is the forward-walking direction
        //
        // Control points (from slide):
        //   P1 = [-T/2,  S,      0]    -- start  (foot at back of stance)
        //   P2 = [ 0,    S+2*A,  0]    -- apex   (foot lifted to height A above stance)
        //   P3 = [ T/2,  S,      0]    -- end    (foot at front of stance)
        //
        // where T = stride length (X extent), S = reach depth (Y), A = step height (Z).
        // Note: Z is stored in the P_z fields; after createSwingTrajectory the curve is
        // entirely in the X-Y plane of the O-frame.  The caller maps Y→depth, Z→height
        // via the coordinate convention used in gait_pattern.cpp.

        class BezierCurve {

            private:

                double P1x_, P1y_, P1z_;
                double P2x_, P2y_, P2z_;
                double P3x_, P3y_, P3z_;

            public:

                BezierCurve(double p1x, double p1y, double p1z,
                            double p2x, double p2y, double p2z,
                            double p3x, double p3y, double p3z);
                
                BezierCurve();

                void setControlPoints(double p1x, double p1y, double p1z,
                                      double p2x, double p2y, double p2z,
                                      double p3x, double p3y, double p3z);

                void getPoint(double t, double& x, double& y, double& z);

                // Build the swing trajectory control points.
                // T = stride length, S = reach depth, A = step height.
                void createSwingTrajectory(double T, double S, double A);
            
        };
    }
}

#endif