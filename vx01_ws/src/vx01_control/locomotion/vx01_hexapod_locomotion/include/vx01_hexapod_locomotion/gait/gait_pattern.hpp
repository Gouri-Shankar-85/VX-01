#ifndef VX01_HEXAPOD_LOCOMOTION_GAIT_PATTERN_HPP
#define VX01_HEXAPOD_LOCOMOTION_GAIT_PATTERN_HPP

#include "vx01_hexapod_locomotion/gait/bezier_curve.hpp"
#include <vector>

namespace vx01_hexapod_locomotion {

    namespace gait {

        // Block phases from the tripod gait timing diagram (slide 11):
        //   LIFT_UP   = 1st half of swing curve (t: 0.0 → 0.5)
        //   HOLD      = foot stationary at current position
        //   LIFT_DOWN = 2nd half of swing curve (t: 0.5 → 1.0)
        //   DRAG      = stance (foot drags backward on ground)
        enum class LegPhase {
            LIFT_UP,
            HOLD,
            LIFT_DOWN,
            DRAG
        };

        // Tripod gait pattern.
        //
        // Parameters:
        //   S = reach depth  (mm) – distance from coxa pivot to neutral foot position
        //                          along the leg's Y axis in the O-frame
        //   T = stride length (mm) – total foot travel along Y in the O-frame
        //   A = step height  (mm) – peak lift above ground
        //
        // getFootPosition() returns coordinates in the O-frame (body-stationary frame):
        //   x = 0  (no lateral displacement in the O-frame leg axis)
        //   y = depth/reach along leg axis
        //   z = height above ground
        class GaitPattern {

            private:

                int current_block_;
                int total_blocks_;

                double S_;  // reach depth
                double T_;  // stride length
                double A_;  // step height

                int data_points_per_line_;

                BezierCurve swing_curve_;

                std::vector<std::vector<LegPhase>> gait_table_;

            public:

                GaitPattern(double S, double T, double A, int data_points_per_line = 10);

                void initializeGaitTable();

                int getCurrentBlock() const;

                void nextBlock();

                void reset();

                LegPhase getLegPhase(int leg_id) const;

                // Returns foot position in the O-frame for the given leg and
                // normalised time within the current block [0, 1].
                //   x = lateral offset (always 0 in forward walking)
                //   y = depth along leg axis (stance = S, swings between S and S+2A)
                //   z = height above ground
                void getFootPosition(int leg_id, double time_in_block,
                                     double& x, double& y, double& z);

                bool isSwingPhase(int leg_id) const;

                double getStrideLength() const;  // returns T_ (stride length)
                double getTrackWidth()   const;  // returns S_ (reach depth, named for compatibility)
                double getStepHeight()   const;  // returns A_ (step height)
        };
    }
}

#endif