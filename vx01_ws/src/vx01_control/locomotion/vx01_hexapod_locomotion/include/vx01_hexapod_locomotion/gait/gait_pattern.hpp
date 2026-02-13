#ifndef VX01_HEXAPOD_LOCOMOTION_GAIT_PATTERN_HPP
#define VX01_HEXAPOD_LOCOMOTION_GAIT_PATTERN_HPP

#include "vx01_hexapod_locomotion/gait/bezier_curve.hpp"
#include <vector>

namespace vx01_hexapod_locomotion {

    namespace gait {

        enum class LegPhase {
            LIFT_UP,
            HOLD,
            LIFT_DOWN,
            DRAG
        };

        class GaitPattern {

            private:

                int current_block_;
                int total_blocks_;

                double S_;
                double T_;
                double A_;

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

                void getFootPosition(int leg_id, double time_in_block, double& x, double& y, double& z);

                bool isSwingPhase(int leg_id) const;

                double getStrideLength() const;
                double getTrackWidth() const;
                double getStepHeight() const;
        };
    }
}

#endif