#ifndef VX01_HEXAPOD_LOCOMOTION_GAIT_PATTERN_HPP
#define VX01_HEXAPOD_LOCOMOTION_GAIT_PATTERN_HPP

#include "vx01_hexapod_locomotion/gait/bezier_curve.hpp"
#include <vector>
#include <cmath>

namespace vx01_hexapod_locomotion {
    namespace gait {

        // Tripod gait phases per block (6 blocks per full cycle)
        enum class LegPhase {
            SWING,   // foot in air, following Bezier arc
            DRAG     // foot on ground, sliding backward (stance)
        };

        class GaitPattern {
        private:
            int    current_block_;
            double S_;   // home reach in leg-local X (mm)
            double T_;   // stride length (mm), measured along body forward axis
            double A_;   // step height (mm)

            BezierCurve swing_curve_;

            // Per-leg stride vectors in leg-local frame.
            // For forward walking (+X_body), the stride direction for leg i is:
            //   stride_x = cos(-leg_angle[i]) * T/2
            //   stride_y = sin(-leg_angle[i]) * T/2
            std::vector<double> leg_stride_x_;  // half-stride in leg-local X per leg
            std::vector<double> leg_stride_y_;  // half-stride in leg-local Y per leg

            // gait_table_[leg_id][block] = phase
            std::vector<std::vector<LegPhase>> gait_table_;

        public:
            // leg_angles: mounting angle of each leg (rad), size 6
            GaitPattern(double S, double T, double A,
                        const std::vector<double>& leg_angles);

            void initializeGaitTable();
            void initializeLegStrides(const std::vector<double>& leg_angles);

            int       getCurrentBlock() const;
            void      nextBlock();
            void      reset();

            LegPhase  getLegPhase(int leg_id) const;
            bool      isSwingPhase(int leg_id) const;

            // Returns foot position in leg-local frame:
            //   x = reach (constant S)
            //   y = stride offset in [-T/2, +T/2]
            //   z = height above ground [0, A]
            void getFootPosition(int leg_id, double t,
                                 double& x, double& y, double& z) const;

            double getStrideLength() const;
            double getReachDepth()   const;
            double getStepHeight()   const;
        };

    }
}

#endif