#ifndef VX01_HEXAPOD_LOCOMOTION_GAIT_PATTERN_HPP
#define VX01_HEXAPOD_LOCOMOTION_GAIT_PATTERN_HPP

#include "vx01_hexapod_locomotion/gait/bezier_curve.hpp"
#include <vector>

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
            double S_;   // reach depth (mm)
            double T_;   // stride length (mm)
            double A_;   // step height (mm)

            BezierCurve swing_curve_;

            // gait_table_[leg_id][block] = phase
            std::vector<std::vector<LegPhase>> gait_table_;

        public:
            GaitPattern(double S, double T, double A);

            void initializeGaitTable();

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