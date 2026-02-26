#include "vx01_hexapod_locomotion/gait/gait_pattern.hpp"

namespace vx01_hexapod_locomotion {
    namespace gait {

        // Tripod gait — 6 blocks per full cycle, duty factor = 0.5
        //
        // Leg groups:
        //   Group A (legs 0, 2, 4): SWING in blocks 0–2, DRAG in blocks 3–5
        //   Group B (legs 1, 3, 5): DRAG in blocks 0–2, SWING in blocks 3–5
        //
        // This gives exactly half-cycle swing and half-cycle stance per leg.
        // While Group A swings, Group B is on the ground (stable tripod support).
        //
        // Foot coordinates in leg-local frame:
        //   x = reach (constant = S)
        //   y = stride direction  [-T/2 .. +T/2]
        //   z = height            [0 .. A]
        //
        // SWING: quadratic Bezier P1=(S,-T/2,0) P2=(S,0,A) P3=(S,+T/2,0)
        //        t maps 0→1 across all 3 swing blocks
        // DRAG : linear from y=+T/2 back to y=-T/2, z=0
        //        t maps 0→1 across all 3 drag blocks

        GaitPattern::GaitPattern(double S, double T, double A)
            : current_block_(0), S_(S), T_(T), A_(A)
        {
            gait_table_.resize(6, std::vector<LegPhase>(6));
            initializeGaitTable();
            swing_curve_.createSwingTrajectory(T_, S_, A_);
        }

        void GaitPattern::initializeGaitTable() {
            for (int leg : {0, 2, 4}) {
                gait_table_[leg][0] = LegPhase::SWING;
                gait_table_[leg][1] = LegPhase::SWING;
                gait_table_[leg][2] = LegPhase::SWING;
                gait_table_[leg][3] = LegPhase::DRAG;
                gait_table_[leg][4] = LegPhase::DRAG;
                gait_table_[leg][5] = LegPhase::DRAG;
            }
            for (int leg : {1, 3, 5}) {
                gait_table_[leg][0] = LegPhase::DRAG;
                gait_table_[leg][1] = LegPhase::DRAG;
                gait_table_[leg][2] = LegPhase::DRAG;
                gait_table_[leg][3] = LegPhase::SWING;
                gait_table_[leg][4] = LegPhase::SWING;
                gait_table_[leg][5] = LegPhase::SWING;
            }
        }

        int  GaitPattern::getCurrentBlock() const { return current_block_; }
        void GaitPattern::nextBlock() { current_block_ = (current_block_ + 1) % 6; }
        void GaitPattern::reset()    { current_block_ = 0; }

        LegPhase GaitPattern::getLegPhase(int leg_id) const {
            if (leg_id < 0 || leg_id >= 6) return LegPhase::DRAG;
            return gait_table_[leg_id][current_block_];
        }

        bool GaitPattern::isSwingPhase(int leg_id) const {
            return getLegPhase(leg_id) == LegPhase::SWING;
        }

        void GaitPattern::getFootPosition(int leg_id, double t,
                                          double& x, double& y, double& z) const {
            double tc = t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
            LegPhase phase = getLegPhase(leg_id);

            // Find which block within the 3-block swing or drag super-phase we're in.
            // current_block_ 0,1,2 = first super-phase; 3,4,5 = second super-phase.
            // Within a 3-block super-phase, determine sub-block position (0,1,2)
            // then compute global t across all 3 blocks:  global_t = (sub_block + tc) / 3
            int sub_block;
            if (current_block_ <= 2) {
                sub_block = current_block_;      // blocks 0,1,2
            } else {
                sub_block = current_block_ - 3;  // blocks 3,4,5 -> 0,1,2
            }

            double global_t = (static_cast<double>(sub_block) + tc) / 3.0;

            if (phase == LegPhase::SWING) {
                swing_curve_.getPoint(global_t, x, y, z);
            } else {
                // DRAG: foot on ground, sweeps from front (+T/2) to rear (-T/2)
                x = S_;
                y = T_/2.0 - global_t * T_;
                z = 0.0;
            }
        }

        double GaitPattern::getStrideLength() const { return T_; }
        double GaitPattern::getReachDepth()   const { return S_; }
        double GaitPattern::getStepHeight()   const { return A_; }

    }
}