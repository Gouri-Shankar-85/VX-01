#include "vx01_hexapod_locomotion/gait/gait_pattern.hpp"

namespace vx01_hexapod_locomotion {
    
    namespace gait {

        GaitPattern::GaitPattern(double S, double T, double A)
            : current_block_(0), S_(S), T_(T), A_(A)
        {
            gait_table_.resize(6, std::vector<LegPhase>(6));
            initializeGaitTable();
            swing_curve_.createSwingTrajectory(T_, S_, A_);
        }

        void GaitPattern::initializeGaitTable()
        {
            // Tripod A: legs 0, 2, 4 — swing blocks 0-1, drag blocks 2-5
            for (int leg : {0, 2, 4}) {
                gait_table_[leg] = {LegPhase::SWING, LegPhase::SWING,
                                    LegPhase::DRAG,  LegPhase::DRAG,
                                    LegPhase::DRAG,  LegPhase::DRAG};
            }

            // Tripod B: legs 1, 3, 5 — drag blocks 0-2, swing blocks 3-4, drag block 5
            for (int leg : {1, 3, 5}) {
                gait_table_[leg] = {LegPhase::DRAG,  LegPhase::DRAG,
                                    LegPhase::DRAG,  LegPhase::SWING,
                                    LegPhase::SWING, LegPhase::DRAG};
            }
        }

        int  GaitPattern::getCurrentBlock() const { return current_block_; }
        void GaitPattern::nextBlock()              { current_block_ = (current_block_ + 1) % 6; }
        void GaitPattern::reset()                  { current_block_ = 0; }

        LegPhase GaitPattern::getLegPhase(int leg_id) const {
            if (leg_id < 0 || leg_id >= 6) return LegPhase::DRAG;
            return gait_table_[leg_id][current_block_];
        }

        bool GaitPattern::isSwingPhase(int leg_id) const {
            return getLegPhase(leg_id) == LegPhase::SWING;
        }


        void GaitPattern::getFootPosition(int leg_id, double t,
                                          double& x, double& y, double& z) const
        {
            double tc = (t < 0.0) ? 0.0 : (t > 1.0 ? 1.0 : t);
            LegPhase phase = getLegPhase(leg_id);

            if (phase == LegPhase::SWING) {
                // Find which sub-block (0 or 1) within the 2-block swing window.
                int swing_start = -1;
                for (int b = 0; b < 6; ++b) {
                    if (gait_table_[leg_id][b] == LegPhase::SWING) {
                        swing_start = b;
                        break;
                    }
                }
                int swing_sub = current_block_ - swing_start;  // 0 or 1
                double global_t = (static_cast<double>(swing_sub) + tc) / 2.0;

                // Bezier returns: x=stride offset [-T/2..+T/2], y=0, z=height [0..A]
                double bx, by, bz;
                swing_curve_.getPoint(global_t, bx, by, bz);

                // x = home reach + stride delta; y = 0 (no lateral); z = lift
                x = S_ + bx;
                y = 0.0;
                z = bz;

            } else {
                // Stance (drag): foot on ground, swept backward in X from +T/2 to -T/2.
                int drag_sub = 0;

                if (leg_id == 0 || leg_id == 2 || leg_id == 4) {
                    // Group A: swings blocks 0-1, drags blocks 2,3,4,5 → sub 0..3
                    drag_sub = current_block_ - 2;
                    if (drag_sub < 0) drag_sub = 0;
                } else {
                    // Group B: swings blocks 3-4, drags blocks 5,0,1,2 → sub 0..3
                    switch (current_block_) {
                        case 5: drag_sub = 0; break;
                        case 0: drag_sub = 1; break;
                        case 1: drag_sub = 2; break;
                        default: drag_sub = 3; break;  // block 2
                    }
                }

                double global_t = (static_cast<double>(drag_sub) + tc) / 4.0;
                // Drag sweeps X from front (+T/2) to rear (-T/2) — stance pushes body forward.
                x = S_ + (T_ / 2.0 - global_t * T_);
                y = 0.0;
                z = 0.0;
            }
        }

        double GaitPattern::getStrideLength() const { return T_; }
        double GaitPattern::getReachDepth()   const { return S_; }
        double GaitPattern::getStepHeight()   const { return A_; }

    }  
}