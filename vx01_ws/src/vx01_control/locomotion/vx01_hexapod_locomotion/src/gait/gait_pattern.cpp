#include "vx01_hexapod_locomotion/gait/gait_pattern.hpp"
#include <cmath>

namespace vx01_hexapod_locomotion {
    
    namespace gait {

        GaitPattern::GaitPattern(double S, double T, double A,
                                 const std::vector<double>& leg_angles)
            : current_block_(0), S_(S), T_(T), A_(A)
        {
            gait_table_.resize(6, std::vector<LegPhase>(6));
            initializeGaitTable();
            swing_curve_.createSwingTrajectory(T_, S_, A_);
            initializeLegStrides(leg_angles);
        }

        void GaitPattern::initializeLegStrides(const std::vector<double>& leg_angles)
        {
            leg_stride_x_.resize(6);
            leg_stride_y_.resize(6);
            for (int i = 0; i < 6; ++i) {
                // Stride direction in leg-local frame for body-forward (+X_body) motion.
                // ROT(-angle) * [1, 0] gives the body +X direction in leg-local frame.
                // Multiply by T/2 to get half-stride vector.
                leg_stride_x_[i] = std::cos(-leg_angles[i]) * (T_ / 2.0);
                leg_stride_y_[i] = std::sin(-leg_angles[i]) * (T_ / 2.0);
            }
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
            // Returns position in LEG-LOCAL frame.
            // The stride vector (leg_stride_x_[leg_id], leg_stride_y_[leg_id]) is the
            // half-stride in leg-local frame that corresponds to body-forward motion.
            // Swing: foot moves from rear (-stride) through lift to front (+stride).
            // Drag:  foot on ground, sweeps from front (+stride) to rear (-stride).

            double tc = (t < 0.0) ? 0.0 : (t > 1.0 ? 1.0 : t);
            LegPhase phase = getLegPhase(leg_id);

            const double sx = leg_stride_x_[leg_id];  // half-stride in leg X
            const double sy = leg_stride_y_[leg_id];  // half-stride in leg Y

            if (phase == LegPhase::SWING) {
                int swing_start = -1;
                for (int b = 0; b < 6; ++b) {
                    if (gait_table_[leg_id][b] == LegPhase::SWING) {
                        swing_start = b;
                        break;
                    }
                }
                int swing_sub   = current_block_ - swing_start;  // 0 or 1
                double global_t = (static_cast<double>(swing_sub) + tc) / 2.0;

                // Bezier arc: bx sweeps [-T/2, +T/2] along "forward" direction,
                // bz lifts to A at midpoint.  by is always 0.
                double bx, by, bz;
                swing_curve_.getPoint(global_t, bx, by, bz);

                // bx is the scalar progress along the stride direction [-T/2..+T/2].
                // Scale by the unit stride direction to get leg-local vector.
                double scale = (T_ > 1e-9) ? (bx / (T_ / 2.0)) : 0.0;
                x = S_ + scale * sx;
                y =      scale * sy;
                z = bz;

            } else {
                // Stance: sweep from front (+stride) to rear (-stride) linearly.
                int drag_sub = 0;
                if (leg_id == 0 || leg_id == 2 || leg_id == 4) {
                    drag_sub = current_block_ - 2;
                    if (drag_sub < 0) drag_sub = 0;
                } else {
                    switch (current_block_) {
                        case 5: drag_sub = 0; break;
                        case 0: drag_sub = 1; break;
                        case 1: drag_sub = 2; break;
                        default: drag_sub = 3; break;
                    }
                }
                double global_t = (static_cast<double>(drag_sub) + tc) / 4.0;
                // t=0 → front (+sx,+sy), t=1 → rear (-sx,-sy)
                double scale = 1.0 - 2.0 * global_t;  // +1 → -1
                x = S_ + scale * sx;
                y =      scale * sy;
                z = 0.0;
            }
        }

        double GaitPattern::getStrideLength() const { return T_; }
        double GaitPattern::getReachDepth()   const { return S_; }
        double GaitPattern::getStepHeight()   const { return A_; }

    }  
}