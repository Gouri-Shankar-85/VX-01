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

        void GaitPattern::initializeLegStrides(const std::vector<double>& /*leg_angles*/)
        {
            leg_stride_x_.resize(6);
            leg_stride_y_.resize(6);
            for (int i = 0; i < 6; ++i) {
                leg_stride_x_[i] = T_ / 2.0;   // always forward in leg-local X
                leg_stride_y_[i] = 0.0;         // never lateral
            }
        }

        void GaitPattern::initializeGaitTable()
        {
            // tripod gait — 3 swing + 3 drag blocks per group
            //
            // Group A (legs 0,2,4): SWING blocks 0-1-2, DRAG blocks 3-4-5
            // Group B (legs 1,3,5): DRAG  blocks 0-1-2, SWING blocks 3-4-5
            //
            // Duty cycle: 50% swing / 50% drag — correct tripod gait.

            for (int leg : {0, 2, 4}) {
                gait_table_[leg] = {LegPhase::SWING, LegPhase::SWING, LegPhase::SWING,
                                    LegPhase::DRAG,  LegPhase::DRAG,  LegPhase::DRAG};
            }
            for (int leg : {1, 3, 5}) {
                gait_table_[leg] = {LegPhase::DRAG,  LegPhase::DRAG,  LegPhase::DRAG,
                                    LegPhase::SWING, LegPhase::SWING, LegPhase::SWING};
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

            const double sx = leg_stride_x_[leg_id];  // half-stride in leg X
            const double sy = leg_stride_y_[leg_id];  // half-stride in leg Y

            if (phase == LegPhase::SWING) {
                int swing_start = (leg_id==0||leg_id==2||leg_id==4) ? 0 : 3;
                int swing_sub   = current_block_ - swing_start;
                double global_t = (static_cast<double>(swing_sub) + tc) / 3.0;

                double bx, by, bz;
                swing_curve_.getPoint(global_t, bx, by, bz);

                double scale = (T_ > 1e-9) ? (bx / (T_ / 2.0)) : 0.0;
                x = S_ + scale * (T_ / 2.0);   // forward in leg-local X
                y = 0.0;                         // no lateral drift
                z = bz;

            } else {
                int drag_start = (leg_id==0||leg_id==2||leg_id==4) ? 3 : 0;
                int drag_sub   = current_block_ - drag_start;
                double global_t = (static_cast<double>(drag_sub) + tc) / 3.0;

                double scale = 1.0 - 2.0 * global_t;
                x = S_ + scale * (T_ / 2.0);   // forward in leg-local X
                y = 0.0;
                z = 0.0;
            }
        }

        double GaitPattern::getStrideLength() const { return T_; }
        double GaitPattern::getReachDepth()   const { return S_; }
        double GaitPattern::getStepHeight()   const { return A_; }

    }  
}