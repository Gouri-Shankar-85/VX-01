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
            // Tripod gait 
            // Group A — legs 0, 2, 4
            for (int leg : {0, 2, 4}) {
                gait_table_[leg][0] = LegPhase::DRAG;
                gait_table_[leg][1] = LegPhase::DRAG;
                gait_table_[leg][2] = LegPhase::DRAG;
                gait_table_[leg][3] = LegPhase::SWING;  // 1st half curve
                gait_table_[leg][4] = LegPhase::SWING;  // 2nd half curve
                gait_table_[leg][5] = LegPhase::DRAG;
            }
            // Group B — legs 1, 3, 5
            for (int leg : {1, 3, 5}) {
                gait_table_[leg][0] = LegPhase::SWING;  // 1st half curve
                gait_table_[leg][1] = LegPhase::SWING;  // 2nd half curve
                gait_table_[leg][2] = LegPhase::DRAG;
                gait_table_[leg][3] = LegPhase::DRAG;
                gait_table_[leg][4] = LegPhase::DRAG;
                gait_table_[leg][5] = LegPhase::DRAG;
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
                
                int swing_sub = 0;

                if (leg_id == 0 || leg_id == 2 || leg_id == 4) {

                    swing_sub = current_block_ - 3;   
                    if (swing_sub < 0) swing_sub = 0;
                    
                } else {

                    swing_sub = current_block_;        
                }
                double global_t = (static_cast<double>(swing_sub) + tc) / 2.0;

                swing_curve_.getPoint(global_t, x, y, z);
                x = S_;  

            } else {

                int drag_sub = 0;

                if (leg_id == 0 || leg_id == 2 || leg_id == 4) {

                    drag_sub = (current_block_ < 3) ? current_block_ : (current_block_ - 2);

                } else {

                    drag_sub = current_block_ - 2;
                    if (drag_sub < 0) drag_sub = 0;
                }

                double global_t = (static_cast<double>(drag_sub) + tc) / 4.0;

                x = S_;
                y = T_ / 2.0 - global_t * T_;   // +T/2 at start, -T/2 at end
                z = 0.0;
            }
        }

        double GaitPattern::getStrideLength() const { return T_; }
        double GaitPattern::getReachDepth()   const { return S_; }
        double GaitPattern::getStepHeight()   const { return A_; }

    }  
} 