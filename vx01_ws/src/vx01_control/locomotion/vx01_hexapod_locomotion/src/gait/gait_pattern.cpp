#include "vx01_hexapod_locomotion/gait/gait_pattern.hpp"

namespace vx01_hexapod_locomotion {

    namespace gait {

        GaitPattern::GaitPattern(double S, double T, double A, int data_points_per_line)
            : current_block_(0), total_blocks_(6),
            S_(S), T_(T), A_(A), data_points_per_line_(data_points_per_line) {

            gait_table_.resize(6);
            for (int i=0; i < 6; ++i) {
                gait_table_[i].resize(6);
            }

            initializeGaitTable();

            swing_curve_.createSwingTrajectory(T_, S_,A_);
        }

        void GaitPattern::initializeGaitTable() {

            // Legs: i=0, j=1, k=2, l=3, m=4, n=5

            // Leg i (0) - starts with LIFT_UP
            gait_table_[0][0] = LegPhase::LIFT_UP;
            gait_table_[0][1] = LegPhase::HOLD;
            gait_table_[0][2] = LegPhase::LIFT_DOWN;
            gait_table_[0][3] = LegPhase::HOLD;
            gait_table_[0][4] = LegPhase::DRAG;
            gait_table_[0][5] = LegPhase::HOLD;
            
            // Leg j (1) - starts with HOLD
            gait_table_[1][0] = LegPhase::HOLD;
            gait_table_[1][1] = LegPhase::DRAG;
            gait_table_[1][2] = LegPhase::HOLD;
            gait_table_[1][3] = LegPhase::LIFT_UP;
            gait_table_[1][4] = LegPhase::HOLD;
            gait_table_[1][5] = LegPhase::LIFT_DOWN;
            
            // Leg k (2) - same as leg i
            gait_table_[2][0] = LegPhase::LIFT_UP;
            gait_table_[2][1] = LegPhase::HOLD;
            gait_table_[2][2] = LegPhase::LIFT_DOWN;
            gait_table_[2][3] = LegPhase::HOLD;
            gait_table_[2][4] = LegPhase::DRAG;
            gait_table_[2][5] = LegPhase::HOLD;
            
            // Leg l (3) - same as leg j
            gait_table_[3][0] = LegPhase::HOLD;
            gait_table_[3][1] = LegPhase::DRAG;
            gait_table_[3][2] = LegPhase::HOLD;
            gait_table_[3][3] = LegPhase::LIFT_UP;
            gait_table_[3][4] = LegPhase::HOLD;
            gait_table_[3][5] = LegPhase::LIFT_DOWN;
            
            // Leg m (4) - same as leg i
            gait_table_[4][0] = LegPhase::LIFT_UP;
            gait_table_[4][1] = LegPhase::HOLD;
            gait_table_[4][2] = LegPhase::LIFT_DOWN;
            gait_table_[4][3] = LegPhase::HOLD;
            gait_table_[4][4] = LegPhase::DRAG;
            gait_table_[4][5] = LegPhase::HOLD;
            
            // Leg n (5) - same as leg j
            gait_table_[5][0] = LegPhase::HOLD;
            gait_table_[5][1] = LegPhase::DRAG;
            gait_table_[5][2] = LegPhase::HOLD;
            gait_table_[5][3] = LegPhase::LIFT_UP;
            gait_table_[5][4] = LegPhase::HOLD;
            gait_table_[5][5] = LegPhase::LIFT_DOWN;
        }

        int GaitPattern::getCurrentBlock() const {
            return current_block_;
        }

        void GaitPattern::nextBlock() {
            current_block_ = (current_block_ + 1) % total_blocks_;
        }

        void GaitPattern::reset() {
            current_block_ = 0;
        }

        LegPhase GaitPattern::getLegPhase(int leg_id) const {

            if (leg_id < 0 || leg_id >= 6) {
                return LegPhase::HOLD;
            }
            return gait_table_[leg_id][current_block_];
        }

        bool GaitPattern::isSwingPhase(int leg_id) const {

            LegPhase phase = getLegPhase(leg_id);
            return (phase == LegPhase::LIFT_UP || phase == LegPhase::LIFT_DOWN);
        }

        void GaitPattern::getFootPosition(int leg_id, double time_in_block, double& x, double& y, double& z) {
            
            LegPhase phase = getLegPhase(leg_id);

            double t = time_in_block < 0.0 ? 0.0 : (time_in_block > 1.0 ? 1.0 : time_in_block);

            switch (phase) {

                case LegPhase::LIFT_UP:
                    swing_curve_.getPoint(t * 0.5, x, y, z);
                    break;

                case LegPhase::LIFT_DOWN:
                    swing_curve_.getPoint(0.5 + t * 0.5, x, y, z);
                    break;

                case LegPhase::DRAG:
                    x = (T_ / 2.0) - t * T_;
                    y = S_;
                    z = 0.0;
                    break;

                case LegPhase::HOLD:
                default:
                    x = 0.0;
                    y = S_;
                    z = 0.0;
                    break;
            }
        }

        double GaitPattern::getStrideLength() const {
            return S_;
        }

        double GaitPattern::getTrackWidth() const {
            return T_;
        }

        double GaitPattern::getStepHeight() const {
            return A_;
        }
    }
}

