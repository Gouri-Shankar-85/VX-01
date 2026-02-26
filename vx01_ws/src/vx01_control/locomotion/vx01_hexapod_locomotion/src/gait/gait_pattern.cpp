#include "vx01_hexapod_locomotion/gait/gait_pattern.hpp"

namespace vx01_hexapod_locomotion {

    namespace gait {

        // =====================================================================
        // GaitPattern
        //
        // Coordinate convention (O-frame, body-stationary, per slides):
        //   x  = stride direction (forward = +x)
        //   y  = leg reach/depth axis (outward from body = +y)
        //   z  = vertical height (up = +z)
        //
        // Bezier control points (slide 6):
        //   P1 = [-T/2,  S,     0]   -- start: foot at rear of stride, at ground
        //   P2 = [  0,   S+2A,  0]   -- apex:  foot extended outward (arc = lift)
        //   P3 = [ T/2,  S,     0]   -- end:   foot at front of stride, at ground
        //
        // The swing-phase "lift" is produced by the y-axis arc of the Bezier:
        //   at t=0.5 the foot reaches y = S + 2A (fully extended = maximum lift).
        //   z is always 0 in the O-frame output (height is implicit in the y arc).
        //
        // Stance (DRAG phase): foot travels linearly from y = T/2 to y = -T/2
        //   along x (moving backward relative to body), depth stays at y = S, z = 0.
        //
        // HOLD phase: foot stays at its current position.  For simplicity we hold at
        //   the natural home position: x=0, y=S, z=0.
        //
        // Tripod gait table (slide 11):
        //   6 blocks per cycle.
        //   Legs i,k,m (0,2,4) share the same phase.
        //   Legs j,l,n (1,3,5) are opposite phase.
        //
        //   Block:       1          2          3          4          5          6
        //   leg i,k,m:  LIFT_UP    HOLD       LIFT_DOWN  HOLD       DRAG       HOLD
        //   leg j,l,n:  HOLD       DRAG       HOLD       LIFT_UP    HOLD       LIFT_DOWN
        // =====================================================================

        GaitPattern::GaitPattern(double S, double T, double A, int data_points_per_line)
            : current_block_(0), total_blocks_(6),
              S_(S), T_(T), A_(A), data_points_per_line_(data_points_per_line) {

            gait_table_.resize(6);
            for (int i = 0; i < 6; ++i) {
                gait_table_[i].resize(6);
            }

            initializeGaitTable();

            swing_curve_.createSwingTrajectory(T_, S_, A_);
        }

        void GaitPattern::initializeGaitTable() {

            // Block index:            0           1           2           3           4           5
            //                     (block 1)   (block 2)   (block 3)   (block 4)   (block 5)   (block 6)

            // Leg i (0), k (2), m (4) -- same phase
            for (int leg : {0, 2, 4}) {
                gait_table_[leg][0] = LegPhase::LIFT_UP;
                gait_table_[leg][1] = LegPhase::HOLD;
                gait_table_[leg][2] = LegPhase::LIFT_DOWN;
                gait_table_[leg][3] = LegPhase::HOLD;
                gait_table_[leg][4] = LegPhase::DRAG;
                gait_table_[leg][5] = LegPhase::HOLD;
            }

            // Leg j (1), l (3), n (5) -- opposite phase
            for (int leg : {1, 3, 5}) {
                gait_table_[leg][0] = LegPhase::HOLD;
                gait_table_[leg][1] = LegPhase::DRAG;
                gait_table_[leg][2] = LegPhase::HOLD;
                gait_table_[leg][3] = LegPhase::LIFT_UP;
                gait_table_[leg][4] = LegPhase::HOLD;
                gait_table_[leg][5] = LegPhase::LIFT_DOWN;
            }
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

        // =====================================================================
        // getFootPosition
        //
        // Returns foot position in the leg-local O-frame:
        //   x = reach along coxa axis (= S, constant during all phases)
        //   y = stride forward/backward (sweeps ±T/2)
        //   z = vertical height above ground level (0 = ground, +A = peak lift)
        //
        // SWING phases (LIFT_UP, LIFT_DOWN):
        //   Traces the Bezier curve P1=(S,-T/2,0) P2=(S,0,A) P3=(S,+T/2,0)
        //   LIFT_UP   : Bezier t = 0.0 → 0.5  (rear to apex)
        //   LIFT_DOWN : Bezier t = 0.5 → 1.0  (apex to front)
        //
        // DRAG (stance):
        //   Foot on ground (z=0), sweeps from front (+T/2) to rear (-T/2) in y.
        //   Reach stays at S.
        //
        // HOLD:
        //   Foot stationary at home: (S, 0, 0).
        // =====================================================================
        void GaitPattern::getFootPosition(int leg_id, double time_in_block,
                                          double& x, double& y, double& z) {
            
            LegPhase phase = getLegPhase(leg_id);
            double t = time_in_block < 0.0 ? 0.0 : (time_in_block > 1.0 ? 1.0 : time_in_block);

            switch (phase) {

                case LegPhase::LIFT_UP:
                    // 1st half of swing Bezier: t_bezier goes 0.0 → 0.5
                    swing_curve_.getPoint(t * 0.5, x, y, z);
                    break;

                case LegPhase::LIFT_DOWN:
                    // 2nd half of swing Bezier: t_bezier goes 0.5 → 1.0
                    swing_curve_.getPoint(0.5 + t * 0.5, x, y, z);
                    break;

                case LegPhase::DRAG:
                    // Stance: foot slides from front (+T/2) to rear (-T/2) along y
                    // Reach stays at S, foot stays on ground (z=0)
                    x =  S_;
                    y =  (T_ / 2.0) - t * T_;
                    z =  0.0;
                    break;

                case LegPhase::HOLD:
                default:
                    // Hold at neutral home position
                    x = S_;
                    y = 0.0;
                    z = 0.0;
                    break;
            }
        }

        double GaitPattern::getStrideLength() const { return T_; }
        double GaitPattern::getTrackWidth()   const { return S_; }
        double GaitPattern::getStepHeight()   const { return A_; }
    }
}