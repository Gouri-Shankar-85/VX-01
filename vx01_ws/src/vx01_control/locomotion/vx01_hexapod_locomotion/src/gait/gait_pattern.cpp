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

        // ─────────────────────────────────────────────────────────────────
        //  initializeGaitTable
        //
        //  FIX: Reference slide Image 9 (Tripod Gait table) shows EXACTLY
        //  2 swing blocks per group per cycle — not 3.
        //
        //  Block index:  0      1      2      3      4      5
        //  Group A (legs 0,2,4): SWING  SWING  DRAG   DRAG   DRAG   DRAG
        //  Group B (legs 1,3,5): DRAG   DRAG   DRAG   DRAG   SWING  SWING
        //
        //  This ensures legs 1,3,5 (including leg 5) lift correctly in
        //  blocks 4 and 5, resolving the "leg 5 does not follow" symptom.
        // ─────────────────────────────────────────────────────────────────
        void GaitPattern::initializeGaitTable()
        {
            // Group A — legs 0, 2, 4
            for (int leg : {0, 2, 4}) {
                gait_table_[leg][0] = LegPhase::SWING;   // 1st half curve
                gait_table_[leg][1] = LegPhase::SWING;   // 2nd half curve
                gait_table_[leg][2] = LegPhase::DRAG;
                gait_table_[leg][3] = LegPhase::DRAG;
                gait_table_[leg][4] = LegPhase::DRAG;
                gait_table_[leg][5] = LegPhase::DRAG;
            }
            // Group B — legs 1, 3, 5
            for (int leg : {1, 3, 5}) {
                gait_table_[leg][0] = LegPhase::DRAG;
                gait_table_[leg][1] = LegPhase::DRAG;
                gait_table_[leg][2] = LegPhase::DRAG;
                gait_table_[leg][3] = LegPhase::DRAG;
                gait_table_[leg][4] = LegPhase::SWING;   // 1st half curve
                gait_table_[leg][5] = LegPhase::SWING;   // 2nd half curve
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

        // ─────────────────────────────────────────────────────────────────
        //  getFootPosition
        //
        //  Returns foot position in leg-local frame for the CURRENT block.
        //
        //  Swing phase:
        //    t in [0,1] across both swing blocks combined.
        //    The Bezier curve interpolates Y from -T/2 to +T/2 with
        //    Z rising to A at mid-swing.  X = S always.
        //
        //  Drag (stance) phase:
        //    Foot is on the ground (Z = 0) and slides from +T/2 back to
        //    -T/2 while the body moves forward. X = S always.
        //
        //  FIX: sub_block calculation was confusing and caused the swing
        //  arc to start/end at wrong Y values.  Now we compute a single
        //  global_t in [0,1] spanning the entire swing (or drag) half-cycle.
        // ─────────────────────────────────────────────────────────────────
        void GaitPattern::getFootPosition(int leg_id, double t,
                                          double& x, double& y, double& z) const
        {
            double tc = (t < 0.0) ? 0.0 : (t > 1.0 ? 1.0 : t);
            LegPhase phase = getLegPhase(leg_id);

            // Determine which half of the 6-block cycle we are in (0..2)
            // and compute a global_t [0,1] spanning the 3 consecutive blocks
            // of the same phase (swing uses 2 blocks, drag uses 4).
            // We use 2-block swing: sub index 0 or 1 within the swing pair.
            // We use 4-block drag:  sub index 0..3 within the drag quad.

            if (phase == LegPhase::SWING) {
                // Which swing block are we in? (0 or 1)
                int swing_sub = 0;
                if (leg_id == 0 || leg_id == 2 || leg_id == 4) {
                    swing_sub = current_block_;        // blocks 0 or 1
                } else {
                    swing_sub = current_block_ - 4;    // blocks 4 or 5  → 0 or 1
                    if (swing_sub < 0) swing_sub = 0;
                }
                double global_t = (static_cast<double>(swing_sub) + tc) / 2.0;

                swing_curve_.getPoint(global_t, x, y, z);
                x = S_;   // X always at reach depth

            } else {
                // Drag phase: foot on ground, Y glides from +T/2 → -T/2
                // 4 drag blocks → sub index 0..3
                int drag_sub = 0;
                if (leg_id == 0 || leg_id == 2 || leg_id == 4) {
                    drag_sub = current_block_ - 2;     // blocks 2..5 → 0..3
                } else {
                    drag_sub = current_block_;          // blocks 0..3 → 0..3
                }
                if (drag_sub < 0) drag_sub = 0;

                double global_t = (static_cast<double>(drag_sub) + tc) / 4.0;

                x = S_;
                y = T_ / 2.0 - global_t * T_;   // +T/2 at start, -T/2 at end
                z = 0.0;
            }
        }

        double GaitPattern::getStrideLength() const { return T_; }
        double GaitPattern::getReachDepth()   const { return S_; }
        double GaitPattern::getStepHeight()   const { return A_; }

    }  // namespace gait
}  // namespace vx01_hexapod_locomotion