#ifndef VX01_HEXAPOD_LOCOMOTION_HEXAPOD_LOCOMOTION_HPP
#define VX01_HEXAPOD_LOCOMOTION_HEXAPOD_LOCOMOTION_HPP

#include "vx01_hexapod_locomotion/control/leg_controller.hpp"
#include "vx01_hexapod_locomotion/gait/gait_pattern.hpp"
#include <vector>
#include <memory>

namespace vx01_hexapod_locomotion {

    class HexapodLocomotion {

        private:

            std::vector<std::shared_ptr<control::LegController>> legs_;
            std::shared_ptr<gait::GaitPattern> gait_;     
            
            double L1_;
            double L2_;
            double L3_;

            double stride_length_;
            double track_width_;
            double step_height_;

            double body_radius_;
            double leg_spacing_;

            bool is_walking_;
            double current_time_in_block_;

        public:
        
            HexapodLocomotion(double L1, double L2, double L3, 
                              double stride_length, double track_width, double step_height,
                              double body_radius, double leg_spacing);

            void initializeLegs();

            void moveToHome(double home_x, double home_y, double home_z);

            void startWalking();

            void stopWalking();

            void updateGait(double dt);

            void turn(double gamma);

            void getLegJointAngles(int leg_id, double& theta1, double& theta2, double& theta3);

            void getLegFootPosition(int leg_id, double& x, double& y, double& z);

            bool isWalking() const;

            int getCurrentBlock() const;
    };
}

#endif