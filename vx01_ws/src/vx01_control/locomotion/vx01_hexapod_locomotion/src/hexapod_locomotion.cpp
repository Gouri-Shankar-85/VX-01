#include "vx01_hexapod_locomotion/hexapod_locomotion.hpp"
#include <cmath>

namespace vx01_hexapod_locomotion {

    HexapodLocomotion::HexapodLocomotion(double L1, double L2, double L3, 
                                         double stride_length, double track_width, double step_height,
                                         double body_radius, double leg_spacing) :
        L1_(L1), L2_(L2), L3_(L3),
        stride_length_(stride_length), track_width_(track_width), step_height_(step_height),
        body_radius_(body_radius), leg_spacing_(leg_spacing),
        is_walking_(false), current_time_in_block_(0.0) {

        gait_ = std::make_shared<gait::GaitPattern>(stride_length_, track_width_, step_height_);

        initializeLegs();
    }

    void HexapodLocomotion::initializeLegs() {

        legs_.clear();

        double beta = leg_spacing_;

        double angle_0 = 0.0;
        legs_.push_back(std::make_shared<control::LegController>(0, angle_0, body_radius_, L1_, L2_, L3_));

        double angle_1 =-beta;
        legs_.push_back(std::make_shared<control::LegController>(1, angle_1, body_radius_, L1_, L2_, L3_));

        double angle_2 = beta;
        legs_.push_back(std::make_shared<control::LegController>(2, angle_2, body_radius_, L1_, L2_, L3_));

        double angle_3 = M_PI;
        legs_.push_back(std::make_shared<control::LegController>(3, angle_3, body_radius_, L1_, L2_, L3_));

        double angle_4 = M_PI - beta;
        legs_.push_back(std::make_shared<control::LegController>(4, angle_4, body_radius_, L1_, L2_, L3_));

        double angle_5 = M_PI + beta;
        legs_.push_back(std::make_shared<control::LegController>(5, angle_5, body_radius_, L1_, L2_, L3_));
        
    }

    void HexapodLocomotion::moveToHome(double x_home, double y_home, double z_home) {

        for (int i =0; i < 6; i++) {

            double leg_x, leg_y, leg_z;

            legs_[i]->bodyToLegFrame(x_home, y_home, z_home, leg_x, leg_y, leg_z);
            legs_[i]->setFootPosition(leg_x, leg_y, leg_z);
        }
    }

    void HexapodLocomotion::startWalking() {
        is_walking_ = true;
        current_time_in_block_ = 0.0;
        gait_->reset();
    }

    void HexapodLocomotion::stopWalking() {
        is_walking_ = false;
    }

    void HexapodLocomotion::updateGait(double dt) {

        if (!is_walking_) {
            return;
        }

        current_time_in_block_ += dt;

        double block_duration = 1.0;

        if(current_time_in_block_ >= block_duration) {
            gait_->nextBlock();
            current_time_in_block_ = 0.0;
        }

        double time_ratio = current_time_in_block_ / block_duration;

        for (int i =0; i < 6; i++) {

            double foot_x, foot_y, foot_z;

            gait_->getFootPosition(i, time_ratio, foot_x, foot_y, foot_z);

            legs_[i]->setFootPosition(foot_x, foot_y, foot_z);
        }
    }

    void HexapodLocomotion::turn(double gamma) {

        for (int i=0; i < 6; i++) {
            double foot_x = legs_[i]->getFootX();
            double foot_y = legs_[i]->getFootY();
            double foot_z = legs_[i]->getFootZ();

            double cos_gamma = std::cos(gamma);
            double sin_gamma = std::sin(gamma);

            double new_x = cos_gamma * foot_x - sin_gamma * foot_y;
            double new_y = sin_gamma * foot_x + cos_gamma * foot_y;
            double new_z = foot_z;

            legs_[i]->setFootPosition(new_x, new_y, new_z);
        }
    }

    void HexapodLocomotion::getLegJointAngles(int leg_id, double& theta1, double& theta2, double& theta3) {

        if (leg_id < 0 || leg_id >= 6) {
            return;
        }

        theta1 = legs_[leg_id]->getTheta1();
        theta2 = legs_[leg_id]->getTheta2();
        theta3 = legs_[leg_id]->getTheta3();
    }

    void HexapodLocomotion::getLegFootPosition(int leg_id, double& x, double& y, double& z) {

        if (leg_id < 0 || leg_id >= 6) {
            return;
        }

        double leg_x = legs_[leg_id]->getFootX();
        double leg_y = legs_[leg_id]->getFootY();
        double leg_z = legs_[leg_id]->getFootZ();

        legs_[leg_id]->legToBodyFrame(leg_x, leg_y, leg_z, x, y, z);
    }

    bool HexapodLocomotion::isWalking() const {
        return is_walking_;
    }

    int HexapodLocomotion::getCurrentBlock() const {
        return gait_->getCurrentBlock();
    }
}