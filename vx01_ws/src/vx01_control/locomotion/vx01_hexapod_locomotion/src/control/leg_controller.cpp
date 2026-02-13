#include "vx01_hexapod_locomotion/control/leg_controller.hpp"
#include <cmath>

namespace vx01_hexapod_locomotion {

    namespace control {

        LegController::LegController(int leg_id, double rotation_angle, double x_start, double L1, double L2, double L3)
            : leg_id_(leg_id), rotation_angle_(rotation_angle), x_start_(x_start),
            L1_(L1), L2_(L2), L3_(L3),
            fk_(L1, L2, L3), ik_(L1, L2, L3),
            theta1_(0.0), theta2_(0.0), theta3_(0.0),
            foot_x_(0.0), foot_y_(0.0), foot_z_(0.0) {}

        void LegController::setJointAngles(double theta1, double theta2, double theta3) {

            theta1_ = theta1;
            theta2_ = theta2;
            theta3_ = theta3;

            fk_.compute(theta1, theta2, theta3, foot_x_, foot_y_, foot_z_);
        }

        bool LegController::setFootPosition(double x, double y, double z) {
            
            double theta1, theta2, theta3;

            bool success = ik_.compute(x, y, z, theta1, theta2, theta3);

            if (success) {
                setJointAngles(theta1, theta2, theta3);
                foot_x_ = x;
                foot_y_ = y;
                foot_z_ = z;
            }

            return success;
        }

        void LegController::bodyToLegFrame(double body_x, double body_y, double body_z,
                                            double& leg_x, double& leg_y, double& leg_z) {

            double cos_rot = std::cos(-rotation_angle_);
            double sin_rot = std::sin(-rotation_angle_);

            double translated_x = body_x - x_start_;
            double translated_y = body_y;

            leg_x = cos_rot * translated_x - sin_rot * translated_y;
            leg_y = sin_rot * translated_x + cos_rot * translated_y;
            leg_z = body_z;
        }

        void LegController::legToBodyFrame(double leg_x, double leg_y, double leg_z,
                                            double& body_x, double& body_y, double& body_z) {

            double cos_rot = std::cos(rotation_angle_);
            double sin_rot = std::sin(rotation_angle_);

            double rotated_x = cos_rot * leg_x - sin_rot * leg_y;
            double rotated_y = sin_rot * leg_x + cos_rot * leg_y;

            body_x = rotated_x + x_start_;
            body_y = rotated_y;
            body_z = leg_z;
        }

        double LegController::getTheta1() const {
            return theta1_;
        }

        double LegController::getTheta2() const {
            return theta2_;
        }

        double LegController::getTheta3() const {
            return theta3_;
        }

        double LegController::getFootX() const {
            return foot_x_;
        }

        double LegController::getFootY() const {
            return foot_y_;
        }

        double LegController::getFootZ() const {
            return foot_z_;
        }

        int LegController::getLegID() const {
            return leg_id_;
        }

        double LegController::getRotationAngle() const {
            return rotation_angle_;
        }
    }
}