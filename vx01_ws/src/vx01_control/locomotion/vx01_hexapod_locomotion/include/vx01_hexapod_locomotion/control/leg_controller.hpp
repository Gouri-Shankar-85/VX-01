#ifndef VX01_HEXAPOD_LOCOMOTION_CONTROL_LEG_CONTROLLER_HPP
#define VX01_HEXAPOD_LOCOMOTION_CONTROL_LEG_CONTROLLER_HPP

#include "vx01_hexapod_locomotion/kinematics/forward_kinematics.hpp"
#include "vx01_hexapod_locomotion/kinematics/inverse_kinematics.hpp"

namespace vx01_hexapod_locomotion {

    namespace control {

        class LegController {

            private:

                int leg_id_;
                double rotation_angle_;
                double x_start_;

                double L1_;
                double L2_;
                double L3_;

                kinematics::ForwardKinematics fk_;
                kinematics::InverseKinematics ik_;

                double theta1_;
                double theta2_;
                double theta3_;

                double foot_x_;
                double foot_y_;
                double foot_z_;

            public:

                LegController(int leg_id, double rotation_angle, double x_start, double L1, double L2, double L3);

                void setJointAngles(double theta1, double theta2, double theta3);   

                bool setFootPosition(double x, double y, double z);

                void bodyToLegFrame(double body_x, double body_y, double body_z, 
                                    double& leg_x, double& leg_y, double& leg_z);
                
                void legToBodyFrame(double leg_x, double leg_y, double leg_z,
                                    double& body_x, double& body_y, double& body_z);

                double getTheta1() const;
                double getTheta2() const;
                double getTheta3() const;

                double getFootX() const;
                double getFootY() const;
                double getFootZ() const;

                int getLegID() const;

                double getRotationAngle() const;
        };
    }
}

#endif