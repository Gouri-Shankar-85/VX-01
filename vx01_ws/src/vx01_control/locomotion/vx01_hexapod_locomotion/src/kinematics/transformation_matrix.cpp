#include "vx01_hexapod_locomotion/kinematics/transformation_matrix.hpp"
#include <cmath>

namespace vx01_hexapod_locomotion {

    namespace kinematics {

        TransformationMatrix::TransformationMatrix() {

            for (int i=0; i<4; i++) {
                for (int j=0; j<4; j++) {
                    matrix_[i][j] = (i==j) ? 1.0 : 0.0;
                }
            }
        }

        TransformationMatrix::TransformationMatrix(const DHParameters& dh) {
            buildFromDH(dh);
        }

        void TransformationMatrix::buildFromDH(const DHParameters& dh) {
            double a     = dh.getA();
            double alpha = dh.getAlpha();
            double d     = dh.getD();
            double theta = dh.getTheta();

            double c_theta = cos(theta);
            double s_theta = sin(theta);
            double c_alpha = cos(alpha);
            double s_alpha = sin(alpha);
            
            // Standard DH transformation matrix  i-1_A_i
            // Row 0
            matrix_[0][0] = c_theta;
            matrix_[0][1] = -s_theta * c_alpha;
            matrix_[0][2] =  s_theta * s_alpha;
            matrix_[0][3] =  a * c_theta;
            
            // Row 1
            matrix_[1][0] = s_theta;
            matrix_[1][1] =  c_theta * c_alpha;
            matrix_[1][2] = -c_theta * s_alpha;
            matrix_[1][3] =  a * s_theta;
            
            // Row 2
            matrix_[2][0] = 0.0;
            matrix_[2][1] = s_alpha;
            matrix_[2][2] = c_alpha;
            matrix_[2][3] = d;
            
            // Row 3
            matrix_[3][0] = 0.0;
            matrix_[3][1] = 0.0;
            matrix_[3][2] = 0.0;
            matrix_[3][3] = 1.0;
        }

        double TransformationMatrix::get(int row, int col) const {
            return matrix_[row][col];
        }

        void TransformationMatrix::set(int row, int col, double value) {
            matrix_[row][col] = value;
        }

        TransformationMatrix TransformationMatrix::multiply(const TransformationMatrix& other) const {
            TransformationMatrix result;
            
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    double sum = 0.0;
                    for (int k = 0; k < 4; k++) {
                        sum += matrix_[i][k] * other.get(k, j);
                    }
                    result.set(i, j, sum);
                }
            }
            
            return result;
        }

        double TransformationMatrix::getX() const {
            return matrix_[0][3];
        }

        double TransformationMatrix::getY() const {
            return matrix_[1][3];
        }

        double TransformationMatrix::getZ() const {
            return matrix_[2][3];
        }
    }
}