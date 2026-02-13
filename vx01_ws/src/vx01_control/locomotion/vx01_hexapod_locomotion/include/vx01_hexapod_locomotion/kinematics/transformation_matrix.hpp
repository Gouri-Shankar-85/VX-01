#ifndef VX01_HEXAPOD_LOCOMOTION_TRANSFORMATION_MARIX_HPP
#define VX01_HEXAPOD_LOCOMOTION_TRANSFORMATION_MARIX_HPP

#include "vx01_hexapod_locomotion/kinematics/dh_parameters.hpp"
#include <array>

namespace vx01_hexapod_locomotion {

    namespace kinematics {

        class TransformationMatrix {

            private:
                std::array<std::array<double, 4>, 4> matrix_;

            public:
                TransformationMatrix();
                TransformationMatrix(const DHParameters& dh);

                double get(int row, int col) const;

                void set(int row, int col, double value);

                TransformationMatrix multiply(const TransformationMatrix& other) const;

                double getX() const;
                double getY() const;
                double getZ() const;

            private:
                void buildFromDH(const DHParameters& dh);
        };
        
    }
}

#endif