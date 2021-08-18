// ######################################################################### //
// Attitude Math Library
// Copyright (c) 2021 Dr. Steven Dumble Phd
// Distributed under the MIT Software License (See accompanying file LICENSE)
// ######################################################################### //

#ifndef AML_AMLDCM_H
#define AML_AMLDCM_H

#include "AMLMatrix33.hpp"
#include "AMLVector3.hpp"
#include <limits>

namespace AML
{

    class DCM
    {
    public:
        static const Matrix33 rotationX(double theta);
        static const Matrix33 rotationY(double theta);
        static const Matrix33 rotationZ(double theta);
    };

    bool isValidDCM(const Matrix33& dcm, double tol = std::numeric_limits<double>::epsilon());
    void normalise(Matrix33& dcm);

    // DCM Kinematic Functions
    Matrix33 integrateDCM(const Matrix33& dcm, const Matrix33& dcmRates, double dt);
    Matrix33 dcmKinematicRates_BodyRates(const Matrix33& dcm, const Vector3& bodyRates);
    Matrix33 dcmKinematicRates_WorldRates(const Matrix33& dcm, const Vector3& worldRates);
}; // namespace AML

#endif //AML_AMLDCM_H