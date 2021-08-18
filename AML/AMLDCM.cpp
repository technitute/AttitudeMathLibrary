// ######################################################################### //
// Attitude Math Library
// Copyright (c) 2021 Dr. Steven Dumble Phd
// Distributed under the MIT Software License (See accompanying file LICENSE)
// ######################################################################### //

#include "AMLDCM.hpp"
#include <cmath>

namespace AML
{

    const Matrix33 DCM::rotationX(double theta)
    {
        const double data[3][3] = { { 1.0, 0.0, 0.0 },
                                    { 0.0, cos(theta), sin(theta) },
                                    { 0.0, -sin(theta), cos(theta) } };
        return Matrix33(data);
    }

    const Matrix33 DCM::rotationY(double theta)
    {
        const double data[3][3] = { { cos(theta), 0.0, -sin(theta) },
                                    { 0.0, 1.0, 0.0 },
                                    { sin(theta), 0.0, cos(theta) } };
        return Matrix33(data);
    }

    const Matrix33 DCM::rotationZ(double theta)
    {
        const double data[3][3] = { { cos(theta), sin(theta), 0.0 },
                                    { -sin(theta), cos(theta), 0.0 },
                                    { 0.0, 0.0, 1.0 } };
        return Matrix33(data);
    }

    void normalise(Matrix33& dcm)
    {
        Vector3 X = Vector3(dcm.m11, dcm.m12, dcm.m13);
        Vector3 Y = Vector3(dcm.m21, dcm.m22, dcm.m23);
        double error = dot(X, Y);

        Vector3 X_orth = X - 0.5 * error * Y;
        Vector3 Y_orth = Y - 0.5 * error * X;
        Vector3 Z_orth = cross(X, Y);

        Vector3 X_norm = 0.5 * (3.0 - dot(X_orth, X_orth)) * X_orth;
        Vector3 Y_norm = 0.5 * (3.0 - dot(Y_orth, Y_orth)) * Y_orth;
        Vector3 Z_norm = 0.5 * (3.0 - dot(Z_orth, Z_orth)) * Z_orth;

        dcm = transpose(Matrix33(X_norm, Y_norm, Z_norm));
    }

    bool isValidDCM(const Matrix33& dcm, double tol)
    {
        double tol_limit = 2.0 * tol;

        // Test Determinant
        bool det_test = fabs(determinant(dcm) - 1.0) < tol_limit;
        bool neg_det_test = determinant(dcm) > 0.0;

        // Test Orthogonal
        Matrix33 test_identity_matrix = (dcm * transpose(dcm)) - Matrix33::identity();
        bool m11_identity_test = (fabs(test_identity_matrix.m11) < tol_limit);
        bool m12_identity_test = (fabs(test_identity_matrix.m12) < tol_limit);
        bool m13_identity_test = (fabs(test_identity_matrix.m13) < tol_limit);
        bool m21_identity_test = (fabs(test_identity_matrix.m21) < tol_limit);
        bool m22_identity_test = (fabs(test_identity_matrix.m22) < tol_limit);
        bool m23_identity_test = (fabs(test_identity_matrix.m23) < tol_limit);
        bool m31_identity_test = (fabs(test_identity_matrix.m31) < tol_limit);
        bool m32_identity_test = (fabs(test_identity_matrix.m32) < tol_limit);
        bool m33_identity_test = (fabs(test_identity_matrix.m33) < tol_limit);
        bool identity_check = m11_identity_test && m12_identity_test && m13_identity_test && m21_identity_test &&
                              m22_identity_test && m23_identity_test && m31_identity_test && m32_identity_test &&
                              m33_identity_test;

        return det_test && neg_det_test && identity_check;
    }

    // DCM Kinematic Functions
    Matrix33 integrateDCM(const Matrix33& dcm, const Matrix33& dcmRates, double dt)
    {
        Matrix33 dcmNew = dcm + dcmRates * dt; // First Order Euler Integration
        normalise(dcmNew);                     // Normalisation
        return dcmNew;
    }

    Matrix33 dcmKinematicRates_BodyRates(const Matrix33& dcm, const Vector3& bodyRates)
    {
        const double p = bodyRates.x;
        const double q = bodyRates.y;
        const double r = bodyRates.z;
        const double skewMatrixData[9] = { 0.0, -r, q, r, 0.0, -p, -q, p, 0.0 };
        const Matrix33 skewMatrix = Matrix33(skewMatrixData);
        return -skewMatrix * dcm;
    }

    Matrix33 dcmKinematicRates_WorldRates(const Matrix33& dcm, const Vector3& worldRates)
    {
        const double p = worldRates.x;
        const double q = worldRates.y;
        const double r = worldRates.z;
        const double skewMatrixData[9] = { 0.0, -r, q, r, 0.0, -p, -q, p, 0.0 };
        const Matrix33 skewMatrix = Matrix33(skewMatrixData);
        return dcm * skewMatrix;
    }
}; // namespace AML