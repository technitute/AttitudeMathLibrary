// ######################################################################### //
// Attitude Math Library
// Copyright (c) 2021 Dr. Steven Dumble Phd
// Distributed under the MIT Software License (See accompanying file LICENSE)
// ######################################################################### //

#ifndef AML_AMLEULERANGLES_H
#define AML_AMLEULERANGLES_H

#include "AMLMatrix33.hpp"
#include "AMLVector3.hpp"
#include <iostream>

namespace AML
{
    class EulerAngles
    {
    public:
        enum class EulerSequence
        {
            ZXZ,
            XYX,
            YZY,
            ZYZ,
            XZX,
            YXY,
            XYZ,
            YZX,
            ZXY,
            XZY,
            ZYX,
            YXZ
        };

        // Constructors
        EulerAngles();
        EulerAngles(double phi_, double theta_, double psi_, EulerSequence seq = EulerSequence::XYZ);

        // Euler Angle Operations
        EulerSequence getEulerSequence() const { return seq_; };

        // Euler Angles
        double phi;
        double theta;
        double psi;

    private:
        // Euler Angle Sequence
        EulerSequence seq_;
    };

    // Stream Functions
    std::ostream& operator<<(std::ostream& os, const EulerAngles& obj);

    // Euler Angle Conversions
    Matrix33 eulerAngles2DCM(const EulerAngles& angles);
    EulerAngles dcm2EulerAngles(const Matrix33& dcm,
                                const EulerAngles::EulerSequence seq = EulerAngles::EulerSequence::XYZ);
    EulerAngles convertEulerAngleSequence(const EulerAngles& angles, const EulerAngles::EulerSequence seq);

    // Euler Angles to DCM Conversions
    Matrix33 eulerAngles2DCM_ZXZ(double phi, double theta, double psi);
    Matrix33 eulerAngles2DCM_XYX(double phi, double theta, double psi);
    Matrix33 eulerAngles2DCM_YZY(double phi, double theta, double psi);
    Matrix33 eulerAngles2DCM_ZYZ(double phi, double theta, double psi);
    Matrix33 eulerAngles2DCM_XZX(double phi, double theta, double psi);
    Matrix33 eulerAngles2DCM_YXY(double phi, double theta, double psi);
    Matrix33 eulerAngles2DCM_XYZ(double phi, double theta, double psi);
    Matrix33 eulerAngles2DCM_YZX(double phi, double theta, double psi);
    Matrix33 eulerAngles2DCM_ZXY(double phi, double theta, double psi);
    Matrix33 eulerAngles2DCM_XZY(double phi, double theta, double psi);
    Matrix33 eulerAngles2DCM_ZYX(double phi, double theta, double psi);
    Matrix33 eulerAngles2DCM_YXZ(double phi, double theta, double psi);

    // DCM to Euler Angle Conversions
    EulerAngles dcm2EulerAngles_ZXZ(const Matrix33& dcm);
    EulerAngles dcm2EulerAngles_XYX(const Matrix33& dcm);
    EulerAngles dcm2EulerAngles_YZY(const Matrix33& dcm);
    EulerAngles dcm2EulerAngles_ZYZ(const Matrix33& dcm);
    EulerAngles dcm2EulerAngles_XZX(const Matrix33& dcm);
    EulerAngles dcm2EulerAngles_YXY(const Matrix33& dcm);
    EulerAngles dcm2EulerAngles_XYZ(const Matrix33& dcm);
    EulerAngles dcm2EulerAngles_YZX(const Matrix33& dcm);
    EulerAngles dcm2EulerAngles_ZXY(const Matrix33& dcm);
    EulerAngles dcm2EulerAngles_XZY(const Matrix33& dcm);
    EulerAngles dcm2EulerAngles_ZYX(const Matrix33& dcm);
    EulerAngles dcm2EulerAngles_YXZ(const Matrix33& dcm);

    // Euler Angle Sequence Conversions
    EulerAngles converEulerAnglesXYZtoZXZ(double phi, double theta, double psi);
    EulerAngles converEulerAnglesZXZtoXYZ(double phi, double theta, double psi);

    // Euler Angle Kinematics
    EulerAngles integrateEulerAngles(const EulerAngles& angles, const EulerAngles& angleRates, double dt);
    EulerAngles eulerAngleKinematicRates(const EulerAngles& angles, const Vector3& bodyRates);

    Matrix33 eulerAngleRatesMatrix_XYZ(double phi, double theta, double psi);
    Matrix33 eulerAngleRatesMatrix_ZXZ(double phi, double theta, double psi);
    Matrix33 eulerAngleRatesMatrix_YZY(double phi, double theta, double psi);
    Matrix33 eulerAngleRatesMatrix_XYX(double phi, double theta, double psi);
    Matrix33 eulerAngleRatesMatrix_ZYZ(double phi, double theta, double psi);
    Matrix33 eulerAngleRatesMatrix_XZX(double phi, double theta, double psi);
    Matrix33 eulerAngleRatesMatrix_YXY(double phi, double theta, double psi);
    Matrix33 eulerAngleRatesMatrix_YZX(double phi, double theta, double psi);
    Matrix33 eulerAngleRatesMatrix_ZXY(double phi, double theta, double psi);
    Matrix33 eulerAngleRatesMatrix_XZY(double phi, double theta, double psi);
    Matrix33 eulerAngleRatesMatrix_ZYX(double phi, double theta, double psi);
    Matrix33 eulerAngleRatesMatrix_YXZ(double phi, double theta, double psi);

    // Euler Angle Interpolation
    EulerAngles interpolate(const EulerAngles& startAngles, const EulerAngles& endAngles, double t);

}; // namespace AML

#endif // AML_AMLEULERANGLES_H