// ######################################################################### //
// Attitude Math Library
// Copyright (c) 2021 Dr. Steven Dumble Phd
// Distributed under the MIT Software License (See accompanying file LICENSE)
// ######################################################################### //

#ifndef AML_AMLQUATERNION_H
#define AML_AMLQUATERNION_H

#include "AMLEulerAngles.hpp"
#include "AMLMatrix33.hpp"
#include "AMLVector3.hpp"
#include <iostream>
#include <limits>

namespace AML
{
    class Quaternion
    {
    public:
        union
        {
            double data[4];
            struct
            {
                double q0, q1, q2, q3;
            };
        };

        // Constructors
        Quaternion();
        explicit Quaternion(double q0_, double q1_, double q2_, double q3_);
        explicit Quaternion(double val);
        explicit Quaternion(const double data[4]);
        explicit Quaternion(double scalar, const Vector3& vec);
        explicit Quaternion(const Vector3& rhs);

        // Operator Assignments
        Quaternion& operator+=(const Quaternion& rhs);
        Quaternion& operator-=(const Quaternion& rhs);
        Quaternion& operator*=(const Quaternion& rhs);

        Quaternion& operator+=(double rhs);
        Quaternion& operator-=(double rhs);
        Quaternion& operator*=(double rhs);
        Quaternion& operator/=(double rhs);

        // Special Object Creators
        static const Quaternion identity();
    };

    // Quaternion / Quaternion Operations
    Quaternion operator-(const Quaternion& rhs);
    Quaternion operator+(const Quaternion& lhs, const Quaternion& rhs);
    Quaternion operator-(const Quaternion& lhs, const Quaternion& rhs);
    Quaternion operator*(const Quaternion& lhs, const Quaternion& rhs);

    // Quaternion / Scalar Operations
    Quaternion operator+(const Quaternion& lhs, double s);
    Quaternion operator-(const Quaternion& lhs, double s);
    Quaternion operator*(const Quaternion& lhs, double s);
    Quaternion operator/(const Quaternion& lhs, double s);
    Quaternion operator+(double s, const Quaternion& rhs);
    Quaternion operator-(double s, const Quaternion& rhs);
    Quaternion operator*(double s, const Quaternion& rhs);
    Quaternion operator/(double s, const Quaternion& rhs);

    // Quaternion / Vector Operations
    Vector3 operator*(const Quaternion& lhs, const Vector3& rhs);

    // Quaternion Operations
    Quaternion conjugate(const Quaternion& rhs);
    double norm(const Quaternion& rhs);
    void normalise(Quaternion& rhs);
    Quaternion inverse(const Quaternion& rhs);
    Quaternion unit(const Quaternion& rhs);
    bool isUnitQuat(const Quaternion& rhs, double tol = std::numeric_limits<double>::epsilon());
    double dot(const Quaternion& lhs, const Quaternion& rhs);

    // Attitude Conversion Functions
    Matrix33 quat2DCM(const Quaternion& quat);
    Quaternion dcm2Quat(const Matrix33& dcm);
    EulerAngles quat2EulerAngles(const Quaternion& quat,
                                 const EulerAngles::EulerSequence seq = EulerAngles::EulerSequence::XYZ);
    Quaternion eulerAngles2Quat(const EulerAngles& angles);

    // Euler Angles to Quaternion Conversions
    Quaternion eulerAngles2Quat_ZXZ(double phi, double theta, double psi);
    Quaternion eulerAngles2Quat_XYX(double phi, double theta, double psi);
    Quaternion eulerAngles2Quat_YZY(double phi, double theta, double psi);
    Quaternion eulerAngles2Quat_ZYZ(double phi, double theta, double psi);
    Quaternion eulerAngles2Quat_XZX(double phi, double theta, double psi);
    Quaternion eulerAngles2Quat_YXY(double phi, double theta, double psi);
    Quaternion eulerAngles2Quat_XYZ(double phi, double theta, double psi);
    Quaternion eulerAngles2Quat_YZX(double phi, double theta, double psi);
    Quaternion eulerAngles2Quat_ZXY(double phi, double theta, double psi);
    Quaternion eulerAngles2Quat_XZY(double phi, double theta, double psi);
    Quaternion eulerAngles2Quat_ZYX(double phi, double theta, double psi);
    Quaternion eulerAngles2Quat_YXZ(double phi, double theta, double psi);

    // Quaternion to Euler Angle Conversions
    EulerAngles quat2EulerAngles_ZXZ(const Quaternion& quat);
    EulerAngles quat2EulerAngles_XYX(const Quaternion& quat);
    EulerAngles quat2EulerAngles_YZY(const Quaternion& quat);
    EulerAngles quat2EulerAngles_ZYZ(const Quaternion& quat);
    EulerAngles quat2EulerAngles_XZX(const Quaternion& quat);
    EulerAngles quat2EulerAngles_YXY(const Quaternion& quat);
    EulerAngles quat2EulerAngles_XYZ(const Quaternion& quat);
    EulerAngles quat2EulerAngles_YZX(const Quaternion& quat);
    EulerAngles quat2EulerAngles_ZXY(const Quaternion& quat);
    EulerAngles quat2EulerAngles_XZY(const Quaternion& quat);
    EulerAngles quat2EulerAngles_ZYX(const Quaternion& quat);
    EulerAngles quat2EulerAngles_YXZ(const Quaternion& quat);

    // Quaternion Kinematic Functions
    Quaternion integrateQuat(const Quaternion& quat, const Quaternion& quatRates, double dt);
    Quaternion quatKinematicRates_BodyRates(const Quaternion& quat, const Vector3& bodyRates);
    Quaternion quatKinematicRates_WorldRates(const Quaternion& quat, const Vector3& worldRates);

    // Quaternion Interpolation Functions
    Quaternion linearInterpolate(const Quaternion& startAngles, const Quaternion& endAngles, double t);
    Quaternion slerpInterpolate(const Quaternion& startAngles, const Quaternion& endAngles, double t);

    // Stream Functions
    std::ostream& operator<<(std::ostream& os, const Quaternion& obj);
}; // namespace AML

#endif //AML_AMLQUATERNION_H