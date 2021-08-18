// ######################################################################### //
// Attitude Math Library
// Copyright (c) 2021 Dr. Steven Dumble Phd
// Distributed under the MIT Software License (See accompanying file LICENSE)
// ######################################################################### //

#include "AMLDCM.hpp"
#include "AMLQuaternion.hpp"
#include <cmath>

namespace AML
{
    // Constructors
    Quaternion::Quaternion() : q0(0.0), q1(0.0), q2(0.0), q3(0.0) {}

    Quaternion::Quaternion(double q0_, double q1_, double q2_, double q3_) : q0(q0_), q1(q1_), q2(q2_), q3(q3_) {}

    Quaternion::Quaternion(double val) : q0(val), q1(val), q2(val), q3(val) {}

    Quaternion::Quaternion(const double data_[4]) : q0(data_[0]), q1(data_[1]), q2(data_[2]), q3(data_[3]) {}

    Quaternion::Quaternion(double scalar, const Vector3& vec) : q0(scalar), q1(vec.x), q2(vec.y), q3(vec.z) {}

    Quaternion::Quaternion(const Vector3& rhs) : q0(0.0), q1(rhs.x), q2(rhs.y), q3(rhs.z) {}

    // Operator Assignments
    Quaternion& Quaternion::operator+=(const Quaternion& rhs)
    {
        q0 += rhs.q0;
        q1 += rhs.q1;
        q2 += rhs.q2;
        q3 += rhs.q3;
        return *this;
    }
    Quaternion& Quaternion::operator-=(const Quaternion& rhs)
    {
        q0 -= rhs.q0;
        q1 -= rhs.q1;
        q2 -= rhs.q2;
        q3 -= rhs.q3;
        return *this;
    }
    Quaternion& Quaternion::operator*=(const Quaternion& rhs)
    {
        double q0_new = (rhs.q0 * q0) - (rhs.q1 * q1) - (rhs.q2 * q2) - (rhs.q3 * q3);
        double q1_new = (rhs.q0 * q1) + (rhs.q1 * q0) - (rhs.q2 * q3) + (rhs.q3 * q2);
        double q2_new = (rhs.q0 * q2) + (rhs.q1 * q3) + (rhs.q2 * q0) - (rhs.q3 * q1);
        double q3_new = (rhs.q0 * q3) - (rhs.q1 * q2) + (rhs.q2 * q1) + (rhs.q3 * q0);
        q0 = q0_new;
        q1 = q1_new;
        q2 = q2_new;
        q3 = q3_new;
        return *this;
    }

    Quaternion& Quaternion::operator+=(double rhs)
    {
        q0 += rhs;
        q1 += rhs;
        q2 += rhs;
        q3 += rhs;
        return *this;
    }
    Quaternion& Quaternion::operator-=(double rhs)
    {
        q0 -= rhs;
        q1 -= rhs;
        q2 -= rhs;
        q3 -= rhs;
        return *this;
    }
    Quaternion& Quaternion::operator*=(double rhs)
    {
        q0 *= rhs;
        q1 *= rhs;
        q2 *= rhs;
        q3 *= rhs;
        return *this;
    }
    Quaternion& Quaternion::operator/=(double rhs)
    {
        q0 /= rhs;
        q1 /= rhs;
        q2 /= rhs;
        q3 /= rhs;
        return *this;
    }

    // Special Object Creators
    const Quaternion Quaternion::identity() { return Quaternion(1.0, 0.0, 0.0, 0.0); }

    // Steam Functions
    std::ostream& operator<<(std::ostream& os, const Quaternion& obj)
    {
        os << "QUAT [" << obj.q0 << "," << obj.q1 << "," << obj.q2 << "," << obj.q3 << "]";
        return os;
    }

    // Quaternion / Quaternion Operations
    Quaternion operator-(const Quaternion& rhs) { return (Quaternion(rhs) *= -1.0); }
    Quaternion operator+(const Quaternion& lhs, const Quaternion& rhs) { return (Quaternion(lhs) += rhs); }
    Quaternion operator-(const Quaternion& lhs, const Quaternion& rhs) { return (Quaternion(lhs) -= rhs); }
    Quaternion operator*(const Quaternion& lhs, const Quaternion& rhs) { return (Quaternion(lhs) *= rhs); }

    // Quaternion / Scalar Operations
    Quaternion operator+(const Quaternion& lhs, double s) { return (Quaternion(lhs) += s); }
    Quaternion operator-(const Quaternion& lhs, double s) { return (Quaternion(lhs) -= s); }
    Quaternion operator*(const Quaternion& lhs, double s) { return (Quaternion(lhs) *= s); }
    Quaternion operator/(const Quaternion& lhs, double s) { return (Quaternion(lhs) /= s); }
    Quaternion operator+(double s, const Quaternion& rhs)
    {
        return Quaternion(s + rhs.q0, s + rhs.q1, s + rhs.q2, s + rhs.q3);
    }
    Quaternion operator-(double s, const Quaternion& rhs)
    {
        return Quaternion(s - rhs.q0, s - rhs.q1, s - rhs.q2, s - rhs.q3);
    }
    Quaternion operator*(double s, const Quaternion& rhs)
    {
        return Quaternion(s * rhs.q0, s * rhs.q1, s * rhs.q2, s * rhs.q3);
    }
    Quaternion operator/(double s, const Quaternion& rhs)
    {
        return Quaternion(s / rhs.q0, s / rhs.q1, s / rhs.q2, s / rhs.q3);
    }

    // Quaternion / Vector Operations
    Vector3 operator*(const Quaternion& lhs, const Vector3& rhs) { return quat2DCM(lhs) * rhs; }

    // Quaternion Operations
    Quaternion conjugate(const Quaternion& rhs) { return Quaternion(rhs.q0, -rhs.q1, -rhs.q2, -rhs.q3); }
    double norm(const Quaternion& rhs)
    {
        return sqrt(rhs.q0 * rhs.q0 + rhs.q1 * rhs.q1 + rhs.q2 * rhs.q2 + rhs.q3 * rhs.q3);
    }
    Quaternion inverse(const Quaternion& rhs) { return (conjugate(rhs) / norm(rhs)); }
    Quaternion unit(const Quaternion& rhs)
    {
        double mag = norm(rhs);
        if (mag > 0.0)
        {
            return (Quaternion(rhs) / mag);
        }
        return Quaternion(rhs);
    }
    void normalise(Quaternion& rhs)
    {
        double mag = norm(rhs);
        if (mag > 0.0)
        {
            rhs /= mag;
        }
    }
    bool isUnitQuat(const Quaternion& rhs, double tol) { return (fabs(norm(rhs) - 1.0) < 2.0 * tol); }
    double dot(const Quaternion& lhs, const Quaternion& rhs)
    {
        return (rhs.q0 * lhs.q0 + rhs.q1 * lhs.q1 + rhs.q2 * lhs.q2 + rhs.q3 * lhs.q3);
    }

    // DCM Conversion Functions
    Matrix33 quat2DCM(const Quaternion& rhs)
    {
        const double TOL = 0.0001;

        // Check if valid rotation quaternion
        if (isUnitQuat(rhs, TOL))
        {
            double data[3][3];
            const double q0 = rhs.q0;
            const double q1 = rhs.q1;
            const double q2 = rhs.q2;
            const double q3 = rhs.q3;
            const double q0_2 = q0 * q0;
            const double q1_2 = q1 * q1;
            const double q2_2 = q2 * q2;
            const double q3_2 = q3 * q3;
            const double q1q2 = q1 * q2;
            const double q0q3 = q0 * q3;
            const double q1q3 = q1 * q3;
            const double q0q2 = q0 * q2;
            const double q2q3 = q2 * q3;
            const double q0q1 = q0 * q1;
            data[0][0] = q0_2 + q1_2 - q2_2 - q3_2;
            data[0][1] = 2.0 * (q1q2 + q0q3);
            data[0][2] = 2.0 * (q1q3 - q0q2);
            data[1][0] = 2.0 * (q1q2 - q0q3);
            data[1][1] = q0_2 - q1_2 + q2_2 - q3_2;
            data[1][2] = 2.0 * (q2q3 + q0q1);
            data[2][0] = 2.0 * (q1q3 + q0q2);
            data[2][1] = 2.0 * (q2q3 - q0q1);
            data[2][2] = q0_2 - q1_2 - q2_2 + q3_2;
            return Matrix33(data);
        }
        return Matrix33::identity();
    }
    Quaternion dcm2Quat(const Matrix33& dcm)
    {
        const double TOL = 0.0001;

        // Check if DCM is Valid
        if (isValidDCM(dcm, TOL))
        {
            double q0 = 0.0;
            double q1 = 0.0;
            double q2 = 0.0;
            double q3 = 0.0;
            const double x4q0_2 = (1.0 + dcm.m11 + dcm.m22 + dcm.m33);
            const double x4q1_2 = (1.0 + dcm.m11 - dcm.m22 - dcm.m33);
            const double x4q2_2 = (1.0 - dcm.m11 + dcm.m22 - dcm.m33);
            const double x4q3_2 = (1.0 - dcm.m11 - dcm.m22 + dcm.m33);
            const double x4q2q3 = dcm.m23 + dcm.m32;
            const double x4q1q3 = dcm.m31 + dcm.m13;
            const double x4q1q2 = dcm.m12 + dcm.m21;
            const double x4q0q1 = dcm.m23 - dcm.m32;
            const double x4q0q2 = dcm.m31 - dcm.m13;
            const double x4q0q3 = dcm.m12 - dcm.m21;

            // Check which Trace is Largest
            if (x4q0_2 >= x4q1_2 && x4q0_2 >= x4q2_2 && x4q0_2 >= x4q3_2) // 4q0_2 Largest
            {
                const double x2q0 = sqrt(x4q0_2);
                q0 = 0.5 * x2q0;
                q1 = 0.5 * x4q0q1 / x2q0;
                q2 = 0.5 * x4q0q2 / x2q0;
                q3 = 0.5 * x4q0q3 / x2q0;
            }
            else if (x4q1_2 >= x4q0_2 && x4q1_2 >= x4q2_2 && x4q1_2 >= x4q3_2) // 4q1_2 Largest
            {
                const double x2q1 = sqrt(x4q1_2);
                q0 = 0.5 * x4q0q1 / x2q1;
                q1 = 0.5 * x2q1;
                q2 = 0.5 * x4q1q2 / x2q1;
                q3 = 0.5 * x4q1q3 / x2q1;
            }
            else if (x4q2_2 >= x4q0_2 && x4q2_2 >= x4q1_2 && x4q2_2 >= x4q3_2) // 4q2_2 Largest
            {
                const double x2q2 = sqrt(x4q2_2);
                q0 = 0.5 * x4q0q2 / x2q2;
                q1 = 0.5 * x4q1q2 / x2q2;
                q2 = 0.5 * x2q2;
                q3 = 0.5 * x4q2q3 / x2q2;
            }
            else if (x4q3_2 >= x4q0_2 && x4q3_2 >= x4q1_2 && x4q3_2 >= x4q2_2) // 4q3_2 Largest
            {
                const double x2q3 = sqrt(x4q3_2);
                q0 = 0.5 * x4q0q3 / x2q3;
                q1 = 0.5 * x4q1q3 / x2q3;
                q2 = 0.5 * x4q2q3 / x2q3;
                q3 = 0.5 * x2q3;
            }
            return Quaternion(q0, q1, q2, q3);
        }
        return Quaternion::identity();
    }

    Quaternion eulerAngles2Quat(const EulerAngles& angles)
    {
        switch (angles.getEulerSequence())
        {
        case EulerAngles::EulerSequence::XYZ:
            return eulerAngles2Quat_XYZ(angles.phi, angles.theta, angles.psi);
        case EulerAngles::EulerSequence::ZXZ:
            return eulerAngles2Quat_ZXZ(angles.phi, angles.theta, angles.psi);
        case EulerAngles::EulerSequence::XYX:
            return eulerAngles2Quat_XYX(angles.phi, angles.theta, angles.psi);
        case EulerAngles::EulerSequence::YZY:
            return eulerAngles2Quat_YZY(angles.phi, angles.theta, angles.psi);
        case EulerAngles::EulerSequence::ZYZ:
            return eulerAngles2Quat_ZYZ(angles.phi, angles.theta, angles.psi);
        case EulerAngles::EulerSequence::XZX:
            return eulerAngles2Quat_XZX(angles.phi, angles.theta, angles.psi);
        case EulerAngles::EulerSequence::YXY:
            return eulerAngles2Quat_YXY(angles.phi, angles.theta, angles.psi);
        case EulerAngles::EulerSequence::YZX:
            return eulerAngles2Quat_YZX(angles.phi, angles.theta, angles.psi);
        case EulerAngles::EulerSequence::ZXY:
            return eulerAngles2Quat_ZXY(angles.phi, angles.theta, angles.psi);
        case EulerAngles::EulerSequence::XZY:
            return eulerAngles2Quat_XZY(angles.phi, angles.theta, angles.psi);
        case EulerAngles::EulerSequence::ZYX:
            return eulerAngles2Quat_ZYX(angles.phi, angles.theta, angles.psi);
        case EulerAngles::EulerSequence::YXZ:
            return eulerAngles2Quat_YXZ(angles.phi, angles.theta, angles.psi);
        }
        return Quaternion::identity();
    }

    EulerAngles quat2EulerAngles(const Quaternion& quat, const EulerAngles::EulerSequence seq)
    {
        const double TOL = 0.0001;

        // Check for valid Quat
        if (isUnitQuat(quat, TOL))
        {
            switch (seq)
            {
            case EulerAngles::EulerSequence::XYZ:
                return quat2EulerAngles_XYZ(quat);
            case EulerAngles::EulerSequence::ZXZ:
                return quat2EulerAngles_ZXZ(quat);
            case EulerAngles::EulerSequence::XYX:
                return quat2EulerAngles_XYX(quat);
            case EulerAngles::EulerSequence::YZY:
                return quat2EulerAngles_YZY(quat);
            case EulerAngles::EulerSequence::ZYZ:
                return quat2EulerAngles_ZYZ(quat);
            case EulerAngles::EulerSequence::XZX:
                return quat2EulerAngles_XZX(quat);
            case EulerAngles::EulerSequence::YXY:
                return quat2EulerAngles_YXY(quat);
            case EulerAngles::EulerSequence::YZX:
                return quat2EulerAngles_YZX(quat);
            case EulerAngles::EulerSequence::ZXY:
                return quat2EulerAngles_ZXY(quat);
            case EulerAngles::EulerSequence::XZY:
                return quat2EulerAngles_XZY(quat);
            case EulerAngles::EulerSequence::ZYX:
                return quat2EulerAngles_ZYX(quat);
            case EulerAngles::EulerSequence::YXZ:
                return quat2EulerAngles_YXZ(quat);
            }
        }
        return EulerAngles();
    }

    Quaternion eulerAngles2Quat_ZXZ(double phi, double theta, double psi)
    {
        const double c1 = cos(0.5 * phi);
        const double s1 = sin(0.5 * phi);
        const double c2 = cos(0.5 * theta);
        const double s2 = sin(0.5 * theta);
        const double c3 = cos(0.5 * psi);
        const double s3 = sin(0.5 * psi);
        const double q0 = c1 * c2 * c3 - s1 * c2 * s3;
        const double q1 = c1 * s2 * c3 + s1 * s2 * s3;
        const double q2 = c1 * s2 * s3 - s1 * s2 * c3;
        const double q3 = c1 * c2 * s3 + s1 * c2 * c3;
        return Quaternion(q0, q1, q2, q3);
    }
    Quaternion eulerAngles2Quat_XYX(double phi, double theta, double psi)
    {
        const double c1 = cos(0.5 * phi);
        const double s1 = sin(0.5 * phi);
        const double c2 = cos(0.5 * theta);
        const double s2 = sin(0.5 * theta);
        const double c3 = cos(0.5 * psi);
        const double s3 = sin(0.5 * psi);
        const double q0 = c1 * c2 * c3 - s1 * c2 * s3;
        const double q1 = c1 * c2 * s3 + s1 * c2 * c3;
        const double q2 = c1 * s2 * c3 + s1 * s2 * s3;
        const double q3 = c1 * s2 * s3 - s1 * s2 * c3;
        return Quaternion(q0, q1, q2, q3);
    }
    Quaternion eulerAngles2Quat_YZY(double phi, double theta, double psi)
    {
        const double c1 = cos(0.5 * phi);
        const double s1 = sin(0.5 * phi);
        const double c2 = cos(0.5 * theta);
        const double s2 = sin(0.5 * theta);
        const double c3 = cos(0.5 * psi);
        const double s3 = sin(0.5 * psi);
        const double q0 = c1 * c2 * c3 - s1 * c2 * s3;
        const double q1 = c1 * s2 * s3 - s1 * s2 * c3;
        const double q2 = c1 * c2 * s3 + s1 * c2 * c3;
        const double q3 = c1 * s2 * c3 + s1 * s2 * s3;
        return Quaternion(q0, q1, q2, q3);
    }
    Quaternion eulerAngles2Quat_ZYZ(double phi, double theta, double psi)
    {
        const double c1 = cos(0.5 * phi);
        const double s1 = sin(0.5 * phi);
        const double c2 = cos(0.5 * theta);
        const double s2 = sin(0.5 * theta);
        const double c3 = cos(0.5 * psi);
        const double s3 = sin(0.5 * psi);
        const double q0 = c1 * c2 * c3 - s1 * c2 * s3;
        const double q1 = s1 * s2 * c3 - c1 * s2 * s3;
        const double q2 = c1 * s2 * c3 + s1 * s2 * s3;
        const double q3 = c1 * c2 * s3 + s1 * c2 * c3;
        return Quaternion(q0, q1, q2, q3);
    }
    Quaternion eulerAngles2Quat_XZX(double phi, double theta, double psi)
    {
        const double c1 = cos(0.5 * phi);
        const double s1 = sin(0.5 * phi);
        const double c2 = cos(0.5 * theta);
        const double s2 = sin(0.5 * theta);
        const double c3 = cos(0.5 * psi);
        const double s3 = sin(0.5 * psi);
        const double q0 = c1 * c2 * c3 - s1 * c2 * s3;
        const double q1 = c1 * c2 * s3 + s1 * c2 * c3;
        const double q2 = s1 * s2 * c3 - c1 * s2 * s3;
        const double q3 = c1 * s2 * c3 + s1 * s2 * s3;
        return Quaternion(q0, q1, q2, q3);
    }
    Quaternion eulerAngles2Quat_YXY(double phi, double theta, double psi)
    {
        const double c1 = cos(0.5 * phi);
        const double s1 = sin(0.5 * phi);
        const double c2 = cos(0.5 * theta);
        const double s2 = sin(0.5 * theta);
        const double c3 = cos(0.5 * psi);
        const double s3 = sin(0.5 * psi);
        const double q0 = c1 * c2 * c3 - s1 * c2 * s3;
        const double q1 = c1 * s2 * c3 + s1 * s2 * s3;
        const double q2 = c1 * c2 * s3 + s1 * c2 * c3;
        const double q3 = s1 * s2 * c3 - c1 * s2 * s3;
        return Quaternion(q0, q1, q2, q3);
    }
    Quaternion eulerAngles2Quat_XYZ(double phi, double theta, double psi)
    {
        const double c1 = cos(0.5 * phi);
        const double s1 = sin(0.5 * phi);
        const double c2 = cos(0.5 * theta);
        const double s2 = sin(0.5 * theta);
        const double c3 = cos(0.5 * psi);
        const double s3 = sin(0.5 * psi);
        const double q0 = c1 * c2 * c3 + s1 * s2 * s3;
        const double q1 = s1 * c2 * c3 - c1 * s2 * s3;
        const double q2 = c1 * s2 * c3 + s1 * c2 * s3;
        const double q3 = c1 * c2 * s3 - s1 * s2 * c3;
        return Quaternion(q0, q1, q2, q3);
    }
    Quaternion eulerAngles2Quat_YZX(double phi, double theta, double psi)
    {
        const double c1 = cos(0.5 * phi);
        const double s1 = sin(0.5 * phi);
        const double c2 = cos(0.5 * theta);
        const double s2 = sin(0.5 * theta);
        const double c3 = cos(0.5 * psi);
        const double s3 = sin(0.5 * psi);
        const double q0 = c1 * c2 * c3 + s1 * s2 * s3;
        const double q1 = c1 * c2 * s3 - s1 * s2 * c3;
        const double q2 = s1 * c2 * c3 - c1 * s2 * s3;
        const double q3 = c1 * s2 * c3 + s1 * c2 * s3;
        return Quaternion(q0, q1, q2, q3);
    }
    Quaternion eulerAngles2Quat_ZXY(double phi, double theta, double psi)
    {
        const double c1 = cos(0.5 * phi);
        const double s1 = sin(0.5 * phi);
        const double c2 = cos(0.5 * theta);
        const double s2 = sin(0.5 * theta);
        const double c3 = cos(0.5 * psi);
        const double s3 = sin(0.5 * psi);
        const double q0 = c1 * c2 * c3 + s1 * s2 * s3;
        const double q1 = c1 * s2 * c3 + s1 * c2 * s3;
        const double q2 = c1 * c2 * s3 - s1 * s2 * c3;
        const double q3 = s1 * c2 * c3 - c1 * s2 * s3;
        return Quaternion(q0, q1, q2, q3);
    }
    Quaternion eulerAngles2Quat_XZY(double phi, double theta, double psi)
    {
        const double c1 = cos(0.5 * phi);
        const double s1 = sin(0.5 * phi);
        const double c2 = cos(0.5 * theta);
        const double s2 = sin(0.5 * theta);
        const double c3 = cos(0.5 * psi);
        const double s3 = sin(0.5 * psi);
        const double q0 = c1 * c2 * c3 - s1 * s2 * s3;
        const double q1 = c1 * s2 * s3 + s1 * c2 * c3;
        const double q2 = c1 * c2 * s3 + s1 * s2 * c3;
        const double q3 = c1 * s2 * c3 - s1 * c2 * s3;
        return Quaternion(q0, q1, q2, q3);
    }
    Quaternion eulerAngles2Quat_ZYX(double phi, double theta, double psi)
    {
        const double c1 = cos(0.5 * phi);
        const double s1 = sin(0.5 * phi);
        const double c2 = cos(0.5 * theta);
        const double s2 = sin(0.5 * theta);
        const double c3 = cos(0.5 * psi);
        const double s3 = sin(0.5 * psi);
        const double q0 = c1 * c2 * c3 - s1 * s2 * s3;
        const double q1 = c1 * c2 * s3 + s1 * s2 * c3;
        const double q2 = c1 * s2 * c3 - s1 * c2 * s3;
        const double q3 = c1 * s2 * s3 + s1 * c2 * c3;
        return Quaternion(q0, q1, q2, q3);
    }
    Quaternion eulerAngles2Quat_YXZ(double phi, double theta, double psi)
    {
        const double c1 = cos(0.5 * phi);
        const double s1 = sin(0.5 * phi);
        const double c2 = cos(0.5 * theta);
        const double s2 = sin(0.5 * theta);
        const double c3 = cos(0.5 * psi);
        const double s3 = sin(0.5 * psi);
        const double q0 = c1 * c2 * c3 - s1 * s2 * s3;
        const double q1 = c1 * s2 * c3 - s1 * c2 * s3;
        const double q2 = c1 * s2 * s3 + s1 * c2 * c3;
        const double q3 = c1 * c2 * s3 + s1 * s2 * c3;
        return Quaternion(q0, q1, q2, q3);
    }

    // Quaternion to Euler Angle Conversions
    EulerAngles quat2EulerAngles_ZXZ(const Quaternion& quat)
    {
        const double q0_2 = quat.q0 * quat.q0;
        const double q1_2 = quat.q1 * quat.q1;
        const double q2_2 = quat.q2 * quat.q2;
        const double q3_2 = quat.q3 * quat.q3;
        const double x2q1q3 = 2.0 * quat.q1 * quat.q3;
        const double x2q0q2 = 2.0 * quat.q0 * quat.q2;
        const double x2q2q3 = 2.0 * quat.q2 * quat.q3;
        const double x2q0q1 = 2.0 * quat.q0 * quat.q1;
        const double phi = atan2(x2q1q3 - x2q0q2, x2q2q3 + x2q0q1);
        const double theta = acos(q0_2 - q1_2 - q2_2 + q3_2);
        const double psi = atan2(x2q1q3 + x2q0q2, -(x2q2q3 - x2q0q1));
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::ZXZ);
    }
    EulerAngles quat2EulerAngles_XYX(const Quaternion& quat)
    {
        const double q0_2 = quat.q0 * quat.q0;
        const double q1_2 = quat.q1 * quat.q1;
        const double q2_2 = quat.q2 * quat.q2;
        const double q3_2 = quat.q3 * quat.q3;
        const double x2q1q2 = 2.0 * quat.q1 * quat.q2;
        const double x2q0q3 = 2.0 * quat.q0 * quat.q3;
        const double x2q1q3 = 2.0 * quat.q1 * quat.q3;
        const double x2q0q2 = 2.0 * quat.q0 * quat.q2;
        const double m11 = q0_2 + q1_2 - q2_2 - q3_2;
        const double m12 = x2q1q2 + x2q0q3;
        const double m13 = x2q1q3 - x2q0q2;
        const double m21 = x2q1q2 - x2q0q3;
        const double m31 = x2q1q3 + x2q0q2;
        const double phi = atan2(m21, m31);
        const double theta = acos(m11);
        const double psi = atan2(m12, -m13);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::XYX);
    }
    EulerAngles quat2EulerAngles_YZY(const Quaternion& quat)
    {
        const double q0_2 = quat.q0 * quat.q0;
        const double q1_2 = quat.q1 * quat.q1;
        const double q2_2 = quat.q2 * quat.q2;
        const double q3_2 = quat.q3 * quat.q3;
        const double x2q1q2 = 2.0 * quat.q1 * quat.q2;
        const double x2q0q3 = 2.0 * quat.q0 * quat.q3;
        const double x2q2q3 = 2.0 * quat.q2 * quat.q3;
        const double x2q0q1 = 2.0 * quat.q0 * quat.q1;
        const double m12 = x2q1q2 + x2q0q3;
        const double m21 = x2q1q2 - x2q0q3;
        const double m22 = q0_2 - q1_2 + q2_2 - q3_2;
        const double m23 = x2q2q3 + x2q0q1;
        const double m32 = x2q2q3 - x2q0q1;
        const double phi = atan2(m32, m12);
        const double theta = acos(m22);
        const double psi = atan2(m23, -m21);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::YZY);
    }
    EulerAngles quat2EulerAngles_ZYZ(const Quaternion& quat)
    {
        const double q0_2 = quat.q0 * quat.q0;
        const double q1_2 = quat.q1 * quat.q1;
        const double q2_2 = quat.q2 * quat.q2;
        const double q3_2 = quat.q3 * quat.q3;
        const double x2q1q3 = 2.0 * quat.q1 * quat.q3;
        const double x2q0q2 = 2.0 * quat.q0 * quat.q2;
        const double x2q2q3 = 2.0 * quat.q2 * quat.q3;
        const double x2q0q1 = 2.0 * quat.q0 * quat.q1;
        const double m13 = x2q1q3 - x2q0q2;
        const double m23 = x2q2q3 + x2q0q1;
        const double m31 = x2q1q3 + x2q0q2;
        const double m32 = x2q2q3 - x2q0q1;
        const double m33 = q0_2 - q1_2 - q2_2 + q3_2;
        const double phi = atan2(m23, -m13);
        const double theta = acos(m33);
        const double psi = atan2(m32, m31);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::ZYZ);
    }
    EulerAngles quat2EulerAngles_XZX(const Quaternion& quat)
    {
        const double q0_2 = quat.q0 * quat.q0;
        const double q1_2 = quat.q1 * quat.q1;
        const double q2_2 = quat.q2 * quat.q2;
        const double q3_2 = quat.q3 * quat.q3;
        const double x2q1q2 = 2.0 * quat.q1 * quat.q2;
        const double x2q0q3 = 2.0 * quat.q0 * quat.q3;
        const double x2q1q3 = 2.0 * quat.q1 * quat.q3;
        const double x2q0q2 = 2.0 * quat.q0 * quat.q2;
        const double m11 = q0_2 + q1_2 - q2_2 - q3_2;
        const double m12 = x2q1q2 + x2q0q3;
        const double m13 = x2q1q3 - x2q0q2;
        const double m21 = x2q1q2 - x2q0q3;
        const double m31 = x2q1q3 + x2q0q2;
        const double phi = atan2(m31, -m21);
        const double theta = acos(m11);
        const double psi = atan2(m13, m12);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::XZX);
    }
    EulerAngles quat2EulerAngles_YXY(const Quaternion& quat)
    {
        const double q0_2 = quat.q0 * quat.q0;
        const double q1_2 = quat.q1 * quat.q1;
        const double q2_2 = quat.q2 * quat.q2;
        const double q3_2 = quat.q3 * quat.q3;
        const double x2q1q2 = 2.0 * quat.q1 * quat.q2;
        const double x2q0q3 = 2.0 * quat.q0 * quat.q3;
        const double x2q2q3 = 2.0 * quat.q2 * quat.q3;
        const double x2q0q1 = 2.0 * quat.q0 * quat.q1;
        const double m12 = x2q1q2 + x2q0q3;
        const double m21 = x2q1q2 - x2q0q3;
        const double m22 = q0_2 - q1_2 + q2_2 - q3_2;
        const double m23 = x2q2q3 + x2q0q1;
        const double m32 = x2q2q3 - x2q0q1;
        const double phi = atan2(m12, -m32);
        const double theta = acos(m22);
        const double psi = atan2(m21, m23);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::YXY);
    }
    EulerAngles quat2EulerAngles_XYZ(const Quaternion& quat)
    {
        const double q0_2 = quat.q0 * quat.q0;
        const double q1_2 = quat.q1 * quat.q1;
        const double q2_2 = quat.q2 * quat.q2;
        const double q3_2 = quat.q3 * quat.q3;
        const double x2q1q2 = 2.0 * quat.q1 * quat.q2;
        const double x2q0q3 = 2.0 * quat.q0 * quat.q3;
        const double x2q1q3 = 2.0 * quat.q1 * quat.q3;
        const double x2q0q2 = 2.0 * quat.q0 * quat.q2;
        const double x2q2q3 = 2.0 * quat.q2 * quat.q3;
        const double x2q0q1 = 2.0 * quat.q0 * quat.q1;
        const double m11 = q0_2 + q1_2 - q2_2 - q3_2;
        const double m12 = x2q1q2 + x2q0q3;
        const double m13 = x2q1q3 - x2q0q2;
        const double m23 = x2q2q3 + x2q0q1;
        const double m33 = q0_2 - q1_2 - q2_2 + q3_2;
        const double phi = atan2(m23, m33);
        const double theta = -asin(m13);
        const double psi = atan2(m12, m11);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::XYZ);
    }
    EulerAngles quat2EulerAngles_YZX(const Quaternion& quat)
    {
        const double q0_2 = quat.q0 * quat.q0;
        const double q1_2 = quat.q1 * quat.q1;
        const double q2_2 = quat.q2 * quat.q2;
        const double q3_2 = quat.q3 * quat.q3;
        const double x2q1q2 = 2.0 * quat.q1 * quat.q2;
        const double x2q0q3 = 2.0 * quat.q0 * quat.q3;
        const double x2q1q3 = 2.0 * quat.q1 * quat.q3;
        const double x2q0q2 = 2.0 * quat.q0 * quat.q2;
        const double x2q2q3 = 2.0 * quat.q2 * quat.q3;
        const double x2q0q1 = 2.0 * quat.q0 * quat.q1;
        const double m11 = q0_2 + q1_2 - q2_2 - q3_2;
        const double m21 = x2q1q2 - x2q0q3;
        const double m22 = q0_2 - q1_2 + q2_2 - q3_2;
        const double m23 = x2q2q3 + x2q0q1;
        const double m31 = x2q1q3 + x2q0q2;
        const double phi = atan2(m31, m11);
        const double theta = -asin(m21);
        const double psi = atan2(m23, m22);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::YZX);
    }
    EulerAngles quat2EulerAngles_ZXY(const Quaternion& quat)
    {
        const double q0_2 = quat.q0 * quat.q0;
        const double q1_2 = quat.q1 * quat.q1;
        const double q2_2 = quat.q2 * quat.q2;
        const double q3_2 = quat.q3 * quat.q3;
        const double x2q1q2 = 2.0 * quat.q1 * quat.q2;
        const double x2q0q3 = 2.0 * quat.q0 * quat.q3;
        const double x2q1q3 = 2.0 * quat.q1 * quat.q3;
        const double x2q0q2 = 2.0 * quat.q0 * quat.q2;
        const double x2q2q3 = 2.0 * quat.q2 * quat.q3;
        const double x2q0q1 = 2.0 * quat.q0 * quat.q1;
        const double m12 = x2q1q2 + x2q0q3;
        const double m22 = q0_2 - q1_2 + q2_2 - q3_2;
        const double m31 = x2q1q3 + x2q0q2;
        const double m32 = x2q2q3 - x2q0q1;
        const double m33 = q0_2 - q1_2 - q2_2 + q3_2;
        const double phi = atan2(m12, m22);
        const double theta = -asin(m32);
        const double psi = atan2(m31, m33);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::ZXY);
    }
    EulerAngles quat2EulerAngles_XZY(const Quaternion& quat)
    {
        const double q0_2 = quat.q0 * quat.q0;
        const double q1_2 = quat.q1 * quat.q1;
        const double q2_2 = quat.q2 * quat.q2;
        const double q3_2 = quat.q3 * quat.q3;
        const double x2q1q2 = 2.0 * quat.q1 * quat.q2;
        const double x2q0q3 = 2.0 * quat.q0 * quat.q3;
        const double x2q1q3 = 2.0 * quat.q1 * quat.q3;
        const double x2q0q2 = 2.0 * quat.q0 * quat.q2;
        const double x2q2q3 = 2.0 * quat.q2 * quat.q3;
        const double x2q0q1 = 2.0 * quat.q0 * quat.q1;
        const double m11 = q0_2 + q1_2 - q2_2 - q3_2;
        const double m12 = x2q1q2 + x2q0q3;
        const double m13 = x2q1q3 - x2q0q2;
        const double m22 = q0_2 - q1_2 + q2_2 - q3_2;
        const double m32 = x2q2q3 - x2q0q1;
        const double phi = atan2(-m32, m22);
        const double theta = asin(m12);
        const double psi = atan2(-m13, m11);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::XZY);
    }
    EulerAngles quat2EulerAngles_ZYX(const Quaternion& quat)
    {
        const double q0_2 = quat.q0 * quat.q0;
        const double q1_2 = quat.q1 * quat.q1;
        const double q2_2 = quat.q2 * quat.q2;
        const double q3_2 = quat.q3 * quat.q3;
        const double x2q1q2 = 2.0 * quat.q1 * quat.q2;
        const double x2q0q3 = 2.0 * quat.q0 * quat.q3;
        const double x2q1q3 = 2.0 * quat.q1 * quat.q3;
        const double x2q0q2 = 2.0 * quat.q0 * quat.q2;
        const double x2q2q3 = 2.0 * quat.q2 * quat.q3;
        const double x2q0q1 = 2.0 * quat.q0 * quat.q1;
        const double m11 = q0_2 + q1_2 - q2_2 - q3_2;
        const double m21 = x2q1q2 - x2q0q3;
        const double m31 = x2q1q3 + x2q0q2;
        const double m32 = x2q2q3 - x2q0q1;
        const double m33 = q0_2 - q1_2 - q2_2 + q3_2;
        const double phi = atan2(-m21, m11);
        const double theta = asin(m31);
        const double psi = atan2(-m32, m33);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::ZYX);
    }
    EulerAngles quat2EulerAngles_YXZ(const Quaternion& quat)
    {
        const double q0_2 = quat.q0 * quat.q0;
        const double q1_2 = quat.q1 * quat.q1;
        const double q2_2 = quat.q2 * quat.q2;
        const double q3_2 = quat.q3 * quat.q3;
        const double x2q1q2 = 2.0 * quat.q1 * quat.q2;
        const double x2q0q3 = 2.0 * quat.q0 * quat.q3;
        const double x2q1q3 = 2.0 * quat.q1 * quat.q3;
        const double x2q0q2 = 2.0 * quat.q0 * quat.q2;
        const double x2q2q3 = 2.0 * quat.q2 * quat.q3;
        const double x2q0q1 = 2.0 * quat.q0 * quat.q1;
        const double m13 = x2q1q3 - x2q0q2;
        const double m21 = x2q1q2 - x2q0q3;
        const double m22 = q0_2 - q1_2 + q2_2 - q3_2;
        const double m23 = x2q2q3 + x2q0q1;
        const double m33 = q0_2 - q1_2 - q2_2 + q3_2;
        const double phi = atan2(-m13, m33);
        const double theta = asin(m23);
        const double psi = atan2(-m21, m22);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::YXZ);
    }

    // Quaternion Kinematic Functions
    Quaternion integrateQuat(const Quaternion& quat, const Quaternion& quatRates, double dt)
    {
        Quaternion quatNew = quat + quatRates * dt; // First Order Euler Integration
        normalise(quatNew);                         // Normalisation
        return quatNew;
    }

    Quaternion quatKinematicRates_BodyRates(const Quaternion& quat, const Vector3& bodyRates)
    {
        const double p = bodyRates.x;
        const double q = bodyRates.y;
        const double r = bodyRates.z;
        const double q0 = 0.5 * (-quat.q1 * p - quat.q2 * q - quat.q3 * r);
        const double q1 = 0.5 * (quat.q0 * p + quat.q3 * q - quat.q2 * r);
        const double q2 = 0.5 * (-quat.q3 * p + quat.q0 * q + quat.q1 * r);
        const double q3 = 0.5 * (quat.q2 * p - quat.q1 * q + quat.q0 * r);
        return Quaternion(q0, q1, q2, q3);
    }
    Quaternion quatKinematicRates_WorldRates(const Quaternion& quat, const Vector3& worldRates)
    {
        const double p = worldRates.x;
        const double q = worldRates.y;
        const double r = worldRates.z;
        const double q0 = 0.5 * (-quat.q1 * p - quat.q2 * q - quat.q3 * r);
        const double q1 = 0.5 * (quat.q0 * p - quat.q3 * q + quat.q2 * r);
        const double q2 = 0.5 * (quat.q3 * p + quat.q0 * q - quat.q1 * r);
        const double q3 = 0.5 * (-quat.q2 * p + quat.q1 * q + quat.q0 * r);
        return Quaternion(q0, q1, q2, q3);
    }

    // Quaternion Interpolation Functions
    Quaternion linearInterpolate(const Quaternion& startQuat, const Quaternion& endQuat, double t)
    {
        Quaternion q0 = unit(startQuat);
        Quaternion q1 = unit(endQuat);

        if (t < 0.0)
        {
            return q0;
        }
        if (t > 1.0)
        {
            return q1;
        }

        double a = (1.0 - t);
        double b = t;
        Quaternion qi = unit(a * q0 + b * q1);
        return qi;
    }
    Quaternion slerpInterpolate(const Quaternion& startQuat, const Quaternion& endQuat, double t)
    {
        Quaternion q0 = unit(startQuat);
        Quaternion q1 = unit(endQuat);

        if (t < 0.0)
        {
            return q0;
        }
        if (t > 1.0)
        {
            return q1;
        }

        double quatDot = dot(q0, q1);

        // Check for Negative Dot Product
        if (quatDot < 0)
        {
            q1 = -q1;
            quatDot = -quatDot;
        }

        double theta = acos(quatDot);

        // Check for Small Angles
        if (theta < 0.0001)
        {
            return linearInterpolate(startQuat, endQuat, t);
        }

        // SLERP
        double a = sin((1.0 - t) * theta) / sin(theta);
        double b = sin(t * theta) / sin(theta);
        Quaternion qt = unit(a * q0 + b * q1);
        return qt;
    }
}; // namespace AML