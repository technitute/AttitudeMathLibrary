// ######################################################################### //
// Attitude Math Library
// Copyright (c) 2021 Dr. Steven Dumble Phd
// Distributed under the MIT Software License (See accompanying file LICENSE)
// ######################################################################### //

#include "AMLDCM.hpp"
#include "AMLEulerAngles.hpp"
#include <cmath>

namespace AML
{
    EulerAngles::EulerAngles() : phi(0.0), theta(0.0), psi(0.0), seq_(EulerSequence::XYZ) {}

    EulerAngles::EulerAngles(double phi_, double theta_, double psi_, EulerSequence seq)
        : phi(phi_), theta(theta_), psi(psi_), seq_(seq)
    {
    }

    // Steam Functions
    std::ostream& operator<<(std::ostream& os, const EulerAngles& obj)
    {
        os << "EULER [" << obj.phi << ", " << obj.theta << ", " << obj.psi << "]";
        return os;
    }

    // Euler Angle Conversions
    Matrix33 eulerAngles2DCM(const EulerAngles& angles)
    {
        switch (angles.getEulerSequence())
        {
        case EulerAngles::EulerSequence::XYZ:
            return eulerAngles2DCM_XYZ(angles.phi, angles.theta, angles.psi);
        case EulerAngles::EulerSequence::ZXZ:
            return eulerAngles2DCM_ZXZ(angles.phi, angles.theta, angles.psi);
        case EulerAngles::EulerSequence::XYX:
            return eulerAngles2DCM_XYX(angles.phi, angles.theta, angles.psi);
        case EulerAngles::EulerSequence::YZY:
            return eulerAngles2DCM_YZY(angles.phi, angles.theta, angles.psi);
        case EulerAngles::EulerSequence::ZYZ:
            return eulerAngles2DCM_ZYZ(angles.phi, angles.theta, angles.psi);
        case EulerAngles::EulerSequence::XZX:
            return eulerAngles2DCM_XZX(angles.phi, angles.theta, angles.psi);
        case EulerAngles::EulerSequence::YXY:
            return eulerAngles2DCM_YXY(angles.phi, angles.theta, angles.psi);
        case EulerAngles::EulerSequence::YZX:
            return eulerAngles2DCM_YZX(angles.phi, angles.theta, angles.psi);
        case EulerAngles::EulerSequence::ZXY:
            return eulerAngles2DCM_ZXY(angles.phi, angles.theta, angles.psi);
        case EulerAngles::EulerSequence::XZY:
            return eulerAngles2DCM_XZY(angles.phi, angles.theta, angles.psi);
        case EulerAngles::EulerSequence::ZYX:
            return eulerAngles2DCM_ZYX(angles.phi, angles.theta, angles.psi);
        case EulerAngles::EulerSequence::YXZ:
            return eulerAngles2DCM_YXZ(angles.phi, angles.theta, angles.psi);
        }
        return Matrix33::identity();
    }

    EulerAngles dcm2EulerAngles(const Matrix33& dcm, const EulerAngles::EulerSequence seq)
    {
        const double TOL = 0.0001;

        if (isValidDCM(dcm, TOL))
        {
            switch (seq)
            {
            case EulerAngles::EulerSequence::XYZ:
                return dcm2EulerAngles_XYZ(dcm);
            case EulerAngles::EulerSequence::ZXZ:
                return dcm2EulerAngles_ZXZ(dcm);
            case EulerAngles::EulerSequence::XYX:
                return dcm2EulerAngles_XYX(dcm);
            case EulerAngles::EulerSequence::YZY:
                return dcm2EulerAngles_YZY(dcm);
            case EulerAngles::EulerSequence::ZYZ:
                return dcm2EulerAngles_ZYZ(dcm);
            case EulerAngles::EulerSequence::XZX:
                return dcm2EulerAngles_XZX(dcm);
            case EulerAngles::EulerSequence::YXY:
                return dcm2EulerAngles_YXY(dcm);
            case EulerAngles::EulerSequence::YZX:
                return dcm2EulerAngles_YZX(dcm);
            case EulerAngles::EulerSequence::ZXY:
                return dcm2EulerAngles_ZXY(dcm);
            case EulerAngles::EulerSequence::XZY:
                return dcm2EulerAngles_XZY(dcm);
            case EulerAngles::EulerSequence::ZYX:
                return dcm2EulerAngles_ZYX(dcm);
            case EulerAngles::EulerSequence::YXZ:
                return dcm2EulerAngles_YXZ(dcm);
            }
        }
        return EulerAngles();
    }

    EulerAngles convertEulerAngleSequence(const EulerAngles& angles, const EulerAngles::EulerSequence seq)
    {
        if (angles.getEulerSequence() == seq)
        {
            return angles;
        }
        else if (angles.getEulerSequence() == EulerAngles::EulerSequence::XYZ && seq == EulerAngles::EulerSequence::ZXZ)
        {
            return converEulerAnglesXYZtoZXZ(angles.phi, angles.theta, angles.psi);
        }
        else if (angles.getEulerSequence() == EulerAngles::EulerSequence::ZXZ && seq == EulerAngles::EulerSequence::XYZ)
        {
            return converEulerAnglesZXZtoXYZ(angles.phi, angles.theta, angles.psi);
        }
        else // General Case
        {
            Matrix33 dcm = eulerAngles2DCM(angles);
            return dcm2EulerAngles(dcm, seq);
        }
    }

    // Euler Angle to DCM Conversions
    Matrix33 eulerAngles2DCM_XYZ(double phi, double theta, double psi)
    {
        const double cosPhi = cos(phi);
        const double sinPhi = sin(phi);
        const double cosTheta = cos(theta);
        const double sinTheta = sin(theta);
        const double cosPsi = cos(psi);
        const double sinPsi = sin(psi);

        double data[3][3];
        data[0][0] = cosPsi * cosTheta;
        data[0][1] = cosTheta * sinPsi;
        data[0][2] = -sinTheta;
        data[1][0] = cosPsi * sinTheta * sinPhi - sinPsi * cosPhi;
        data[1][1] = sinPsi * sinTheta * sinPhi + cosPsi * cosPhi;
        data[1][2] = sinPhi * cosTheta;
        data[2][0] = cosPsi * sinTheta * cosPhi + sinPsi * sinPhi;
        data[2][1] = sinPsi * sinTheta * cosPhi - cosPsi * sinPhi;
        data[2][2] = cosTheta * cosPhi;
        return Matrix33(data);
    }
    Matrix33 eulerAngles2DCM_ZXZ(double phi, double theta, double psi)
    {
        const double cosPhi = cos(phi);
        const double sinPhi = sin(phi);
        const double cosTheta = cos(theta);
        const double sinTheta = sin(theta);
        const double cosPsi = cos(psi);
        const double sinPsi = sin(psi);

        double data[3][3];
        data[0][0] = cosPhi * cosPsi - sinPhi * cosTheta * sinPsi;
        data[0][1] = cosPhi * sinPsi + sinPhi * cosTheta * cosPsi;
        data[0][2] = sinPhi * sinTheta;
        data[1][0] = -sinPhi * cosPsi - cosPhi * cosTheta * sinPsi;
        data[1][1] = -sinPhi * sinPsi + cosPhi * cosTheta * cosPsi;
        data[1][2] = cosPhi * sinTheta;
        data[2][0] = sinTheta * sinPsi;
        data[2][1] = -sinTheta * cosPsi;
        data[2][2] = cosTheta;
        return Matrix33(data);
    }
    Matrix33 eulerAngles2DCM_XYX(double phi, double theta, double psi)
    {
        return (DCM::rotationX(phi) * DCM::rotationY(theta) * DCM::rotationX(psi));
    }
    Matrix33 eulerAngles2DCM_YZY(double phi, double theta, double psi)
    {
        return (DCM::rotationY(phi) * DCM::rotationZ(theta) * DCM::rotationY(psi));
    }
    Matrix33 eulerAngles2DCM_ZYZ(double phi, double theta, double psi)
    {
        return (DCM::rotationZ(phi) * DCM::rotationY(theta) * DCM::rotationZ(psi));
    }
    Matrix33 eulerAngles2DCM_XZX(double phi, double theta, double psi)
    {
        return (DCM::rotationX(phi) * DCM::rotationZ(theta) * DCM::rotationX(psi));
    }
    Matrix33 eulerAngles2DCM_YXY(double phi, double theta, double psi)
    {
        return (DCM::rotationY(phi) * DCM::rotationX(theta) * DCM::rotationY(psi));
    }
    Matrix33 eulerAngles2DCM_YZX(double phi, double theta, double psi)
    {
        return (DCM::rotationY(phi) * DCM::rotationZ(theta) * DCM::rotationX(psi));
    }
    Matrix33 eulerAngles2DCM_ZXY(double phi, double theta, double psi)
    {
        return (DCM::rotationZ(phi) * DCM::rotationX(theta) * DCM::rotationY(psi));
    }
    Matrix33 eulerAngles2DCM_XZY(double phi, double theta, double psi)
    {
        return (DCM::rotationX(phi) * DCM::rotationZ(theta) * DCM::rotationY(psi));
    }
    Matrix33 eulerAngles2DCM_ZYX(double phi, double theta, double psi)
    {
        return (DCM::rotationZ(phi) * DCM::rotationY(theta) * DCM::rotationX(psi));
    }
    Matrix33 eulerAngles2DCM_YXZ(double phi, double theta, double psi)
    {
        return (DCM::rotationY(phi) * DCM::rotationX(theta) * DCM::rotationZ(psi));
    }

    // DCM to Euler Angle Conversions
    EulerAngles dcm2EulerAngles_XYZ(const Matrix33& dcm)
    {
        double phi = atan2(dcm.m23, dcm.m33);
        double theta = -asin(dcm.m13);
        double psi = atan2(dcm.m12, dcm.m11);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::XYZ);
    }
    EulerAngles dcm2EulerAngles_ZXZ(const Matrix33& dcm)
    {
        double phi = atan2(dcm.m13, dcm.m23);
        double theta = acos(dcm.m33);
        double psi = atan2(dcm.m31, -dcm.m32);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::ZXZ);
    }
    EulerAngles dcm2EulerAngles_XYX(const Matrix33& dcm)
    {
        double phi = atan2(dcm.m21, dcm.m31);
        double theta = acos(dcm.m11);
        double psi = atan2(dcm.m12, -dcm.m13);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::XYX);
    }
    EulerAngles dcm2EulerAngles_YZY(const Matrix33& dcm)
    {
        double phi = atan2(dcm.m32, dcm.m12);
        double theta = acos(dcm.m22);
        double psi = atan2(dcm.m23, -dcm.m21);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::YZY);
    }
    EulerAngles dcm2EulerAngles_ZYZ(const Matrix33& dcm)
    {
        double phi = atan2(dcm.m23, -dcm.m13);
        double theta = acos(dcm.m33);
        double psi = atan2(dcm.m32, dcm.m31);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::ZYZ);
    }
    EulerAngles dcm2EulerAngles_XZX(const Matrix33& dcm)
    {
        double phi = atan2(dcm.m31, -dcm.m21);
        double theta = acos(dcm.m11);
        double psi = atan2(dcm.m13, dcm.m12);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::XZX);
    }
    EulerAngles dcm2EulerAngles_YXY(const Matrix33& dcm)
    {
        double phi = atan2(dcm.m12, -dcm.m32);
        double theta = acos(dcm.m22);
        double psi = atan2(dcm.m21, dcm.m23);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::YXY);
    }
    EulerAngles dcm2EulerAngles_YZX(const Matrix33& dcm)
    {
        double phi = atan2(dcm.m31, dcm.m11);
        double theta = -asin(dcm.m21);
        double psi = atan2(dcm.m23, dcm.m22);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::YZX);
    }
    EulerAngles dcm2EulerAngles_ZXY(const Matrix33& dcm)
    {
        double phi = atan2(dcm.m12, dcm.m22);
        double theta = -asin(dcm.m32);
        double psi = atan2(dcm.m31, dcm.m33);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::ZXY);
    }
    EulerAngles dcm2EulerAngles_XZY(const Matrix33& dcm)
    {
        double phi = atan2(-dcm.m32, dcm.m22);
        double theta = asin(dcm.m12);
        double psi = atan2(-dcm.m13, dcm.m11);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::XZY);
    }
    EulerAngles dcm2EulerAngles_ZYX(const Matrix33& dcm)
    {
        double phi = atan2(-dcm.m21, dcm.m11);
        double theta = asin(dcm.m31);
        double psi = atan2(-dcm.m32, dcm.m33);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::ZYX);
    }
    EulerAngles dcm2EulerAngles_YXZ(const Matrix33& dcm)
    {
        double phi = atan2(-dcm.m13, dcm.m33);
        double theta = asin(dcm.m23);
        double psi = atan2(-dcm.m21, dcm.m22);
        return EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::YXZ);
    }

    // Euler Angle Sequence Conversions
    EulerAngles converEulerAnglesXYZtoZXZ(double phi, double theta, double psi)
    {
        const double cosPhi = cos(phi);
        const double sinPhi = sin(phi);
        const double cosTheta = cos(theta);
        const double sinTheta = sin(theta);
        const double cosPsi = cos(psi);
        const double sinPsi = sin(psi);
        const double phiZXZ = atan2(-sinTheta, sinPhi * cosTheta);
        const double thetaZXZ = acos(cosPhi * cosTheta);
        const double psiZXZ =
            atan2(cosPhi * sinTheta * cosPsi + sinPhi * sinPsi, -cosPhi * sinTheta * sinPsi + sinPhi * cosPsi);
        return EulerAngles(phiZXZ, thetaZXZ, psiZXZ, EulerAngles::EulerSequence::ZXZ);
    }
    EulerAngles converEulerAnglesZXZtoXYZ(double phi, double theta, double psi)
    {
        const double cosPhi = cos(phi);
        const double sinPhi = sin(phi);
        const double cosTheta = cos(theta);
        const double sinTheta = sin(theta);
        const double cosPsi = cos(psi);
        const double sinPsi = sin(psi);
        const double phiXYZ = atan2(cosPhi * sinTheta, cosTheta);
        const double thetaXYZ = -asin(sinPhi * sinTheta);
        const double psiXYZ =
            atan2(cosPhi * sinPsi + sinPhi * cosTheta * cosPsi, cosPhi * cosPsi - sinPhi * cosTheta * sinPsi);
        return EulerAngles(phiXYZ, thetaXYZ, psiXYZ, EulerAngles::EulerSequence::XYZ);
    }

    // Euler Angle Rates
    EulerAngles integrateEulerAngles(const EulerAngles& angles, const EulerAngles& angleRates, double dt)
    {
        EulerAngles::EulerSequence seq = angles.getEulerSequence();
        if (seq == angleRates.getEulerSequence())
        {
            double phiNew = angles.phi + angleRates.phi * dt;
            double thetaNew = angles.theta + angleRates.theta * dt;
            double psiNew = angles.psi + angleRates.psi * dt;
            return EulerAngles(phiNew, thetaNew, psiNew, seq);
        }
        return angles;
    }

    EulerAngles eulerAngleKinematicRates(const EulerAngles& angles, const Vector3& bodyRates)
    {
        Vector3 eulerRates;
        EulerAngles::EulerSequence seq = angles.getEulerSequence();
        switch (seq)
        {
        case EulerAngles::EulerSequence::XYZ:
            eulerRates = eulerAngleRatesMatrix_XYZ(angles.phi, angles.theta, angles.psi) * bodyRates;
            break;
        case EulerAngles::EulerSequence::ZXZ:
            eulerRates = eulerAngleRatesMatrix_ZXZ(angles.phi, angles.theta, angles.psi) * bodyRates;
            break;
        case EulerAngles::EulerSequence::YZY:
            eulerRates = eulerAngleRatesMatrix_YZY(angles.phi, angles.theta, angles.psi) * bodyRates;
            break;
        case EulerAngles::EulerSequence::XYX:
            eulerRates = eulerAngleRatesMatrix_XYX(angles.phi, angles.theta, angles.psi) * bodyRates;
            break;
        case EulerAngles::EulerSequence::ZYZ:
            eulerRates = eulerAngleRatesMatrix_ZYZ(angles.phi, angles.theta, angles.psi) * bodyRates;
            break;
        case EulerAngles::EulerSequence::XZX:
            eulerRates = eulerAngleRatesMatrix_XZX(angles.phi, angles.theta, angles.psi) * bodyRates;
            break;
        case EulerAngles::EulerSequence::YXY:
            eulerRates = eulerAngleRatesMatrix_YXY(angles.phi, angles.theta, angles.psi) * bodyRates;
            break;
        case EulerAngles::EulerSequence::YZX:
            eulerRates = eulerAngleRatesMatrix_YZX(angles.phi, angles.theta, angles.psi) * bodyRates;
            break;
        case EulerAngles::EulerSequence::ZXY:
            eulerRates = eulerAngleRatesMatrix_ZXY(angles.phi, angles.theta, angles.psi) * bodyRates;
            break;
        case EulerAngles::EulerSequence::XZY:
            eulerRates = eulerAngleRatesMatrix_XZY(angles.phi, angles.theta, angles.psi) * bodyRates;
            break;
        case EulerAngles::EulerSequence::ZYX:
            eulerRates = eulerAngleRatesMatrix_ZYX(angles.phi, angles.theta, angles.psi) * bodyRates;
            break;
        case EulerAngles::EulerSequence::YXZ:
            eulerRates = eulerAngleRatesMatrix_YXZ(angles.phi, angles.theta, angles.psi) * bodyRates;
            break;
        }
        return EulerAngles(eulerRates.x, eulerRates.y, eulerRates.z, seq);
    }

    Matrix33 eulerAngleRatesMatrix_XYZ(double phi, double theta, double psi)
    {
        const double cosPhi = cos(phi);
        const double sinPhi = sin(phi);
        const double cosTheta = cos(theta);
        const double tanTheta = tan(theta);
        const double secTheta = 1.0 / cosTheta;

        double data[3][3] = { { 1.0, sinPhi * tanTheta, cosPhi * tanTheta },
                              { 0.0, cosPhi, -sinPhi },
                              { 0.0, sinPhi * secTheta, cosPhi * secTheta } };

        return Matrix33(data);
    }
    Matrix33 eulerAngleRatesMatrix_ZXZ(double phi, double theta, double psi)
    {
        const double cosPhi = cos(phi);
        const double sinPhi = sin(phi);
        const double cosTheta = cos(theta);
        const double sinTheta = sin(theta);
        const double cscTheta = 1.0 / sinTheta;

        double data[3][3] = { { -sinPhi * cosTheta * cscTheta, -cosPhi * cosTheta * cscTheta, sinTheta * cscTheta },
                              { cosPhi * sinTheta * cscTheta, -sinPhi * sinTheta * cscTheta, 0.0 },
                              { sinPhi * cscTheta, cosPhi * cscTheta, 0.0 } };

        return Matrix33(data);
    }
    Matrix33 eulerAngleRatesMatrix_YZY(double phi, double theta, double psi)
    {
        const double cosPhi = cos(phi);
        const double sinPhi = sin(phi);
        const double cosTheta = cos(theta);
        const double sinTheta = sin(theta);
        const double cscTheta = 1.0 / sinTheta;

        double data[3][3] = { { -cosPhi * cosTheta * cscTheta, sinTheta * cscTheta, -sinPhi * cosTheta * cscTheta },
                              { -sinPhi * sinTheta * cscTheta, 0.0, cosPhi * sinTheta * cscTheta },
                              { cosPhi * cscTheta, 0.0, sinPhi * cscTheta } };

        return Matrix33(data);
    }
    Matrix33 eulerAngleRatesMatrix_ZYZ(double phi, double theta, double psi)
    {
        const double cosPhi = cos(phi);
        const double sinPhi = sin(phi);
        const double cosTheta = cos(theta);
        const double sinTheta = sin(theta);
        const double cscTheta = 1.0 / sinTheta;

        double data[3][3] = { { cosPhi * cosTheta * cscTheta, -sinPhi * cosTheta * cscTheta, sinTheta },
                              { sinPhi * sinTheta * cscTheta, cosPhi * sinTheta * cscTheta, 0.0 },
                              { -cosPhi * cscTheta, sinPhi * cscTheta, 0.0 } };

        return Matrix33(data);
    }
    Matrix33 eulerAngleRatesMatrix_XYX(double phi, double theta, double psi)
    {
        const double cosPhi = cos(phi);
        const double sinPhi = sin(phi);
        const double cosTheta = cos(theta);
        const double sinTheta = sin(theta);
        const double cscTheta = 1.0 / sinTheta;

        double data[3][3] = { { sinTheta * cscTheta, -sinPhi * cosTheta * cscTheta, -cosPhi * cosTheta * cscTheta },
                              { 0.0, cosPhi * sinTheta * cscTheta, -sinPhi * sinTheta * cscTheta },
                              { 0.0, sinPhi * cscTheta, cosPhi * cscTheta } };

        return Matrix33(data);
    }
    Matrix33 eulerAngleRatesMatrix_XZX(double phi, double theta, double psi)
    {
        const double cosPhi = cos(phi);
        const double sinPhi = sin(phi);
        const double cosTheta = cos(theta);
        const double sinTheta = sin(theta);
        const double cscTheta = 1.0 / sinTheta;

        double data[3][3] = { { sinTheta * cscTheta, cosPhi * cosTheta * cscTheta, -sinPhi * cosTheta * cscTheta },
                              { 0.0, sinPhi * sinTheta * cscTheta, cosPhi * sinTheta * cscTheta },
                              { 0.0, -cosPhi * cscTheta, sinPhi * cscTheta } };

        return Matrix33(data);
    }
    Matrix33 eulerAngleRatesMatrix_YXY(double phi, double theta, double psi)
    {
        const double cosPhi = cos(phi);
        const double sinPhi = sin(phi);
        const double cosTheta = cos(theta);
        const double sinTheta = sin(theta);
        const double cscTheta = 1.0 / sinTheta;

        double data[3][3] = { { -sinPhi * cosTheta * cscTheta, sinTheta * cscTheta, cosPhi * cosTheta * cscTheta },
                              { sinTheta * cosPhi * cscTheta, 0.0, sinTheta * sinPhi * cscTheta },
                              { sinPhi * cscTheta, 0.0, -cosPhi * cscTheta } };

        return Matrix33(data);
    }
    Matrix33 eulerAngleRatesMatrix_YZX(double phi, double theta, double psi)
    {
        const double cosPhi = cos(phi);
        const double sinPhi = sin(phi);
        const double cosTheta = cos(theta);
        const double sinTheta = sin(theta);
        const double secTheta = 1.0 / cosTheta;

        double data[3][3] = { { cosPhi * sinTheta * secTheta, cosTheta * secTheta, sinPhi * sinTheta * secTheta },
                              { -sinPhi * cosTheta * secTheta, 0.0, cosPhi * cosTheta * secTheta },
                              { cosPhi * secTheta, 0.0, sinPhi * secTheta } };

        return Matrix33(data);
    }
    Matrix33 eulerAngleRatesMatrix_ZXY(double phi, double theta, double psi)
    {
        const double cosPhi = cos(phi);
        const double sinPhi = sin(phi);
        const double cosTheta = cos(theta);
        const double sinTheta = sin(theta);
        const double secTheta = 1.0 / cosTheta;

        double data[3][3] = { { sinPhi * sinTheta * secTheta, cosPhi * sinTheta * secTheta, cosTheta * secTheta },
                              { cosTheta * cosPhi * secTheta, -sinPhi * cosTheta * secTheta, 0.0 },
                              { sinPhi * secTheta, cosPhi * secTheta, 0.0 } };

        return Matrix33(data);
    }
    Matrix33 eulerAngleRatesMatrix_XZY(double phi, double theta, double psi)
    {
        const double cosPhi = cos(phi);
        const double sinPhi = sin(phi);
        const double cosTheta = cos(theta);
        const double sinTheta = sin(theta);
        const double secTheta = 1.0 / cosTheta;

        double data[3][3] = { { cosTheta * secTheta, -cosPhi * sinTheta * secTheta, sinPhi * sinTheta * secTheta },
                              { 0.0, sinPhi * cosTheta * secTheta, cosPhi * cosTheta * secTheta },
                              { 0.0, cosPhi * secTheta, -sinPhi * secTheta } };

        return Matrix33(data);
    }
    Matrix33 eulerAngleRatesMatrix_ZYX(double phi, double theta, double psi)
    {
        const double cosPhi = cos(phi);
        const double sinPhi = sin(phi);
        const double cosTheta = cos(theta);
        const double sinTheta = sin(theta);
        const double secTheta = 1.0 / cosTheta;

        double data[3][3] = { { -cosPhi * sinTheta * secTheta, sinPhi * sinTheta * secTheta, cosTheta * secTheta },
                              { sinPhi * cosTheta * secTheta, cosPhi * cosTheta * secTheta, 0.0 },
                              { cosPhi * secTheta, -sinPhi * secTheta, 0.0 } };

        return Matrix33(data);
    }
    Matrix33 eulerAngleRatesMatrix_YXZ(double phi, double theta, double psi)
    {
        const double cosPhi = cos(phi);
        const double sinPhi = sin(phi);
        const double cosTheta = cos(theta);
        const double sinTheta = sin(theta);
        const double secTheta = 1.0 / cosTheta;

        double data[3][3] = { { sinPhi * sinTheta * secTheta, cosTheta * secTheta, -cosPhi * sinTheta * secTheta },
                              { cosPhi * cosTheta * secTheta, 0.0, sinPhi * cosTheta * secTheta },
                              { -sinPhi * secTheta, 0.0, cosPhi * secTheta } };

        return Matrix33(data);
    }

    EulerAngles linearInterpolate(const EulerAngles& startAngles, const EulerAngles& endAngles, double t)
    {
        if (startAngles.getEulerSequence() == endAngles.getEulerSequence())
        {
            if (t < 0.0)
            {
                return startAngles;
            }
            if (t > 1.0)
            {
                return endAngles;
            }
            double phiNew = (1 - t) * startAngles.phi + t * endAngles.phi;
            double thetaNew = (1 - t) * startAngles.theta + t * endAngles.theta;
            double psiNew = (1 - t) * startAngles.psi + t * endAngles.psi;
            return EulerAngles(phiNew, thetaNew, psiNew, startAngles.getEulerSequence());
        }
        return EulerAngles();
    }

    EulerAngles smoothInterpolate(const EulerAngles& startAngles, const EulerAngles& endAngles, double t)
    {
        if (startAngles.getEulerSequence() == endAngles.getEulerSequence())
        {
            if (t < 0.0)
            {
                return startAngles;
            }
            if (t > 1.0)
            {
                return endAngles;
            }

            double t2 = t * t;
            double t3 = t2 * t;
            double t4 = t3 * t;
            double t5 = t4 * t;

            double deltaPhi = endAngles.phi - startAngles.phi;
            double deltaTheta = endAngles.theta - startAngles.theta;
            double deltaPsi = endAngles.psi - startAngles.psi;

            double phiNew = 6 * deltaPhi * t5 + -15 * deltaPhi * t4 + 10 * deltaPhi * t3 + startAngles.phi;
            double thetaNew = 6 * deltaTheta * t5 + -15 * deltaTheta * t4 + 10 * deltaTheta * t3 + startAngles.theta;
            double psiNew = 6 * deltaPsi * t5 + -15 * deltaPsi * t4 + 10 * deltaPsi * t3 + startAngles.psi;

            return EulerAngles(phiNew, thetaNew, psiNew, startAngles.getEulerSequence());
        }
        return EulerAngles();
    }
}; // namespace AML
