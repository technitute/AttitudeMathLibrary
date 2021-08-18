// ######################################################################### //
// Attitude Math Library
// Copyright (c) 2021 Dr. Steven Dumble Phd
// Distributed under the MIT Software License (See accompanying file LICENSE)
// ######################################################################### //

#include "AML.hpp"
#include "catch/catch.hpp"
#include <vector>

using namespace AML;

namespace
{
    void checkIsApprox(const Matrix33& a, const Matrix33& b)
    {
        CHECK(a.m11 == Approx(b.m11));
        CHECK(a.m12 == Approx(b.m12));
        CHECK(a.m13 == Approx(b.m13));
        CHECK(a.m21 == Approx(b.m21));
        CHECK(a.m22 == Approx(b.m22));
        CHECK(a.m23 == Approx(b.m23));
        CHECK(a.m31 == Approx(b.m31));
        CHECK(a.m32 == Approx(b.m32));
        CHECK(a.m33 == Approx(b.m33));
    }
}; // namespace

TEST_CASE("Construction", "[EulerAngles]")
{
    // Default
    EulerAngles angles;
    CHECK(angles.phi == 0.0);
    CHECK(angles.theta == 0.0);
    CHECK(angles.psi == 0.0);
    CHECK(angles.getEulerSequence() == EulerAngles::EulerSequence::XYZ);

    // Value
    double phi = 0.2;
    double theta = -0.8;
    double psi = 1.2;
    auto seq = EulerAngles::EulerSequence::ZXZ;
    angles = EulerAngles(phi, theta, psi, seq);
    CHECK(angles.phi == phi);
    CHECK(angles.theta == theta);
    CHECK(angles.psi == psi);
    CHECK(angles.getEulerSequence() == seq);
}

TEST_CASE("eulerAngles2DCM", "[EulerAngles]")
{
    double phi = 0.2;
    double theta = -0.8;
    double psi = 1.2;
    Matrix33 dcmTest;

    auto dcmConstruct = [phi, theta, psi](int a1, int a2, int a3) {
        Matrix33 dcm = Matrix33::identity();
        // Axis 3
        if (1 == a3)
            dcm = DCM::rotationX(psi) * dcm;
        if (2 == a3)
            dcm = DCM::rotationY(psi) * dcm;
        if (3 == a3)
            dcm = DCM::rotationZ(psi) * dcm;
        // Axis 2
        if (1 == a2)
            dcm = DCM::rotationX(theta) * dcm;
        if (2 == a2)
            dcm = DCM::rotationY(theta) * dcm;
        if (3 == a2)
            dcm = DCM::rotationZ(theta) * dcm;
        // Axis 1
        if (1 == a1)
            dcm = DCM::rotationX(phi) * dcm;
        if (2 == a1)
            dcm = DCM::rotationY(phi) * dcm;
        if (3 == a1)
            dcm = DCM::rotationZ(phi) * dcm;
        return dcm;
    };

    // XYZ
    dcmTest = eulerAngles2DCM(EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::XYZ));
    checkIsApprox(dcmTest, dcmConstruct(1, 2, 3));
    // ZXZ
    dcmTest = eulerAngles2DCM(EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::ZXZ));
    checkIsApprox(dcmTest, dcmConstruct(3, 1, 3));
    // XYX
    dcmTest = eulerAngles2DCM(EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::XYX));
    checkIsApprox(dcmTest, dcmConstruct(1, 2, 1));
    // YZY
    dcmTest = eulerAngles2DCM(EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::YZY));
    checkIsApprox(dcmTest, dcmConstruct(2, 3, 2));
    // ZYZ
    dcmTest = eulerAngles2DCM(EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::ZYZ));
    checkIsApprox(dcmTest, dcmConstruct(3, 2, 3));
    // XZX
    dcmTest = eulerAngles2DCM(EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::XZX));
    checkIsApprox(dcmTest, dcmConstruct(1, 3, 1));
    // YXY
    dcmTest = eulerAngles2DCM(EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::YXY));
    checkIsApprox(dcmTest, dcmConstruct(2, 1, 2));
    // YZX
    dcmTest = eulerAngles2DCM(EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::YZX));
    checkIsApprox(dcmTest, dcmConstruct(2, 3, 1));
    // ZXY
    dcmTest = eulerAngles2DCM(EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::ZXY));
    checkIsApprox(dcmTest, dcmConstruct(3, 1, 2));
    // XZY
    dcmTest = eulerAngles2DCM(EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::XZY));
    checkIsApprox(dcmTest, dcmConstruct(1, 3, 2));
    // ZYX
    dcmTest = eulerAngles2DCM(EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::ZYX));
    checkIsApprox(dcmTest, dcmConstruct(3, 2, 1));
    // YXZ
    dcmTest = eulerAngles2DCM(EulerAngles(phi, theta, psi, EulerAngles::EulerSequence::YXZ));
    checkIsApprox(dcmTest, dcmConstruct(2, 1, 3));
}

TEST_CASE("dcm2EulerAngles", "[EulerAngles]")
{
    double phi = 0.6;
    double theta = -0.2;
    double psi = 1.2;

    std::vector<EulerAngles::EulerSequence> testSeq = {
        EulerAngles::EulerSequence::ZXZ, EulerAngles::EulerSequence::XYX, EulerAngles::EulerSequence::YZY,
        EulerAngles::EulerSequence::ZYZ, EulerAngles::EulerSequence::XZX, EulerAngles::EulerSequence::YXY,
        EulerAngles::EulerSequence::XYZ, EulerAngles::EulerSequence::YZX, EulerAngles::EulerSequence::ZXY,
        EulerAngles::EulerSequence::XZY, EulerAngles::EulerSequence::ZYX, EulerAngles::EulerSequence::YXZ,
    };

    for (const auto seq : testSeq)
    {
        EulerAngles angles = EulerAngles(phi, theta, psi, seq);
        Matrix33 dcm = eulerAngles2DCM(angles);
        EulerAngles anglesTest = dcm2EulerAngles(dcm, seq);
        Matrix33 dcmTest = eulerAngles2DCM(anglesTest);
        INFO("Seq=" << static_cast<int>(seq));
        checkIsApprox(dcm, dcmTest);
    }
}

TEST_CASE("convertEulerAngleSequence", "[EulerAngles]")
{
    double phi = 0.6;
    double theta = -0.2;
    double psi = 1.2;

    std::vector<EulerAngles::EulerSequence> testSeq = {
        EulerAngles::EulerSequence::ZXZ, EulerAngles::EulerSequence::XYX, EulerAngles::EulerSequence::YZY,
        EulerAngles::EulerSequence::ZYZ, EulerAngles::EulerSequence::XZX, EulerAngles::EulerSequence::YXY,
        EulerAngles::EulerSequence::XYZ, EulerAngles::EulerSequence::YZX, EulerAngles::EulerSequence::ZXY,
        EulerAngles::EulerSequence::XZY, EulerAngles::EulerSequence::ZYX, EulerAngles::EulerSequence::YXZ,
    };

    for (const auto seq1 : testSeq)
        for (const auto seq2 : testSeq)
        {
            EulerAngles angles1 = EulerAngles(phi, theta, psi, seq1);
            EulerAngles angles2 = convertEulerAngleSequence(angles1, seq2);

            INFO("Seq1=" << static_cast<int>(seq1) << " Seq2=" << static_cast<int>(seq2));
            Matrix33 dcm1 = eulerAngles2DCM(angles1);
            Matrix33 dcm2 = eulerAngles2DCM(angles2);
            checkIsApprox(dcm1, dcm2);
        }
}
