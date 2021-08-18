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

TEST_CASE("quat2DCM", "[Quaternion]")
{
    Quaternion quat(0.9057665, 0.3547626, -0.1611254, 0.1666407);
    double data[9] = { 0.8925390, 0.1875527,  0.4101198,  -0.4161977, 0.6927487,
                       0.5889640, -0.1736482, -0.6963642, 0.6963642 };
    Matrix33 dcmResult(data);
    Matrix33 dcmTest = quat2DCM(quat);
    checkIsApprox(dcmTest, dcmResult);
}

TEST_CASE("dcm2Quat", "[Quaternion]")
{
    double data[9] = { 0.8925390, 0.1875527,  0.4101198,  -0.4161977, 0.6927487,
                       0.5889640, -0.1736482, -0.6963642, 0.6963642 };
    Matrix33 dcm(data);
    Quaternion quatResult = Quaternion(0.9057665, 0.3547626, -0.1611254, 0.1666407);
    Quaternion quatTest = dcm2Quat(dcm);
    CHECK(quatTest.q0 == Approx(quatResult.q0));
    CHECK(quatTest.q1 == Approx(quatResult.q1));
    CHECK(quatTest.q2 == Approx(quatResult.q2));
    CHECK(quatTest.q3 == Approx(quatResult.q3));
}

TEST_CASE("quat2EulerAngles", "[Quaternion]")
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

    Quaternion quat = Quaternion(0.9057665, 0.3547626, -0.1611254, 0.1666407);
    Matrix33 dcmQuat = quat2DCM(quat);

    for (const auto seq : testSeq)
    {
        EulerAngles angles = quat2EulerAngles(quat, seq);
        EulerAngles anglesDcm = dcm2EulerAngles(dcmQuat, seq);
        Matrix33 dcmAngles = eulerAngles2DCM(angles);
        INFO("Seq=" << static_cast<int>(seq));
        checkIsApprox(dcmAngles, dcmQuat);
    }
}

TEST_CASE("eulerAngles2Quat", "[Quaternion]")
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
        Matrix33 dcmAngles = eulerAngles2DCM(angles);

        Quaternion quat = eulerAngles2Quat(angles);
        Matrix33 dcmQuat = quat2DCM(quat);

        INFO("Seq=" << static_cast<int>(seq));
        checkIsApprox(dcmQuat, dcmAngles);
    }
}
