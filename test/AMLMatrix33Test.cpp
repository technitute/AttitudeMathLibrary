// ######################################################################### //
// Attitude Math Library
// Copyright (c) 2021 Dr. Steven Dumble Phd
// Distributed under the MIT Software License (See accompanying file LICENSE)
// ######################################################################### //

#include "AML.hpp"
#include "catch/catch.hpp"

using namespace AML;

TEST_CASE("Matrix33 Constructors", "[Matrix33]")
{
    // Case 1
    Matrix33 m;
    CHECK(m.m11 == 0.0);
    CHECK(m.m12 == 0.0);
    CHECK(m.m13 == 0.0);
    CHECK(m.m21 == 0.0);
    CHECK(m.m22 == 0.0);
    CHECK(m.m23 == 0.0);
    CHECK(m.m31 == 0.0);
    CHECK(m.m32 == 0.0);
    CHECK(m.m33 == 0.0);
    // Case 2
    m = Matrix33(5.0);
    CHECK(m.m11 == 5.0);
    CHECK(m.m12 == 5.0);
    CHECK(m.m13 == 5.0);
    CHECK(m.m21 == 5.0);
    CHECK(m.m22 == 5.0);
    CHECK(m.m23 == 5.0);
    CHECK(m.m31 == 5.0);
    CHECK(m.m32 == 5.0);
    CHECK(m.m33 == 5.0);
    // Case 3
    double data1d[9] = { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0 };
    m = Matrix33(data1d);
    CHECK(m.m11 == 1.0);
    CHECK(m.m12 == 2.0);
    CHECK(m.m13 == 3.0);
    CHECK(m.m21 == 4.0);
    CHECK(m.m22 == 5.0);
    CHECK(m.m23 == 6.0);
    CHECK(m.m31 == 7.0);
    CHECK(m.m32 == 8.0);
    CHECK(m.m33 == 9.0);
    // Case 4
    double data2d[3][3] = { { 1.0, 2.0, 3.0 }, { 4.0, 5.0, 6.0 }, { 7.0, 8.0, 9.0 } };
    m = Matrix33(data2d);
    CHECK(m.m11 == 1.0);
    CHECK(m.m12 == 2.0);
    CHECK(m.m13 == 3.0);
    CHECK(m.m21 == 4.0);
    CHECK(m.m22 == 5.0);
    CHECK(m.m23 == 6.0);
    CHECK(m.m31 == 7.0);
    CHECK(m.m32 == 8.0);
    CHECK(m.m33 == 9.0);
    CHECK(m.data[0][0] == 1.0);
    CHECK(m.data[0][1] == 2.0);
    CHECK(m.data[0][2] == 3.0);
    CHECK(m.data[1][0] == 4.0);
    CHECK(m.data[1][1] == 5.0);
    CHECK(m.data[1][2] == 6.0);
    CHECK(m.data[2][0] == 7.0);
    CHECK(m.data[2][1] == 8.0);
    CHECK(m.data[2][2] == 9.0);
    // Case 5
    Vector3 v1(1.0, 4.0, 7.0);
    Vector3 v2(2.0, 5.0, 8.0);
    Vector3 v3(3.0, 6.0, 9.0);
    m = Matrix33(v1, v2, v3);
    CHECK(m.m11 == 1.0);
    CHECK(m.m12 == 2.0);
    CHECK(m.m13 == 3.0);
    CHECK(m.m21 == 4.0);
    CHECK(m.m22 == 5.0);
    CHECK(m.m23 == 6.0);
    CHECK(m.m31 == 7.0);
    CHECK(m.m32 == 8.0);
    CHECK(m.m33 == 9.0);
}

TEST_CASE("Matrix33 Addition Operator Assignment with Matrix33", "[Matrix33]")
{
    // Case 1
    double data1[3][3] = { { 1.0, 2.0, 3.0 }, { 4.0, 5.0, 6.0 }, { 7.0, 8.0, 9.0 } };
    double data2[3][3] = { { 1.5, 2.5, 3.5 }, { 4.5, 5.5, 6.5 }, { 7.5, 8.5, 9.5 } };
    Matrix33 m1(data1);
    Matrix33 m2(data2);
    Matrix33 m = m1 += m2;

    CHECK(m.m11 == Approx(2.5));
    CHECK(m.m12 == Approx(4.5));
    CHECK(m.m13 == Approx(6.5));
    CHECK(m.m21 == Approx(8.5));
    CHECK(m.m22 == Approx(10.5));
    CHECK(m.m23 == Approx(12.5));
    CHECK(m.m31 == Approx(14.5));
    CHECK(m.m32 == Approx(16.5));
    CHECK(m.m33 == Approx(18.5));

    CHECK(m1.m11 == Approx(2.5));
    CHECK(m1.m12 == Approx(4.5));
    CHECK(m1.m13 == Approx(6.5));
    CHECK(m1.m21 == Approx(8.5));
    CHECK(m1.m22 == Approx(10.5));
    CHECK(m1.m23 == Approx(12.5));
    CHECK(m1.m31 == Approx(14.5));
    CHECK(m1.m32 == Approx(16.5));
    CHECK(m1.m33 == Approx(18.5));
}

TEST_CASE("Matrix33 Subtraction Operator Assignment with Matrix33", "[Matrix33]")
{
    // Case 1
    double data1[3][3] = { { 1.0, 2.0, 3.0 }, { 4.0, 5.0, 6.0 }, { 7.0, 8.0, 9.0 } };
    double data2[3][3] = { { 1.5, 2.5, 3.5 }, { 4.5, 5.5, 6.5 }, { 7.5, 8.5, 9.5 } };
    Matrix33 m1(data1);
    Matrix33 m2(data2);
    Matrix33 m = m1 -= m2;

    CHECK(m1.m11 == Approx(-0.5));
    CHECK(m1.m12 == Approx(-0.5));
    CHECK(m1.m13 == Approx(-0.5));
    CHECK(m1.m21 == Approx(-0.5));
    CHECK(m1.m22 == Approx(-0.5));
    CHECK(m1.m23 == Approx(-0.5));
    CHECK(m1.m31 == Approx(-0.5));
    CHECK(m1.m32 == Approx(-0.5));
    CHECK(m1.m33 == Approx(-0.5));

    CHECK(m1.m11 == m.m11);
    CHECK(m1.m12 == m.m12);
    CHECK(m1.m13 == m.m13);
    CHECK(m1.m21 == m.m21);
    CHECK(m1.m22 == m.m22);
    CHECK(m1.m23 == m.m23);
    CHECK(m1.m31 == m.m31);
    CHECK(m1.m32 == m.m32);
    CHECK(m1.m33 == m.m33);
}

TEST_CASE("Matrix33 Multiplication Operator Assignment with Matrix33", "[Matrix33]")
{
    // Case 1
    double data1[3][3] = { { 1.0, 2.0, 3.0 }, { 4.0, 5.0, 6.0 }, { 7.0, 8.0, 9.0 } };
    double data2[3][3] = { { 1.5, 2.5, 3.5 }, { 4.5, 5.5, 6.5 }, { 7.5, 8.5, 9.5 } };
    Matrix33 m1(data1);
    Matrix33 m2(data2);
    Matrix33 m = m1 *= m2;

    CHECK(m1.m11 == Approx(33.0));
    CHECK(m1.m12 == Approx(39.0));
    CHECK(m1.m13 == Approx(45.0));
    CHECK(m1.m21 == Approx(73.5));
    CHECK(m1.m22 == Approx(88.5));
    CHECK(m1.m23 == Approx(103.5));
    CHECK(m1.m31 == Approx(114.0));
    CHECK(m1.m32 == Approx(138.0));
    CHECK(m1.m33 == Approx(162.));

    CHECK(m1.m11 == m.m11);
    CHECK(m1.m12 == m.m12);
    CHECK(m1.m13 == m.m13);
    CHECK(m1.m21 == m.m21);
    CHECK(m1.m22 == m.m22);
    CHECK(m1.m23 == m.m23);
    CHECK(m1.m31 == m.m31);
    CHECK(m1.m32 == m.m32);
    CHECK(m1.m33 == m.m33);
}

TEST_CASE("Matrix33 Division Operator Assignment with Matrix33", "[Matrix33]")
{
    // Case 1
    double data1[3][3] = { { -2.0, -3.0, 2.0 }, { 1.0, 0.0, 1.0 }, { 6.0, -8.0, 7.0 } };
    double data2[3][3] = { { -2.0, 2.0, 3.0 }, { -1.0, 1.0, 3.0 }, { 2.0, 0.0, -1.0 } };
    Matrix33 m1(data1);
    Matrix33 m2(data2);
    Matrix33 m = m1 /= m2;

    CHECK(m1.m11 == Approx(-17.0 / 6.0));
    CHECK(m1.m12 == Approx(8.0 / 3.0));
    CHECK(m1.m13 == Approx(-5.0 / 2.0));
    CHECK(m1.m21 == Approx(-0.5));
    CHECK(m1.m22 == Approx(1.0));
    CHECK(m1.m23 == Approx(0.5));
    CHECK(m1.m31 == Approx(-10.0));
    CHECK(m1.m32 == Approx(12.0));
    CHECK(m1.m33 == Approx(-1.0));

    CHECK(m1.m11 == m.m11);
    CHECK(m1.m12 == m.m12);
    CHECK(m1.m13 == m.m13);
    CHECK(m1.m21 == m.m21);
    CHECK(m1.m22 == m.m22);
    CHECK(m1.m23 == m.m23);
    CHECK(m1.m31 == m.m31);
    CHECK(m1.m32 == m.m32);
    CHECK(m1.m33 == m.m33);
}

TEST_CASE("Matrix33 Addition Operator Assignment with Scalar", "[Matrix33]")
{
    // Case 1
    double data1[3][3] = { { -2.0, -3.0, 2.0 }, { 1.0, 0.0, 1.0 }, { 6.0, -8.0, 7.0 } };
    Matrix33 m1(data1);
    Matrix33 m = m1 += 0.5;

    CHECK(m1.m11 == Approx(-1.5));
    CHECK(m1.m12 == Approx(-2.5));
    CHECK(m1.m13 == Approx(2.5));
    CHECK(m1.m21 == Approx(1.5));
    CHECK(m1.m22 == Approx(0.5));
    CHECK(m1.m23 == Approx(1.5));
    CHECK(m1.m31 == Approx(6.5));
    CHECK(m1.m32 == Approx(-7.5));
    CHECK(m1.m33 == Approx(7.5));

    CHECK(m1.m11 == m.m11);
    CHECK(m1.m12 == m.m12);
    CHECK(m1.m13 == m.m13);
    CHECK(m1.m21 == m.m21);
    CHECK(m1.m22 == m.m22);
    CHECK(m1.m23 == m.m23);
    CHECK(m1.m31 == m.m31);
    CHECK(m1.m32 == m.m32);
    CHECK(m1.m33 == m.m33);
}

TEST_CASE("Matrix33 Subtraction Operator Assignment with Scalar", "[Matrix33]")
{
    // Case 1
    double data1[3][3] = { { -2.0, -3.0, 2.0 }, { 1.0, 0.0, 1.0 }, { 6.0, -8.0, 7.0 } };
    Matrix33 m1(data1);
    Matrix33 m = m1 -= 0.5;

    CHECK(m1.m11 == Approx(-2.5));
    CHECK(m1.m12 == Approx(-3.5));
    CHECK(m1.m13 == Approx(1.5));
    CHECK(m1.m21 == Approx(0.5));
    CHECK(m1.m22 == Approx(-0.5));
    CHECK(m1.m23 == Approx(0.5));
    CHECK(m1.m31 == Approx(5.5));
    CHECK(m1.m32 == Approx(-8.5));
    CHECK(m1.m33 == Approx(6.5));

    CHECK(m1.m11 == m.m11);
    CHECK(m1.m12 == m.m12);
    CHECK(m1.m13 == m.m13);
    CHECK(m1.m21 == m.m21);
    CHECK(m1.m22 == m.m22);
    CHECK(m1.m23 == m.m23);
    CHECK(m1.m31 == m.m31);
    CHECK(m1.m32 == m.m32);
    CHECK(m1.m33 == m.m33);
}

TEST_CASE("Matrix33 Multiplication Operator Assignment with Scalar", "[Matrix33]")
{
    // Case 1
    double data1[3][3] = { { -2.0, -3.0, 2.0 }, { 1.0, 0.0, 1.0 }, { 6.0, -8.0, 7.0 } };
    Matrix33 m1(data1);
    Matrix33 m = m1 *= 0.5;

    CHECK(m1.m11 == Approx(-1.0));
    CHECK(m1.m12 == Approx(-1.5));
    CHECK(m1.m13 == Approx(1.0));
    CHECK(m1.m21 == Approx(0.5));
    CHECK(m1.m22 == Approx(0.0));
    CHECK(m1.m23 == Approx(0.5));
    CHECK(m1.m31 == Approx(3.0));
    CHECK(m1.m32 == Approx(-4.0));
    CHECK(m1.m33 == Approx(3.5));

    CHECK(m1.m11 == m.m11);
    CHECK(m1.m12 == m.m12);
    CHECK(m1.m13 == m.m13);
    CHECK(m1.m21 == m.m21);
    CHECK(m1.m22 == m.m22);
    CHECK(m1.m23 == m.m23);
    CHECK(m1.m31 == m.m31);
    CHECK(m1.m32 == m.m32);
    CHECK(m1.m33 == m.m33);
}

TEST_CASE("Matrix33 Division Operator Assignment with Scalar", "[Matrix33]")
{
    // Case 1
    double data1[3][3] = { { -2.0, -3.0, 2.0 }, { 1.0, 0.0, 1.0 }, { 6.0, -8.0, 7.0 } };
    Matrix33 m1(data1);
    Matrix33 m = m1 /= 0.5;

    CHECK(m1.m11 == Approx(-4.0));
    CHECK(m1.m12 == Approx(-6.0));
    CHECK(m1.m13 == Approx(4.0));
    CHECK(m1.m21 == Approx(2.0));
    CHECK(m1.m22 == Approx(0.0));
    CHECK(m1.m23 == Approx(2.0));
    CHECK(m1.m31 == Approx(12.0));
    CHECK(m1.m32 == Approx(-16.0));
    CHECK(m1.m33 == Approx(14.0));

    CHECK(m1.m11 == m.m11);
    CHECK(m1.m12 == m.m12);
    CHECK(m1.m13 == m.m13);
    CHECK(m1.m21 == m.m21);
    CHECK(m1.m22 == m.m22);
    CHECK(m1.m23 == m.m23);
    CHECK(m1.m31 == m.m31);
    CHECK(m1.m32 == m.m32);
    CHECK(m1.m33 == m.m33);
}

TEST_CASE("Matrix33 Identity", "[Matrix33]")
{
    // Case 1
    Matrix33 m = Matrix33::identity();

    CHECK(m.m11 == Approx(1.0));
    CHECK(m.m12 == Approx(0.0));
    CHECK(m.m13 == Approx(0.0));
    CHECK(m.m21 == Approx(0.0));
    CHECK(m.m22 == Approx(1.0));
    CHECK(m.m23 == Approx(0.0));
    CHECK(m.m31 == Approx(0.0));
    CHECK(m.m32 == Approx(0.0));
    CHECK(m.m33 == Approx(1.0));
}

TEST_CASE("Matrix33 Negative Operator", "[Matrix33]")
{
    // Case 1
    double data1[3][3] = { { -2.0, -3.0, 2.0 }, { 1.0, 0.0, 1.0 }, { 6.0, -8.0, 7.0 } };
    Matrix33 m = -Matrix33(data1);

    CHECK(m.m11 == Approx(2.0));
    CHECK(m.m12 == Approx(3.0));
    CHECK(m.m13 == Approx(-2.0));
    CHECK(m.m21 == Approx(-1.0));
    CHECK(m.m22 == Approx(0.0));
    CHECK(m.m23 == Approx(-1.0));
    CHECK(m.m31 == Approx(-6.0));
    CHECK(m.m32 == Approx(8.0));
    CHECK(m.m33 == Approx(-7.0));
}

TEST_CASE("Matrix33 Addition Operator with Matrix33", "[Matrix33]")
{
    // Case 1
    double data1[3][3] = { { 1.0, 2.0, 3.0 }, { 4.0, 5.0, 6.0 }, { 7.0, 8.0, 9.0 } };
    double data2[3][3] = { { 1.5, 2.5, 3.5 }, { 4.5, 5.5, 6.5 }, { 7.5, 8.5, 9.5 } };
    Matrix33 m1(data1);
    Matrix33 m2(data2);
    Matrix33 m = m1 + m2;

    CHECK(m.m11 == Approx(2.5));
    CHECK(m.m12 == Approx(4.5));
    CHECK(m.m13 == Approx(6.5));
    CHECK(m.m21 == Approx(8.5));
    CHECK(m.m22 == Approx(10.5));
    CHECK(m.m23 == Approx(12.5));
    CHECK(m.m31 == Approx(14.5));
    CHECK(m.m32 == Approx(16.5));
    CHECK(m.m33 == Approx(18.5));
}

TEST_CASE("Matrix33 Subtraction Operator with Matrix33", "[Matrix33]")
{
    // Case 1
    double data1[3][3] = { { 1.0, 2.0, 3.0 }, { 4.0, 5.0, 6.0 }, { 7.0, 8.0, 9.0 } };
    double data2[3][3] = { { 1.5, 2.5, 3.5 }, { 4.5, 5.5, 6.5 }, { 7.5, 8.5, 9.5 } };
    Matrix33 m1(data1);
    Matrix33 m2(data2);
    Matrix33 m = m1 - m2;

    CHECK(m.m11 == Approx(-0.5));
    CHECK(m.m12 == Approx(-0.5));
    CHECK(m.m13 == Approx(-0.5));
    CHECK(m.m21 == Approx(-0.5));
    CHECK(m.m22 == Approx(-0.5));
    CHECK(m.m23 == Approx(-0.5));
    CHECK(m.m31 == Approx(-0.5));
    CHECK(m.m32 == Approx(-0.5));
    CHECK(m.m33 == Approx(-0.5));
}

TEST_CASE("Matrix33 Multiplication Operator with Matrix33", "[Matrix33]")
{
    // Case 1
    double data1[3][3] = { { 1.0, 2.0, 3.0 }, { 4.0, 5.0, 6.0 }, { 7.0, 8.0, 9.0 } };
    double data2[3][3] = { { 1.5, 2.5, 3.5 }, { 4.5, 5.5, 6.5 }, { 7.5, 8.5, 9.5 } };
    Matrix33 m1(data1);
    Matrix33 m2(data2);
    Matrix33 m = m1 * m2;

    CHECK(m.m11 == Approx(33.0));
    CHECK(m.m12 == Approx(39.0));
    CHECK(m.m13 == Approx(45.0));
    CHECK(m.m21 == Approx(73.5));
    CHECK(m.m22 == Approx(88.5));
    CHECK(m.m23 == Approx(103.5));
    CHECK(m.m31 == Approx(114.0));
    CHECK(m.m32 == Approx(138.0));
    CHECK(m.m33 == Approx(162.));
}

TEST_CASE("Matrix33 Division Operator with Matrix33", "[Matrix33]")
{
    // Case 1
    double data1[3][3] = { { -2.0, -3.0, 2.0 }, { 1.0, 0.0, 1.0 }, { 6.0, -8.0, 7.0 } };
    double data2[3][3] = { { -2.0, 2.0, 3.0 }, { -1.0, 1.0, 3.0 }, { 2.0, 0.0, -1.0 } };
    Matrix33 m1(data1);
    Matrix33 m2(data2);
    Matrix33 m = m1 / m2;

    CHECK(m.m11 == Approx(-17.0 / 6.0));
    CHECK(m.m12 == Approx(8.0 / 3.0));
    CHECK(m.m13 == Approx(-5.0 / 2.0));
    CHECK(m.m21 == Approx(-0.5));
    CHECK(m.m22 == Approx(1.0));
    CHECK(m.m23 == Approx(0.5));
    CHECK(m.m31 == Approx(-10.0));
    CHECK(m.m32 == Approx(12.0));
    CHECK(m.m33 == Approx(-1.0));
}

TEST_CASE("Matrix33 Multiplication Operator with Vector3", "[Matrix33]")
{
    // Case 1
    double data1[3][3] = { { -2.0, -3.0, 2.0 }, { 1.0, 0.0, 1.0 }, { 6.0, -8.0, 7.0 } };
    double data2[3] = { -2.0, 2.0, 3.0 };
    Matrix33 m1(data1);
    Vector3 v1(data2);
    Vector3 v = m1 * v1;

    CHECK(v.x == Approx(4.0));
    CHECK(v.y == Approx(1.0));
    CHECK(v.z == Approx(-7.0));
}

TEST_CASE("Matrix33 Addition Operator with Scalar", "[Matrix33]")
{
    double data1[3][3] = { { -2.0, -3.0, 2.0 }, { 1.0, 0.0, 1.0 }, { 6.0, -8.0, 7.0 } };
    Matrix33 m1(data1);

    // Case 1
    Matrix33 m = m1 + 0.5;
    CHECK(m.m11 == Approx(-1.5));
    CHECK(m.m12 == Approx(-2.5));
    CHECK(m.m13 == Approx(2.5));
    CHECK(m.m21 == Approx(1.5));
    CHECK(m.m22 == Approx(0.5));
    CHECK(m.m23 == Approx(1.5));
    CHECK(m.m31 == Approx(6.5));
    CHECK(m.m32 == Approx(-7.5));
    CHECK(m.m33 == Approx(7.5));

    // Case 2
    m = 0.5 + m1;
    CHECK(m.m11 == Approx(-1.5));
    CHECK(m.m12 == Approx(-2.5));
    CHECK(m.m13 == Approx(2.5));
    CHECK(m.m21 == Approx(1.5));
    CHECK(m.m22 == Approx(0.5));
    CHECK(m.m23 == Approx(1.5));
    CHECK(m.m31 == Approx(6.5));
    CHECK(m.m32 == Approx(-7.5));
    CHECK(m.m33 == Approx(7.5));
}

TEST_CASE("Matrix33 Subtraction Operator with Scalar", "[Matrix33]")
{
    double data1[3][3] = { { -2.0, -3.0, 2.0 }, { 1.0, 0.0, 1.0 }, { 6.0, -8.0, 7.0 } };
    Matrix33 m1(data1);

    // Case 1
    Matrix33 m = m1 - 0.5;
    CHECK(m.m11 == Approx(-2.5));
    CHECK(m.m12 == Approx(-3.5));
    CHECK(m.m13 == Approx(1.5));
    CHECK(m.m21 == Approx(0.5));
    CHECK(m.m22 == Approx(-0.5));
    CHECK(m.m23 == Approx(0.5));
    CHECK(m.m31 == Approx(5.5));
    CHECK(m.m32 == Approx(-8.5));
    CHECK(m.m33 == Approx(6.5));

    // Case 2
    m = 0.5 - m1;
    CHECK(m.m11 == Approx(2.5));
    CHECK(m.m12 == Approx(3.5));
    CHECK(m.m13 == Approx(-1.5));
    CHECK(m.m21 == Approx(-0.5));
    CHECK(m.m22 == Approx(0.5));
    CHECK(m.m23 == Approx(-0.5));
    CHECK(m.m31 == Approx(-5.5));
    CHECK(m.m32 == Approx(8.5));
    CHECK(m.m33 == Approx(-6.5));
}

TEST_CASE("Matrix33 Multiplication Operator with Scalar", "[Matrix33]")
{
    double data1[3][3] = { { -2.0, -3.0, 2.0 }, { 1.0, 0.0, 1.0 }, { 6.0, -8.0, 7.0 } };
    Matrix33 m1(data1);

    // Case 1
    Matrix33 m = m1 * 0.5;
    CHECK(m.m11 == Approx(-1.0));
    CHECK(m.m12 == Approx(-1.5));
    CHECK(m.m13 == Approx(1.0));
    CHECK(m.m21 == Approx(0.5));
    CHECK(m.m22 == Approx(0.0));
    CHECK(m.m23 == Approx(0.5));
    CHECK(m.m31 == Approx(3.0));
    CHECK(m.m32 == Approx(-4.0));
    CHECK(m.m33 == Approx(3.5));

    // Case 1
    m = 0.5 * m1;
    CHECK(m.m11 == Approx(-1.0));
    CHECK(m.m12 == Approx(-1.5));
    CHECK(m.m13 == Approx(1.0));
    CHECK(m.m21 == Approx(0.5));
    CHECK(m.m22 == Approx(0.0));
    CHECK(m.m23 == Approx(0.5));
    CHECK(m.m31 == Approx(3.0));
    CHECK(m.m32 == Approx(-4.0));
    CHECK(m.m33 == Approx(3.5));
}

TEST_CASE("Matrix33 Division Operator with Scalar", "[Matrix33]")
{
    double data1[3][3] = { { -2.0, -3.0, 2.0 }, { 1.0, -1.0, 1.0 }, { 6.0, -8.0, 7.0 } };
    Matrix33 m1(data1);

    // Case 1
    Matrix33 m = m1 / 0.5;
    CHECK(m.m11 == Approx(-4.0));
    CHECK(m.m12 == Approx(-6.0));
    CHECK(m.m13 == Approx(4.0));
    CHECK(m.m21 == Approx(2.0));
    CHECK(m.m22 == Approx(-2.0));
    CHECK(m.m23 == Approx(2.0));
    CHECK(m.m31 == Approx(12.0));
    CHECK(m.m32 == Approx(-16.0));
    CHECK(m.m33 == Approx(14.0));

    // Case 2
    m = 0.5 / m1;
    CHECK(m.m11 == Approx(0.5 / -2.0));
    CHECK(m.m12 == Approx(0.5 / -3.0));
    CHECK(m.m13 == Approx(0.5 / 2.0));
    CHECK(m.m21 == Approx(0.5 / 1.0));
    CHECK(m.m22 == Approx(0.5 / -1.0));
    CHECK(m.m23 == Approx(0.5 / 1.0));
    CHECK(m.m31 == Approx(0.5 / 6.0));
    CHECK(m.m32 == Approx(0.5 / -8.0));
    CHECK(m.m33 == Approx(0.5 / 7.0));
}

TEST_CASE("Matrix33 Diag of Matrix33", "[Matrix33]")
{
    // Case 1
    double data1[3][3] = { { -2.0, -3.0, 2.0 }, { 1.0, 0.0, 1.0 }, { 6.0, -8.0, 7.0 } };
    Vector3 m = diag(Matrix33(data1));

    CHECK(m.x == Approx(-2.0));
    CHECK(m.y == Approx(0.0));
    CHECK(m.z == Approx(7.0));
}

TEST_CASE("Matrix33 Vector3 to Diag Matrix33", "[Matrix33]")
{
    // Case 1
    double data[3] = { -2.0, -3.0, 2.0 };
    Vector3 v = Vector3(data);
    Matrix33 m = diag(v);

    CHECK(m.m11 == Approx(-2.0));
    CHECK(m.m12 == Approx(0.0));
    CHECK(m.m13 == Approx(0.0));
    CHECK(m.m21 == Approx(0.0));
    CHECK(m.m22 == Approx(-3.0));
    CHECK(m.m23 == Approx(0.0));
    CHECK(m.m31 == Approx(0.0));
    CHECK(m.m32 == Approx(0.0));
    CHECK(m.m33 == Approx(2.0));
}

TEST_CASE("Matrix33 Transpose", "[Matrix33]")
{
    double data1[3][3] = { { -2.0, -3.0, 2.0 }, { 1.0, -1.0, 1.0 }, { 6.0, -8.0, 7.0 } };
    Matrix33 m1(data1);

    // Case 1
    Matrix33 m = transpose(m1);
    CHECK(m.m11 == Approx(-2.0));
    CHECK(m.m21 == Approx(-3.0));
    CHECK(m.m31 == Approx(2.0));
    CHECK(m.m12 == Approx(1.0));
    CHECK(m.m22 == Approx(-1.0));
    CHECK(m.m32 == Approx(1.0));
    CHECK(m.m13 == Approx(6.0));
    CHECK(m.m23 == Approx(-8.0));
    CHECK(m.m33 == Approx(7.0));
}

TEST_CASE("Matrix33 Determinant", "[Matrix33]")
{
    double data1[3][3] = { { -2.0, -3.0, 2.0 }, { 1.0, -1.0, 1.0 }, { 6.0, -8.0, 7.0 } };
    Matrix33 m1(data1);

    // Case 1
    double det = determinant(m1);
    CHECK(det == Approx(-3.0));
}

TEST_CASE("Matrix33 Inverse", "[Matrix33]")
{
    double data1[3][3] = { { -2.0, -3.0, 2.0 }, { 1.0, -1.0, 1.0 }, { 6.0, -8.0, 7.0 } };
    Matrix33 m1(data1);

    // Case 1
    Matrix33 m = inverse(m1);
    CHECK(m.m11 == Approx(-1.0 / 3.0));
    CHECK(m.m12 == Approx(-5.0 / 3.0));
    CHECK(m.m13 == Approx(1.0 / 3.0));
    CHECK(m.m21 == Approx(1.0 / 3.0));
    CHECK(m.m22 == Approx(26.0 / 3.0));
    CHECK(m.m23 == Approx(-4.0 / 3.0));
    CHECK(m.m31 == Approx(2.0 / 3.0));
    CHECK(m.m32 == Approx(34.0 / 3.0));
    CHECK(m.m33 == Approx(-5.0 / 3.0));
}
