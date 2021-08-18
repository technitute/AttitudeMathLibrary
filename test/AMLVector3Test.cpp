// ######################################################################### //
// Attitude Math Library
// Copyright (c) 2021 Dr. Steven Dumble Phd
// Distributed under the MIT Software License (See accompanying file LICENSE)
// ######################################################################### //

#include "AML.hpp"
#include "catch/catch.hpp"

using namespace AML;

TEST_CASE("Vector3 Constructors", "[Vector3]")
{
    // Case 1
    Vector3 v;
    CHECK(v.x == 0.0);
    CHECK(v.y == 0.0);
    CHECK(v.z == 0.0);
    // Case 2
    v = Vector3(5.0);
    CHECK(v.x == 5.0);
    CHECK(v.y == 5.0);
    CHECK(v.z == 5.0);
    // Case 3
    v = Vector3(1.0, 2.0, 3.0);
    CHECK(v.x == 1.0);
    CHECK(v.y == 2.0);
    CHECK(v.z == 3.0);
    // Case 4
    double data[3] = { 1.0, 2.0, 3.0 };
    v = Vector3(data);
    CHECK(v.x == 1.0);
    CHECK(v.y == 2.0);
    CHECK(v.z == 3.0);
}

TEST_CASE("Vector3 plus scalar assignments", "[Vector3]")
{
    // Case 1
    Vector3 v = Vector3(2, -5, 4);
    v += 3;
    CHECK(v.x == Approx(5));
    CHECK(v.y == Approx(-2));
    CHECK(v.z == Approx(7));
    // Case 2
    v = Vector3(0.24, 0.0082, -0.03);
    v += 0.2;
    CHECK(v.x == Approx(0.44));
    CHECK(v.y == Approx(0.2082));
    CHECK(v.z == Approx(0.17));
    // Case 3
    v = Vector3(-27, 83, -163);
    v += 13;
    CHECK(v.x == Approx(-14));
    CHECK(v.y == Approx(96));
    CHECK(v.z == Approx(-150));
}

TEST_CASE("Vector3 plus scalar", "[Vector3]")
{
    // Case 1
    Vector3 v = Vector3(2, -5, 4);
    v = v + 3;
    CHECK(v.x == Approx(5));
    CHECK(v.y == Approx(-2));
    CHECK(v.z == Approx(7));
    // Case 2
    v = Vector3(0.24, 0.0082, -0.03);
    v = v + 0.2;
    CHECK(v.x == Approx(0.44));
    CHECK(v.y == Approx(0.2082));
    CHECK(v.z == Approx(0.17));
    // Case 3
    v = Vector3(-27, 83, -163);
    v = v + 13;
    CHECK(v.x == Approx(-14));
    CHECK(v.y == Approx(96));
    CHECK(v.z == Approx(-150));
}

TEST_CASE("Vector3 minus scalar assignments", "[Vector3]")
{
    // Case 1
    Vector3 v = Vector3(2, -5, 4);
    v -= 3;
    CHECK(v.x == Approx(-1));
    CHECK(v.y == Approx(-8));
    CHECK(v.z == Approx(1));
    // Case 2
    v = Vector3(0.24, 0.0082, -0.03);
    v -= 0.2;
    CHECK(v.x == Approx(0.04));
    CHECK(v.y == Approx(-0.1918));
    CHECK(v.z == Approx(-0.23));
    // Case 3
    v = Vector3(-27, 83, -163);
    v -= 13;
    CHECK(v.x == Approx(-40));
    CHECK(v.y == Approx(70));
    CHECK(v.z == Approx(-176));
}

TEST_CASE("Vector3 minus scalar", "[Vector3]")
{
    // Case 1
    Vector3 v = Vector3(2, -5, 4);
    v = v - 3;
    CHECK(v.x == Approx(-1));
    CHECK(v.y == Approx(-8));
    CHECK(v.z == Approx(1));
    // Case 2
    v = Vector3(0.24, 0.0082, -0.03);
    v = v - 0.2;
    CHECK(v.x == Approx(0.04));
    CHECK(v.y == Approx(-0.1918));
    CHECK(v.z == Approx(-0.23));
    // Case 3
    v = Vector3(-27, 83, -163);
    v = v - 13;
    CHECK(v.x == Approx(-40));
    CHECK(v.y == Approx(70));
    CHECK(v.z == Approx(-176));
}

TEST_CASE("Vector3 times scalar assignment", "[Vector3]")
{
    // Case 1
    Vector3 v = Vector3(2, -5, 4);
    v *= 3;
    CHECK(v.x == Approx(6));
    CHECK(v.y == Approx(-15));
    CHECK(v.z == Approx(12));
    // Case 2
    v = Vector3(0.24, 0.0082, -0.03);
    v *= 0.2;
    CHECK(v.x == Approx(0.048));
    CHECK(v.y == Approx(0.00164));
    CHECK(v.z == Approx(-0.006));
    // Case 3
    v = Vector3(-27, 83, -163);
    v *= 13;
    CHECK(v.x == Approx(-351));
    CHECK(v.y == Approx(1079));
    CHECK(v.z == Approx(-2119));
}

TEST_CASE("Vector3 times scalar", "[Vector3]")
{
    // Case 1
    Vector3 v = Vector3(2, -5, 4);
    v = v * 3;
    CHECK(v.x == Approx(6));
    CHECK(v.y == Approx(-15));
    CHECK(v.z == Approx(12));
    // Case 2
    v = Vector3(0.24, 0.0082, -0.03);
    v = v * 0.2;
    CHECK(v.x == Approx(0.048));
    CHECK(v.y == Approx(0.00164));
    CHECK(v.z == Approx(-0.006));
    // Case 3
    v = Vector3(-27, 83, -163);
    v = v * 13;
    CHECK(v.x == Approx(-351));
    CHECK(v.y == Approx(1079));
    CHECK(v.z == Approx(-2119));
}

TEST_CASE("Vector3 divided by scalar assignment", "[Vector3]")
{
    // Case 1
    Vector3 v = Vector3(2, -5, 4);
    v /= 3;
    CHECK(v.x == Approx(0.66666667));
    CHECK(v.y == Approx(-1.66666667));
    CHECK(v.z == Approx(1.33333333));
    // Case 2
    v = Vector3(0.24, 0.0082, -0.03);
    v /= 0.2;
    CHECK(v.x == Approx(1.2));
    CHECK(v.y == Approx(0.041));
    CHECK(v.z == Approx(-0.15));
    // Case 3
    v = Vector3(-27, 83, -163);
    v /= 13;
    CHECK(v.x == Approx(-2.0769231));
    CHECK(v.y == Approx(6.3846154));
    CHECK(v.z == Approx(-12.538462));
}

TEST_CASE("Vector3 divided by scalar", "[Vector3]")
{
    // Case 1
    Vector3 v = Vector3(2, -5, 4);
    v = v / 3;
    CHECK(v.x == Approx(0.66666667));
    CHECK(v.y == Approx(-1.66666667));
    CHECK(v.z == Approx(1.33333333));
    // Case 2
    v = Vector3(0.24, 0.0082, -0.03);
    v = v / 0.2;
    CHECK(v.x == Approx(1.2));
    CHECK(v.y == Approx(0.041));
    CHECK(v.z == Approx(-0.15));
    // Case 3
    v = Vector3(-27, 83, -163);
    v = v / 13;
    CHECK(v.x == Approx(-2.0769231));
    CHECK(v.y == Approx(6.3846154));
    CHECK(v.z == Approx(-12.538462));
}

TEST_CASE("Vector3 plus Vector3 assignment", "[Vector3]")
{
    // Case 1
    Vector3 v1 = Vector3(2, -5, 4);
    Vector3 v2 = Vector3(6, 2, -8);
    v1 += v2;
    CHECK(v1.x == Approx(8));
    CHECK(v1.y == Approx(-3));
    CHECK(v1.z == Approx(-4));
    // Case 2
    v1 = Vector3(0.24, 0.0082, -0.03);
    v2 = Vector3(0.53, -0.0532, -1.53);
    v1 += v2;
    CHECK(v1.x == Approx(0.77));
    CHECK(v1.y == Approx(-0.045));
    CHECK(v1.z == Approx(-1.56));
    // Case 3
    v1 = Vector3(-27, 83, -163);
    v2 = Vector3(36, -64, 264);
    v1 += v2;
    CHECK(v1.x == Approx(9));
    CHECK(v1.y == Approx(19));
    CHECK(v1.z == Approx(101));
}

TEST_CASE("Vector3 plus Vector3", "[Vector3]")
{
    // Case 1
    Vector3 v1 = Vector3(2, -5, 4);
    Vector3 v2 = Vector3(6, 2, -8);
    Vector3 v = v1 + v2;
    CHECK(v.x == Approx(8));
    CHECK(v.y == Approx(-3));
    CHECK(v.z == Approx(-4));
    // Case 2
    v1 = Vector3(0.24, 0.0082, -0.03);
    v2 = Vector3(0.53, -0.0532, -1.53);
    v = v1 + v2;
    CHECK(v.x == Approx(0.77));
    CHECK(v.y == Approx(-0.045));
    CHECK(v.z == Approx(-1.56));
    // Case 3
    v1 = Vector3(-27, 83, -163);
    v2 = Vector3(36, -64, 264);
    v = v1 + v2;
    CHECK(v.x == Approx(9));
    CHECK(v.y == Approx(19));
    CHECK(v.z == Approx(101));
}

TEST_CASE("Vector3 minus Vector3 assignment", "[Vector3]")
{
    // Case 1
    Vector3 v1 = Vector3(2, -5, 4);
    Vector3 v2 = Vector3(6, 2, -8);
    v1 -= v2;
    CHECK(v1.x == Approx(-4));
    CHECK(v1.y == Approx(-7));
    CHECK(v1.z == Approx(12));
    // Case 2
    v1 = Vector3(0.24, 0.0082, -0.03);
    v2 = Vector3(0.53, -0.0532, -1.53);
    v1 -= v2;
    CHECK(v1.x == Approx(-0.29));
    CHECK(v1.y == Approx(0.0614));
    CHECK(v1.z == Approx(1.5));
    // Case 3
    v1 = Vector3(-27, 83, -163);
    v2 = Vector3(36, -64, 264);
    v1 -= v2;
    CHECK(v1.x == Approx(-63));
    CHECK(v1.y == Approx(147));
    CHECK(v1.z == Approx(-427));
}

TEST_CASE("Vector3 minus Vector3", "[Vector3]")
{
    // Case 1
    Vector3 v1 = Vector3(2, -5, 4);
    Vector3 v2 = Vector3(6, 2, -8);
    Vector3 v = v1 - v2;
    CHECK(v.x == Approx(-4));
    CHECK(v.y == Approx(-7));
    CHECK(v.z == Approx(12));
    // Case 2
    v1 = Vector3(0.24, 0.0082, -0.03);
    v2 = Vector3(0.53, -0.0532, -1.53);
    v = v1 - v2;
    CHECK(v.x == Approx(-0.29));
    CHECK(v.y == Approx(0.0614));
    CHECK(v.z == Approx(1.5));
    // Case 3
    v1 = Vector3(-27, 83, -163);
    v2 = Vector3(36, -64, 264);
    v = v1 - v2;
    CHECK(v.x == Approx(-63));
    CHECK(v.y == Approx(147));
    CHECK(v.z == Approx(-427));
}

TEST_CASE("Cross product of Vector3s", "[Vector3]")
{
    // Case 1
    Vector3 v1 = Vector3(2, -5, 4);
    Vector3 v2 = Vector3(6, 2, -8);
    Vector3 v = cross(v1, v2);
    CHECK(v.x == Approx(32));
    CHECK(v.y == Approx(40));
    CHECK(v.z == Approx(34));
    // Case 2
    v1 = Vector3(0.24, 0.0082, -0.03);
    v2 = Vector3(0.53, -0.0532, -1.53);
    v = cross(v1, v2);
    CHECK(v.x == Approx(-0.014142));
    CHECK(v.y == Approx(0.3513));
    CHECK(v.z == Approx(-0.017114));
    // Case 3
    v1 = Vector3(-27, 83, -163);
    v2 = Vector3(36, -64, 264);
    v = cross(v1, v2);
    CHECK(v.x == Approx(11480));
    CHECK(v.y == Approx(1260));
    CHECK(v.z == Approx(-1260));
    // Cross product of vector and itself is zero
    v = cross(v1, v1);
    CHECK(v.x == Approx(0));
    CHECK(v.y == Approx(0));
    CHECK(v.z == Approx(0));
}

TEST_CASE("Dot product of Vector3s", "[Vector3]")
{
    // Case 1
    Vector3 v1 = Vector3(2, -5, 4);
    Vector3 v2 = Vector3(6, 2, -8);
    CHECK(dot(v1, v2) == Approx(-30));
    // Case 2
    v1 = Vector3(0.24, 0.0082, -0.03);
    v2 = Vector3(0.53, -0.0532, -1.53);
    CHECK(dot(v1, v2) == Approx(0.172663));
    // Case 3
    v1 = Vector3(-27, 83, -163);
    v2 = Vector3(36, -64, 264);
    CHECK(dot(v1, v2) == Approx(-49316));
}

TEST_CASE("Norm of a Vector3", "[Vector3]")
{
    // Case 1
    Vector3 v = Vector3(2, -5, 4);
    CHECK(norm(v) == Approx(6.70820393));
    // Case 2
    v = Vector3(0.24, 0.0082, -0.03);
    CHECK(norm(v) == Approx(0.242007));
    // Case 3
    v = Vector3(-27, 83, -163);
    CHECK(norm(v) == Approx(184.897269));
}

TEST_CASE("Unit Vector3", "[Vector3]")
{
    // Case 1
    Vector3 v = Vector3(2, -5, 4);
    Vector3 n = unit(v);
    CHECK(n.x == Approx(0.298142));
    CHECK(n.y == Approx(-0.745356));
    CHECK(n.z == Approx(0.596285));
    // Case 2
    v = Vector3(0.24, 0.0082, -0.03);
    n = unit(v);
    CHECK(n.x == Approx(0.991708));
    CHECK(n.y == Approx(0.0338834));
    CHECK(n.z == Approx(-0.123964));
    // Case 3
    v = Vector3(-27, 83, -163);
    n = unit(v);
    CHECK(n.x == Approx(-0.146027));
    CHECK(n.y == Approx(0.448898));
    CHECK(n.z == Approx(-0.881571));
}

TEST_CASE("Normalized Vector3", "[Vector3]")
{
    // Case 1
    Vector3 n = Vector3(2, -5, 4);
    normalise(n);
    CHECK(n.x == Approx(0.298142));
    CHECK(n.y == Approx(-0.745356));
    CHECK(n.z == Approx(0.596285));
    // Case 2
    n = Vector3(0.24, 0.0082, -0.03);
    normalise(n);
    CHECK(n.x == Approx(0.991708));
    CHECK(n.y == Approx(0.0338834));
    CHECK(n.z == Approx(-0.123964));
    // Case 3
    n = Vector3(-27, 83, -163);
    normalise(n);
    CHECK(n.x == Approx(-0.146027));
    CHECK(n.y == Approx(0.448898));
    CHECK(n.z == Approx(-0.881571));
}