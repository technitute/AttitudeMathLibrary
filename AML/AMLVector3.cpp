// ######################################################################### //
// Attitude Math Library
// Copyright (c) 2021 Dr. Steven Dumble Phd
// Distributed under the MIT Software License (See accompanying file LICENSE)
// ######################################################################### //

#include "AMLVector3.hpp"
#include <cmath>

namespace AML
{

    // Constructors
    Vector3::Vector3() : x(0.0), y(0.0), z(0.0) {}

    Vector3::Vector3(double val) : x(val), y(val), z(val) {}

    Vector3::Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vector3::Vector3(const double data_[3]) : x(data_[0]), y(data_[1]), z(data_[2]) {}

    // Operator Assignments (Vector)
    Vector3& Vector3::operator+=(const Vector3& rhs)
    {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }
    Vector3& Vector3::operator-=(const Vector3& rhs)
    {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }
    Vector3& Vector3::operator*=(const Vector3& rhs)
    {
        x *= rhs.x;
        y *= rhs.y;
        z *= rhs.z;
        return *this;
    }
    Vector3& Vector3::operator/=(const Vector3& rhs)
    {
        x /= rhs.x;
        y /= rhs.y;
        z /= rhs.z;
        return *this;
    }

    // Operator Assignments (Scalar)
    Vector3& Vector3::operator+=(double s)
    {
        x += s;
        y += s;
        z += s;
        return *this;
    }
    Vector3& Vector3::operator-=(double s)
    {
        x -= s;
        y -= s;
        z -= s;
        return *this;
    }
    Vector3& Vector3::operator*=(double s)
    {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }
    Vector3& Vector3::operator/=(double s)
    {
        x /= s;
        y /= s;
        z /= s;
        return *this;
    }

    // Special Object Creators
    const Vector3 Vector3::xAxis() { return Vector3(1.0, 0.0, 0.0); }
    const Vector3 Vector3::yAxis() { return Vector3(0.0, 1.0, 0.0); }
    const Vector3 Vector3::zAxis() { return Vector3(0.0, 0.0, 1.0); }

    // Vector / Vector Elementwise Operations
    Vector3 operator-(const Vector3& rhs) { return Vector3(-rhs.x, -rhs.y, -rhs.z); }
    Vector3 operator+(const Vector3& lhs, const Vector3& rhs) { return (Vector3(lhs) += rhs); }
    Vector3 operator-(const Vector3& lhs, const Vector3& rhs) { return (Vector3(lhs) -= rhs); }
    Vector3 operator*(const Vector3& lhs, const Vector3& rhs) { return (Vector3(lhs) *= rhs); }
    Vector3 operator/(const Vector3& lhs, const Vector3& rhs) { return (Vector3(lhs) /= rhs); }

    // Vector / Scalar Operations
    Vector3 operator+(const Vector3& lhs, double s) { return (Vector3(lhs) += s); }
    Vector3 operator-(const Vector3& lhs, double s) { return (Vector3(lhs) -= s); }
    Vector3 operator*(const Vector3& lhs, double s) { return (Vector3(lhs) *= s); }
    Vector3 operator/(const Vector3& lhs, double s) { return (Vector3(lhs) /= s); }
    Vector3 operator+(double s, const Vector3& rhs) { return (Vector3(s) += rhs); }
    Vector3 operator-(double s, const Vector3& rhs) { return (Vector3(s) -= rhs); }
    Vector3 operator*(double s, const Vector3& rhs) { return (Vector3(s) *= rhs); }
    Vector3 operator/(double s, const Vector3& rhs) { return (Vector3(s) /= rhs); }

    // Vector Operations
    double norm(const Vector3& rhs) { return sqrt(rhs.x * rhs.x + rhs.y * rhs.y + rhs.z * rhs.z); }
    double norm2d(const Vector3& rhs) { return sqrt(rhs.x * rhs.x + rhs.y * rhs.y); }
    Vector3 unit(const Vector3& rhs)
    {
        double mag = norm(rhs);
        if (mag > 0.0)
        {
            return (Vector3(rhs) / mag);
        }
        return Vector3(rhs);
    }
    void normalise(Vector3& rhs)
    {
        double mag = norm(rhs);
        if (mag > 0.0)
        {
            rhs /= mag;
        }
    }
    Vector3 cross(const Vector3& lhs, const Vector3& rhs)
    {
        double x = (lhs.y * rhs.z) - (lhs.z * rhs.y);
        double y = (lhs.z * rhs.x) - (lhs.x * rhs.z);
        double z = (lhs.x * rhs.y) - (lhs.y * rhs.x);
        return Vector3(x, y, z);
    }
    double dot(const Vector3& rhs, const Vector3& lhs) { return (rhs.x * lhs.x + rhs.y * lhs.y + rhs.z * lhs.z); }

    // Steam Functions
    std::ostream& operator<<(std::ostream& os, const Vector3& obj)
    {
        os << "[" << obj.x << ", " << obj.y << ", " << obj.z << "]";
        return os;
    }
}; // namespace AML