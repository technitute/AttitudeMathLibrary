// ######################################################################### //
// Attitude Math Library
// Copyright (c) 2021 Dr. Steven Dumble Phd
// Distributed under the MIT Software License (See accompanying file LICENSE)
// ######################################################################### //

#ifndef AML_AMLVECTOR3_H
#define AML_AMLVECTOR3_H

#include <iostream>

namespace AML
{

    class Vector3
    {
    public:
        // Public Element Data
        union
        {
            double data[3];
            struct
            {
                double x, y, z;
            };
        };

        // Constructors
        Vector3();
        Vector3(double val);
        Vector3(double x, double y, double z);
        Vector3(const double data[3]);

        // Operator Assignments (Vector)
        Vector3& operator+=(const Vector3& rhs);
        Vector3& operator-=(const Vector3& rhs);
        Vector3& operator*=(const Vector3& rhs);
        Vector3& operator/=(const Vector3& rhs);

        // Operator Assignments (Scalar)
        Vector3& operator+=(double s);
        Vector3& operator-=(double s);
        Vector3& operator*=(double s);
        Vector3& operator/=(double s);

        // Special Object Creators
        static const Vector3 xAxis();
        static const Vector3 yAxis();
        static const Vector3 zAxis();
    };

    // Vector / Vector Elementwise Operations
    Vector3 operator-(const Vector3& rhs);
    Vector3 operator+(const Vector3& lhs, const Vector3& rhs);
    Vector3 operator-(const Vector3& lhs, const Vector3& rhs);
    Vector3 operator*(const Vector3& lhs, const Vector3& rhs);
    Vector3 operator/(const Vector3& lhs, const Vector3& rhs);

    // Vector / Scalar Operations
    Vector3 operator+(const Vector3& lhs, double s);
    Vector3 operator-(const Vector3& lhs, double s);
    Vector3 operator*(const Vector3& lhs, double s);
    Vector3 operator/(const Vector3& lhs, double s);
    Vector3 operator+(double s, const Vector3& rhs);
    Vector3 operator-(double s, const Vector3& rhs);
    Vector3 operator*(double s, const Vector3& rhs);
    Vector3 operator/(double s, const Vector3& rhs);

    // Vector Operations
    double norm(const Vector3& rhs);
    void normalise(Vector3& rhs);
    Vector3 unit(const Vector3& rhs);
    Vector3 cross(const Vector3& lhs, const Vector3& rhs);
    double dot(const Vector3& lhs, const Vector3& rhs);

    // Stream Functions
    std::ostream& operator<<(std::ostream& os, const Vector3& obj);

}; // namespace AML

#endif