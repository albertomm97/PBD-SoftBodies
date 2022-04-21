#ifndef QUATERNION_F_H
#define QUATERNION_F_H

#include <Std/CMath.h>

#include <Eigen/Dense>

namespace Eigen
{
}

namespace utad
{
using Quaternionf = Eigen::Quaternionf;

inline Quaternionf operator*(const Quaternionf &lhs, float rhs)
{
    return Quaternionf(lhs.coeffs() * rhs);
}

inline Quaternionf operator*(float lhs, const Quaternionf &rhs)
{
    return Quaternionf(rhs.coeffs() * lhs);
}

inline Quaternionf operator+(const Quaternionf &lhs, const Quaternionf &rhs)
{
    return Quaternionf(lhs.coeffs() + rhs.coeffs());
}

inline void operator+=(Quaternionf &lhs, const Quaternionf &rhs)
{
    lhs.coeffs() += rhs.coeffs();
}
}  // namespace utad

#endif