/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_VECTOR2_H
#define ZIMA_VECTOR2_H

#include <cstdio>
#include <ostream>
#include "zima/common/maths.h"

namespace zima {

template <typename T>
class Vector2 {
 public:
  Vector2() {
    values_[0] = 0;
    values_[1] = 0;
  }

  Vector2(T x, T y) {
    values_[0] = x;
    values_[1] = y;
  }

  inline const T& X() const { return values_[0]; }

  inline void X(const T& x) { values_[0] = x; }

  inline const T& Y() const { return values_[1]; }

  inline void Y(const T& y) { values_[1] = y; }

  inline void MakeFloor(const Vector2& r_other) {
    if (r_other.values_[0] < values_[0]) values_[0] = r_other.values_[0];
    if (r_other.values_[1] < values_[1]) values_[1] = r_other.values_[1];
  }

  inline void MakeCeil(const Vector2& r_other) {
    if (r_other.values_[0] > values_[0]) values_[0] = r_other.values_[0];
    if (r_other.values_[1] > values_[1]) values_[1] = r_other.values_[1];
  }

  inline double SquaredLength() const {
    return Square(values_[0]) + Square(values_[1]);
  }

  inline double Length() const { return sqrt(SquaredLength()); }

  inline double SquaredDistance(const Vector2& r_other) const {
    return (*this - r_other).SquaredLength();
  }

  inline double Distance(const Vector2& r_other) const {
    return sqrt(SquaredDistance(r_other));
  }

  inline Vector2 operator=(const Vector2& r_other) {
    values_[0] = r_other.values_[0];
    values_[1] = r_other.values_[1];
    return *this;
  }

  inline void operator+=(const Vector2& r_other) {
    values_[0] += r_other.values_[0];
    values_[1] += r_other.values_[1];
  }

  inline void operator-=(const Vector2& r_other) {
    values_[0] -= r_other.values_[0];
    values_[1] -= r_other.values_[1];
  }

  inline const Vector2 operator+(const Vector2& r_other) const {
    return Vector2(values_[0] + r_other.values_[0],
                   values_[1] + r_other.values_[1]);
  }

  inline const Vector2 operator-(const Vector2& r_other) const {
    return Vector2(values_[0] - r_other.values_[0],
                   values_[1] - r_other.values_[1]);
  }

  inline void operator/=(T scalar) {
    values_[0] /= scalar;
    values_[1] /= scalar;
  }

  inline const Vector2 operator/(T scalar) const {
    return Vector2(values_[0] / scalar, values_[1] / scalar);
  }

  inline double operator*(const Vector2& r_other) const {
    return values_[0] * r_other.values_[0] + values_[1] * r_other.values_[1];
  }

  inline const Vector2 operator*(T scalar) const {
    return Vector2(values_[0] * scalar, values_[1] * scalar);
  }

  inline const Vector2 operator-(T scalar) const {
    return Vector2(values_[0] - scalar, values_[1] - scalar);
  }

  inline void operator*=(T scalar) {
    values_[0] *= scalar;
    values_[1] *= scalar;
  }

  inline bool operator==(const Vector2& r_other) const {
    return (values_[0] == r_other.values_[0] &&
            values_[1] == r_other.values_[1]);
  }

  inline bool operator!=(const Vector2& r_other) const {
    return (values_[0] != r_other.values_[0] ||
            values_[1] != r_other.values_[1]);
  }

  inline bool operator<(const Vector2& r_other) const {
    if (values_[0] < r_other.values_[0])
      return true;
    else if (values_[0] > r_other.values_[0])
      return false;
    else
      return (values_[1] < r_other.values_[1]);
  }

  friend inline std::ostream& operator<<(std::ostream& r_stream,
                                         const Vector2& r_vector) {
    r_stream << "(" << r_vector.X() << ", " << r_vector.Y() << "),";
    return r_stream;
  }

 private:
  T values_[2];
};  // Vector2<T>

}  // namespace zima

#endif  // ZIMA_VECTOR2_H
