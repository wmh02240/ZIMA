/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_MATHS_H
#define ZIMA_MATHS_H

#include <cfloat>
#include <cmath>
#include <deque>
#include <memory>
#include <string>

namespace zima {

constexpr double _PI = M_PI;
constexpr double _2PI = 2 * M_PI;
constexpr double _PI_2 = M_PI_2;
constexpr double _PI_180 = M_PI / 180.0;
constexpr double _180_PI = 180.0 / M_PI;

inline double DegreesToRadians(const double &degrees) { return degrees * _PI_180; }

inline double RadiansToDegrees(const double &radians) { return radians * _180_PI; }

inline double Round(const double& value) {
  return value >= 0.0 ? floor(value + 0.5) : ceil(value - 0.5);
}

double NormalizeRadian(const double& radian, const float& range_start,
                       const float& range_end);

double NormalizeRadian(const double& radian);

float NormalizeDegree(const float& degree, const float& range_start,
                      const float& range_end);

float NormalizeDegree(const float& degree);

template <typename T>
inline T Square(const T& value) {
  return (value * value);
}

template <typename T>
inline const T& Minimum(const T& value1, const T& value2) {
  return value1 < value2 ? value1 : value2;
}

template <typename T>
inline const T& Maximum(const T& value1, const T& value2) {
  return value1 > value2 ? value1 : value2;
}

template <typename T>
inline const T& Clip(const T& n, const T& minValue, const T& maxValue) {
  return Minimum(Maximum(n, minValue), maxValue);
}

inline bool DoubleEqual(const double& a, const double& b) {
  return (fabs(a - b) <= DBL_EPSILON);
}

inline bool FloatEqual(const float& a, const float& b) {
  return (fabs(a - b) <= FLT_EPSILON);
}

template <typename T>
inline bool IsUpTo(const T& value, const T& maximum) {
  return (value >= 0 && value < maximum);
}

template <typename T>
inline bool InRange(const T& value, const T& a, const T& b) {
  if (a < b) {
    return (value >= a && value <= b);
  } else {
    return (value >= b && value <= a);
  }
}

class Line {
  // ax + by + c = 0
 public:
  Line(const float& a, const float& b, const float& c);
  Line(const float& x1, const float& y1, const float& x2, const float& y2);
  ~Line() = default;

  using SPtr = std::shared_ptr<Line>;

  float GetA() const { return a_; }
  float GetB() const { return b_; }
  float GetC() const { return c_; }

  bool GetX(const float& y, float& x);
  bool GetY(const float& x, float& y);

  bool IsValid() const { return valid_; }

  std::string DebugString(const uint8_t& precision) const;

 protected:
  Line() = default;
  float a_;
  float b_;
  float c_;

  bool valid_;
};

class LineSegment : public Line {
 public:
  LineSegment() = delete;
  LineSegment(const float& x1, const float& y1, const float& x2,
              const float& y2);
  ~LineSegment() = default;

  using SPtr = std::shared_ptr<LineSegment>;

  float GetX1() const { return x1_; }
  float GetY1() const { return y1_; }
  float GetX2() const { return x2_; }
  float GetY2() const { return y2_; }

  double Length() const;

  std::string DebugString(const uint8_t& precision) const;

 private:
  float x1_;
  float y1_;
  float x2_;
  float y2_;
};

using LineSegments = std::deque<LineSegment>;

}  // namespace zima

#endif  // ZIMA_MATHS_H
