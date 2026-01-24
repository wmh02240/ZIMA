/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/common/maths.h"

#include "zima/common/util.h"
#include "zima/logger/logger.h"

namespace zima {

double NormalizeRadian(const double& radian, const float& range_start,
                       const float& range_end) {
  if (range_start > range_end || !FloatEqual(range_end - range_start, _2PI)) {
    ZERROR << "Invalid range: " << range_start << " to " << range_end;
    return radian;
  }

  auto _radian = radian;
  while (_radian < range_start) {
    _radian += _2PI;
  }

  while (_radian >= range_end) {
    _radian -= _2PI;
  }

  return _radian;
}

double NormalizeRadian(const double& radian) {
  return NormalizeRadian(radian, -_PI, _PI);
}

float NormalizeDegree(const float& degree, const float& range_start,
                      const float& range_end) {
  if (range_start > range_end || !FloatEqual(range_end - range_start, 360.0)) {
    ZERROR << "Invalid range: " << range_start << " to " << range_end;
    return degree;
  }

  auto _degree = degree;
  while (_degree < range_start) {
    _degree += 360.0;
  }

  while (_degree >= range_end) {
    _degree -= 360.0;
  }

  return _degree;
}

float NormalizeDegree(const float& degree) {
  return NormalizeDegree(degree, -180.0, 180.0);
}

Line::Line(const float& a, const float& b, const float& c) {
  a_ = a;
  b_ = b;
  c_ = c;
  valid_ = !FloatEqual(a_, 0) || !FloatEqual(b_, 0);
}

// Line::Line(const float& x, const float& y, const float& angle) {
//   auto _angle = NormalizeDegree(angle + 90);
//   float A, B;
//   if (DoubleEqual(_angle, 90) || DoubleEqual(_angle, -90)) {
//     A = 0;
//     B = 1;
//   } else {
//     A = 1;
//     B = atan(DegreesToRadians(_angle));
//   }

//   a_ = A;
//   b_ = B;
//   c_ = -(A * x + B * y);
// }

Line::Line(const float& x1, const float& y1, const float& x2, const float& y2) {
  a_ = y2 - y1;
  b_ = x1 - x2;
  c_ = y1 * (x2 - x1) - x1 * (y2 - y1);
  valid_ = !FloatEqual(a_, 0) || !FloatEqual(b_, 0);
}

bool Line::GetX(const float& y, float& x) {
  if (FloatEqual(a_, 0)) {
    return false;
  }
  x = -1 * b_ * y / a_ - c_ / a_;
  return true;
}

bool Line::GetY(const float& x, float& y) {
  if (FloatEqual(b_, 0)) {
    return false;
  }
  y = -1 * a_ * x / b_ - c_ / b_;

  return true;
}

std::string Line::DebugString(const uint8_t& precision) const {
  std::string msg = FloatToString(a_, precision) + "x + " +
                    FloatToString(b_, precision) + "y + " +
                    FloatToString(c_, precision) + " = 0";
  if (FloatEqual(b_, 0)) {
    msg += "(x = " + FloatToString(-1 * c_ / a_, precision) + ")";
  } else {
    msg += "(y = ";
    if (FloatEqual(a_, 0)) {
      msg += FloatToString(-1 * c_ / b_, precision) + ")";
    } else {
      msg += FloatToString(-1 * a_ / b_, precision) + "x + " +
             FloatToString(-1 * c_ / b_, precision) + ")";
    }
  }
  return msg;
}

LineSegment::LineSegment(const float& x1, const float& y1, const float& x2,
                         const float& y2)
    : Line(x1, y1, x2, y2), x1_(x1), y1_(y1), x2_(x2), y2_(y2) {}

double LineSegment::Length() const {
  return sqrt(Square(x1_ - x2_) + Square(y1_ - y2_));
}

std::string LineSegment::DebugString(const uint8_t& precision) const {
  return Line::DebugString(precision) + ", (" + FloatToString(x1_, precision) +
         ", " + FloatToString(y1_, precision) + ")->(" +
         FloatToString(x2_, precision) + ", " + FloatToString(y2_, precision) +
         ")";
}

}  // namespace zima
