/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_BOUNDINGBOX_H
#define ZIMA_BOUNDINGBOX_H

#include "zima/common/vector2.h"

namespace zima {

template <typename T>
class BoundingBox {
 public:
  BoundingBox() = delete;
  BoundingBox(const T& x_min, const T& x_max, const T& y_min, const T& y_max)
      : min_(x_min, y_min), max_(x_max, y_max) {}
  BoundingBox(const BoundingBox& box) {
    min_ = box.min_;
    max_ = box.max_;
  }

  ~BoundingBox() = default;

  BoundingBox& operator=(const BoundingBox& box) {
    min_ = box.min_;
    max_ = box.max_;
    return *this;
  }

  void Reset(const T& x_min, const T& x_max, const T& y_min, const T& y_max) {
    min_.X(x_min);
    min_.Y(y_min);
    max_.X(x_max);
    max_.Y(y_max);
  }

  bool Contain(const Vector2<T>& target) const {
    return Contain(target.X(), target.Y());
  }

  bool Contain(const T& x, const T& y) const {
    return (x >= min_.X() && y >= min_.Y() && x <= max_.X() && y <= max_.Y());
  }

  bool Expend(const Vector2<T>& target) {
    if (target.X() < min_.X()) {
      min_.X(target.X());
    }
    if (target.Y() < min_.Y()) {
      min_.Y(target.Y());
    }
    if (target.X() > max_.X()) {
      max_.X(target.X());
    }
    if (target.Y() > max_.Y()) {
      max_.Y(target.Y());
    }
    return true;
  }

  bool Expend(const T& x, const T& y) { return Expend(Vector2<T>(x, y)); }

  bool EqualShrink(const T& value) {
    if (!IsValid() || value <= 0) {
      return false;
    }
    if (max_.X() - min_.X() < value * 2 || max_.Y() - min_.Y() < value * 2) {
      return false;
    }

    max_.X(max_.X() - value);
    max_.Y(max_.Y() - value);
    min_.X(min_.X() + value);
    min_.Y(min_.Y() + value);

    return true;
  }

  Vector2<T> GetMin() const { return min_; }

  Vector2<T> GetMax() const { return max_; }

  bool IsValid() const { return min_.X() <= max_.X() && min_.Y() <= max_.Y(); }

  std::string DebugString() const {
    std::string str;
    str += "Bound min(" + std::to_string(min_.X()) + ", " +
           std::to_string(min_.Y()) + "), max(" + std::to_string(max_.X()) +
           ", " + std::to_string(max_.Y()) + ")";
    return str;
  }

 protected:
  Vector2<T> min_;
  Vector2<T> max_;
};

}  // namespace zima

#endif  // ZIMA_BOUNDINGBOX_H
