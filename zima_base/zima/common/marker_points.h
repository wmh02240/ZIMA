/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_MARKER_POINTS_H
#define ZIMA_MARKER_POINTS_H

#include <deque>
#include <list>

#include "zima/common/point_cell.h"
#include "zima/grid_map/map_2d.h"
#include "zima/logger/logger.h"

namespace zima {

class MarkerPoint {
 public:
  MarkerPoint() = delete;
  MarkerPoint(const MapPoint& point, const CharGridMap2D::DataType& value)
      : point_(point), value_(value) {}
  ~MarkerPoint() = default;

  const MapPoint& Point() const { return point_; }
  const CharGridMap2D::DataType& Value() const { return value_; }

  std::string DebugString() const;

 private:
  MapPoint point_;
  CharGridMap2D::DataType value_;
};

using MarkerPoints = std::deque<MarkerPoint>;
using MarkerPointsSPtr = std::shared_ptr<MarkerPoints>;

class StepPoint;
using Steps = std::list<StepPoint>;
class StepPoint {
 public:
  enum Type {
    kNormalStep,
    kPauseStep,
  };

  StepPoint() = delete;
  StepPoint(const MapPoint& pose, const MarkerPoints& marker_points,
            const Type& type = kNormalStep)
      : pose_(pose), marker_points_(nullptr), type_(type) {
    if (!marker_points.empty()) {
      marker_points_.reset(new MarkerPoints(marker_points));
    }
  };
  StepPoint(const MapPoint& pose, const MarkerPointsSPtr& marker_points = nullptr,
            const Type& type = kNormalStep)
      : pose_(pose), marker_points_(marker_points), type_(type){};
  ~StepPoint() = default;

  StepPoint& operator=(const StepPoint& ref);

  const MapPoint& Pose() const { return pose_; }
  const MarkerPointsSPtr& Markers() const { return marker_points_; }
  Type GetType() const { return type_; }

  std::string DebugString() const;

  static Steps GenerateStepsBetweenTwoSteps(CharGridMap2D::SPtr map,
                                            const StepPoint& a,
                                            const StepPoint& b);

 private:
  MapPoint pose_;
  MarkerPointsSPtr marker_points_;
  Type type_;
};

}  // namespace zima

#endif  // ZIMA_MARKER_POINTS_H
