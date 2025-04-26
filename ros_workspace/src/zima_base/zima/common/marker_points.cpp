/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/common/marker_points.h"

namespace zima {

std::string MarkerPoint::DebugString() const {
  std::string str;
  str.push_back(value_);
  str += ":" + point_.DebugString();
  return str;
}

StepPoint& StepPoint::operator=(const StepPoint& ref) {
  if (&ref == this) {
    return *this;
  }
  this->pose_ = ref.pose_;
  if (ref.marker_points_ == nullptr) {
    this->marker_points_ = nullptr;
  } else {
    this->marker_points_.reset(new MarkerPoints(*ref.marker_points_));
  }
  this->type_ = ref.type_;

  return *this;
}

std::string StepPoint::DebugString() const {
  std::string str;
  str += "Pose:" + pose_.DebugString() + "mark{";
  if (marker_points_ != nullptr) {
    for (auto&& marker_point : *marker_points_) {
      str += marker_point.DebugString();
    }
  }
  str += "}";
  return str;
}

Steps StepPoint::GenerateStepsBetweenTwoSteps(CharGridMap2D::SPtr map,
                                              const StepPoint& a,
                                              const StepPoint& b) {
  MapCell a_cell, b_cell;
  map->WorldToMap(a.Pose(), a_cell);
  map->WorldToMap(b.Pose(), b_cell);
  MapCells cells = map->GenerateCellsBetweenTwoCells(a_cell, b_cell);

  float degree = 0;
  if (DoubleEqual(a.Pose().X(), b.Pose().X())) {
    if (a.Pose().Y() > b.Pose().Y()) {
      degree = -90;
    } else {
      degree = 90;
    }
  } else {
    degree = RadiansToDegrees(
        atan((b.Pose().Y() - a.Pose().Y()) / (b.Pose().X() - a.Pose().X())));
    if (b.Pose().X() - a.Pose().X() < 0) {
      degree = NormalizeDegree(degree + 180);
    }
  }
  // ZINFO << "Point degree: " << degree;

  Steps steps;
  for (auto&& cell : cells) {
    MapPoint pose(0, 0, degree);
    map->MapToWorld(cell, pose);
    steps.emplace_back(StepPoint(pose));
  }

  if (!steps.empty()) {
    auto step = StepPoint(steps.back().Pose(), b.Markers());
    steps.pop_back();
    steps.emplace_back(step);
  }

  return steps;
};

}  // namespace zima
