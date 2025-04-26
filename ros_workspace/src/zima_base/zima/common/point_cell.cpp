/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/common/point_cell.h"

#include "zima/logger/logger.h"

namespace zima {

bool MapCell::ParseFromJson(const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }
  if (!json->is_array()) {
    ZERROR << "Json type not array, json: " << json->dump();
    return false;
  }
  if (json->size() != 2) {
    ZERROR << "Array size not 2, json: " << json->dump();
    return false;
  }
  if (!json->at(0).is_number_integer() || !json->at(1).is_number_integer()) {
    ZERROR << "Array data type not integer, json: " << json->dump();
    return false;
  }

  SetX(json->at(0));
  SetY(json->at(1));

  return true;
}

bool MapPoint::ParseFromJson(const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }
  if (!json->is_array()) {
    ZERROR << "Json type not array, json: " << json->dump();
    return false;
  }
  if (json->size() != 2 && json->size() != 3) {
    ZERROR << "Array size not 2 or 3, json: " << json->dump();
    return false;
  }
  if (!json->at(0).is_number() || !json->at(1).is_number() ||
      (json->size() == 3 && !json->at(2).is_number())) {
    ZERROR << "Array data type not number, json: " << json->dump();
    return false;
  }

  SetX(json->at(0));
  SetY(json->at(1));
  if (json->size() == 3) {
    SetDegree(json->at(2));
  }

  return true;
}

MapPoint MapPoint::GetVector(const MapPoint& source, const MapPoint& target) {
  MapPoint ret;
  ret.SetX(target.X() - source.X()).SetY(target.Y() - source.Y());
  float degree;
  if (DoubleEqual(ret.X(), 0)) {
    degree = ret.Y() > 0 ? 90.0 : (ret.Y() < 0 ? -90.0 : source.Degree());
  } else {
    degree = RadiansToDegrees(atan2(ret.Y(), ret.X()));
  }
  degree = NormalizeDegree(degree);

  ret.SetDegree(degree);

  return ret;
}

TimedMapPoint::TimedMapPoint() : MapPoint(), timestamp_(0) {}

TimedMapPoint::TimedMapPoint(const double& timestamp) : timestamp_(timestamp) {}
TimedMapPoint::TimedMapPoint(const double& timestamp, const MapPoint& point)
    : MapPoint(point), timestamp_(timestamp) {}

double TimedMapPoint::GetTimestamp() const { return timestamp_; }

void TimedMapPoint::SetTimestamp(const double& timestamp) {
  timestamp_ = timestamp;
}

MapPoints GenerateInterpolationPoints(const MapPoint& a, const MapPoint& b,
                                      const float& interval) {
  MapPoints points;
  points.emplace_back(a);
  if (interval < 0) {
    ZWARN << "Interval invalid: " << interval << ", should be larger than 0.";
    points.emplace_back(b);
    return points;
  }

  static const float kIntervalMin = 0.01;
  if (interval < kIntervalMin) {
    ZWARN << "Interval too small: " << interval << ", should be larger than"
          << FloatToString(kIntervalMin, 3) << ".";
    points.emplace_back(b);
    return points;
  }
  auto vector = MapPoint::GetVector(a, b);
  auto interval_vector = MapPoint(interval * cos(vector.Radian()),
                                  interval * sin(vector.Radian()));
  auto interval_vector_length = interval_vector.Length();
  auto multiple = vector.Length() / interval_vector_length;
  static const float kMultipleMax = 10000;
  if (multiple > kMultipleMax) {
    ZWARN << "Multiple too large: " << multiple
          << ", should be smaller than: " << FloatToString(kMultipleMax, 1)
          << ".";
    points.emplace_back(b);
    return points;
  }

  auto interpolate_count = static_cast<int32_t>(floor(multiple));
  // ZINFO << "Interpolate for " << interpolate_count << " points.";
  while (interpolate_count > 0) {
    points.emplace_back(MapPoint(points.back().X() + interval_vector.X(),
                                 points.back().Y() + interval_vector.Y(),
                                 vector.Degree()));
    interpolate_count--;
  }
  points.emplace_back(b);
  return points;
}

}  // namespace zima
