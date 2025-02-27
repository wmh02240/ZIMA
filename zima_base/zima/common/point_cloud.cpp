/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/common/point_cloud.h"

#include <algorithm>

#include "zima/common/transform.h"
#include "zima/common/util.h"
#include "zima/grid_map/map_2d.h"
#include "zima/logger/logger.h"

namespace zima {

PointCloud::Point::Point(const double& timestamp, const float& polar_distance,
                         const float& polar_degree, const float& intensity)
    : timestamp_(timestamp),
      polar_dis_(polar_distance),
      polar_degree_(NormalizeDegree(polar_degree)),
      intensity_(intensity) {
  cartesian_coordinate_.SetX(polar_dis_ * cos(DegreesToRadians(polar_degree_)));
  cartesian_coordinate_.SetY(polar_dis_ * sin(DegreesToRadians(polar_degree_)));
}

PointCloud::Point::Point(const double& timestamp,
                         const MapPoint& cartesian_coordinate,
                         const float& intensity)
    : timestamp_(timestamp),
      cartesian_coordinate_(cartesian_coordinate),
      intensity_(intensity) {
  polar_dis_ = sqrt(Square(cartesian_coordinate_.X()) +
                    Square(cartesian_coordinate_.Y()));
  if (FloatEqual(cartesian_coordinate_.X(), 0)) {
    polar_degree_ = FloatEqual(cartesian_coordinate_.Y(), 0)
                        ? 0
                        : (cartesian_coordinate_.Y() > 0 ? 90.0 : -90.0);
  } else {
    polar_degree_ = RadiansToDegrees(
        atan(cartesian_coordinate_.Y() / cartesian_coordinate_.X()));
    if (cartesian_coordinate_.X() < 0) {
      polar_degree_ = NormalizeDegree(polar_degree_ + 180);
    }
  }
}

PointCloud::Point& PointCloud::Point::operator=(
    const PointCloud::Point& point) {
  timestamp_ = point.timestamp_;
  polar_dis_ = point.polar_dis_;
  polar_degree_ = point.polar_degree_;
  cartesian_coordinate_ = point.cartesian_coordinate_;
  intensity_ = point.intensity_;
  return *this;
}

std::string PointCloud::Point::DebugString() const {
  std::string str;
  str += "xy(" + FloatToString(X(), 3) + ", " + FloatToString(Y(), 3) +
         "), polar(" + FloatToString(Distance(), 3) + ", " +
         FloatToString(Degree(), 3) + "°)";
  return str;
}

PointCloud::PointCloud(const std::string& name)
    : name_(name),
      timestamp_(0),
      seq_(0),
      scan_time_(0),
      lock_(std::make_shared<ReadWriteLock>()),
      points_({}),
      equal_degree_interval_(true),
      is_ascend_degree_(true),
      filter_distance_(-1) {}

PointCloud::PointCloud(const PointCloud& ref)
    : name_(ref.name_),
      timestamp_(ref.timestamp_),
      seq_(ref.seq_),
      scan_time_(ref.scan_time_),
      lock_(std::make_shared<ReadWriteLock>()),
      points_(ref.points_),
      equal_degree_interval_(ref.equal_degree_interval_.load()),
      is_ascend_degree_(ref.is_ascend_degree_.load()),
      filter_distance_(ref.filter_distance_) {}

bool PointCloud::Empty() const {
  if (!lock_->IsLockedByThisThread()) {
    ZWARN << "Access " << name_ << " for empty checking without lock!";
  }
  return points_.empty();
}

size_t PointCloud::Size() const {
  if (!lock_->IsLockedByThisThread()) {
    ZWARN << "Access " << name_ << " points size without lock!";
  }
  return points_.size();
}

PointCloud::Points& PointCloud::GetPointsRef() {
  if (!lock_->InWriteByThisThread()) {
    ZWARN << "Access " << name_ << " points without lock!";
  }
  return points_;
}

const PointCloud::Points& PointCloud::GetPointsConstRef() const {
  if (!lock_->IsLockedByThisThread()) {
    ZWARN << "Access " << name_ << " points without lock!";
  }
  return points_;
}

bool PointCloud::FilterFromPointCloud(const PointCloud& source_point_cloud,
                                      const float& filter_distance) {
  ReadLocker source_lock(source_point_cloud.GetLock());
  if (source_point_cloud.Size() < 2) {
    ZWARN << "Insufficient source point cloud size: "
          << source_point_cloud.Size() << ".";
    return false;
  }
  auto source_points = source_point_cloud.GetPointsConstRef();
  source_lock.Unlock();

  WriteLocker lock(lock_);
  filter_distance_ = filter_distance;
  auto first_point = source_points.front();
  auto& _pc = GetPointsRef();
  _pc.clear();
  _pc.emplace_back(first_point);
  auto last_point = first_point;
  for (auto&& point : source_points) {
    if (fabs(point.X() - last_point.X()) > filter_distance_ ||
        fabs(point.Y() - last_point.Y()) > filter_distance_) {
      last_point = point;
      _pc.emplace_back(point);
    }
  }

  // ZINFO << "Filter from " << source_points.size() << " to " << _pc.size()
  //       << " points.";
  return true;
}

void PointCloud::SetTimeStamp(const double& time) {
  WriteLocker lock(lock_, false);
  if (!lock_->InWriteByThisThread()) {
    lock.Lock();
  }
  timestamp_ = time;
}

double PointCloud::GetTimeStamp() const {
  ReadLocker lock(lock_, false);
  if (!lock_->IsLockedByThisThread()) {
    lock.Lock();
  }
  return timestamp_;
}

void PointCloud::SetSeq(const uint32_t& seq) {
  WriteLocker lock(lock_, false);
  if (!lock_->InWriteByThisThread()) {
    lock.Lock();
  }
  seq_ = seq;
}

uint32_t PointCloud::GetSeq() const {
  ReadLocker lock(lock_, false);
  if (!lock_->IsLockedByThisThread()) {
    lock.Lock();
  }
  return seq_;
}

void PointCloud::SetScanTime(const float& time) {
  WriteLocker lock(lock_, false);
  if (!lock_->InWriteByThisThread()) {
    lock.Lock();
  }
  scan_time_ = time;
}

float PointCloud::GetScanTime() const {
  ReadLocker lock(lock_, false);
  if (!lock_->IsLockedByThisThread()) {
    lock.Lock();
  }
  return scan_time_;
}

void PointCloud::SetFilterDistance(const float& distance) {
  WriteLocker lock(lock_, false);
  if (!lock_->InWriteByThisThread()) {
    lock.Lock();
  }
  filter_distance_ = distance;
}

float PointCloud::GetFilterDistance() const {
  if (!lock_->IsLockedByThisThread()) {
    ZWARN << "Access " << name_ << " filter distance without lock!";
  }
  return filter_distance_;
}

PointCloud::Points PointCloud::GetRangedPoints(const float& start_degree,
                                               const float& end_degree) {
  if (!lock_->IsLockedByThisThread()) {
    ZWARN << "Access " << name_ << " time stamp without lock!";
  }

  Points points;
  if (points_.size() < 2) {
    ZWARN << "Point cloud size " << points_.size()
          << " invalid, stop processing.";
    return points;
  }

  if (fabs(start_degree - end_degree) > 360.0 || start_degree > 360.0 ||
      start_degree < -180.0 || end_degree > 360.0 || end_degree < -180.0) {
    ZERROR << "Invalid range: " << start_degree << " to " << end_degree;
    return points;
  }
  float expend_normalize_range_start;
  float expend_normalize_range_end;

  auto ascending = end_degree > start_degree;
  if (ascending) {
    expend_normalize_range_start = std::max(-180.0, end_degree - 360.0);
    expend_normalize_range_end =
        std::min(360.0, expend_normalize_range_start + 360.0);
  } else {
    expend_normalize_range_start = std::max(-180.0, start_degree - 360.0);
    expend_normalize_range_end =
        std::min(360.0, expend_normalize_range_start + 360.0);
  }

  for (auto&& point : points_) {
    auto _degree = NormalizeDegree(point.Degree(), expend_normalize_range_start,
                                   expend_normalize_range_end);
    if (_degree >= start_degree && _degree <= end_degree) {
      points.emplace_back(point);
    }
  }
  std::sort(points.begin(), points.end(), [&](const Point& a, const Point& b) {
    return ascending
               ? NormalizeDegree(a.Degree(), expend_normalize_range_start,
                                 expend_normalize_range_end) <
                     NormalizeDegree(b.Degree(), expend_normalize_range_start,
                                     expend_normalize_range_end)
               : NormalizeDegree(a.Degree(), expend_normalize_range_start,
                                 expend_normalize_range_end) >
                     NormalizeDegree(b.Degree(), expend_normalize_range_start,
                                     expend_normalize_range_end);
  });

  return points;
}

void PointCloud::ProcessRangedPoints(const float& start_degree,
                                     const float& end_degree,
                                     ProcessFunc process) {
  if (!lock_->IsLockedByThisThread()) {
    ZWARN << "Access " << name_ << " time stamp without lock!";
  }

  auto points = GetRangedPoints(start_degree, end_degree);
  for (auto&& point : points) {
    process(point);
  }
}

PointCloud::SPtr PointCloud::TransformBA(const std::string& name,
                                         const MapPoint& transform_B_to_A) {
  ReadLocker lock(lock_);

  PointCloud::SPtr pc(new PointCloud(name));
  WriteLocker tmp_lock(pc->GetLock());
  pc->SetTimeStamp(GetTimeStamp());
  pc->SetScanTime(GetScanTime());
  pc->SetSeq(GetSeq());
  pc->SetFilterDistance(GetFilterDistance());
  pc->SetAscendDegree(IsAscendDegree());
  pc->SetEqualDegreeInterval(IsEqualDegreeInterval());
  auto& _pc = pc->GetPointsRef();
  _pc.clear();
  for (auto&& point : GetPointsConstRef()) {
    double x, y;
    Transform::CoordinateTransformationBA(
        point.X(), point.Y(), transform_B_to_A.X(), transform_B_to_A.Y(),
        transform_B_to_A.Radian(), x, y);
    _pc.emplace_back(PointCloud::Point(point.TimeStamp(), MapPoint(x, y),
                                       point.Intensity()));
  }

  return pc;
}

void PointCloud::Print(const float& map_resolution) {
  ReadLocker lock(lock_);
  if (points_.empty()) {
    ZWARN << "Point cloud " << name_ << " is empty.";
    return;
  }

  DynamicMapPointBound bound(
      std::numeric_limits<float>::max(), std::numeric_limits<float>::min(),
      std::numeric_limits<float>::max(), std::numeric_limits<float>::min());
  for (auto&& point : points_) {
    bound.Expend(point.X(), point.Y());
  }

  // + 2 is for making sure it will not overflow.
  auto x_range = static_cast<uint16_t>(
      (bound.GetMax().X() - bound.GetMin().X()) / map_resolution + 2);
  auto y_range = static_cast<uint16_t>(
      (bound.GetMax().Y() - bound.GetMin().Y()) / map_resolution + 2);

  SlamValueGridMap2D::SPtr debug_map(
      new SlamValueGridMap2D("candidate debug map", x_range, y_range,
                             map_resolution));
  // ZINFO << "default:" << std::to_string(debug_map->GetDefaultValue());
  WriteLocker map_lock(debug_map->GetLock());
  for (auto&& point : points_) {
    MapCell cell;
    debug_map->WorldToMap({point.X(), point.Y()}, cell);
    debug_map->SetValue(cell.X(), cell.Y(),
                        SlamValueGridMap2D::GetPredefineMaxObstacleValue() + 1);
  }
  debug_map->SetValue(0, 0, debug_map->GetDefaultValue() - 1);
  debug_map->Print(__FILE__, __FUNCTION__, __LINE__);
}

}  // namespace zima
