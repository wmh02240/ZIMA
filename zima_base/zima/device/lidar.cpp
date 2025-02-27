/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/device/lidar.h"

#include "zima/common/config.h"
#include "zima/common/time.h"
#include "zima/common/transform.h"
#include "zima/logger/logger.h"

namespace zima {

Lidar::Config::Config() : Config(nullptr) {}

Lidar::Config::Config(const JsonSPtr& json) {
  min_range_ = 0.1;
  max_range_ = 8;
  filter_distance_ = 0.04;

  // Override if json config is provided.
  if (json != nullptr) {
    // ZINFO << "Load config from json.";
    config_valid_ = ParseFromJson(json);
  } else {
    config_valid_ = false;
  }
}

bool Lidar::Config::ParseFromJson(const JsonSPtr& json) {
  // TODO(Austin): Support config for blocking range.
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }

  if (!DeviceBase::Config::ParseFromJson(json)) {
    return false;
  }

  // Override.
  if (!JsonHelper::GetFloat(*json, kMaxRangeKey_, max_range_)) {
    ZGERROR << "Config " << kMaxRangeKey_ << " not found.";
    // return false;
  }
  if (!JsonHelper::GetFloat(*json, kMinRangeKey_, min_range_)) {
    ZGERROR << "Config " << kMinRangeKey_ << " not found.";
    // return false;
  }
  if (!JsonHelper::GetFloat(*json, kFilterDistanceKey_, filter_distance_)) {
    ZGERROR << "Config " << kFilterDistanceKey_ << " not found.";
    // return false;
  }

  if (!InRange(filter_distance_, 0.f, 0.1f)) {
    ZERROR << "Config " << kFilterDistanceKey_ << ": "
           << FloatToString(filter_distance_, 2) << " invalid.";
    return false;
  }

  if (!InRange(min_range_, 0.f, max_range_)) {
    ZERROR << "Config " << kMinRangeKey_ << ": " << FloatToString(min_range_, 2)
           << " and " << kMaxRangeKey_ << ": " << FloatToString(max_range_, 2)
           << " invalid.";
    return false;
  }

  return true;
}

const std::string Lidar::kPointCloudInLidarFrameName_ = "lidar pc";
const std::string Lidar::kPointCloudInChassisFrameName_ = "chassis pc";

Lidar::Lidar(const std::string name, const Config& config)
    : DeviceBase(name, config),
      lock_(std::make_shared<ReadWriteLock>()),
      config_(config),
      point_cloud_in_lidar_frame_(new PointCloud("Empty point cloud")),
      point_cloud_in_chassis_frame_(new PointCloud("Empty point cloud")),
      max_range_(config.max_range_),
      min_range_(config.min_range_) {}

bool Lidar::HasPointCloud() const {
  ReadLocker lock(lock_);
  return point_cloud_in_lidar_frame_ != nullptr;
}

bool Lidar::GetPointCloudTimeStamp(double& timestamp) const {
  ReadLocker lock(lock_);
  if (!HasPointCloud()) {
    ZWARN << "No point cloud received.";
    return false;
  }
  ReadLocker point_cloud_lock(point_cloud_in_lidar_frame_->GetLock());
  timestamp = point_cloud_in_lidar_frame_->GetTimeStamp();
  return true;
}

bool Lidar::GetPointCloudSeq(uint32_t& seq) const {
  ReadLocker lock(lock_);
  if (!HasPointCloud()) {
    ZWARN << "No point cloud received.";
    return false;
  }
  ReadLocker point_cloud_lock(point_cloud_in_lidar_frame_->GetLock());
  seq = point_cloud_in_lidar_frame_->GetSeq();
  return true;
}

bool Lidar::CheckFresh(const double& limit) const {
  double lidar_timestamp;
  if (GetPointCloudTimeStamp(lidar_timestamp) &&
      Time::Now() - lidar_timestamp < limit) {
    return true;
  }
  return false;
}

PointCloud::SPtr Lidar::GetPointCloudInLidarFrame() const {
  ReadLocker lock(lock_);
  return point_cloud_in_lidar_frame_;
}

PointCloud::SPtr Lidar::GetPointCloudInChassisFrame() const {
  ReadLocker lock(lock_);
  return point_cloud_in_chassis_frame_;
}

PointCloud::SPtr Lidar::ConvertToChassisFrame(
    const PointCloud::SPtr& point_cloud_in_lidar_frame) {
  ReadLocker point_cloud_in_lidar_frame_lock(
      point_cloud_in_lidar_frame->GetLock());

  return point_cloud_in_lidar_frame->TransformBA(kPointCloudInChassisFrameName_,
                                                 tf_to_base_);
}

void Lidar::UpdatePointCloudInLidarFrame(
    const PointCloud::SPtr& new_point_cloud) {
  {
    WriteLocker lidar_lock(lock_);
    point_cloud_in_lidar_frame_ = new_point_cloud;
    point_cloud_in_lidar_frame_->FilterFromPointCloud(
        *point_cloud_in_lidar_frame_, config_.filter_distance_);
  }

  auto point_cloud_in_chassis_frame =
      ConvertToChassisFrame(point_cloud_in_lidar_frame_);

  {
    WriteLocker tmp_lock(point_cloud_in_chassis_frame->GetLock());
    point_cloud_in_chassis_frame->SetEqualDegreeInterval(false);
  }
  {
    WriteLocker lidar_lock(lock_);
    point_cloud_in_chassis_frame_ = point_cloud_in_chassis_frame;
  }
}

}  // namespace zima
