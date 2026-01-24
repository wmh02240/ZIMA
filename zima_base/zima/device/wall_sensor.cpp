/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/device/wall_sensor.h"

#include "zima/common/config.h"
#include "zima/common/time.h"
#include "zima/logger/logger.h"

namespace zima {
WallSensor::Config::Config(const float& chassis_radius, const JsonSPtr& json) {
  // Load default setting.
  install_degree_ = 0;
  mark_distance_on_y_ = 0;
  mark_point_ = MapPoint();

  // Override if json config is provided.
  if (json != nullptr) {
    // ZINFO << "Load config from json.";
    config_valid_ = ParseFromJson(chassis_radius, json);
    // ZINFO << "Wall sensor tf " << tf_to_base_.DebugString();
    // ZINFO << "mark point " << mark_point_.DebugString();
  } else {
    config_valid_ = false;
  }
}

bool WallSensor::Config::ParseFromJson(const float& chassis_radius,
                                       const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }

  // Override.
  bool config_has_degree_setting =
      JsonHelper::GetFloat(*json, kInstallDegreeKey_, install_degree_);

  if (!DeviceBase::Config::ParseFromJson(json)) {
    if (!config_has_degree_setting) {
      ZERROR << "Config " << kInstallDegreeKey_ << " and " << kTfToBaseKey_
             << " not found.";
      return false;
    }
    // Calculate tf by install degree.
    tf_to_base_ =
        MapPoint(chassis_radius * cos(DegreesToRadians(install_degree_)),
                 chassis_radius * sin(DegreesToRadians(install_degree_)),
                 install_degree_);
  }

  if (tf_to_base_.Degree() > 90 || tf_to_base_.Degree() < -90) {
    ZERROR << "Install degree invalid, should be -90~90.";
    return false;
  }

  bool config_has_mark_distance_setting =
      JsonHelper::GetFloat(*json, kMarkDistanceOnYKey_, mark_distance_on_y_);

  JsonSPtr array(new Json());
  if (!JsonHelper::GetArray(*json, kMarkPointKey_, *array)) {
    if (!config_has_mark_distance_setting) {
      ZERROR << "Config " << kMarkPointKey_ << " and " << kMarkDistanceOnYKey_
             << " not found.";
      return false;
    }
    // Calculate tf by mark distance.
    auto y_mark_distance = chassis_radius + mark_distance_on_y_;
    if (tf_to_base_.Degree() > 0) {
      auto degree = Maximum(45.0f, tf_to_base_.Degree() - 10.0f);
      mark_point_ =
          MapPoint(y_mark_distance * tan(DegreesToRadians(90 - degree)),
                   y_mark_distance, degree);
    } else {
      auto degree = Minimum(-45.0f, tf_to_base_.Degree() + 10.0f);
      mark_point_ =
          MapPoint(y_mark_distance * tan(DegreesToRadians(degree - (-90))),
                   -y_mark_distance, degree);
    }
  } else {
    if (!mark_point_.ParseFromJson(array)) {
      ZERROR << "Config " << kMarkPointKey_ << " invalid.";
      return false;
    }
  }

  return true;
}

double WallSensor::GetDataTimeStamp() const {
  ReadLocker lock(lock_);
  return data_timestamp_;
}

bool WallSensor::CheckFresh(const double& limit) const {
  ReadLocker lock(lock_);
  if (Time::Now() - data_timestamp_ < limit) {
    return true;
  }
  return false;
}

}  // namespace zima
