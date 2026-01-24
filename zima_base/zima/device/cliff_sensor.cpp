/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/device/cliff_sensor.h"

#include "zima/common/config.h"
#include "zima/common/time.h"
#include "zima/logger/logger.h"

namespace zima {

CliffSensor::Config::Config(const float& map_resolution,
                            const uint& robot_cell_width,
                            const float& chassis_radius, const JsonSPtr& json) {
  // Load default setting.
  trace_path_movement_mark_point_ = MapPoint();
  encircle_obstacle_movement_mark_point_ = MapPoint();

  // Override if json config is provided.
  if (json != nullptr) {
    // ZINFO << "Load config from json.";
    config_valid_ =
        ParseFromJson(map_resolution, robot_cell_width, chassis_radius, json);
    // ZINFO << "CliffSensor Tf: " << tf_to_base_.DebugString();
    // ZINFO << "tp: " << trace_path_movement_mark_point_.DebugString();
    // ZINFO << "left: "
    //       << left_encircle_obstacle_movement_mark_point_.DebugString();
    // ZINFO << "right: "
    //       << right_encircle_obstacle_movement_mark_point_.DebugString();
  } else {
    config_valid_ = false;
  }
}

bool CliffSensor::Config::ParseFromJson(const float& map_resolution,
                                        const uint& robot_cell_width,
                                        const float& chassis_radius,
                                        const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }

  // Override.
  if (!JsonHelper::GetFloat(*json, kTriggerDistanceValueKey_,
                            trigger_distance_value_)) {
    ZERROR << "Config " << kTriggerDistanceValueKey_ << " not found.";
    return false;
  }

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

  auto x_direction_mark_distance = (robot_cell_width + 1) * map_resolution;
  auto y_direction_mark_distance = (robot_cell_width + 1) * map_resolution;
  JsonSPtr array(new Json());
  if (!JsonHelper::GetArray(*json, kTracePathMovementMarkPointKey_, *array)) {
    if (!config_has_degree_setting) {
      ZERROR << "Config " << kTracePathMovementMarkPointKey_ << " not found.";
      return false;
    }
    // Calculate tf by install degree.
    // Always mark as front obstacle for trace path movement (unless it was a
    // sensor behind wheels).
    trace_path_movement_mark_point_ = MapPoint(
        x_direction_mark_distance,
        x_direction_mark_distance *
            tan(DegreesToRadians(Clip(tf_to_base_.Degree(), -30.0f, 30.0f))));
  } else {
    if (!trace_path_movement_mark_point_.ParseFromJson(array)) {
      ZERROR << "Config " << kTracePathMovementMarkPointKey_ << " invalid.";
      return false;
    }
  }
  if (!JsonHelper::GetArray(*json, kEncircleObstacleMovementMarkPointKey_,
                            *array)) {
    if (!config_has_degree_setting) {
      ZERROR << "Config " << kEncircleObstacleMovementMarkPointKey_
             << " not found.";
      return false;
    }
    // Calculate tf by install degree.
    if (install_degree_ > 45) {
      encircle_obstacle_movement_mark_point_ = MapPoint(
          y_direction_mark_distance / tan(DegreesToRadians(install_degree_)),
          y_direction_mark_distance);
    } else if (install_degree_ < -45) {
      encircle_obstacle_movement_mark_point_ = MapPoint(
          -y_direction_mark_distance / tan(DegreesToRadians(install_degree_)),
          -y_direction_mark_distance);
    } else if (FloatEqual(install_degree_, 0)) {
      encircle_obstacle_movement_mark_point_ =
          MapPoint(x_direction_mark_distance, 0);
    } else {
      encircle_obstacle_movement_mark_point_ = MapPoint(
          x_direction_mark_distance,
          x_direction_mark_distance * tan(DegreesToRadians(install_degree_)));
    }
  } else {
    if (!encircle_obstacle_movement_mark_point_.ParseFromJson(array)) {
      ZERROR << "Config " << kEncircleObstacleMovementMarkPointKey_
             << " invalid.";
      return false;
    }
  }

  return true;
}

CliffSensor::CliffSensor(const std::string name, const Config& config)
    : DeviceBase(name, config),
      lock_(std::make_shared<ReadWriteLock>()),
      trigger_distance_value_(config.trigger_distance_value_),
      last_triggered_state_(false),
      real_time_triggered_state_(false),
      triggered_event_(false),
      data_timestamp_(0),
      trace_path_movement_mark_point_(config.trace_path_movement_mark_point_),
      encircle_obstacle_movement_mark_point_(
          config.encircle_obstacle_movement_mark_point_) {}

void CliffSensor::UpdateRealtimeDistanceValue(const float& value) {
  distance_value_ = value;

  auto triggered = distance_value_ > trigger_distance_value_;
  real_time_triggered_state_.store(triggered);
  if (triggered) {
    if (!last_triggered_state_) {
      ZGINFO << name_ << " Triggered";
    }
    triggered_event_.store(true);
  } else {
    if (last_triggered_state_) {
      ZGINFO << name_ << " Released";
    }
  }
  WriteLocker lock(lock_);
  data_timestamp_ = Time::Now();
  last_triggered_state_ = triggered;
}

float CliffSensor::GetDistanceValue() const {
  ReadLocker lock(lock_);
  return distance_value_;
}

double CliffSensor::GetDataTimeStamp() const {
  ReadLocker lock(lock_);
  return data_timestamp_;
}

bool CliffSensor::CheckFresh(const double& limit) const {
  ReadLocker lock(lock_);
  if (Time::Now() - data_timestamp_ < limit) {
    return true;
  }
  return false;
}

}  // namespace zima
