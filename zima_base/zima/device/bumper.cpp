/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/common/config.h"
#include "zima/common/time.h"
#include "zima/device/bumper.h"
#include "zima/logger/logger.h"

namespace zima {

Bumper::Config::Config(const float& map_resolution,
                       const uint& robot_cell_width,
                       const float& chassis_radius, const JsonSPtr& json) {
  // Load default setting.
  cover_range_min_degree_ = 0;
  cover_range_max_degree_ = 0;
  trace_path_movement_mark_point_ = MapPoint();
  left_encircle_obstacle_movement_mark_point_ = MapPoint();
  right_encircle_obstacle_movement_mark_point_ = MapPoint();

  // Override if json config is provided.
  if (json != nullptr) {
    // ZINFO << "Load config from json.";
    config_valid_ =
        ParseFromJson(map_resolution, robot_cell_width, chassis_radius, json);
    // ZINFO << "Bumper Tf: " << tf_to_base_.DebugString();
    // ZINFO << "tp: " << trace_path_movement_mark_point_.DebugString();
    // ZINFO << "left: "
    //       << left_encircle_obstacle_movement_mark_point_.DebugString();
    // ZINFO << "right: "
    //       << right_encircle_obstacle_movement_mark_point_.DebugString();
  } else {
    config_valid_ = false;
  }
}

bool Bumper::Config::ParseFromJson(const float& map_resolution,
                                   const uint& robot_cell_width,
                                   const float& chassis_radius,
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

  if (!JsonHelper::GetFloat(*json, kCoverRangeMinDegreeKey_,
                            cover_range_min_degree_)) {
    ZERROR << "Config " << kCoverRangeMinDegreeKey_ << " not found.";
    return false;
  }
  if (!JsonHelper::GetFloat(*json, kCoverRangeMaxDegreeKey_,
                            cover_range_max_degree_)) {
    ZERROR << "Config " << kCoverRangeMaxDegreeKey_ << " not found.";
    return false;
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
  if (!JsonHelper::GetArray(*json, kLeftEncircleObstacleMovementMarkPointKey_,
                            *array)) {
    if (!config_has_degree_setting) {
      ZERROR << "Config " << kLeftEncircleObstacleMovementMarkPointKey_
             << " not found.";
      return false;
    }
    // Calculate tf by install degree.
    if (install_degree_ > 0) {
      left_encircle_obstacle_movement_mark_point_ =
          MapPoint(y_direction_mark_distance *
                       tan(DegreesToRadians(
                           90.0f - Minimum(cover_range_max_degree_, 60.0f))),
                   y_direction_mark_distance);
    } else {
      left_encircle_obstacle_movement_mark_point_ =
          MapPoint(x_direction_mark_distance,
                   x_direction_mark_distance *
                       tan(DegreesToRadians(Maximum(cover_range_max_degree_, 30.0f))));
    }
  } else {
    if (!left_encircle_obstacle_movement_mark_point_.ParseFromJson(array)) {
      ZERROR << "Config " << kLeftEncircleObstacleMovementMarkPointKey_
             << " invalid.";
      return false;
    }
  }
  if (!JsonHelper::GetArray(*json, kRightEncircleObstacleMovementMarkPointKey_,
                            *array)) {
    if (!config_has_degree_setting) {
      ZERROR << "Config " << kRightEncircleObstacleMovementMarkPointKey_
             << " not found.";
      return false;
    }
    // Calculate tf by install degree.
    if (install_degree_ < 0) {
      right_encircle_obstacle_movement_mark_point_ = MapPoint(
          y_direction_mark_distance *
              tan(DegreesToRadians((Maximum(cover_range_min_degree_, -60.0f)) -
                                   (-90.0f))),
          -y_direction_mark_distance);
    } else {
      right_encircle_obstacle_movement_mark_point_ = MapPoint(
          x_direction_mark_distance,
          x_direction_mark_distance * tan(DegreesToRadians(Maximum(
                                          cover_range_min_degree_, -30.0f))));
    }
  } else {
    if (!right_encircle_obstacle_movement_mark_point_.ParseFromJson(array)) {
      ZERROR << "Config " << kRightEncircleObstacleMovementMarkPointKey_
             << " invalid.";
      return false;
    }
  }

  return true;
}

Bumper::Bumper(const std::string name, const Config& config)
    : DeviceBase(name, config),
      lock_(std::make_shared<ReadWriteLock>()),
      last_triggered_state_(false),
      real_time_triggered_state_(false),
      triggered_event_(false),
      data_timestamp_(0),
      cover_range_min_degree_(config.cover_range_min_degree_),
      cover_range_max_degree_(config.cover_range_max_degree_),
      trace_path_movement_mark_point_(config.trace_path_movement_mark_point_),
      left_encircle_obstacle_movement_mark_point_(
          config.left_encircle_obstacle_movement_mark_point_),
      right_encircle_obstacle_movement_mark_point_(
          config.right_encircle_obstacle_movement_mark_point_) {}

void Bumper::UpdateRealtimeTriggeredState(const bool& triggered) {
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

double Bumper::GetDataTimeStamp() const {
  ReadLocker lock(lock_);
  return data_timestamp_;
}

bool Bumper::CheckFresh(const double& limit) const {
  ReadLocker lock(lock_);
  if (Time::Now() - data_timestamp_ < limit) {
    return true;
  }
  return false;
}

}  // namespace zima
