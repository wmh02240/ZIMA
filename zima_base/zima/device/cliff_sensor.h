/*
 * This file is part of Project Zima.
 * Copyright © 2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_CLIFF_SENSOR_H
#define ZIMA_CLIFF_SENSOR_H

#include <atomic>
#include <memory>
#include <string>

#include "zima/common/lock.h"
#include "zima/common/point_cell.h"
#include "zima/device/device.h"

namespace zima {

class CliffSensor : public DeviceBase {
 public:
  class Config : public DeviceBase::Config {
   public:
    Config() = delete;
    Config(const float& map_resolution, const uint& robot_cell_width_2,
           const float& chassis_radius, const JsonSPtr& json = nullptr);
    ~Config() = default;

    bool ParseFromJson(const float& map_resolution,
                       const uint& robot_cell_width,
                       const float& chassis_radius, const JsonSPtr& json);

    static const std::string kInstallDegreeKey_;
    float install_degree_;
    static const std::string kTracePathMovementMarkPointKey_;
    MapPoint trace_path_movement_mark_point_;
    static const std::string kEncircleObstacleMovementMarkPointKey_;
    MapPoint encircle_obstacle_movement_mark_point_;
    static const std::string kTriggerDistanceValueKey_;
    float trigger_distance_value_;
  };

  CliffSensor() = delete;
  CliffSensor(const std::string name, const Config& config);
  ~CliffSensor() = default;

  using SPtr = std::shared_ptr<CliffSensor>;

  void UpdateRealtimeDistanceValue(const float& value);
  float GetDistanceValue() const;

  bool TriggeredEvent() const { return triggered_event_.load(); }
  void ClearEvent() { triggered_event_.store(false); }
  double GetDataTimeStamp() const;
  bool CheckFresh(const double& limit) const;

  MapPoint GetTracePathMovementMarkPoint() const {
    return trace_path_movement_mark_point_;
  }
  MapPoint GetEncircleObstacleMovementMarkPoint() const {
    return encircle_obstacle_movement_mark_point_;
  }

  static const std::string kNullName_;

 private:
  ReadWriteLock::SPtr lock_;
  float distance_value_;
  float trigger_distance_value_;
  bool last_triggered_state_;
  std::atomic_bool real_time_triggered_state_;
  std::atomic_bool triggered_event_;
  double data_timestamp_;
  MapPoint trace_path_movement_mark_point_;
  MapPoint encircle_obstacle_movement_mark_point_;
};

}  // namespace zima

#endif  // ZIMA_CLIFF_SENSOR_H
