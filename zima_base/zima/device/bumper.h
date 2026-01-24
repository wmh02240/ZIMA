/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_BUMPER_H
#define ZIMA_BUMPER_H

#include <atomic>
#include <memory>
#include <string>

#include "zima/common/lock.h"
#include "zima/common/point_cell.h"
#include "zima/device/device.h"

namespace zima {

class Bumper : public DeviceBase {
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
    static const std::string kCoverRangeMinDegreeKey_;
    float cover_range_min_degree_;
    static const std::string kCoverRangeMaxDegreeKey_;
    float cover_range_max_degree_;
    static const std::string kTracePathMovementMarkPointKey_;
    MapPoint trace_path_movement_mark_point_;
    static const std::string kLeftEncircleObstacleMovementMarkPointKey_;
    MapPoint left_encircle_obstacle_movement_mark_point_;
    static const std::string kRightEncircleObstacleMovementMarkPointKey_;
    MapPoint right_encircle_obstacle_movement_mark_point_;
  };

  Bumper() = delete;
  Bumper(const std::string name, const Config& config);
  ~Bumper() = default;

  using SPtr = std::shared_ptr<Bumper>;

  void UpdateRealtimeTriggeredState(const bool& triggered);

  bool TriggeredEvent() const { return triggered_event_.load(); }
  void ClearEvent() { triggered_event_.store(false); }
  double GetDataTimeStamp() const;
  bool CheckFresh(const double& limit) const;

  double GetCoverRangeMinDegree() const { return cover_range_min_degree_; }
  double GetCoverRangeMaxDegree() const { return cover_range_max_degree_; }

  MapPoint GetTracePathMovementMarkPoint() const {
    return trace_path_movement_mark_point_;
  }
  MapPoint GetEncircleObstacleMovementMarkPoint(const bool& on_left) const {
    if (on_left) {
      return left_encircle_obstacle_movement_mark_point_;
    }
    return right_encircle_obstacle_movement_mark_point_;
  }

  static const std::string kNullName_;

 private:
  ReadWriteLock::SPtr lock_;
  bool last_triggered_state_;
  std::atomic_bool real_time_triggered_state_;
  std::atomic_bool triggered_event_;
  double data_timestamp_;
  double cover_range_min_degree_;
  double cover_range_max_degree_;
  MapPoint trace_path_movement_mark_point_;
  MapPoint left_encircle_obstacle_movement_mark_point_;
  MapPoint right_encircle_obstacle_movement_mark_point_;
};

}  // namespace zima

#endif  // ZIMA_BUMPER_H
