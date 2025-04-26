/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_WALL_SENSOR_H
#define ZIMA_WALL_SENSOR_H

#include <atomic>
#include <memory>
#include <string>

#include "zima/common/lock.h"
#include "zima/common/point_cell.h"
#include "zima/device/device.h"

namespace zima {

class WallSensor : public DeviceBase {
 public:
  class Config : public DeviceBase::Config {
   public:
    Config() = delete;
    explicit Config(const float& chassis_radius,
                    const JsonSPtr& json = nullptr);
    ~Config() = default;

    bool ParseFromJson(const float& chassis_radius, const JsonSPtr& json);

    static const std::string kInstallDegreeKey_;
    float install_degree_;
    static const std::string kMarkDistanceOnYKey_;
    float mark_distance_on_y_;
    static const std::string kMarkPointKey_;
    MapPoint mark_point_;
  };

  WallSensor() = delete;
  WallSensor(const std::string name, const Config& config)
      : DeviceBase(name, config),
        lock_(std::make_shared<ReadWriteLock>()),
        data_timestamp_(0),
        distance_(10000 * 100),
        mark_point_(config.mark_point_){};
  ~WallSensor() = default;

  using SPtr = std::shared_ptr<WallSensor>;

  void UpdateDistance(const float& distance_m) {
    distance_.store(static_cast<int32_t>(distance_m * 100 * kDistanceScale_));
  }
  float GetDistance() const {
    return static_cast<float>(distance_.load()) / 100 / kDistanceScale_;
  }

  MapPoint GetMarkPoint() const { return mark_point_; }

  double GetDataTimeStamp() const;
  bool CheckFresh(const double& limit) const;

  const int kDistanceScale_ = 100;

  static const std::string kNullName_;

 private:
  ReadWriteLock::SPtr lock_;
  double data_timestamp_;
  // cm * kDistanceScale_
  std::atomic_int32_t distance_;
  MapPoint mark_point_;
};

}  // namespace zima

#endif  // ZIMA_WALL_SENSOR_H
