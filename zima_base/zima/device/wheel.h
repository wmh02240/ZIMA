/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_WHEEL_H
#define ZIMA_WHEEL_H

#include <atomic>
#include <memory>
#include <string>

#include "zima/common/lock.h"
#include "zima/common/point_cell.h"
#include "zima/device/device.h"

namespace zima {

class Wheel : public DeviceBase {
 public:
  class Config : public DeviceBase::Config {
   public:
    Config();
    explicit Config(const JsonSPtr& json);
    ~Config() = default;

    bool ParseFromJson(const JsonSPtr& json);

    static const std::string kMinSpeedKey_;
    float min_speed_;
    static const std::string kMaxSpeedKey_;
    float max_speed_;
    static const std::string kWheelTickToDistanceX10kKey_;
    float wheel_tick_to_distance_x_10k_;

    static const std::string kWheelSpeedControlP_;
    float wheel_speed_control_p_;
    static const std::string kWheelSpeedControlI_;
    float wheel_speed_control_i_;
    static const std::string kWheelSpeedControlD_;
    float wheel_speed_control_d_;
  };

  Wheel() = delete;
  Wheel(const std::string name, const Config& config);
  ~Wheel() = default;

  using SPtr = std::shared_ptr<Wheel>;

  void SetTargetSpeed(const float& speed);
  void SetCurrentSpeed(const float& speed);
  float TargetSpeed() const;
  float CurrentSpeed() const;
  float MaxSpeed() const { return kMaxSpeed_; };
  float MinSpeed() const { return kMinSpeed_; };

  double GetDataTimeStamp() const;
  bool CheckFresh(const double& limit) const;

  float GetWheelTickToDistance() const { return kWheelTickToDistance_; };

  float GetSpeedControlP() const { return kWheelSpeedControlP_; };
  float GetSpeedControlI() const { return kWheelSpeedControlI_; };
  float GetSpeedControlD() const { return kWheelSpeedControlD_; };

  const int kSpeedScale_ = 10000;

  static const std::string kNullName_;

 private:
  ReadWriteLock::SPtr lock_;
  double data_timestamp_;
  // m/s * speed_scale_
  std::atomic_int32_t current_speed_;
  std::atomic_int32_t target_speed_;
  const float kMaxSpeed_;
  const float kMinSpeed_;
  // distance for meter.
  const float kWheelTickToDistance_;

  const float kWheelSpeedControlP_;
  const float kWheelSpeedControlI_;
  const float kWheelSpeedControlD_;
};

}  // namespace zima

#endif  // ZIMA_WHEEL_H
