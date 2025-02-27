/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_BATTERY_H
#define ZIMA_BATTERY_H

#include <stdint.h>

#include <atomic>
#include <memory>

#include "zima/common/lock.h"
#include "zima/device/device.h"

namespace zima {

class Battery : public DeviceBase {
 public:
  class Config : public DeviceBase::Config {
   public:
    Config();
    explicit Config(const JsonSPtr& json);
    ~Config() = default;

    bool ParseFromJson(const JsonSPtr& json);

    static const std::string kFullyChargedStateVoltageKey_;
    float fully_charged_state_voltage_;
    static const std::string kLowStateVoltageKey_;
    float low_state_voltage_;
    static const std::string kDangerousStateVoltageKey_;
    float dangerous_state_voltage_;
    static const std::string kDesignCapacityKey_;
    float design_capacity_;
  };

  Battery() = delete;
  Battery(const std::string& name, const Config& config);
  ~Battery();

  using SPtr = std::shared_ptr<Battery>;

  enum State {
    kCharging,
    kFullyCharged,
    kDisCharging,
    kLow,
    kDangerous,
  };

  void UpdateBatteryVoltage(const float& voltage);

  void SetChargingState();
  void SetFullyChargedState();
  void SetDisChargingState();
  bool IsFullyCharged();
  bool IsLowState();
  bool IsDangerousState();

  double GetDataTimeStamp() const;
  bool CheckFresh(const double& limit) const;

  static const std::string kNullName_;

 protected:
  virtual float VoltageToPercentage(const float& voltage);

 private:
  ReadWriteLock::SPtr access_;
  State state_;
  float percentage_;
  double data_timestamp_;
  double last_print_log_time_;

  float voltage_;
  float fully_charged_state_voltage_;
  float low_state_voltage_;
  float dangerous_state_voltage_;

  float design_capacity_;
};

}  // namespace zima

#endif  // ZIMA_BATTERY_H
