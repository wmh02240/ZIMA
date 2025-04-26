/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/device/battery.h"

#include "zima/common/config.h"
#include "zima/common/maths.h"
#include "zima/common/time.h"
#include "zima/common/util.h"
#include "zima/logger/logger.h"

namespace zima {

Battery::Config::Config() : Config(nullptr) {}

Battery::Config::Config(const JsonSPtr& json) {
  // Load default setting.
  fully_charged_state_voltage_ = 0;
  low_state_voltage_ = 0;
  dangerous_state_voltage_ = 0;
  design_capacity_ = 0;

  // Override if json config is provided.
  if (json != nullptr) {
    // ZINFO << "Load config from json.";
    config_valid_ = ParseFromJson(json);
  } else {
    config_valid_ = false;
  }
}

bool Battery::Config::ParseFromJson(const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }

  if (!DeviceBase::Config::ParseFromJson(json)) {
    return false;
  }

  // Override.
  if (!JsonHelper::GetFloat(*json, kFullyChargedStateVoltageKey_,
                            fully_charged_state_voltage_)) {
    ZERROR << "Config " << kFullyChargedStateVoltageKey_ << " not found.";
    return false;
  }
  if (!JsonHelper::GetFloat(*json, kLowStateVoltageKey_, low_state_voltage_)) {
    ZERROR << "Config " << kLowStateVoltageKey_ << " not found.";
    return false;
  }
  if (!JsonHelper::GetFloat(*json, kDangerousStateVoltageKey_,
                            dangerous_state_voltage_)) {
    ZERROR << "Config " << kDangerousStateVoltageKey_ << " not found.";
    return false;
  }
  if (!JsonHelper::GetFloat(*json, kDesignCapacityKey_, design_capacity_)) {
    ZERROR << "Config " << kDesignCapacityKey_ << " not found.";
    return false;
  }

  return true;
}

Battery::Battery(const std::string& name, const Config& config)
    : DeviceBase(name, config),
      access_(std::make_shared<ReadWriteLock>()),
      state_(kDisCharging),
      percentage_(0),
      data_timestamp_(0),
      last_print_log_time_(0),
      voltage_(0),
      fully_charged_state_voltage_(config.fully_charged_state_voltage_),
      low_state_voltage_(config.low_state_voltage_),
      dangerous_state_voltage_(config.dangerous_state_voltage_),
      design_capacity_(config.design_capacity_) {}

Battery::~Battery() {}

float Battery::VoltageToPercentage(const float& voltage) {
  const float kVoltageNormalRange =
      fully_charged_state_voltage_ - low_state_voltage_;
  const float kVoltageLowRange =
      low_state_voltage_ - dangerous_state_voltage_;
  float percentage;
  if (voltage > low_state_voltage_) {
    percentage = (voltage - low_state_voltage_) / kVoltageNormalRange * 80 + 20;
    percentage = Minimum(percentage, 100.0f);
  } else if (voltage > dangerous_state_voltage_) {
    percentage = (voltage - dangerous_state_voltage_) / kVoltageLowRange * 20;
  } else {
    percentage = 0;
  }
  return percentage;
}

void Battery::UpdateBatteryVoltage(const float& voltage) {
  WriteLocker lock(access_);
  switch (state_) {
    case kCharging:
    case kFullyCharged: {
      if (voltage > voltage_ || FloatEqual(voltage_,0)) {
        voltage_ = voltage;
        percentage_ = VoltageToPercentage(voltage);
        ZINFO << FloatToString(voltage_, 2) << "V("
              << FloatToString(percentage_, 2) << "%)";
      } else if (Time::Now() - last_print_log_time_ > 20) {
        last_print_log_time_ = Time::Now();
        ZINFO << FloatToString(voltage_, 2) << "V("
              << FloatToString(percentage_, 2) << "%)";
      }
      break;
    }
    case kDisCharging:
    case kLow:
    case kDangerous: {
      if (voltage < voltage_ || FloatEqual(voltage_,0)) {
        voltage_ = voltage;
        percentage_ = VoltageToPercentage(voltage);
        ZINFO << FloatToString(voltage_, 2) << "V("
              << FloatToString(percentage_, 2) << "%)";
      } else if (Time::Now() - last_print_log_time_ > 20) {
        last_print_log_time_ = Time::Now();
        ZINFO << FloatToString(voltage_, 2) << "V("
              << FloatToString(percentage_, 2) << "%)";
      }
      break;
    }
    default: {
      ZERROR << "Invalid state " << state_;
      break;
    }
  }
  data_timestamp_ = Time::Now();
}

void Battery::SetChargingState() {
  WriteLocker lock(access_);
  if (state_ == kCharging) {
    return;
  }
  state_ = kCharging;
  data_timestamp_ = Time::Now();
}

void Battery::SetFullyChargedState() {
  WriteLocker lock(access_);
  if (state_ == kFullyCharged) {
    return;
  }
  state_ = kFullyCharged;
  data_timestamp_ = Time::Now();
}

void Battery::SetDisChargingState() {
  WriteLocker lock(access_);
  switch (state_) {
    case kDisCharging: {
      if (voltage_ <= low_state_voltage_) {
        ZWARN << "Battery Change to low state.";
        state_ = kLow;
      }
      break;
    }
    case kLow: {
      if (voltage_ <= dangerous_state_voltage_) {
        ZWARN << "Battery Change to dangerous state.";
        state_ = kDangerous;
      }
      break;
    }
    case kDangerous: {
      // Do nothing.
      break;
    }
    default: {
      state_ = kDisCharging;
      break;
    }
  }
  data_timestamp_ = Time::Now();
}

bool Battery::IsLowState() {
  ReadLocker lock(access_);
  return state_ == kLow;
}

bool Battery::IsDangerousState() {
  ReadLocker lock(access_);
  return state_ == kDangerous;
}

double Battery::GetDataTimeStamp() const {
  ReadLocker lock(access_);
  return data_timestamp_;
}

bool Battery::CheckFresh(const double& limit) const {
  ReadLocker lock(access_);
  if (Time::Now() - data_timestamp_ < limit) {
    return true;
  }
  return false;
}

}  // namespace zima
