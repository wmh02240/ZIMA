/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/device/wheel.h"

#include "zima/common/config.h"
#include "zima/common/time.h"
#include "zima/logger/logger.h"

namespace zima {

Wheel::Config::Config() : Config(nullptr) {}

Wheel::Config::Config(const JsonSPtr& json) {
  min_speed_ = -0.1;
  max_speed_ = 0.1;
  wheel_tick_to_distance_x_10k_ = 0;
  wheel_speed_control_p_ = 0;
  wheel_speed_control_i_ = 0;
  wheel_speed_control_d_ = 0;

  // Override if json config is provided.
  if (json != nullptr) {
    // ZINFO << "Load config from json.";
    config_valid_ = ParseFromJson(json);
  } else {
    config_valid_ = false;
  }
}

bool Wheel::Config::ParseFromJson(const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }

  if (!DeviceBase::Config::ParseFromJson(json)) {
    return false;
  }

  // Override.
  if (!JsonHelper::GetFloat(*json, kMinSpeedKey_, min_speed_)) {
    ZERROR << "Config " << kMinSpeedKey_ << " not found.";
    return false;
  }
  if (!JsonHelper::GetFloat(*json, kMaxSpeedKey_, max_speed_)) {
    ZERROR << "Config " << kMaxSpeedKey_ << " not found.";
    return false;
  }

  JsonHelper::GetFloat(*json, kWheelTickToDistanceX10kKey_,
                       wheel_tick_to_distance_x_10k_);
  JsonHelper::GetFloat(*json, kWheelSpeedControlP_, wheel_speed_control_p_);
  JsonHelper::GetFloat(*json, kWheelSpeedControlI_, wheel_speed_control_i_);
  JsonHelper::GetFloat(*json, kWheelSpeedControlD_, wheel_speed_control_d_);

  return true;
}

Wheel::Wheel(const std::string name, const Config& config)
    : DeviceBase(name, config),
      current_speed_(0),
      target_speed_(0),
      kMaxSpeed_(config.max_speed_),
      kMinSpeed_(config.min_speed_),
      kWheelTickToDistance_(config.wheel_tick_to_distance_x_10k_ / 10000),
      kWheelSpeedControlP_(config.wheel_speed_control_p_),
      kWheelSpeedControlI_(config.wheel_speed_control_i_),
      kWheelSpeedControlD_(config.wheel_speed_control_d_) {}

void Wheel::SetTargetSpeed(const float& speed) {
  target_speed_.store(static_cast<int32_t>(speed * kSpeedScale_));
}

void Wheel::SetCurrentSpeed(const float& speed) {
  current_speed_.store(static_cast<int32_t>(speed * kSpeedScale_));
}

float Wheel::TargetSpeed() const {
  return static_cast<float>(target_speed_.load()) / kSpeedScale_;
}

float Wheel::CurrentSpeed() const {
  return static_cast<float>(current_speed_.load()) / kSpeedScale_;
}

double Wheel::GetDataTimeStamp() const {
  ReadLocker lock(lock_);
  return data_timestamp_;
}

bool Wheel::CheckFresh(const double& limit) const {
  ReadLocker lock(lock_);
  if (Time::Now() - data_timestamp_ < limit) {
    return true;
  }
  return false;
}

}  // namespace zima
