/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/device/gyro.h"

#include "zima/common/config.h"
#include "zima/common/time.h"
#include "zima/logger/logger.h"

namespace zima {

Gyro::Config::Config() : Config(nullptr) {}

Gyro::Config::Config(const JsonSPtr& json) {
  // Override if json config is provided.
  if (json != nullptr) {
    // ZINFO << "Load config from json.";
    config_valid_ = ParseFromJson(json);
  } else {
    config_valid_ = false;
  }
}

bool Gyro::Config::ParseFromJson(const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }

  if (!DeviceBase::Config::ParseFromJson(json)) {
    return false;
  }

  return true;
}

Gyro::Gyro(const std::string name, const Config& config)
    : DeviceBase(name, config),
      lock_(std::make_shared<ReadWriteLock>()),
      data_timestamp_(0){};

void Gyro::SetDegree(const float& degree, const double& data_timestamp) {
  WriteLocker lock(lock_);
  degree_ = degree;
  data_timestamp_ = data_timestamp;
}

float Gyro::GetDegree() const {
  ReadLocker lock(lock_);
  return degree_;
}

double Gyro::GetDataTimeStamp() const {
  ReadLocker lock(lock_);
  return data_timestamp_;
}

bool Gyro::CheckFresh(const double& limit) const {
  ReadLocker lock(lock_);
  if (Time::Now() - data_timestamp_ < limit) {
    return true;
  }
  return false;
}

}  // namespace zima
