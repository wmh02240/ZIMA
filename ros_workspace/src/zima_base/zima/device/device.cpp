/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/device/device.h"
#include "zima/logger/logger.h"

namespace zima {

DeviceBase::Config::Config() : config_valid_(true) {
  tf_to_base_ = MapPoint();
}

bool DeviceBase::Config::ParseFromJson(const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }

  // Override.
  JsonSPtr array(new Json());
  if (!JsonHelper::GetArray(*json, kTfToBaseKey_, *array)) {
    // ZINFO << "Tf to base config not found.";
    return false;
  }
  if (!tf_to_base_.ParseFromJson(array)) {
    return false;
  }

  return true;
}

}  // namespace zima
