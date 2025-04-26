/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/common/config.h"

#include "zima/hal/system/file.h"
#include "zima/logger/logger.h"

namespace zima {

GlobalJsonConfig::GlobalJsonConfig()
    : access_(std::make_shared<ReadWriteLock>()),
      global_json_config_(nullptr) {
  LoadFromFile();
}

bool GlobalJsonConfig::ParseFromJson(const JsonSPtr& json) {
  if (json == nullptr) {
    return false;
  }

  if (!JsonHelper::KeyExists(*json, kConfigKey_)) {
    ZERROR << "Missing global config.";
    return false;
  }

  WriteLocker lock(access_);
  global_json_config_.reset(new Json((*json).at(kConfigKey_)));
  const Json& config = *global_json_config_;

  std::string version("");
  if (JsonHelper::GetString(config, kVersionName_, version)) {
    version_ = version;
    ZINFO << "Config version: " << version;
  } else {
    ZWARN << "Global config missing version.";
  }

  std::string description;
  if (JsonHelper::GetString(config, KdescriptionName_, description)) {
    description_ = description;
    ZINFO << "Config description: " << description;
  } else {
    ZWARN << "Global config missing description.";
  }

  return true;
}

bool GlobalJsonConfig::GetGlobalConfig(JsonSPtr& config) {
  ReadLocker lock(access_);
  if (global_json_config_ == nullptr) {
    return false;
  }

  config = global_json_config_;
  return true;
}

bool GlobalJsonConfig::GetGlobalConfigObject(const std::string& name,
                                             JsonSPtr& object) {
  JsonSPtr global_config;
  if (!GetGlobalConfig(global_config)) {
    return false;
  }

  object.reset(new Json());
  if (!JsonHelper::GetObject(*global_config, name, *object)) {
    return false;
  }

  return true;
}

std::string GlobalJsonConfig::GetConfigVersion() {
  ReadLocker lock(access_);
  return version_;
}

std::string GlobalJsonConfig::GetConfigDescription() {
  ReadLocker lock(access_);
  return description_;
}

bool GlobalJsonConfig::LoadFromFile() {
  LocalJsonFileLoader loader(kConfigFilePath_, kConfigFileName_);

  JsonSPtr json(new Json());
  if (!loader.GetJsonFromFile(*json)) {
    ZWARN << "Load json config from file " << kConfigFilePath_
          << kConfigFileName_ << " failed.";
    return false;
  }

  return ParseFromJson(json);
}

}  // namespace zima
