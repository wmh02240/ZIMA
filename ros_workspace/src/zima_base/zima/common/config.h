/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_CONFIG_H
#define ZIMA_CONFIG_H

#include "zima/common/json.h"
#include "zima/common/lock.h"
#include "zima/common/macro.h"

namespace zima {

class GlobalJsonConfig {
  DECLARE_SINGLETON(GlobalJsonConfig)

 public:
  bool ParseFromJson(const JsonSPtr& json);
  bool GetGlobalConfig(JsonSPtr& config);
  bool GetGlobalConfigObject(const std::string& name, JsonSPtr& object);

  std::string GetConfigVersion();
  std::string GetConfigDescription();

 private:
  bool LoadFromFile();

  ReadWriteLock::SPtr access_;

  const std::string kConfigFilePath_ = "/tmp/";
  const std::string kConfigFileName_ = "zima_config.json";
  const std::string kConfigKey_ = "zima global config";
  const std::string kVersionName_ = "config version";
  const std::string KdescriptionName_ = "config description";

  JsonSPtr global_json_config_;

  std::string version_;
  std::string description_;
};

}  // namespace zima

#endif  // ZIMA_JSON_H
