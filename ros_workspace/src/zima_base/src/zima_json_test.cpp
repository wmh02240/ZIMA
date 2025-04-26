/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/common/config.h"
#include "zima/logger/logger.h"
#include "zima/zima_base_version.h"

int main(int argc, char **argv) {
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  auto global_json_config = zima::GlobalJsonConfig::Instance();
  global_json_config->GetConfigDescription();
  return 0;
}
