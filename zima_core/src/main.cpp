/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/logger/logger.h"
#include "zima/zima_core_version.h"

int main(int argc, char **argv) {
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  ZINFO << "Hello Zima";
  ZINFO << zima::GetCoreVersionInfo();

  return 0;
}
