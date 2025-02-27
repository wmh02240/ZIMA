/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/zima_core_version.h"

#include "zima/common/util.h"
#include "zima/zima_core_version_hash.h"

namespace zima {

#define ZIMA_CORE_MAJOR_VERSION "0"
#define ZIMA_CORE_MINOR_VERSION "1"
#define ZIMA_CORE_PATCH_VERSION "1"

std::string GetCoreVersionInfo() {
  static std::string version_info;
  if (version_info.empty()) {
    version_info += GetZimaPrintString();
    version_info +=
        "Zima core version " + std::string(ZIMA_CORE_MAJOR_VERSION) + "." +
        std::string(ZIMA_CORE_MINOR_VERSION) + "." +
        std::string(ZIMA_CORE_PATCH_VERSION) + ", build at " + __DATE__ + " " +
        __TIME__ + ", hash:" + ZIMA_CORE_VERSION_HASH;
  }
  return version_info;
}

}  // namespace zima
