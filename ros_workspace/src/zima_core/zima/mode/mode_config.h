/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

/*
  NOTICE:
  This header file should ONLY be included by algorithm_config.cpp.
 */

#ifndef ZIMA_MODE_CONFIG_H
#define ZIMA_MODE_CONFIG_H

#include "zima/mode/auto_cleaning_mode.h"
#include "zima/mode/auto_scan_house_mode.h"
#include "zima/mode/mode_base.h"

namespace zima {

// ====================
const std::string OperationMode::Config::kSmallRangeLinearRangeKey_ =
    "small range linear range";
const std::string OperationMode::Config::kRelocationCountMaxKey_ =
    "relocation count max";
const std::string OperationMode::Config::kOptimizedSmallRangeSearchConfigKey_ =
    "optimized small range search config";
const std::string OperationMode::Config::kSmallRangeSearchConfigKey_ =
    "small range search config";
const std::string OperationMode::Config::kOptimizedGlobalSearchConfigKey_ =
    "optimized global search config";
const std::string OperationMode::Config::kGlobalSearchConfigKey_ =
    "global search config";

// ====================
const std::string AutoCleaningMode::Config::kConfigKey_ =
    "auto cleaning mode config";

// ====================
const std::string AutoScanHouseMode::Config::kConfigKey_ =
    "auto scan house mode config";

}  // namespace zima

#endif  // ZIMA_MODE_CONFIG_H
