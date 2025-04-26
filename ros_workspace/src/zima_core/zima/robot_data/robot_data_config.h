/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

/*
  NOTICE:
  This header file should ONLY be included by robot_data_config.cpp.
 */

#ifndef ZIMA_ROBOT_DATA_CONFIG_H
#define ZIMA_ROBOT_DATA_CONFIG_H

#include "zima/robot_data/nav_data.h"
#include "zima/robot_data/cleaning_record.h"
#include "zima/robot_data/local_nav_data.h"

namespace zima {

// ====================
const std::string RoomInfo::Config::kRoomInfoConfigKey_ = "room info config";
const std::string RoomInfo::Config::kMaxSectionWidthKey_ = "max section width";

// ====================
const std::string CleaningRecordManager::Config::kConfigKey_ =
    "cleaning record config";
const std::string CleaningRecordManager::Config::kCleaningRecordPathKey_ =
    "cleaning record path";
const std::string CleaningRecordManager::Config::kCleaningRecordCountLimitKey_ =
    "cleaning record count limit";

// ====================
const std::string LocalNavDataManager::Config::kConfigKey_ =
    "local nav data config";
const std::string LocalNavDataManager::Config::kLocalNavDataPathKey_ =
    "local nav data path";
const std::string LocalNavDataManager::Config::kLocalNavDataCountLimitKey_ =
    "local nav data count limit";


}  // namespace zima

#endif  // ZIMA_ROBOT_CONFIG_H
