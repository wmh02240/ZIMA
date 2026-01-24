/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

/*
  NOTICE:
  This header file should ONLY be included by grid_map_config.cpp.
 */

#ifndef ZIMA_GRID_MAP_CONFIG_H
#define ZIMA_GRID_MAP_CONFIG_H

#include "zima/grid_map/map_2d.h"
#include "zima/grid_map/nav_map_2d.h"

namespace zima {

// ====================
const std::string SlamValueGridMap2D::Config::kConfigKey_ =
    "slam value grid map config";
const std::string SlamValueGridMap2D::Config::kDefaultValueKey_ =
    "default value";
const std::string SlamValueGridMap2D::Config::kMinSpaceValueKey_ =
    "min space value";
const std::string SlamValueGridMap2D::Config::kMediumValueKey_ = "medium value";
const std::string SlamValueGridMap2D::Config::kMaxObstacleValueKey_ =
    "max obstacle value";

// ====================
const std::string NavMap::Config::kConfigKey_ = "nav map config";
const std::string NavMap::Config::kResolutionKey_ = "resolution";
const std::string NavMap::Config::kMaxWidthKey_ = "max width";
const std::string NavMap::Config::kMaxHeightKey_ = "max height";
const std::string NavMap::Config::kMaxCleanWidthKey_ = "max clean width";
const std::string NavMap::Config::kMaxCleanHeightKey_ = "max clean height";

}  // namespace zima

#endif  // ZIMA_GRID_MAP_CONFIG_H
