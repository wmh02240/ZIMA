/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

/*
  NOTICE:
  This header file should ONLY be included by movement_config.cpp.
 */

#ifndef ZIMA_MOVEMENT_CONFIG_H
#define ZIMA_MOVEMENT_CONFIG_H

#include "zima/movement/encircle_map_edge_movement.h"
#include "zima/movement/encircle_obstacle_movement.h"
#include "zima/movement/trace_path_movement.h"

namespace zima {

// ====================
const std::string TracePathMovement::Config::kConfigKey_ =
    "trace path movement config";
const std::string TracePathMovement::Config::kRetreatDistanceKey_ =
    "retreat distance";
const std::string
    TracePathMovement::Config::kTracePathObstacleSlowDownDistanceKey_ =
        "trace path obstacle slow down distance";
const std::string
    TracePathMovement::Config::kTracePathObstacleSlowDownSpeedKey_ =
        "trace path obstacle slow down speed";
const std::string
    TracePathMovement::Config::kTracePathObstacleSlowDownSpeedStepKey_ =
        "trace path obstacle slow down speed step";

// ====================
const std::string EncircleObstacleMovement::Config::kConfigKey_ =
    "encircle obstacle movement config";
const std::string EncircleObstacleMovement::Config::kMoveForwardDistanceKey_ =
    "move forward distance";
const std::string EncircleObstacleMovement::Config::kMoveForwardTimeKey_ =
    "move forward time";
const std::string EncircleObstacleMovement::Config::kRetreatDistanceKey_ =
    "retreat distance";
const std::string
    EncircleObstacleMovement::Config::kRotateLidarCalDegreeCompensateKey_ =
        "rotate lidar cal degree compensate";

// ====================
const std::string EncircleMapEdgeMovement::Config::kConfigKey_ =
    "encircle map edge movement config";
const std::string EncircleMapEdgeMovement::Config::kRetreatDistanceKey_ =
    "retreat distance";

}  // namespace zima

#endif  // ZIMA_MOVEMENT_CONFIG_H
