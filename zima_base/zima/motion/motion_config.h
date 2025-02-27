/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

/*
  NOTICE:
  This header file should ONLY be included by motion_config.cpp.
 */

#ifndef ZIMA_MOTION_CONFIG_H
#define ZIMA_MOTION_CONFIG_H

#include "zima/motion/encircle_map_edge_motion.h"
#include "zima/motion/encircle_obstacle_motion.h"
#include "zima/motion/motion.h"
#include "zima/motion/move_forward_motion.h"
#include "zima/motion/retreat_motion.h"
#include "zima/motion/rotate_motion.h"
#include "zima/motion/trace_path_motion.h"

namespace zima {

// ====================
const std::string MotionBase::MotionConfigBase::kCycleRateKey_ = "cycle rate";
const std::string MotionBase::MotionConfigBase::kTargetSpeedKey_ =
    "target speed";
const std::string MotionBase::MotionConfigBase::kSpeedUpStepKey_ =
    "speed up step";
const std::string MotionBase::MotionConfigBase::kTimeoutKey_ = "timeout";

// ====================
const std::string MoveForwardMotion::Config::kConfigKey_ =
    "move forward motion config";
const std::string MoveForwardMotion::Config::kMinTurnCircleRadiusKey_ =
    "min turn circle radius";

// ====================
const std::string RotateMotion::Config::kConfigKey_ = "rotate motion config";
const std::string RotateMotion::Config::kAccuracyAngleKey_ = "accuracy angle";
const std::string RotateMotion::Config::kMinSpeedKey_ = "min speed";

// ====================
const std::string RetreatMotion::Config::kConfigKey_ = "retreat motion config";
const std::string RetreatMotion::Config::kMinSpeedKey_ = "min speed";

// ====================
const std::string TracePathMotion::Config::kConfigKey_ =
    "trace path motion config";
const std::string TracePathMotion::Config::kMinTurnCircleRadiusKey_ =
    "min turn circle radius";
const std::string TracePathMotion::Config::kInterpolationDistanceKey_ =
    "interpolation distance";
const std::string TracePathMotion::Config::kMaxNearDistanceKey_ =
    "max near distance";
const std::string TracePathMotion::Config::kMaxReachDistanceKey_ =
    "max reach distance";
const std::string TracePathMotion::Config::kStopForRoomValueDistanceKey_ =
    "stop for room value distance";
const std::string TracePathMotion::Config::kStopForUserBlockDistanceKey_ =
    "stop for user block distance";
const std::string
    TracePathMotion::Config::kStopForCleaningAreaEdgeDistanceKey_ =
        "stop for cleaning area edge distance";
const std::string TracePathMotion::Config::kStopForLidarObsDistanceKey_ =
    "stop for lidar obs distance";
const std::string TracePathMotion::Config::kStopForLidarCompensateKey_ =
    "stop for lidar compensate";

// ====================
const std::string EncircleMapEdgeMotion::Config::kConfigKey_ =
    "encircle map edge motion config";
const std::string EncircleMapEdgeMotion::Config::kTargetMapEdgeDistanceKey_ =
    "target map edge distance";
const std::string
    EncircleMapEdgeMotion::Config::kMaxValidMapEdgeDistanceKey_ =
        "max valid map edge distance";
const std::string EncircleMapEdgeMotion::Config::kTurnCircleRadiusKey_ =
    "turn circle radius";
const std::string EncircleMapEdgeMotion::Config::kDelayDistanceKey_ =
    "delay distance";

// ====================
const std::string EncircleObstacleMotion::Config::kConfigKey_ =
    "encircle obstacle motion config";
const std::string EncircleObstacleMotion::Config::kTargetObstacleDistanceKey_ =
    "target obstacle distance";
const std::string
    EncircleObstacleMotion::Config::kMaxValidObstacleDistanceKey_ =
        "max valid obstacle distance";
const std::string EncircleObstacleMotion::Config::kTurnCircleRadiusKey_ =
    "turn circle radius";
const std::string EncircleObstacleMotion::Config::kDelayDistanceKey_ =
    "delay distance";
const std::string EncircleObstacleMotion::Config::kReachTempTargetDistanceKey_ =
    "reach temp target distance";
const std::string
    EncircleObstacleMotion::Config::kStopForRoomValueDistanceKey_ =
        "stop for room value distance";
const std::string
    EncircleObstacleMotion::Config::kStopForUserBlockDistanceKey_ =
        "stop for user block distance";
const std::string
    EncircleObstacleMotion::Config::kStopForCleaningAreaEdgeDistanceKey_ =
        "stop for cleaning area edge distance";
const std::string EncircleObstacleMotion::Config::kStopForLidarObsDistanceKey_ =
    "stop for lidar obs distance";
const std::string EncircleObstacleMotion::Config::kStopForLidarCompensateKey_ =
    "stop for lidar compensate";

}  // namespace zima

#endif  // ZIMA_MOTION_CONFIG_H
