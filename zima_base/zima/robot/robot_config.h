/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

/*
  NOTICE:
  This header file should ONLY be included by robot_config.cpp.
 */

#ifndef ZIMA_ROBOT_CONFIG_H
#define ZIMA_ROBOT_CONFIG_H

#include "zima/robot/chassis.h"

namespace zima {

// ====================
const std::string Chassis::kLeftWheel_ = "left wheel";
const std::string Chassis::kRightWheel_ = "right wheel";
const std::string Chassis::kLeftBumper_ = "left bumper";
const std::string Chassis::kCenterBumper_ = "center bumper";
const std::string Chassis::kRightBumper_ = "right bumper";
const std::string Chassis::kButton1_ = "button 1";
const std::string Chassis::kLeftWallSensor_ = "left wall sensor";
const std::string Chassis::kRightWallSensor_ = "right wall sensor";
const std::string Chassis::kGyro_ = "gyro";
const std::string Chassis::kLidar_ = "lidar";
const std::string Chassis::kBattery_ = "battery";

const std::string Chassis::Config::kChassisConfigKey_ = "chassis config";
const std::string Chassis::Config::kDeviceConfigKey_ = "device config";
const std::string Chassis::Config::kTrackLengthKey_ = "track length";
const std::string Chassis::Config::kRadiusKey_ = "radius";
const std::string Chassis::Config::kEncircleObstacleOnLeftKey_ =
    "encircle obstacle on left";

}  // namespace zima

#endif  // ZIMA_ROBOT_CONFIG_H
