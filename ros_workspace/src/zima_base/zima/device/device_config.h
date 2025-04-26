/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

/*
  NOTICE:
  This header file should ONLY be included by device_config.cpp.
 */

#ifndef ZIMA_DEVICE_CONFIG_H
#define ZIMA_DEVICE_CONFIG_H

#include "zima/device/battery.h"
#include "zima/device/bumper.h"
#include "zima/device/button.h"
#include "zima/device/device.h"
#include "zima/device/gyro.h"
#include "zima/device/lidar.h"
#include "zima/device/wall_sensor.h"
#include "zima/device/wheel.h"

namespace zima {

// ====================
const std::string DeviceBase::Config::kTfToBaseKey_ = "tf to base";

// ====================
const std::string Battery::Config::kFullyChargedStateVoltageKey_ =
    "fully charged state voltage";
const std::string Battery::Config::kLowStateVoltageKey_ =
    "low state voltage";
const std::string Battery::Config::kDangerousStateVoltageKey_ =
    "dangerous state voltage";
const std::string Battery::Config::kDesignCapacityKey_ =
    "design capacity";
const std::string Battery::kNullName_ = "Null battery";

// ====================
const std::string Bumper::Config::kCoverRangeMinDegreeKey_ =
    "cover range min degree";
const std::string Bumper::Config::kCoverRangeMaxDegreeKey_ =
    "cover range max degree";
const std::string Bumper::Config::kInstallDegreeKey_ =
    "install degree";
const std::string Bumper::Config::kTracePathMovementMarkPointKey_ =
    "trace path movement mark point";
const std::string Bumper::Config::kLeftEncircleObstacleMovementMarkPointKey_ =
    "left encircle obstacle movement mark point";
const std::string Bumper::Config::kRightEncircleObstacleMovementMarkPointKey_ =
    "right encircle obstacle movement mark point";
const std::string Bumper::kNullName_ = "Null bumper";

// ====================
const std::string Button::Config::kReleaseDelayKey_ = "release delay";
const std::string Button::kNullName_ = "null button";

// ====================
const std::string Gyro::kNullName_ = "null gyro";

// ====================
const std::string Lidar::Config::kMaxRangeKey_ = "max range";
const std::string Lidar::Config::kMinRangeKey_ = "min range";
const std::string Lidar::Config::kFilterDistanceKey_ = "filter distance";
const std::string Lidar::kNullName_ = "Null lidar";

// ====================
const std::string WallSensor::Config::kInstallDegreeKey_ =
    "install degree";
const std::string WallSensor::Config::kMarkDistanceOnYKey_ =
    "mark distance on y";
const std::string WallSensor::Config::kMarkPointKey_ =
    "mark point";
const std::string WallSensor::kNullName_ = "Null wall sensor";

// ====================
const std::string Wheel::Config::kMinSpeedKey_ =
    "min speed";
const std::string Wheel::Config::kMaxSpeedKey_ =
    "max speed";
const std::string Wheel::Config::kWheelTickToDistanceX10kKey_ =
    "wheel tick to distance x 10k";
const std::string Wheel::Config::kWheelSpeedControlP_ =
    "wheel speed control p";
const std::string Wheel::Config::kWheelSpeedControlI_ =
    "wheel speed control i";
const std::string Wheel::Config::kWheelSpeedControlD_ =
    "wheel speed control d";
const std::string Wheel::kNullName_ = "Null wheel";

}  // namespace zima

#endif  // ZIMA_DEVICE_CONFIG_H
