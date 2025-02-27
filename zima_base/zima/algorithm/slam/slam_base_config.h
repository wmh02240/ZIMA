/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

/*
  NOTICE:
  This header file should ONLY be included by slam_base_config.cpp.
 */

#ifndef ZIMA_SLAM_BASE_CONFIG_H
#define ZIMA_SLAM_BASE_CONFIG_H

#include "zima/algorithm/slam/slam_base.h"
namespace zima {

// ====================
const std::string SlamBase::Config::kConfigKey_ = "slam base config";
const std::string SlamBase::Config::kOdomDataListMaxSizeKey_ =
    "odom data list max size";
const std::string SlamBase::Config::kOdomDataListMaxTimeIntervalKey_ =
    "odom data list max time interval";
const std::string SlamBase::Config::kPointCloudListMaxSizeKey_ =
    "point cloud list max size";
const std::string SlamBase::Config::kPointCloudListMaxTimeIntervalKey_ =
    "point cloud list max time interval";

const std::string SlamBase::Config::kOdomSampleRatioKey_ = "odom sample ratio";
const std::string SlamBase::Config::kPointCloudSampleRatioKey_ =
    "point cloud sample ratio";
const std::string SlamBase::Config::kPoseFilterMaxLinearDistanceKey_ =
    "pose filter max linear distance";
const std::string SlamBase::Config::kPoseFilterMaxAngleDegreeKey_ =
    "pose filter max angle degree";
const std::string SlamBase::Config::kPoseFilterMaxTimeIntervalKey_ =
    "pose filter max time interval";

}  // namespace zima

#endif  // ZIMA_SLAM_BASE_CONFIG_H
