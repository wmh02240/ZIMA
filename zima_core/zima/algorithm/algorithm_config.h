/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

/*
  NOTICE:
  This header file should ONLY be included by algorithm_config.cpp.
 */

#ifndef ZIMA_ALGORITHM_CONFIG_H
#define ZIMA_ALGORITHM_CONFIG_H

#include "zima/algorithm/point_cloud_matcher.h"
#include "zima/algorithm/pose_interpolator.h"
#include "zima/algorithm/slam/probability_map.h"
#include "zima/algorithm/slam/simple_slam.h"

namespace zima {

// ====================
const std::string PoseInterpolator::Config::kConfigKey_ =
    "pose interpolator config";
const std::string PoseInterpolator::Config::kBufferTimeLimitKey_ =
    "buffer time limit";
const std::string PoseInterpolator::Config::kBufferCountLimitKey_ =
    "buffer count limit";

// ====================
const std::string ProbabilityIndexGridMap2D::Config::kConfigKey_ =
    "probability index grid map config";
const std::string
    ProbabilityIndexGridMap2D::Config::kHitProbabilityOnObstacleKey_ =
        "hit probability on obstacle";
const std::string
    ProbabilityIndexGridMap2D::Config::kMissProbabilityOnObstacleKey_ =
        "miss probability on obstacle";
const std::string
    ProbabilityIndexGridMap2D::Config::kMaxProbabilityForObstacleInCellKey_ =
        "max probability for obstacle in cell";
const std::string
    ProbabilityIndexGridMap2D::Config::kMinProbabilityForObstacleInCellKey_ =
        "min probability for obstacle in cell";
const std::string ProbabilityIndexGridMap2D::Config::kProbabilityCountKey_ =
    "probability count";

// ====================
const std::string PointCloudMatcher::Config::kConfigKey_ =
    "point cloud matcher config";
const std::string PointCloudMatcher::Config::kTranslationDeltaCostWeightKey_ =
    "translation delta cost weight";
const std::string
    PointCloudMatcher::Config::kRotationDegreeDeltaCostWeightKey_ =
        "rotation degree delta cost weight";
const std::string PointCloudMatcher::BABSearchConfig::kDegreeRangeKey_ =
    "degree range";
const std::string PointCloudMatcher::BABSearchConfig::kDegreeStepKey_ =
    "degree step";
const std::string PointCloudMatcher::BABSearchConfig::kSearchDepthKey_ =
    "search depth";
const std::string PointCloudMatcher::BABSearchConfig::kMinScoreKey_ =
    "min score";

// ====================
const std::string SimpleSlam::Config::kConfigKey_ = "simple slam config";
const std::string SimpleSlam::Config::kSlamLinearRangeKey_ =
    "slam linear range";
const std::string SimpleSlam::Config::kSlamResolutionKey_ =
    "slam resolution";
const std::string SimpleSlam::Config::kMinRequiredMatchScoreKey_ =
    "min required match score";
const std::string SimpleSlam::Config::kMaxRequiredMatchScoreKey_ =
    "max required match score";
const std::string SimpleSlam::Config::kMinSearchLinearRangeKey_ =
    "min search linear range";
const std::string SimpleSlam::Config::kMaxSearchLinearRangeKey_ =
    "max search linear range";
const std::string SimpleSlam::Config::kMinSearchDegreeRangeKey_ =
    "min search degree range";
const std::string SimpleSlam::Config::kMaxSearchDegreeRangeKey_ =
    "max search degree range";
const std::string SimpleSlam::Config::kRawSearchLinearStepKey_ =
    "raw search linear step";
const std::string SimpleSlam::Config::kRawSearchDegreeStepKey_ =
    "raw search degree step";
const std::string SimpleSlam::Config::kPreciseSearchLinearRangeKey_ =
    "precise search linear range";
const std::string SimpleSlam::Config::kPreciseSearchDegreeRangeKey_ =
    "precise search degree range";
const std::string SimpleSlam::Config::kPreciseSearchLinearStepKey_ =
    "precise search linear step";
const std::string SimpleSlam::Config::kPreciseSearchDegreeStepKey_ =
    "precise search degree step";
const std::string SimpleSlam::Config::kRawMatcherConfigKey_ =
    "raw matcher config";
const std::string SimpleSlam::Config::kPreciseMatcherConfigKey_ =
    "precise matcher config";
const std::string SimpleSlam::Config::kMotionDetectorTimeInterval_ =
    "motion detector time interval";
const std::string SimpleSlam::Config::kMotionDetectorDegreeLimit_ =
    "motion detector degree limit";

}  // namespace zima

#endif  // ZIMA_ALGORITHM_CONFIG_H
