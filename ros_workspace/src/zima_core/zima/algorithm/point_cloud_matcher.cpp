/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/algorithm/point_cloud_matcher.h"

#include <algorithm>
#include <thread>
#include <vector>

#include "zima/common/config.h"
#include "zima/common/time.h"
#include "zima/grid_map/map_util.h"
#include "zima/hal/system/cpu.h"

// 1 thread cost time like 80% ~ 100% of non-boost
// 2 thread cost time like 60% ~ 80% of non-boost
// 4 thread cost time like 30% ~ 70% of non-boost

// static const uint8_t boost_thread_count = thread::hardware_concurrency() > 1
//                                     ? thread::hardware_concurrency() - 1
//                                     : thread::hardware_concurrency();
static const uint8_t boost_thread_count = 4;
namespace zima {

PointCloudMatcher::BABSearchParameter::BABSearchParameter(
    const SlamValueGridMap2D::SPtr& map, const MapPoint& init_pose,
    const DynamicMapPointBound& search_linear_bound_in_map_frame,
    const float& degree_range, const float& degree_step,
    const uint8_t& search_depth, const float& min_score,
    const bool& optimized_for_known_wall_degree)
    : init_pose_(init_pose),
      search_linear_cell_bound_(0, 0, 0, 0),
      degree_range_(degree_range),
      degree_step_(degree_step),
      search_depth_(search_depth),
      min_score_(min_score),
      optimized_for_known_wall_degree_(optimized_for_known_wall_degree) {
  MapCell min, max;
  map->WorldToMap(MapPoint(search_linear_bound_in_map_frame.GetMin().X(),
                           search_linear_bound_in_map_frame.GetMin().Y()),
                  min);
  map->WorldToMap(MapPoint(search_linear_bound_in_map_frame.GetMax().X(),
                           search_linear_bound_in_map_frame.GetMax().Y()),
                  max);
  search_linear_cell_bound_ =
      DynamicMapCellBound(min.X(), max.X(), min.Y(), max.Y());
  ZGINFO << "Search for cell bound: "
         << search_linear_cell_bound_.DebugString();

  num_scans_ = 1;
  for (auto degree = degree_step_; degree <= degree_range_;
       degree += degree_step_) {
    num_scans_++;
  }
  for (auto degree = -degree_step_; degree >= -degree_range_;
       degree -= degree_step_) {
    num_scans_++;
  }

  if (optimized_for_known_wall_degree_) {
    ZGINFO << "Search with wall degree: "
           << FloatToString(init_pose_.Degree(), 2);
    if (degree_range_ > 45.0) {
      ZGWARN
          << "Range should never be larger than 45 with wall degree provided.";
      degree_range_ = 45.0;
    }

    // For 4 wall degree.
    num_scans_ *= 4;
  } else {
    if (degree_range_ > 180.0) {
      ZGWARN << "Range should never be larger than 180.";
      degree_range_ = 180.0;
    }
  }
};

PointCloudMatcher::BABSearchConfig::BABSearchConfig()
    : BABSearchConfig(nullptr) {}

PointCloudMatcher::BABSearchConfig::BABSearchConfig(const JsonSPtr& json)
    : config_valid_(false) {
  // Load default setting.
  degree_range_ = 180;
  degree_step_ = 1;
  search_depth_ = 3;
  min_score_ = 0.5;

  if (json != nullptr) {
    // ZINFO << "Load config from json.";
    config_valid_ = ParseFromJson(json);
  } else {
    ZGERROR;
  }
}

bool PointCloudMatcher::BABSearchConfig::ParseFromJson(const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }

  // Override.
  if (!JsonHelper::GetFloat(*json, kDegreeRangeKey_, degree_range_)) {
    ZGERROR << "Config " << kDegreeRangeKey_ << " not found.";
  }
  if (degree_range_ < 0) {
    ZERROR << "Config " << kDegreeRangeKey_
           << " invalid: " << FloatToString(degree_range_, 3) << ".";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kDegreeStepKey_, degree_step_)) {
    ZGERROR << "Config " << kDegreeStepKey_ << " not found.";
  }
  if (degree_step_ < 0) {
    ZERROR << "Config " << kDegreeStepKey_
           << " invalid: " << FloatToString(degree_step_, 3) << ".";
    return false;
  }

  if (!JsonHelper::GetUInt(*json, kSearchDepthKey_, search_depth_)) {
    ZGERROR << "Config " << kSearchDepthKey_ << " not found.";
  }
  if (search_depth_ < 0 || search_depth_ > UINT8_MAX) {
    ZERROR << "Config " << kSearchDepthKey_
           << " invalid: " << FloatToString(search_depth_, 3) << ".";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kMinScoreKey_, min_score_)) {
    ZGERROR << "Config " << kMinScoreKey_ << " not found.";
  }
  if (min_score_ < 0 || min_score_ > 1) {
    ZERROR << "Config " << kMinScoreKey_
           << " invalid: " << FloatToString(min_score_, 3) << ".";
    return false;
  }

  return true;
}

PointCloudMatcher::NormalSearchParameter::NormalSearchParameter(
    const MapPoint& init_pose,
    const DynamicMapPointBound& search_linear_bound_in_map_frame,
    const float& linear_search_step, const float& degree_range,
    const float& degree_step, const float& min_score)
    : init_pose_(init_pose),
      search_linear_bound_(search_linear_bound_in_map_frame),
      linear_search_step_(linear_search_step),
      degree_range_(degree_range),
      degree_step_(degree_step),
      min_score_(min_score) {
  // ZINFO << "Search for point bound: " << search_linear_bound_.DebugString();
  num_scans_ = 1;
  for (auto degree = degree_step_; degree <= degree_range_;
       degree += degree_step_) {
    num_scans_++;
  }
  for (auto degree = -degree_step_; degree >= -degree_range_;
       degree -= degree_step_) {
    num_scans_++;
  }
}
MapCell PointCloudMatcher::NormalSearchParameter::ConvertToScoreMapCell(
    const float& x, const float& y) const {
  auto _x = x - init_pose_.X();
  auto _y = y - init_pose_.Y();
  auto cell_x = static_cast<int16_t>(round(_x / linear_search_step_));
  auto cell_y = static_cast<int16_t>(round(_y / linear_search_step_));
  return MapCell(cell_x, cell_y);
}

MapPoint PointCloudMatcher::NormalSearchParameter::ScoreMapCellConvertToPose(
    const MapCell& score_map_cell) const {
  return MapPoint(init_pose_.X() + score_map_cell.X() * linear_search_step_,
                  init_pose_.Y() + score_map_cell.Y() * linear_search_step_);
}

FloatValueGridMap2D::SPtr
PointCloudMatcher::NormalSearchParameter::GenerateNewScoreMap() const {
  uint16_t x_half_count = 0;
  for (auto x = init_pose_.X() + linear_search_step_;
       x <= search_linear_bound_.GetMax().X(); x += linear_search_step_) {
    x_half_count++;
  }
  uint16_t y_half_count = 0;
  for (auto y = init_pose_.Y() + linear_search_step_;
       y <= search_linear_bound_.GetMax().Y(); y += linear_search_step_) {
    y_half_count++;
  }
  return std::make_shared<FloatValueGridMap2D>(
      "Score map", 2 * x_half_count + 1, 2 * y_half_count + 1,
      linear_search_step_, -1);
}

PointCloudMatcher::GradientSearchParameter::GradientSearchParameter(
    const MapPoint& init_pose,
    const DynamicMapPointBound& search_linear_bound_in_map_frame,
    const float& linear_search_step, const float& degree_range,
    const float& degree_step, const uint8_t& max_iteration_num,
    const float& min_score)
    : NormalSearchParameter(init_pose, search_linear_bound_in_map_frame,
                            linear_search_step, degree_range, degree_step,
                            min_score),
      max_iteration_num_(max_iteration_num) {}

PointCloudMatcher::BABSearchCandidate::BABSearchCandidate(
    const MapCell& cell_pose, const MapCell& offset_cell,
    const uint16_t& scan_index, const float& degree, const float& offset_degree,
    const float& score)
    : cell_pose_(cell_pose),
      offset_cell_(offset_cell),
      scan_index_(scan_index),
      degree_(degree),
      offset_degree_(offset_degree),
      score_(score) {}


PointCloudMatcher::NormalSearchCandidate::NormalSearchCandidate(
    const MapPoint& pose, const MapPoint& offset_point,
    const uint16_t& scan_index, const float& score)
    : pose_(pose),
      offset_point_(offset_point),
      scan_index_(scan_index),
      score_(score) {}

void PointCloudMatcher::MatchResult::SetMatchResult(const MapPoint& pose,
                                                    const float& score,
                                                    const float& match_time) {
  pose_ = pose;
  score_ = score;
  match_time_ = match_time;

  valid_.store(true);
}

PointCloudMatcher::Config::Config() : Config(nullptr){};

PointCloudMatcher::Config::Config(const JsonSPtr& json) : config_valid_(false) {
  // Load default setting.
  translation_delta_cost_weight_ = 0;
  rotation_degree_delta_cost_weight_ = 0;

  if (json != nullptr) {
    // ZINFO << "Load config from json.";
    config_valid_ = ParseFromJson(json);
  } else {
    JsonSPtr config;
    if (GlobalJsonConfig::Instance()->GetGlobalConfigObject(kConfigKey_,
                                                            config)) {
      // ZINFO << "Load config from global json config.";
      config_valid_ = ParseFromJson(config);
    } else {
      ZGERROR;
    }
  }
};

bool PointCloudMatcher::Config::ParseFromJson(const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }

  // Override.
  if (!JsonHelper::GetFloat(*json, kTranslationDeltaCostWeightKey_,
                            translation_delta_cost_weight_)) {
    ZGERROR << "Config " << kTranslationDeltaCostWeightKey_ << " not found.";
    return false;
  }
  if (translation_delta_cost_weight_ < 0) {
    ZERROR << "Config " << kTranslationDeltaCostWeightKey_
           << " invalid: " << FloatToString(translation_delta_cost_weight_, 3)
           << ".";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kRotationDegreeDeltaCostWeightKey_,
                            rotation_degree_delta_cost_weight_)) {
    ZGERROR << "Config " << kRotationDegreeDeltaCostWeightKey_ << " not found.";
    return false;
  }
  if (rotation_degree_delta_cost_weight_ < 0) {
    ZERROR << "Config " << kRotationDegreeDeltaCostWeightKey_ << " invalid: "
           << FloatToString(rotation_degree_delta_cost_weight_, 3) << ".";
    return false;
  }

  return true;
}

PointCloudMatcher::PointCloudMatcher(const Config& config)
    : translation_delta_cost_weight_(config.translation_delta_cost_weight_),
      rotation_degree_delta_cost_weight_(
          config.rotation_degree_delta_cost_weight_),
      searching_(false),
      stop_cmd_(false){
          // ZINFO << "translation_delta_cost_weight_: "
          //       << FloatToString(translation_delta_cost_weight_, 3)
          //       << ", rotation_degree_delta_cost_weight_: "
          //       << FloatToString(rotation_degree_delta_cost_weight_, 3);
      };

PointCloudMatcher::~PointCloudMatcher() { StopSearch(); }

VMapCells PointCloudMatcher::ToMapCells(const PointCloud::SPtr& point_cloud,
                                        const SlamValueGridMap2D::SPtr& map) {
  ReadLocker lock(point_cloud->GetLock());
  ReadLocker lock2(map->GetLock());
  VMapCells cells;
  for (auto&& point : point_cloud->GetPointsConstRef()) {
    MapCell cell;
    map->WorldToMap(point.ToMapPoint(), cell);
    cells.emplace_back(cell);
  }
  return cells;
}

VMapPoints PointCloudMatcher::ToMapPoints(const PointCloud::SPtr& point_cloud) {
  ReadLocker lock(point_cloud->GetLock());
  VMapPoints points;
  for (auto&& point : point_cloud->GetPointsConstRef()) {
    points.emplace_back(point.ToMapPoint());
  }
  return points;
}

float PointCloudMatcher::SimpleMatchScore(
    const SlamValueGridMap2D::SPtr& map, const MapCell& pose,
    const VMapCells& point_cloud_cells_in_map_frame) {
  float score = 0;
  int64_t sum_value = 0;
  ReadLocker lock(map->GetLock());
  // std::string str;
  for (auto&& cell : point_cloud_cells_in_map_frame) {
    SlamValueGridMap2D::DataType value =
        SlamValueGridMap2D::GetPredefineDefaultValue();
    MapCell point_cloud_cell_in_map(cell.X() + pose.X(), cell.Y() + pose.Y());
    map->GetValue(point_cloud_cell_in_map.X(), point_cloud_cell_in_map.Y(),
                  value);
    // str += std::to_string(value) + ", ";
    sum_value += value;
  }
  // ZINFO << str;

  if (!point_cloud_cells_in_map_frame.empty()) {
    auto average_value =
        (sum_value * 1.0 / point_cloud_cells_in_map_frame.size());
    // ZWARN << "Average: " << FloatToString(average_value, 2);
    score =
        (average_value - map->GetPredefineDefaultValue()) /
        (map->GetPredefineMaxObstacleValue() - map->GetPredefineDefaultValue());
  }

  return score;
}

float PointCloudMatcher::SimpleMatchScore(
    const SlamValueGridMap2D::SPtr& map, const MapPoint& pose,
    const VMapPoints& point_cloud_points_in_map_frame) {
  float score = 0;
  int64_t sum_value = 0;
  ReadLocker lock(map->GetLock());
  // std::string str;
  for (auto&& point : point_cloud_points_in_map_frame) {
    SlamValueGridMap2D::DataType value =
        SlamValueGridMap2D::GetPredefineDefaultValue();
    MapPoint point_cloud_point_in_map(point.X() + pose.X(),
                                      point.Y() + pose.Y());
    MapCell point_cloud_cell_in_map;
    map->WorldToMap(point_cloud_point_in_map, point_cloud_cell_in_map);
    map->GetValue(point_cloud_cell_in_map.X(), point_cloud_cell_in_map.Y(),
                  value);
    // str += std::to_string(value) + ", ";
    sum_value += value;
  }
  // ZINFO << str;

  if (!point_cloud_points_in_map_frame.empty()) {
    auto average_value =
        (sum_value * 1.0 / point_cloud_points_in_map_frame.size());
    // ZWARN << "Average: " << FloatToString(average_value, 2);
    score =
        (average_value - map->GetPredefineDefaultValue()) /
        (map->GetPredefineMaxObstacleValue() - map->GetPredefineDefaultValue());
  }

  return score;
}

float PointCloudMatcher::SimpleMatchScore(
    const ProbabilityIndexGridMap2D::SPtr& map, const MapPoint& pose,
    const VMapPoints& point_cloud_points_in_map_frame,
    const bool& use_interpolator,
    const FloatValueGridStaticMap2D::SPtr cached_probability_map) {
  float score = 0;
  float sum_value = 0;
  ReadLocker lock(map->GetLock());
  // std::string str;
  float probability = 0;
  auto medium_probability = map->GetMediumProbabilityForObstacleInCell();
  for (auto&& point : point_cloud_points_in_map_frame) {
    MapPoint point_cloud_point_in_map(point.X() + pose.X(),
                                      point.Y() + pose.Y());
    if (!use_interpolator) {
      // Directly get probability from map.
      MapCell point_cloud_cell_in_map;
      map->WorldToMap(point_cloud_point_in_map, point_cloud_cell_in_map);
      if (map->IsUnknown(point_cloud_cell_in_map.X(),
                         point_cloud_cell_in_map.Y())) {
        // sum_value += map->GetMinProbabilityForObstacleInCell();
        sum_value += medium_probability;
      } else {
        map->GetProbability(point_cloud_cell_in_map.X(),
                            point_cloud_cell_in_map.Y(), probability);
        // str += std::to_string(value) + ", ";
        // ZINFO << "Probability: " << FloatToString(probability, 4);
        sum_value += probability;
      }
    } else {
      // Get probability from map using bicubic interpolation.
      if (cached_probability_map != nullptr) {
        MapCell cached_cell;
        ReadLocker lock(cached_probability_map->GetLock());
        cached_probability_map->WorldToMap(point_cloud_point_in_map, cached_cell);
        if (cached_probability_map->GetValue(cached_cell.X(), cached_cell.Y(),
                                             probability)) {
          sum_value += probability;
          // ZINFO << "Use cached probability.";
          continue;
        }
      }

      auto _probability = BicubicInterpolator::Interpolate(
          map, point_cloud_point_in_map);
      if (_probability != nullptr) {
        sum_value += *_probability;
        if (cached_probability_map != nullptr) {
          MapCell cached_cell;
          cached_probability_map->WorldToMap(point_cloud_point_in_map,
                                             cached_cell);
          WriteLocker lock(cached_probability_map->GetLock());
          cached_probability_map->SetValue(cached_cell.X(), cached_cell.Y(),
                                           *_probability);
        }
      } else {
        ZERROR << "Failed to get probability for "
               << point_cloud_point_in_map.DebugString();
        sum_value += medium_probability;
      }
    }
  }
  // ZINFO << str;

  if (!point_cloud_points_in_map_frame.empty()) {
    auto average_value = (sum_value / point_cloud_points_in_map_frame.size());
    // ZWARN << "Average: " << FloatToString(average_value, 4);
    score = average_value;
  }

  return score;
}

bool PointCloudMatcher::NormalSearch(
    const SlamValueGridMap2D::SPtr& map,
    const PointCloud::SPtr& point_cloud_in_chassis_frame,
    const NormalSearchParameter& parameter, MatchResult& match_result,
    const bool& enable_weight) {
  if (!map->IsMarked()) {
    ZERROR << "Input map empty.";
    match_result.SetMatchResult(MapPoint(), 0, 0);
    return false;
  }

  StopWatch stop_watch("search stop watch", true, true, true);
  searching_.store(true);
  stop_cmd_.store(false);

  auto rotated_scans_in_map_frame = GenerateRotatedPointCloudPointsInMapFrame(
      point_cloud_in_chassis_frame, parameter);
  ZGINFO << "Rotate scans count: " << rotated_scans_in_map_frame.size();
  auto candidates =
      GenerateNormalCandidates(map, rotated_scans_in_map_frame, parameter);
  ZGINFO << "Candidate count: " << candidates.size();
  if (candidates.empty()) {
    ZERROR << "No candidate generated.";
    return false;
  }

  // point_cloud_in_chassis_frame->Print(map->GetResolution());

  // if (false) {
  //   SlamValueGridMap2D::SPtr debug_map(new SlamValueGridMap2D(
  //       "candidate debug map", map->GetRangeX(), map->GetRangeY(),
  //       map->GetResolution(), map->GetDefaultValue()));
  //   ZINFO << "default:" << std::to_string(map->GetDefaultValue());
  //   WriteLocker lock(debug_map->GetLock());
  //   for (auto&& candidate : lowest_resolution_candidates) {
  //     debug_map->SetValue(candidate.cell_pose_.X(), candidate.cell_pose_.Y(),
  //                         debug_map->GetPredefineMaxObstacleValue() + 1);
  //   }
  //   debug_map.Print(__FILE__, __FUNCTION__, __LINE__);
  // }

  MatchAndSortCandidates(map, rotated_scans_in_map_frame, candidates,
                         enable_weight);
  auto best_candidate = candidates.front();
  ZGINFO << "Match finish, best candidate: " << best_candidate.DebugString()
         << ".";
  if (FLAGS_debug_enable && best_candidate.score_ < parameter.min_score_) {
    DebugPointCloudInMap(
        map, best_candidate.pose_,
        rotated_scans_in_map_frame.at(best_candidate.scan_index_));
  }

  MapPoint match_pose = best_candidate.pose_;
  MapCell match_cell;
  map->WorldToMap(match_pose, match_cell);
  auto pose_score = best_candidate.score_;
  auto cost_time = stop_watch.Duration();
  match_result.SetMatchResult(match_pose, pose_score, cost_time);
  ZGINFO << "Found best pose: " << match_pose.DebugString() << " cell "
         << match_cell.DebugString() << ", degree offset: "
         << FloatToString(best_candidate.offset_point_.Degree(), 1)
         << ", score: " << FloatToString(pose_score, 4);
  ZGINFO << "Search toke " << FloatToString(cost_time, 3) << "s.";

  if (!stop_cmd_.load() && best_candidate.score_ > parameter.min_score_) {
    if (FLAGS_debug_enable) {
      DebugPointCloudInMap(
          map, best_candidate.pose_,
          rotated_scans_in_map_frame.at(best_candidate.scan_index_));
    }
    // auto debug_map_resolution = map->GetResolution();
    // SlamValueGridMap2D debug_map(
    //     "debug", static_cast<uint16_t>(10 / debug_map_resolution),
    //     static_cast<uint16_t>(10 / debug_map_resolution),
    //     debug_map_resolution, -1);
    // auto debug_cells = [&](const BABRotateScan& scan) {
    //   debug_map.ResetMap(debug_map.GetRangeX(), debug_map.GetRangeY(),
    //                      debug_map.GetResolution(),
    //                      debug_map.GetDefaultValue());
    //   WriteLocker lock(debug_map.GetLock());
    //   for (auto&& cell : scan.scan_) {
    //     MapPoint point;
    //     map->MapToWorld(cell, point);
    //     MapCell _cell;
    //     debug_map.WorldToMap(point, _cell);
    //     debug_map.SetValue(_cell.X(), _cell.Y(), 100);
    //   }
    //   ZINFO << "Degree: " << FloatToString(scan.degree_, 2);
    //   debug_map.Print(__FILE__, __FUNCTION__, __LINE__);
    // };
    // debug_cells(rotated_scans_in_map_frame.at(best_candidate.scan_index_));

    ZGINFO << "Pose match requirement.";
    searching_.store(false);
    return true;
  }

  if (stop_cmd_.load()) {
    ZWARN << "Stop for stop cmd.";
  }
  ZWARN << "Failed to find pose that match requirement.";
  point_cloud_in_chassis_frame->Print(map->GetResolution());
  searching_.store(false);
  return false;
}

bool PointCloudMatcher::NormalSearch(
    const ProbabilityIndexGridMap2D::SPtr& map,
    const PointCloud::SPtr& point_cloud_in_chassis_frame,
    const NormalSearchParameter& parameter, MatchResult& match_result,
    const bool& enable_weight) {
  if (!map->IsMarked()) {
    ZGWARN << "Input map empty, consider as match success at init pose.";
    match_result.SetMatchResult(parameter.init_pose_, 1, 0);
    return true;
  }

  if (point_cloud_in_chassis_frame == nullptr) {
    ZGERROR << "Inpub point cloud invalid.";
    return false;
  }

  StopWatch stop_watch("search stop watch", true, true, true);
  searching_.store(true);
  stop_cmd_.store(false);

  auto rotated_scans_in_map_frame = GenerateRotatedPointCloudPointsInMapFrame(
      point_cloud_in_chassis_frame, parameter);
  // {
  //   ReadLocker lock(point_cloud_in_chassis_frame->GetLock());
  //   ZGINFO << "Rotate scans count: " << rotated_scans_in_map_frame.size()
  //          << ", each point cloud size: "
  //          << point_cloud_in_chassis_frame->GetPointsConstRef().size();
  // }
  auto candidates =
      GenerateNormalCandidates(map, rotated_scans_in_map_frame, parameter);
  // ZGINFO << "Candidate count: " << candidates.size();
  if (candidates.empty()) {
    ZERROR << "No candidate generated.";
    return false;
  }

  // point_cloud_in_chassis_frame->Print(map->GetResolution());

  // if (false) {
  //   SlamValueGridMap2D::SPtr debug_map(new SlamValueGridMap2D(
  //       "candidate debug map", map->GetRangeX(), map->GetRangeY(),
  //       map->GetResolution(), map->GetDefaultValue()));
  //   ZINFO << "default:" << std::to_string(map->GetDefaultValue());
  //   WriteLocker lock(debug_map->GetLock());
  //   for (auto&& candidate : lowest_resolution_candidates) {
  //     debug_map->SetValue(candidate.cell_pose_.X(), candidate.cell_pose_.Y(),
  //                         debug_map->GetPredefineMaxObstacleValue() + 1);
  //   }
  //   debug_map.Print(__FILE__, __FUNCTION__, __LINE__);
  // }

  MatchAndSortCandidates(map, rotated_scans_in_map_frame, candidates,
                         enable_weight);
  auto best_candidate = candidates.front();
  // ZGINFO << "Match finish, best candidate: " << best_candidate.DebugString();
  if (FLAGS_debug_enable && best_candidate.score_ < parameter.min_score_) {
    // DebugPointCloudInMap(
    //     map, best_candidate.pose_,
    //     rotated_scans_in_map_frame.at(best_candidate.scan_index_));
  }

  MapPoint match_pose = best_candidate.pose_;
  MapCell match_cell;
  map->WorldToMap(match_pose, match_cell);
  auto pose_score = best_candidate.score_;
  auto cost_time = stop_watch.Duration();
  match_result.SetMatchResult(match_pose, pose_score, cost_time);
  // ZINFO << "Found best pose: " << match_pose.DebugString() << " cell "
  //       << match_cell.DebugString() << ", degree offset: "
  //       << FloatToString(best_candidate.offset_point_.Degree(), 1)
  //       << ", score: " << FloatToString(pose_score, 4) << ". Search toke "
  //       << FloatToString(cost_time, 3) << "s.";
  if (IsDebugFuncEnabled()) {
    GenerateScoreGraph(candidates, parameter);
  }

  if (!stop_cmd_.load() && best_candidate.score_ > parameter.min_score_) {
    // if (FLAGS_debug_enable) {
    //   DebugPointCloudInMap(
    //       map, best_candidate.pose_,
    //       rotated_scans_in_map_frame.at(best_candidate.scan_index_));
    // }
    // auto debug_map_resolution = map->GetResolution();
    // SlamValueGridMap2D debug_map(
    //     "debug", static_cast<uint16_t>(10 / debug_map_resolution),
    //     static_cast<uint16_t>(10 / debug_map_resolution),
    //     debug_map_resolution, -1);
    // auto debug_cells = [&](const BABRotateScan& scan) {
    //   debug_map.ResetMap(debug_map.GetRangeX(), debug_map.GetRangeY(),
    //                      debug_map.GetResolution(),
    //                      debug_map.GetDefaultValue());
    //   WriteLocker lock(debug_map.GetLock());
    //   for (auto&& cell : scan.scan_) {
    //     MapPoint point;
    //     map->MapToWorld(cell, point);
    //     MapCell _cell;
    //     debug_map.WorldToMap(point, _cell);
    //     debug_map.SetValue(_cell.X(), _cell.Y(), 100);
    //   }
    //   ZINFO << "Degree: " << FloatToString(scan.degree_, 2);
    //   debug_map.Print(__FILE__, __FUNCTION__, __LINE__);
    // };
    // debug_cells(rotated_scans_in_map_frame.at(best_candidate.scan_index_));

    // ZINFO << "Pose match requirement.";
    searching_.store(false);
    return true;
  }

  if (stop_cmd_.load()) {
    ZWARN << "Stop for stop cmd.";
  }
  ZWARN << "Failed to find pose that match requirement.";
  point_cloud_in_chassis_frame->Print(map->GetResolution());
  searching_.store(false);
  return false;
}

bool PointCloudMatcher::GradientSearch(
    const ProbabilityIndexGridMap2D::SPtr& map,
    const PointCloud::SPtr& point_cloud_in_chassis_frame,
    const GradientSearchParameter& parameter, MatchResult& match_result,
    const bool& enable_weight,
    const FloatValueGridStaticMap2D::SPtr cached_probability_map) {
  if (!map->IsMarked()) {
    ZGWARN << "Input map empty, consider as match success at init pose.";
    match_result.SetMatchResult(parameter.init_pose_, 1, 0);
    return true;
  }

  StopWatch stop_watch("search stop watch", true, true, true);
  searching_.store(true);
  stop_cmd_.store(false);

  auto rotated_scans_in_map_frame = static_cast<GradientRotateScans>(
      GenerateRotatedPointCloudPointsInMapFrame(point_cloud_in_chassis_frame,
                                                parameter));
  // ZGINFO << "Rotate scans count: " << rotated_scans_in_map_frame.size();

  auto match_pose = parameter.init_pose_;
  FloatValueGridMap2D::DataType pose_score = -1;
  auto index = 0;

  uint16_t x_half_count = 0;
  for (auto x = parameter.init_pose_.X() + parameter.linear_search_step_;
       x <= parameter.search_linear_bound_.GetMax().X();
       x += parameter.linear_search_step_) {
    x_half_count++;
  }
  uint16_t y_half_count = 0;
  for (auto y = parameter.init_pose_.Y() + parameter.linear_search_step_;
       y <= parameter.search_linear_bound_.GetMax().Y();
       y += parameter.linear_search_step_) {
    y_half_count++;
  }
  if (score_map_ == nullptr) {
    score_map_ = std::make_shared<FloatValueGridMap2D>(
        "Score map", 2 * x_half_count + 1, 2 * y_half_count + 1,
        parameter.linear_search_step_, -1);
  }
  // auto score_map = parameter.GenerateNewScoreMap();
  auto score_map = score_map_;
  for (auto&& scan : rotated_scans_in_map_frame) {
    // ZINFO << "Process scan " << std::to_string(index) << ", degree: " <<
    // scan.degree_;

    score_map->ResetMap(2 * x_half_count + 1, 2 * y_half_count + 1,
                        parameter.linear_search_step_, -1);
    score_map->ChangeName(std::to_string(index) + " " + score_map->Name());

    auto curr_cell = MapCell(0, 0);
    auto iteration_count = 1;
    // auto move_count = 0;
    const bool use_8_direction = IsRunningOnX86();
    while (true) {
      auto next_cell =
          MatchForGradient(map, score_map, scan, curr_cell, parameter,
                           use_8_direction, true, cached_probability_map);
      if (next_cell == curr_cell) {
        // Get Best match.
        break;
      }
      if (++iteration_count > parameter.max_iteration_num_) {
        // Reach max iteration count.
        break;
      }
      // ZINFO << "Move from " << curr_cell.DebugString() << " to "
      //       << next_cell.DebugString();
      curr_cell = next_cell;
      // move_count++;
    }

    // ZGINFO << "Process scan " << std::to_string(index)
    //        << ", degree: " << scan.degree_ << ", move count " << move_count;
    FloatValueGridMap2D::DataType score = score_map->GetDefaultValue();
    ReadLocker lock(score_map->GetLock());
    score_map->GetValue(curr_cell.X(), curr_cell.Y(), score);
    // ZINFO << "Scan best Score: " << FloatToString(score, 8);
    if (score > pose_score) {
      pose_score = score;
      match_pose = parameter.ScoreMapCellConvertToPose(curr_cell);
      match_pose.SetDegree(scan.degree_);
    }

    // ZINFO << "Current best pose: " << match_pose.DebugString()
    //       << ", degree offset: "
    //       << FloatToString(match_pose.Degree() -
    //       parameter.init_pose_.Degree(),
    //                        1)
    //       << ", score: " << FloatToString(pose_score, 8);

    if (IsDebugFuncEnabled()) {
      score_map->PrintWithPrecision(__FILE__, __FUNCTION__, __LINE__, 2);
      score_map->Print(__FILE__, __FUNCTION__, __LINE__);
    }

    index++;
  }

  MapCell match_cell;
  map->WorldToMap(match_pose, match_cell);
  auto cost_time = stop_watch.Duration();
  match_result.SetMatchResult(match_pose, pose_score, cost_time);
  // ZGINFO << "Found best pose: " << match_pose.DebugString() << " cell "
  //        << match_cell.DebugString() << ", degree offset: "
  //        << FloatToString(match_pose.Degree() - parameter.init_pose_.Degree(),
  //                         1)
  //        << ", score: " << FloatToString(pose_score, 8) << ". Search toke "
  //        << FloatToString(cost_time, 3) << "s.";

  // ZINFO << "Match finish, best candidate: " << best_candidate.DebugString();
  if (FLAGS_debug_enable && pose_score < parameter.min_score_) {
    // DebugPointCloudInMap(
    //     map, best_candidate.pose_,
    //     rotated_scans_in_map_frame.at(best_candidate.scan_index_));
  }

  if (!stop_cmd_.load() && pose_score > parameter.min_score_) {
    // if (FLAGS_debug_enable) {
    //   DebugPointCloudInMap(
    //       map, best_candidate.pose_,
    //       rotated_scans_in_map_frame.at(best_candidate.scan_index_));
    // }
    // auto debug_map_resolution = map->GetResolution();
    // SlamValueGridMap2D debug_map(
    //     "debug", static_cast<uint16_t>(10 / debug_map_resolution),
    //     static_cast<uint16_t>(10 / debug_map_resolution),
    //     debug_map_resolution, -1);
    // auto debug_cells = [&](const BABRotateScan& scan) {
    //   debug_map.ResetMap(debug_map.GetRangeX(), debug_map.GetRangeY(),
    //                      debug_map.GetResolution(),
    //                      debug_map.GetDefaultValue());
    //   WriteLocker lock(debug_map.GetLock());
    //   for (auto&& cell : scan.scan_) {
    //     MapPoint point;
    //     map->MapToWorld(cell, point);
    //     MapCell _cell;
    //     debug_map.WorldToMap(point, _cell);
    //     debug_map.SetValue(_cell.X(), _cell.Y(), 100);
    //   }
    //   ZINFO << "Degree: " << FloatToString(scan.degree_, 2);
    //   debug_map.Print(__FILE__, __FUNCTION__, __LINE__);
    // };
    // debug_cells(rotated_scans_in_map_frame.at(best_candidate.scan_index_));

    // ZINFO << "Pose match requirement.";
    searching_.store(false);
    return true;
  }

  if (stop_cmd_.load()) {
    ZWARN << "Stop for stop cmd.";
  }
  ZWARN << "Failed to find pose that match requirement.";
  point_cloud_in_chassis_frame->Print(map->GetResolution());
  searching_.store(false);
  return false;
}

bool PointCloudMatcher::BABSearch(
    const SlamValueGridMap2D::SPtr& map,
    const PointCloud::SPtr& point_cloud_in_chassis_frame,
    const BABSearchParameter& parameter, MatchResult& match_result) {
  if (!map->IsMarked()) {
    ZERROR << "Input map empty.";
    match_result.SetMatchResult(MapPoint(), 0, 0);
    return false;
  }

  StopWatch stop_watch("search stop watch", true, true, true);
  searching_.store(true);
  stop_cmd_.store(false);
  MultiResolutionSlamValueGridMap2D::SPtr multi_resolution_slam_map(
      new MultiResolutionSlamValueGridMap2D("Multi resolution slam map", map,
                                            parameter.search_depth_));
  auto rotated_scans_in_map_frame = GenerateRotatedPointCloudCellsInMapFrame(
      point_cloud_in_chassis_frame, map, parameter);
  ZGINFO << "Rotate scans count: " << rotated_scans_in_map_frame.size();
  auto lowest_resolution_candidates =
      GenerateBABCandidates(map, rotated_scans_in_map_frame, parameter);
  ZGINFO << "Lowest resolution candidate count: "
         << lowest_resolution_candidates.size();

  // point_cloud_in_chassis_frame->Print(map->GetResolution());

  // if (false) {
  //   SlamValueGridMap2D::SPtr debug_map(new SlamValueGridMap2D(
  //       "candidate debug map", map->GetRangeX(), map->GetRangeY(),
  //       map->GetResolution(), map->GetDefaultValue()));
  //   ZINFO << "default:" << std::to_string(map->GetDefaultValue());
  //   WriteLocker lock(debug_map->GetLock());
  //   for (auto&& candidate : lowest_resolution_candidates) {
  //     debug_map->SetValue(candidate.cell_pose_.X(), candidate.cell_pose_.Y(),
  //                         debug_map->GetPredefineMaxObstacleValue() + 1);
  //   }
  //   debug_map.Print(__FILE__, __FUNCTION__, __LINE__);
  // }

  SlamValueGridMap2D::SPtr lowest_resolution_map;
  if (!multi_resolution_slam_map->GetLayer(parameter.search_depth_ - 1,
                                           lowest_resolution_map)) {
    ZERROR << "Insufficient map layer, expecting layer "
           << (parameter.search_depth_ - 1);
    return false;
  }
  MatchAndSortCandidates(lowest_resolution_map, rotated_scans_in_map_frame,
                         lowest_resolution_candidates);
  ZGINFO << "First match finish, best candidate: "
         << lowest_resolution_candidates.front().DebugString() << ".";
  if (FLAGS_debug_enable &&
      lowest_resolution_candidates.front().score_ < parameter.min_score_) {
    DebugPointCloudInMap(map, lowest_resolution_candidates.front().cell_pose_,
                         rotated_scans_in_map_frame.at(
                             lowest_resolution_candidates.front().scan_index_));
  }

  ZGINFO << "Start BAB search.";
  auto best_candidate =
      BranchAndBound(multi_resolution_slam_map, rotated_scans_in_map_frame,
                     parameter, lowest_resolution_candidates,
                     parameter.search_depth_ - 1, parameter.min_score_);

  MapPoint match_pose;
  map->MapToWorld(best_candidate.cell_pose_, match_pose);
  match_pose.SetDegree(NormalizeDegree(best_candidate.degree_));
  auto pose_score = best_candidate.score_;
  auto cost_time = stop_watch.Duration();
  match_result.SetMatchResult(match_pose, pose_score, cost_time);
  ZGINFO << "Found best pose: " << match_pose.DebugString() << " cell "
         << best_candidate.cell_pose_.DebugString()
         << ", degree offset: " << FloatToString(best_candidate.degree_, 1)
         << ", score: " << FloatToString(pose_score, 4);
  ZGINFO << "Search toke " << FloatToString(cost_time, 3) << "s.";

  if (!stop_cmd_.load() && best_candidate.score_ > parameter.min_score_) {
    if (FLAGS_debug_enable) {
      DebugPointCloudInMap(
          map, best_candidate.cell_pose_,
          rotated_scans_in_map_frame.at(best_candidate.scan_index_));
    }
    // auto debug_map_resolution = map->GetResolution();
    // SlamValueGridMap2D debug_map(
    //     "debug", static_cast<uint16_t>(10 / debug_map_resolution),
    //     static_cast<uint16_t>(10 / debug_map_resolution),
    //     debug_map_resolution, -1);
    // auto debug_cells = [&](const BABRotateScan& scan) {
    //   debug_map.ResetMap(debug_map.GetRangeX(), debug_map.GetRangeY(),
    //                      debug_map.GetResolution(),
    //                      debug_map.GetDefaultValue());
    //   WriteLocker lock(debug_map.GetLock());
    //   for (auto&& cell : scan.scan_) {
    //     MapPoint point;
    //     map->MapToWorld(cell, point);
    //     MapCell _cell;
    //     debug_map.WorldToMap(point, _cell);
    //     debug_map.SetValue(_cell.X(), _cell.Y(), 100);
    //   }
    //   ZINFO << "Degree: " << FloatToString(scan.degree_, 2);
    //   debug_map.Print(__FILE__, __FUNCTION__, __LINE__);
    // };
    // debug_cells(rotated_scans_in_map_frame.at(best_candidate.scan_index_));

    ZGINFO << "Pose match requirement, search toke "
          << FloatToString(cost_time, 3) << "s.";
    searching_.store(false);
    return true;
  }

  if (stop_cmd_.load()) {
    ZWARN << "Stop for stop cmd.";
  }
  ZWARN << "Failed to find pose that match requirement, search toke "
        << FloatToString(cost_time, 3) << "s.";
  point_cloud_in_chassis_frame->Print(map->GetResolution());
  searching_.store(false);
  return false;
}

bool PointCloudMatcher::BoostBABSearch(
    const SlamValueGridMap2D::SPtr& map,
    const PointCloud::SPtr& point_cloud_in_chassis_frame,
    const BABSearchParameter& parameter, MatchResult& match_result) {
  if (!map->IsMarked()) {
    ZERROR << "Input map empty.";
    match_result.SetMatchResult(MapPoint(), 0, 0);
    return false;
  }

  StopWatch stop_watch("search stop watch", true, true, true);
  searching_.store(true);
  stop_cmd_.store(false);
  MultiResolutionSlamValueGridMap2D::SPtr multi_resolution_slam_map(
      new MultiResolutionSlamValueGridMap2D("Multi resolution slam map", map,
                                            parameter.search_depth_));
  auto rotated_scans_in_map_frame = GenerateRotatedPointCloudCellsInMapFrame(
      point_cloud_in_chassis_frame, map, parameter);
  ZGINFO << "Rotate scans count: " << rotated_scans_in_map_frame.size();
  auto lowest_resolution_candidates =
      GenerateBABCandidates(map, rotated_scans_in_map_frame, parameter);
  ZGINFO << "Lowest resolution candidate count: "
        << lowest_resolution_candidates.size();

  // if (false) {
  //   SlamValueGridMap2D::SPtr debug_map(new SlamValueGridMap2D(
  //       "candidate debug map", map->GetRangeX(), map->GetRangeY(),
  //       map->GetResolution(), map->GetDefaultValue()));
  //   ZINFO << "default:" << std::to_string(map->GetDefaultValue());
  //   WriteLocker lock(debug_map->GetLock());
  //   for (auto&& candidate : lowest_resolution_candidates) {
  //     debug_map->SetValue(candidate.cell_pose_.X(), candidate.cell_pose_.Y(),
  //                         debug_map->GetPredefineMaxObstacleValue() + 1);
  //   }
  //   debug_map->Print(__FILE__, __FUNCTION__, __LINE__);
  // }

  SlamValueGridMap2D::SPtr lowest_resolution_map;
  if (!multi_resolution_slam_map->GetLayer(parameter.search_depth_ - 1,
                                           lowest_resolution_map)) {
    ZERROR << "Insufficient map layer, expecting layer "
           << (parameter.search_depth_ - 1);
    return false;
  }
  BoostMatchAndSortCandidates(lowest_resolution_map, rotated_scans_in_map_frame,
                              lowest_resolution_candidates);
  ZGINFO << "First match finish.";

  ReadWriteLock::SPtr best_candidate_lock(std::make_shared<ReadWriteLock>());
  ReadWriteLock::SPtr lowest_resolution_candidates_iter_lock(
      std::make_shared<ReadWriteLock>());
  auto lowest_resolution_candidates_iter = lowest_resolution_candidates.begin();
  BABSearchCandidate best_candidate(MapCell(0, 0), MapCell(0, 0), 0, 0, 0,
                                    parameter.min_score_);
  SharedScoreBoard score_board(parameter.min_score_);
  std::vector<std::thread> thread_pool;

  auto boost_BAB_func = [&](const uint16_t& index) -> void {
    ZGINFO << "Thread " << index << " start.";
    while (true) {
      if (stop_cmd_.load()) {
        ZWARN << "Stop for stop cmd.";
        break;
      }
      BABSearchCandidates tmp_candidate_list;
      {
        // ZINFO << "Thread " << index << " want to access lock";
        WriteLocker lock(lowest_resolution_candidates_iter_lock);
        // ZINFO << "Thread " << index << " access lock";
        if (lowest_resolution_candidates_iter ==
            lowest_resolution_candidates.end()) {
          break;
        }
        // ZINFO << "Process for "
        //       << lowest_resolution_candidates_iter->DebugString();
        tmp_candidate_list.emplace_back(*lowest_resolution_candidates_iter);
        lowest_resolution_candidates_iter++;
      }

      auto _best_candidate = BoostBranchAndBound(
          multi_resolution_slam_map, rotated_scans_in_map_frame, parameter,
          tmp_candidate_list, parameter.search_depth_ - 1,
          score_board.GetCurrentScore(), score_board);

      {
        WriteLocker lock(best_candidate_lock);

        if (_best_candidate.score_ > best_candidate.score_) {
          ZGINFO << "Get best candidate: " << _best_candidate.DebugString();
          ZGINFO << "Current candidate: " << best_candidate.DebugString();
          best_candidate = _best_candidate;
          ZGINFO << "Update best candidate as: " << best_candidate.DebugString();
        }
      }
    };
    ZGINFO << "Thread " << index << " exit.";
  };

  for (auto index = 0u; index < boost_thread_count; index++) {
    thread_pool.emplace_back(std::thread(boost_BAB_func, index));
  }

  for (auto&& t : thread_pool) {
    t.join();
  }

  MapPoint match_pose;
  map->MapToWorld(best_candidate.cell_pose_, match_pose);
  match_pose.SetDegree(NormalizeDegree(best_candidate.degree_));
  auto pose_score = best_candidate.score_;
  auto cost_time = stop_watch.Duration();
  match_result.SetMatchResult(match_pose, pose_score, cost_time);
  ZGINFO << "Found best pose: " << match_pose.DebugString() << " cell "
         << best_candidate.cell_pose_.DebugString()
         << " score: " << FloatToString(pose_score, 4);
  ZGINFO << "Search toke " << FloatToString(cost_time, 3) << "s.";

  if (!stop_cmd_.load() && best_candidate.score_ > parameter.min_score_) {
    if (FLAGS_debug_enable) {
      DebugPointCloudInMap(
          map, best_candidate.cell_pose_,
          rotated_scans_in_map_frame.at(best_candidate.scan_index_));
    }
    ZGINFO << "Found best pose: " << match_pose.DebugString() << " cell "
           << best_candidate.cell_pose_.DebugString()
           << " score: " << FloatToString(pose_score, 4);
    ZGINFO << "Search toke " << FloatToString(cost_time, 3) << "s.";
    searching_.store(false);
    return true;
  }

  if (stop_cmd_.load()) {
    ZWARN << "Stop for stop cmd.";
  }
  ZWARN << "Failed to find pose that match requirement.";
  point_cloud_in_chassis_frame->Print(map->GetResolution());
  searching_.store(false);
  return false;
}

void PointCloudMatcher::StopSearch() {
  ZGINFO << "Wait for stopping search.";
  while (searching_.load()) {
    if (!stop_cmd_.load()) {
      ZGINFO << "Stop search.";
      stop_cmd_.store(true);
    }
    Time::SleepMSec(10);
  }
  ZGINFO << "Search stopped.";
}

void PointCloudMatcher::DebugPointCloudInMap(
    const SlamValueGridMap2D::SPtr& map, const MapCell& cell_pose,
    const BABRotateScan& scan) {
  SlamValueGridMap2D::SPtr debug_map(new SlamValueGridMap2D(*map));
  WriteLocker lock(debug_map->GetLock());
  ZGINFO << "Scan center cell " << cell_pose.DebugString();
  for (auto&& cell : scan.scan_) {
    auto cell_in_map = cell + cell_pose;
    SlamValueGridMap2D::DataType value =
        SlamValueGridMap2D::GetPredefineDefaultValue();
    debug_map->GetValue(cell_in_map.X(), cell_in_map.Y(), value);
    if (value < SlamValueGridMap2D::GetPredefineMediumValue()) {
      debug_map->SetValue(cell_in_map.X(), cell_in_map.Y(),
                          SlamValueGridMap2D::GetPredefineDefaultValue() - 1);
    } else {
      debug_map->SetValue(
          cell_in_map.X(), cell_in_map.Y(),
          SlamValueGridMap2D::GetPredefineMaxObstacleValue() + 1);
    }
  }
  debug_map->Print(__FILE__, __FUNCTION__, __LINE__);
}

void PointCloudMatcher::DebugPointCloudInMap(
    const SlamValueGridMap2D::SPtr& map, const MapPoint& pose,
    const NormalRotateScan& scan) {
  SlamValueGridMap2D::SPtr debug_map(new SlamValueGridMap2D(*map));
  WriteLocker lock(debug_map->GetLock());
  MapCell cell_pose;
  debug_map->WorldToMap(pose, cell_pose);
  ZGINFO << "Scan center cell " << cell_pose.DebugString();
  for (auto&& point : scan.scan_) {
    MapCell cell;
    debug_map->WorldToMap(point, cell);
    auto cell_in_map = cell + cell_pose;
    SlamValueGridMap2D::DataType value =
        SlamValueGridMap2D::GetPredefineDefaultValue();
    debug_map->GetValue(cell_in_map.X(), cell_in_map.Y(), value);
    if (value < SlamValueGridMap2D::GetPredefineMediumValue()) {
      debug_map->SetValue(cell_in_map.X(), cell_in_map.Y(),
                          SlamValueGridMap2D::GetPredefineDefaultValue() - 1);
    } else {
      debug_map->SetValue(
          cell_in_map.X(), cell_in_map.Y(),
          SlamValueGridMap2D::GetPredefineMaxObstacleValue() + 1);
    }
  }
  debug_map->Print(__FILE__, __FUNCTION__, __LINE__);
}

void PointCloudMatcher::DebugPointCloudInMap(
    const ProbabilityIndexGridMap2D::SPtr& map, const MapPoint& pose,
    const NormalRotateScan& scan) {
  SlamValueGridMap2D::SPtr debug_map(new SlamValueGridMap2D(
      "Debug map", map->GetRangeX(), map->GetRangeY(), map->GetResolution()));
  WriteLocker lock(debug_map->GetLock());
  MapCell cell_pose;
  debug_map->WorldToMap(pose, cell_pose);
  ZGINFO << "Scan center cell " << cell_pose.DebugString();
  for (auto&& point : scan.scan_) {
    MapCell cell;
    debug_map->WorldToMap(point, cell);
    auto cell_in_map = cell + cell_pose;
    SlamValueGridMap2D::DataType value =
        SlamValueGridMap2D::GetPredefineDefaultValue();
    debug_map->GetValue(cell_in_map.X(), cell_in_map.Y(), value);
    if (value < SlamValueGridMap2D::GetPredefineMediumValue()) {
      debug_map->SetValue(cell_in_map.X(), cell_in_map.Y(),
                          SlamValueGridMap2D::GetPredefineDefaultValue() - 1);
    } else {
      debug_map->SetValue(
          cell_in_map.X(), cell_in_map.Y(),
          SlamValueGridMap2D::GetPredefineMaxObstacleValue() + 1);
    }
  }
  debug_map->Print(__FILE__, __FUNCTION__, __LINE__);
}

PointCloudMatcher::BABRotateScans
PointCloudMatcher::GenerateRotatedPointCloudCellsInMapFrame(
    const PointCloud::SPtr& point_cloud_in_chassis_frame,
    const SlamValueGridMap2D::SPtr& map, const BABSearchParameter& parameter) {
  BABRotateScans rotated_scans_in_map_frame;

  auto pose = parameter.init_pose_;
  // auto debug_map_resolution = map->GetResolution();
  // SlamValueGridMap2D debug_map(
  //     "debug", static_cast<uint16_t>(10 / debug_map_resolution),
  //     static_cast<uint16_t>(10 / debug_map_resolution), debug_map_resolution,
  //     -1);
  // auto debug_cells = [&](const BABRotateScan& scan) {
  //   debug_map.ResetMap(debug_map.GetRangeX(), debug_map.GetRangeY(),
  //                      debug_map.GetResolution(),
  //                      debug_map.GetDefaultValue());
  //   WriteLocker lock(debug_map.GetLock());
  //   for (auto&& cell : scan.scan_) {
  //     MapPoint point;
  //     map->MapToWorld(cell, point);
  //     MapCell _cell;
  //     debug_map.WorldToMap(point, _cell);
  //     debug_map.SetValue(_cell.X(), _cell.Y(), 100);
  //   }
  //   ZINFO << "Degree: " << FloatToString(scan.degree_, 2);
  //   debug_map->Print(__FILE__, __FUNCTION__, __LINE__);
  // };
  {
    ReadLocker lock(point_cloud_in_chassis_frame->GetLock());
    ZGINFO << "Point cloud size: " << point_cloud_in_chassis_frame->Size();
  }
  ZGINFO << "Search range: " << FloatToString(parameter.degree_range_, 2);

  // if (parameter.wall_degree_ != nullptr) {
  //   pose.SetDegree(NormalizeDegree(*parameter.wall_degree_));
  // }

  float pose_degree_offset_for_wall = 0;
  for (auto pose_count = 1;
       pose_count <= (parameter.optimized_for_known_wall_degree_ ? 4 : 1);
       pose_count++) {
    ZGINFO << "Generate for pose: " << pose.DebugString()
           << " with pose degree offset for wall: "
           << FloatToString(pose_degree_offset_for_wall, 1);
    rotated_scans_in_map_frame.emplace_back(
        ToMapCells(point_cloud_in_chassis_frame->TransformBA(
                       point_cloud_in_chassis_frame->Name(),
                       MapPoint(0, 0,
                                NormalizeDegree(pose.Degree() +
                                                pose_degree_offset_for_wall))),
                   map),
        NormalizeDegree(pose.Degree() + pose_degree_offset_for_wall),
        pose_degree_offset_for_wall);
    // debug_cells(rotated_scans_in_map_frame.back());
    for (auto degree_offset = parameter.degree_step_;
         degree_offset <= parameter.degree_range_;
         degree_offset += parameter.degree_step_) {
      rotated_scans_in_map_frame.emplace_back(
          ToMapCells(point_cloud_in_chassis_frame->TransformBA(
                         point_cloud_in_chassis_frame->Name(),
                         MapPoint(0, 0,
                                  NormalizeDegree(pose.Degree() +
                                                  pose_degree_offset_for_wall +
                                                  degree_offset))),
                     map),
          NormalizeDegree(pose.Degree() + pose_degree_offset_for_wall +
                          degree_offset),
          pose_degree_offset_for_wall + degree_offset);
      // debug_cells(rotated_scans_in_map_frame.back());
    }
    for (auto degree_offset = -parameter.degree_step_;
         degree_offset >= -parameter.degree_range_;
         degree_offset -= parameter.degree_step_) {
      rotated_scans_in_map_frame.emplace_back(
          ToMapCells(point_cloud_in_chassis_frame->TransformBA(
                         point_cloud_in_chassis_frame->Name(),
                         MapPoint(0, 0,
                                  NormalizeDegree(pose.Degree() +
                                                  pose_degree_offset_for_wall +
                                                  degree_offset))),
                     map),
          NormalizeDegree(pose.Degree() + pose_degree_offset_for_wall +
                          degree_offset),
          pose_degree_offset_for_wall + degree_offset);
      // debug_cells(rotated_scans_in_map_frame.back());
    }

    // Rotate 90 degree for next pose.
    pose_degree_offset_for_wall += 90;
  }

  if (rotated_scans_in_map_frame.size() != parameter.num_scans_) {
    ZERROR << "Scans count(" << rotated_scans_in_map_frame.size()
           << ") but parameter is set to " << parameter.num_scans_;
    rotated_scans_in_map_frame.clear();
    rotated_scans_in_map_frame.emplace_back(
        ToMapCells(point_cloud_in_chassis_frame->TransformBA(
                       point_cloud_in_chassis_frame->Name(),
                       MapPoint(0, 0, pose.Degree())),
                   map),
        pose.Degree(), 0);
  }
  return rotated_scans_in_map_frame;
}

PointCloudMatcher::NormalRotateScans
PointCloudMatcher::GenerateRotatedPointCloudPointsInMapFrame(
    const PointCloud::SPtr& point_cloud_in_chassis_frame,
    const NormalSearchParameter& parameter) {
  NormalRotateScans rotated_scans_in_map_frame;

  auto pose = parameter.init_pose_;
  // {
  //   ReadLocker lock(point_cloud_in_chassis_frame->GetLock());
  //   ZINFO << "Point cloud size: " << point_cloud_in_chassis_frame->Size();
  // }
  // ZINFO << "Search range: " << FloatToString(parameter.degree_range_, 2);

  // if (parameter.wall_degree_ != nullptr) {
  //   pose.SetDegree(NormalizeDegree(*parameter.wall_degree_));
  // }

  // ZINFO << "Generate for pose: " << pose.DebugString();
  rotated_scans_in_map_frame.emplace_back(
      ToMapPoints(point_cloud_in_chassis_frame->TransformBA(
          point_cloud_in_chassis_frame->Name(), MapPoint(0, 0, pose.Degree()))),
      NormalizeDegree(pose.Degree()), 0);
  // debug_cells(rotated_scans_in_map_frame.back());
  for (auto degree_offset = parameter.degree_step_;
       degree_offset <= parameter.degree_range_;
       degree_offset += parameter.degree_step_) {
    rotated_scans_in_map_frame.emplace_back(
        ToMapPoints(point_cloud_in_chassis_frame->TransformBA(
            point_cloud_in_chassis_frame->Name(),
            MapPoint(0, 0, NormalizeDegree(pose.Degree() + degree_offset)))),
        NormalizeDegree(pose.Degree() + degree_offset), degree_offset);
    // debug_cells(rotated_scans_in_map_frame.back());
  }
  for (auto degree_offset = -parameter.degree_step_;
       degree_offset >= -parameter.degree_range_;
       degree_offset -= parameter.degree_step_) {
    rotated_scans_in_map_frame.emplace_back(
        ToMapPoints(point_cloud_in_chassis_frame->TransformBA(
            point_cloud_in_chassis_frame->Name(),
            MapPoint(0, 0, NormalizeDegree(pose.Degree() + degree_offset)))),
        NormalizeDegree(pose.Degree() + degree_offset), degree_offset);
    // debug_cells(rotated_scans_in_map_frame.back());
  }

  if (rotated_scans_in_map_frame.size() != parameter.num_scans_) {
    ZERROR << "Scans count(" << rotated_scans_in_map_frame.size()
           << ") but parameter is set to " << parameter.num_scans_;
    rotated_scans_in_map_frame.clear();
    rotated_scans_in_map_frame.emplace_back(
        ToMapPoints(point_cloud_in_chassis_frame->TransformBA(
            point_cloud_in_chassis_frame->Name(),
            MapPoint(0, 0, pose.Degree()))),
        pose.Degree(), 0);
  }
  return rotated_scans_in_map_frame;
}

PointCloudMatcher::BABSearchCandidates PointCloudMatcher::GenerateBABCandidates(
    const SlamValueGridMap2D::SPtr& map,
    const BABRotateScans& rotated_scans_in_map_frame,
    const BABSearchParameter& parameter) {
  BABSearchCandidates candidates;
  uint32_t origin_candidate_count = 0;

  const int linear_step_size = 1 << (parameter.search_depth_ - 1);
  ReadLocker lock(map->GetLock());
  SlamValueGridMap2D::DataType value;

  MapCell init_cell;
  map->WorldToMap(parameter.init_pose_, init_cell);
  for (auto y = parameter.search_linear_cell_bound_.GetMin().Y();
       y <= parameter.search_linear_cell_bound_.GetMax().Y();
       y += linear_step_size) {
    for (auto x = parameter.search_linear_cell_bound_.GetMin().X();
         x <= parameter.search_linear_cell_bound_.GetMax().X();
         x += linear_step_size) {
      bool cell_in_valid_area = false;
      if (map->GetValue(x, y, value) &&
          value != SlamValueGridMap2D::GetPredefineDefaultValue()) {
        cell_in_valid_area = true;
      }
      for (auto scan_index = 0u; scan_index < parameter.num_scans_;
           scan_index++) {
        origin_candidate_count++;
        if (cell_in_valid_area) {
          candidates.emplace_back(BABSearchCandidate(
              MapCell(x + linear_step_size - 1, y + linear_step_size - 1),
              MapCell(x + linear_step_size - 1 - init_cell.X(),
                      y + linear_step_size - 1 - init_cell.Y()),
              scan_index, rotated_scans_in_map_frame.at(scan_index).degree_,
              rotated_scans_in_map_frame.at(scan_index).offset_degree_,
              parameter.min_score_));
        }
      }
    }
  }
  ZGINFO << "Candidate count: " << origin_candidate_count << " filter to "
         << candidates.size();

  return candidates;
}

PointCloudMatcher::NormalSearchCandidates
PointCloudMatcher::GenerateNormalCandidates(
    const SlamValueGridMap2D::SPtr& map,
    const NormalRotateScans& rotated_scans_in_map_frame,
    const NormalSearchParameter& parameter) {
  NormalSearchCandidates candidates;
  uint32_t origin_candidate_count = 0;

  ReadLocker lock(map->GetLock());

  auto generate_candidate = [&](const float& x, const float& y) -> void {
    MapCell cell;
    map->WorldToMap(MapPoint(x, y), cell);
    for (auto scan_index = 0u; scan_index < parameter.num_scans_;
         scan_index++) {
      origin_candidate_count++;
      candidates.emplace_back(NormalSearchCandidate(
          MapPoint(x, y, rotated_scans_in_map_frame.at(scan_index).degree_),
          MapPoint(x - parameter.init_pose_.X(), y - parameter.init_pose_.Y(),
                   rotated_scans_in_map_frame.at(scan_index).offset_degree_),
          scan_index, 0));
    }
  };

  generate_candidate(parameter.init_pose_.X(), parameter.init_pose_.Y());
  {
    auto y = parameter.init_pose_.Y();
    for (auto x = parameter.init_pose_.X() + parameter.linear_search_step_;
         x <= parameter.search_linear_bound_.GetMax().X();
         x += parameter.linear_search_step_) {
      generate_candidate(x, y);
    }
    for (auto x = parameter.init_pose_.X() - parameter.linear_search_step_;
         x >= parameter.search_linear_bound_.GetMin().X();
         x -= parameter.linear_search_step_) {
      generate_candidate(x, y);
    }
  }

  for (auto y = parameter.init_pose_.Y() + parameter.linear_search_step_;
       y <= parameter.search_linear_bound_.GetMax().Y();
       y += parameter.linear_search_step_) {
    for (auto x = parameter.init_pose_.X() + parameter.linear_search_step_;
         x <= parameter.search_linear_bound_.GetMax().X();
         x += parameter.linear_search_step_) {
      generate_candidate(x, y);
    }
    for (auto x = parameter.init_pose_.X() - parameter.linear_search_step_;
         x >= parameter.search_linear_bound_.GetMin().X();
         x -= parameter.linear_search_step_) {
      generate_candidate(x, y);
    }
    generate_candidate(parameter.init_pose_.X(), y);
  }
  for (auto y = parameter.init_pose_.Y() - parameter.linear_search_step_;
       y >= parameter.search_linear_bound_.GetMin().Y();
       y -= parameter.linear_search_step_) {
    for (auto x = parameter.init_pose_.X() + parameter.linear_search_step_;
         x <= parameter.search_linear_bound_.GetMax().X();
         x += parameter.linear_search_step_) {
      generate_candidate(x, y);
    }
    for (auto x = parameter.init_pose_.X() - parameter.linear_search_step_;
         x >= parameter.search_linear_bound_.GetMin().X();
         x -= parameter.linear_search_step_) {
      generate_candidate(x, y);
    }
    generate_candidate(parameter.init_pose_.X(), y);
  }

  ZGINFO << "Candidate count: " << origin_candidate_count << " filter to "
         << candidates.size();

  return candidates;
}

PointCloudMatcher::NormalSearchCandidates
PointCloudMatcher::GenerateNormalCandidates(
    const ProbabilityIndexGridMap2D::SPtr& map,
    const NormalRotateScans& rotated_scans_in_map_frame,
    const NormalSearchParameter& parameter) {
  NormalSearchCandidates candidates;
  uint32_t origin_candidate_count = 0;

  ReadLocker lock(map->GetLock());

  auto generate_candidate = [&](const float& x, const float& y) -> void {
    MapCell cell;
    map->WorldToMap(MapPoint(x, y), cell);
    for (auto scan_index = 0u; scan_index < parameter.num_scans_;
         scan_index++) {
      origin_candidate_count++;
      candidates.emplace_back(NormalSearchCandidate(
          MapPoint(x, y, rotated_scans_in_map_frame.at(scan_index).degree_),
          MapPoint(x - parameter.init_pose_.X(), y - parameter.init_pose_.Y(),
                   rotated_scans_in_map_frame.at(scan_index).offset_degree_),
          scan_index, 0));
    }
  };

  generate_candidate(parameter.init_pose_.X(), parameter.init_pose_.Y());
  {
    auto y = parameter.init_pose_.Y();
    for (auto x = parameter.init_pose_.X() + parameter.linear_search_step_;
         x <= parameter.search_linear_bound_.GetMax().X();
         x += parameter.linear_search_step_) {
      generate_candidate(x, y);
    }
    for (auto x = parameter.init_pose_.X() - parameter.linear_search_step_;
         x >= parameter.search_linear_bound_.GetMin().X();
         x -= parameter.linear_search_step_) {
      generate_candidate(x, y);
    }
  }

  for (auto y = parameter.init_pose_.Y() + parameter.linear_search_step_;
       y <= parameter.search_linear_bound_.GetMax().Y();
       y += parameter.linear_search_step_) {
    for (auto x = parameter.init_pose_.X() + parameter.linear_search_step_;
         x <= parameter.search_linear_bound_.GetMax().X();
         x += parameter.linear_search_step_) {
      generate_candidate(x, y);
    }
    for (auto x = parameter.init_pose_.X() - parameter.linear_search_step_;
         x >= parameter.search_linear_bound_.GetMin().X();
         x -= parameter.linear_search_step_) {
      generate_candidate(x, y);
    }
    generate_candidate(parameter.init_pose_.X(), y);
  }
  for (auto y = parameter.init_pose_.Y() - parameter.linear_search_step_;
       y >= parameter.search_linear_bound_.GetMin().Y();
       y -= parameter.linear_search_step_) {
    for (auto x = parameter.init_pose_.X() + parameter.linear_search_step_;
         x <= parameter.search_linear_bound_.GetMax().X();
         x += parameter.linear_search_step_) {
      generate_candidate(x, y);
    }
    for (auto x = parameter.init_pose_.X() - parameter.linear_search_step_;
         x >= parameter.search_linear_bound_.GetMin().X();
         x -= parameter.linear_search_step_) {
      generate_candidate(x, y);
    }
    generate_candidate(parameter.init_pose_.X(), y);
  }

  // ZINFO << "Candidate count: " << origin_candidate_count << " filter to "
  //       << candidates.size();

  return candidates;
}

void PointCloudMatcher::MatchAndSortCandidates(
    const SlamValueGridMap2D::SPtr& map,
    const PointCloudMatcher::BABRotateScans& rotated_scans_in_map_frame,
    PointCloudMatcher::BABSearchCandidates& candidates) {
  // map->Print(__FILE__, __FUNCTION__, __LINE__);
  for (auto&& candidate : candidates) {
    candidate.score_ = SimpleMatchScore(
        map, candidate.cell_pose_,
        rotated_scans_in_map_frame.at(candidate.scan_index_).scan_);
    // ZINFO << "Candidiate: " << candidate.DebugString();
    // DebugPointCloudInMap(
    //     map, candidate.cell_pose_,
    //     rotated_scans_in_map_frame.at(candidate.scan_index_).scan_);
  }
  std::sort(candidates.begin(), candidates.end(),
            std::greater<BABSearchCandidate>());
}

void PointCloudMatcher::MatchAndSortCandidates(
    const SlamValueGridMap2D::SPtr& map,
    const PointCloudMatcher::NormalRotateScans& rotated_scans_in_map_frame,
    PointCloudMatcher::NormalSearchCandidates& candidates,
    const bool& enable_weight) {
  // map->Print(__FILE__, __FUNCTION__, __LINE__);

  // if (enable_weight) {
  //   ZINFO << "Enable weight.";
  // }
  for (auto&& candidate : candidates) {
    candidate.score_ = SimpleMatchScore(
        map, candidate.pose_,
        rotated_scans_in_map_frame.at(candidate.scan_index_).scan_);
    if (enable_weight) {
      // auto old_score = candidate.score_;
      candidate.score_ =
          WeighingScore(candidate, translation_delta_cost_weight_,
                        rotation_degree_delta_cost_weight_);
      // auto new_score = candidate.score_;
      // ZINFO << "Update " << candidate.pose_.DebugString() << " score from "
      //       << FloatToString(old_score, 7) << " to "
      //       << FloatToString(new_score, 7);
    }
    // ZINFO << "Candidiate: " << candidate.DebugString();
    // DebugPointCloudInMap(
    //     map, candidate.cell_pose_,
    //     rotated_scans_in_map_frame.at(candidate.scan_index_).scan_);
  }
  std::sort(candidates.begin(), candidates.end(),
            std::greater<NormalSearchCandidate>());
}

void PointCloudMatcher::MatchAndSortCandidates(
    const ProbabilityIndexGridMap2D::SPtr& map,
    const PointCloudMatcher::NormalRotateScans& rotated_scans_in_map_frame,
    PointCloudMatcher::NormalSearchCandidates& candidates,
    const bool& enable_weight) {
  // map->Print(__FILE__, __FUNCTION__, __LINE__);

  // if (enable_weight) {
  //   ZINFO << "Enable weight.";
  // }
  for (auto&& candidate : candidates) {
    candidate.score_ = SimpleMatchScore(
        map, candidate.pose_,
        rotated_scans_in_map_frame.at(candidate.scan_index_).scan_);
    if (enable_weight) {
      // auto old_score = candidate.score_;
      candidate.score_ =
          WeighingScore(candidate, translation_delta_cost_weight_,
                        rotation_degree_delta_cost_weight_);
      // auto new_score = candidate.score_;
      // ZINFO << "Update " << candidate.pose_.DebugString() << " score from "
      //       << FloatToString(old_score, 7) << " to "
      //       << FloatToString(new_score, 7);
    }
    // ZINFO << "Candidiate: " << candidate.DebugString();
    // DebugPointCloudInMap(
    //     map, candidate.cell_pose_,
    //     rotated_scans_in_map_frame.at(candidate.scan_index_).scan_);
  }
  std::sort(candidates.begin(), candidates.end(),
            std::greater<NormalSearchCandidate>());
}

MapCell PointCloudMatcher::MatchForGradient(
    const ProbabilityIndexGridMap2D::SPtr& map,
    const FloatValueGridMap2D::SPtr& score_map,
    const NormalRotateScan& rotated_scans_in_map_frame,
    const MapCell& init_score_map_cell,
    const GradientSearchParameter& parameter, const bool& use_8_direction,
    const bool& enable_weight,
    const FloatValueGridStaticMap2D::SPtr cached_probability_map) {
  static const MapCells relative_8_direction_expend_step_cells{
      MapCell(0, 0),  MapCell(1, 0),  MapCell(-1, 0),
      MapCell(0, 1),  MapCell(0, -1), MapCell(1, 1),
      MapCell(-1, 1), MapCell(1, -1), MapCell(-1, -1)};
  static const MapCells relative_4_direction_expend_step_cells{
      MapCell(0, 0), MapCell(1, 0), MapCell(-1, 0), MapCell(0, 1),
      MapCell(0, -1)};
  if (map == nullptr) {
    ZERROR << "Map pointer invalid.";
    return init_score_map_cell;
  }
  if (score_map == nullptr) {
    ZERROR << "Score map pointer invalid.";
    return init_score_map_cell;
  }

  bool use_interpolator = true;
  // ZINFO << "Use interpolator " << use_interpolator;
  // ZINFO << "Scan offset degree: "
  //       << FloatToString(rotated_scans_in_map_frame.offset_degree_, 2);
  MapCell result_cell = init_score_map_cell;
  FloatValueGridMap2D::DataType max_score = -1;
  WriteLocker lock(score_map->GetLock(), false);
  for (auto&& expend_cell :
       (use_8_direction ? relative_8_direction_expend_step_cells
                        : relative_4_direction_expend_step_cells)) {
    FloatValueGridMap2D::DataType score = 0;
    auto curr_cell = init_score_map_cell + expend_cell;
    lock.Lock();
    if (!score_map->GetValue(curr_cell.X(), curr_cell.Y(), score) ||
        score < 0) {
      lock.Unlock();
      // Calculate score.
      // ZINFO << "Calculate score for " << curr_cell.DebugString();
      auto curr_point = parameter.ScoreMapCellConvertToPose(curr_cell);
      curr_point.SetDegree(rotated_scans_in_map_frame.degree_);
      score =
          SimpleMatchScore(map, curr_point, rotated_scans_in_map_frame.scan_,
                           use_interpolator, cached_probability_map);
      if (enable_weight) {
        // auto old_score = score;
        auto offset_length = curr_point.Distance(parameter.init_pose_);
        auto offset_degree = curr_point.AngleDiff(parameter.init_pose_);
        // ZINFO << "Offset degree: " << FloatToString(offset_degree, 2);
        score = WeighingScore(score, offset_length, offset_degree,
                              translation_delta_cost_weight_,
                              rotation_degree_delta_cost_weight_);
        // auto new_score = score;
        // ZINFO << "Update " << curr_point.DebugString() << " score from "
        //       << FloatToString(old_score, 7) << " to "
        //       << FloatToString(new_score, 7);
      }

      lock.Lock();
      score_map->SetValue(curr_cell.X(), curr_cell.Y(), score);
    }
    lock.Unlock();

    if (score > max_score) {
      max_score = score;
      result_cell = curr_cell;
      // ZINFO << "Update cell to " << result_cell.DebugString();
    }
  }
  return result_cell;
}

void PointCloudMatcher::BoostMatchAndSortCandidates(
    const SlamValueGridMap2D::SPtr& map,
    const PointCloudMatcher::BABRotateScans& rotated_scans_in_map_frame,
    PointCloudMatcher::BABSearchCandidates& candidates) {
  // map->Print(__FILE__, __FUNCTION__, __LINE__);

  ReadWriteLock::SPtr candidates_iter_lock(std::make_shared<ReadWriteLock>());
  auto candidates_iter = candidates.begin();
  std::vector<std::thread> thread_pool;

  auto boost_match_func = [&](const uint16_t& index) -> void {
    ZGINFO << "Thread " << index << " start.";
    while (true) {
      auto tmp_candidates_iter = candidates.begin();
      {
        // ZINFO << "Thread " << index << " want to access lock";
        WriteLocker lock(candidates_iter_lock);
        // ZINFO << "Thread " << index << " access lock";
        if (candidates_iter == candidates.end()) {
          break;
        }
        // ZINFO << "Process for "
        //       << lowest_resolution_candidates_iter->DebugString();
        tmp_candidates_iter = candidates_iter;
        candidates_iter++;
      }
      tmp_candidates_iter->score_ = SimpleMatchScore(
          map, tmp_candidates_iter->cell_pose_,
          rotated_scans_in_map_frame.at(tmp_candidates_iter->scan_index_)
              .scan_);
      // ZINFO << "Candidiate: " << candidate.DebugString();
      // DebugPointCloudInMap(
      //     map, candidate.cell_pose_,
      //     rotated_scans_in_map_frame.at(candidate.scan_index_).scan_);
    };
    ZGINFO << "Thread " << index << " exit.";
  };

  for (auto index = 0u; index < boost_thread_count; index++) {
    thread_pool.emplace_back(std::thread(boost_match_func, index));
  }

  for (auto&& t : thread_pool) {
    t.join();
  }

  std::sort(candidates.begin(), candidates.end(),
            std::greater<BABSearchCandidate>());
}

PointCloudMatcher::BABSearchCandidate PointCloudMatcher::BranchAndBound(
    const MultiResolutionSlamValueGridMap2D::SPtr& maps,
    const BABRotateScans rotated_scans_in_map_frame,
    const PointCloudMatcher::BABSearchParameter& parameter,
    const PointCloudMatcher::BABSearchCandidates& candidates,
    const uint8_t& search_level, const float& min_score) {
  if (search_level == 0) {
    return candidates.front();
  }
  // ZINFO << "Search for level " << search_level;

  BABSearchCandidate tmp_candidate(MapCell(0, 0), MapCell(0, 0), 0, 0, 0,
                                   min_score);
  for (auto&& candidate : candidates) {
    if (stop_cmd_.load()) {
      ZGWARN << "Stop for stop cmd.";
      break;
    }
    // ZINFO << "Check for " << candidate.cell_pose_.DebugString()
    //       << " degree offset: " << candidate.degree_
    //       << ", score: " << FloatToString(candidate.score_, 4);
    if (candidate.score_ <= min_score) {
      break;
    }

    // if (search_level == 1) {
    //   ZINFO << "Check for " << candidate.cell_pose_.DebugString()
    //         << " degree offset: " << candidate.degree_
    //         << ", score: " << FloatToString(candidate.score_, 4);
    // }
    BABSearchCandidates higher_resolution_candidates;
    auto linear_step = 1 << (search_level - 1);
    for (int x_offset : {0, linear_step}) {
      if (candidate.cell_pose_.X() - x_offset <=
          parameter.search_linear_cell_bound_.GetMin().X()) {
        break;
      }
      for (int y_offset : {0, linear_step}) {
        if (candidate.cell_pose_.Y() - y_offset <=
            parameter.search_linear_cell_bound_.GetMin().Y()) {
          break;
        }
        higher_resolution_candidates.emplace_back(
            BABSearchCandidate(MapCell(candidate.cell_pose_.X() - x_offset,
                                       candidate.cell_pose_.Y() - y_offset),
                               MapCell(candidate.offset_cell_.X() - x_offset,
                                       candidate.offset_cell_.Y() - y_offset),
                               candidate.scan_index_, candidate.degree_,
                               candidate.offset_degree_, 0));
        // if (search_level == 1) {
        //   ZINFO << "Check for "
        //         <<
        //         higher_resolution_candidates.back().cell_pose_.DebugString();
        // }
      }
    }
    SlamValueGridMap2D::SPtr map;
    if (!maps->GetLayer(search_level - 1, map)) {
      ZERROR << "Insufficient map layer, expecting layer "
             << (search_level - 1);
      break;
    }
    MatchAndSortCandidates(map, rotated_scans_in_map_frame,
                           higher_resolution_candidates);
    auto new_candidate = BranchAndBound(maps, rotated_scans_in_map_frame,
                                        parameter, higher_resolution_candidates,
                                        search_level - 1, tmp_candidate.score_);
    if (tmp_candidate.score_ < new_candidate.score_) {
      tmp_candidate = new_candidate;
      ZGINFO << "Update candidate as " << tmp_candidate.DebugString();
    }
  }

  return tmp_candidate;
}

PointCloudMatcher::SharedScoreBoard::SharedScoreBoard(const float& min_score)
    : access_(std::make_shared<ReadWriteLock>()) {
  WriteLocker lock(access_);
  current_min_score_ = min_score;
}
void PointCloudMatcher::SharedScoreBoard::UpdateScore(
    const float& new_min_score) {
  ReadLocker read_lock(access_);
  if (new_min_score <= current_min_score_) {
    return;
  }
  read_lock.Unlock();

  WriteLocker write_lock(access_);
  if (current_min_score_ < new_min_score) {
    current_min_score_ = new_min_score;
    ZGINFO << "Update score to " << current_min_score_;
  }
}

float PointCloudMatcher::SharedScoreBoard::GetCurrentScore() const {
  ReadLocker lock(access_);
  return current_min_score_;
}

PointCloudMatcher::BABSearchCandidate PointCloudMatcher::BoostBranchAndBound(
    const MultiResolutionSlamValueGridMap2D::SPtr& maps,
    const BABRotateScans& rotated_scans_in_map_frame,
    const PointCloudMatcher::BABSearchParameter& parameter,
    const PointCloudMatcher::BABSearchCandidates& candidates,
    const uint8_t& search_level, const float& min_score,
    SharedScoreBoard& score_board) {
  if (search_level == 0) {
    return candidates.front();
  }
  // ZINFO << "Search for level " << search_level;

  BABSearchCandidate tmp_candidate(MapCell(0, 0), MapCell(0, 0), 0, 0, 0,
                                   min_score);

  // auto current_score = min_score;
  auto current_score = score_board.GetCurrentScore();
  for (auto&& candidate : candidates) {
    if (stop_cmd_.load()) {
      ZWARN << "Stop for stop cmd.";
      break;
    }

    // ZINFO << "Check for " << candidate.cell_pose_.DebugString()
    //       << " degree offset: " << candidate.degree_
    //       << ", score: " << FloatToString(candidate.score_, 4);
    if (candidate.score_ <= current_score) {
      break;
    }

    // if (search_level == 1) {
    //   ZINFO << "Check for " << candidate.cell_pose_.DebugString()
    //         << " degree offset: " << candidate.degree_
    //         << ", score: " << FloatToString(candidate.score_, 4);
    // }
    BABSearchCandidates higher_resolution_candidates;
    auto linear_step = 1 << (search_level - 1);
    for (int x_offset : {0, linear_step}) {
      if (candidate.cell_pose_.X() - x_offset <=
          parameter.search_linear_cell_bound_.GetMin().X()) {
        break;
      }
      for (int y_offset : {0, linear_step}) {
        if (candidate.cell_pose_.Y() - y_offset <=
            parameter.search_linear_cell_bound_.GetMin().Y()) {
          break;
        }
        higher_resolution_candidates.emplace_back(
            BABSearchCandidate(MapCell(candidate.cell_pose_.X() - x_offset,
                                       candidate.cell_pose_.Y() - y_offset),
                               MapCell(candidate.offset_cell_.X() - x_offset,
                                       candidate.offset_cell_.Y() - y_offset),
                               candidate.scan_index_, candidate.degree_,
                               candidate.offset_degree_, 0));
        // if (search_level == 1) {
        //   ZINFO << "Check for "
        //         <<
        //         higher_resolution_candidates.back().cell_pose_.DebugString();
        // }
      }
    }
    SlamValueGridMap2D::SPtr map;
    if (!maps->GetLayer(search_level - 1, map)) {
      ZERROR << "Insufficient map layer, expecting layer "
             << (search_level - 1);
      break;
    }
    MatchAndSortCandidates(map, rotated_scans_in_map_frame,
                           higher_resolution_candidates);
    auto new_candidate =
        BoostBranchAndBound(maps, rotated_scans_in_map_frame, parameter,
                            higher_resolution_candidates, search_level - 1,
                            tmp_candidate.score_, score_board);
    if (tmp_candidate.score_ < new_candidate.score_) {
      tmp_candidate = new_candidate;
      // ZINFO << "Update candidate as " << tmp_candidate.DebugString();
      score_board.UpdateScore(tmp_candidate.score_);
      current_score = tmp_candidate.score_;
    }
  }

  return tmp_candidate;
}

float PointCloudMatcher::WeighingScore(
    const float& score, const float& offset_length, const float& offset_degree,
    const float& translation_delta_cost_weight,
    const float& rotation_degree_delta_cost_weight) const {
  auto factor =
      std::exp(-Square(fabs(offset_length) * translation_delta_cost_weight +
                       fabs(offset_degree) * rotation_degree_delta_cost_weight));
  // ZINFO << "Factor: " << FloatToString(factor, 5);

  return score * factor;
}

float PointCloudMatcher::WeighingScore(
    const NormalSearchCandidate& candidate,
    const float& translation_delta_cost_weight,
    const float& rotation_degree_delta_cost_weight) const {
  return WeighingScore(candidate.score_, candidate.offset_point_.Length(),
                       candidate.offset_point_.Degree(),
                       translation_delta_cost_weight,
                       rotation_degree_delta_cost_weight);
}

void PointCloudMatcher::GenerateScoreGraph(
    const NormalSearchCandidates& result_candidates,
    const NormalSearchParameter& parameter) const {
  auto best_candidate = result_candidates.front();
  ZGINFO << "Total index: " << std::to_string(parameter.num_scans_)
         << ", best candidate scan index: "
         << std::to_string(best_candidate.scan_index_);
  for (auto index = 0; index < parameter.num_scans_; index++) {
    if (index != best_candidate.scan_index_) {
      continue;
    }
    auto score_map = parameter.GenerateNewScoreMap();
    WriteLocker lock(score_map->GetLock());
    MapCell best_pose_cell;
    for (auto&& candidate : result_candidates) {
      if (candidate.scan_index_ != index) {
        continue;
      }
      auto score_map_cell = parameter.ConvertToScoreMapCell(
          candidate.pose_.X(), candidate.pose_.Y());
      if (candidate.pose_ == best_candidate.pose_) {
        best_pose_cell = score_map_cell;
        auto x = candidate.pose_.X() - parameter.init_pose_.X();
        auto y = candidate.pose_.Y() - parameter.init_pose_.Y();
        ZGINFO << "x: " << FloatToString(x, 4) << ", y: " << FloatToString(y, 4)
               << ", best cell: " << best_pose_cell.DebugString();
      }

      score_map->SetValue(score_map_cell.X(), score_map_cell.Y(),
                          candidate.score_ * 1000);
    }
    score_map->PrintWithPrecision(__FILE__, __FUNCTION__, __LINE__, 2);
    score_map->Print(__FILE__, __FUNCTION__, __LINE__);

    // Try trace for gradient peak value.
    auto peak_cell = score_map->GradientMoveFor8Direction(
        0, 0,
        Maximum((score_map->GetRangeX() - 1) / 2,
                (score_map->GetRangeY() - 1) / 2),
        true);
    if (peak_cell != best_pose_cell) {
      ZWARN << "Peak " << peak_cell.DebugString()
            << ", but best cell: " << best_pose_cell.DebugString();
    }
  }
}

AsyncPointCloudMatcher::AsyncPointCloudMatcher(
    const std::string thread_name, const SlamValueGridMap2D::SPtr& map,
    const PointCloud::SPtr& point_cloud_in_chassis_frame,
    const PointCloudMatcher::BABSearchParameter& parameter)
    : access_(std::make_shared<ReadWriteLock>()),
      matcher_(
          std::make_shared<PointCloudMatcher>(PointCloudMatcher::Config())),
      parameter_(parameter) {
  auto thread_manager = ZimaThreadManager::Instance();
  thread_param_ = ZimaThreadWrapper::ThreadParam(
      thread_name, ZimaThreadManager::kSlamThreadIndex_, 100, 2, 5);
  thread_manager->RegisterThread(
      std::thread(&AsyncPointCloudMatcher::AsyncMatch, this, thread_param_, map,
                  point_cloud_in_chassis_frame, parameter),
      thread_param_);
}

AsyncPointCloudMatcher::~AsyncPointCloudMatcher() {
  auto thread_manager = ZimaThreadManager::Instance();
  if (thread_manager->IsThreadRunning(thread_param_.thread_name_)) {
    matcher_->StopSearch();
    Time::SleepMSec(5);
  }
}

bool AsyncPointCloudMatcher::IsMatching() {
  ReadLocker lock(access_);
  if (match_result_.IsValid()) {
    return false;
  }
  return true;
}

bool AsyncPointCloudMatcher::GetMatchResult(
    PointCloudMatcher::MatchResult& match_result) {
  ReadLocker lock(access_);
  if (!match_result_.IsValid()) {
    return false;
  }
  match_result.SetMatchResult(match_result_.pose_, match_result_.score_,
                              match_result_.match_time_);
  return true;
}

void AsyncPointCloudMatcher::AsyncMatch(
    const ZimaThreadWrapper::ThreadParam& param,
    const SlamValueGridMap2D::SPtr& map,
    const PointCloud::SPtr& point_cloud_in_chassis_frame,
    const PointCloudMatcher::BABSearchParameter& parameter) {
  ZGINFO << "Thread \"" << param.thread_name_ << "\" start.";
  auto thread_manager = ZimaThreadManager::Instance();
  if (param.bind_cpu_id_ >= 0) {
    thread_manager->BindCPUCore(param.thread_name_, param.bind_cpu_id_);
  }

  thread_manager->UpdateThreadCycle(param.thread_name_);
  PointCloudMatcher::MatchResult match_result;
  matcher_->BABSearch(map, point_cloud_in_chassis_frame, parameter,
                      match_result);

  {
    WriteLocker lock(access_);
    match_result_.SetMatchResult(match_result.pose_, match_result.score_,
                                 match_result.match_time_);
  }
  // ZINFO << parameter.init_pose_.DebugString();

  ZGINFO << "Thread \"" << param.thread_name_ << "\" exiting.";
  thread_manager->MarkThreadExited(param.thread_name_, __FILE__, __FUNCTION__,
                                   __LINE__);
  ZGINFO << "Thread \"" << param.thread_name_ << "\" exit.";
}

}  // namespace zima
