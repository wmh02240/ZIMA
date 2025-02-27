/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_POINT_CLOUD_MATCHER_H
#define ZIMA_POINT_CLOUD_MATCHER_H

#include "zima/algorithm/slam/probability_map.h"
#include "zima/common/debug.h"
#include "zima/common/point_cloud.h"
#include "zima/common/thread.h"
#include "zima/common/util.h"
#include "zima/grid_map/map_2d.h"
#include "zima/grid_map/multi_layers_map_2d.h"

namespace zima {

/*
 * These search methods are inspired by Cartographer project.
 */
class PointCloudMatcher : public DebugBase {
 public:
  class Config {
   public:
    Config();
    Config(const JsonSPtr& json);
    ~Config() = default;

    using SPtr = std::shared_ptr<Config>;

    bool ParseFromJson(const JsonSPtr& json);

    bool config_valid_;

    static const std::string kConfigKey_;
    static const std::string kTranslationDeltaCostWeightKey_;
    float translation_delta_cost_weight_;
    static const std::string kRotationDegreeDeltaCostWeightKey_;
    float rotation_degree_delta_cost_weight_;
  };

  PointCloudMatcher() = delete;
  PointCloudMatcher(const Config& config);
  ~PointCloudMatcher();

  using SPtr = std::shared_ptr<PointCloudMatcher>;

  VMapCells ToMapCells(const PointCloud::SPtr& point_cloud,
                       const SlamValueGridMap2D::SPtr& map);

  VMapPoints ToMapPoints(const PointCloud::SPtr& point_cloud);

  /*
   * @brief  This function returns a matching score from 0 to 1.
   */
  float SimpleMatchScore(const SlamValueGridMap2D::SPtr& map,
                         const MapCell& pose,
                         const VMapCells& point_cloud_cells_in_map_frame);

  /*
   * @brief  This function returns a matching score from 0 to 1.
   */
  float SimpleMatchScore(const SlamValueGridMap2D::SPtr& map,
                         const MapPoint& pose,
                         const VMapPoints& point_cloud_points_in_map_frame);

  /*
   * @brief  This function returns a matching score from 0 to 1.
   */
  float SimpleMatchScore(
      const ProbabilityIndexGridMap2D::SPtr& map, const MapPoint& pose,
      const VMapPoints& point_cloud_points_in_map_frame,
      const bool& use_interpolator = false,
      const FloatValueGridStaticMap2D::SPtr cached_probability_map = nullptr);

  class BABSearchParameter {
   public:
    BABSearchParameter() = delete;
    BABSearchParameter(
        const SlamValueGridMap2D::SPtr& map, const MapPoint& init_pose,
        const DynamicMapPointBound& search_linear_bound_in_map_frame,
        const float& degree_range, const float& degree_step,
        const uint8_t& search_depth, const float& min_score,
        const bool& optimized_for_known_wall_degree = false);

    MapPoint init_pose_;
    // This bound is in map frame.
    DynamicMapCellBound search_linear_cell_bound_;
    // Search for init pose degree ± degree_range_.
    float degree_range_;
    float degree_step_;
    uint8_t search_depth_;
    float min_score_;
    bool optimized_for_known_wall_degree_;
    uint16_t num_scans_;
  };

  class BABSearchConfig {
   public:
    BABSearchConfig();
    BABSearchConfig(const JsonSPtr& json);
    ~BABSearchConfig() = default;

    using SPtr = std::shared_ptr<BABSearchConfig>;

    bool ParseFromJson(const JsonSPtr& json);

    bool config_valid_;

    static const std::string kDegreeRangeKey_;
    float degree_range_;
    static const std::string kDegreeStepKey_;
    float degree_step_;
    static const std::string kSearchDepthKey_;
    uint search_depth_;
    static const std::string kMinScoreKey_;
    float min_score_;
  };

  class NormalSearchParameter {
   public:
    NormalSearchParameter() = delete;
    NormalSearchParameter(
        const MapPoint& init_pose,
        const DynamicMapPointBound& search_linear_bound_in_map_frame,
        const float& linear_search_step, const float& degree_range,
        const float& degree_step, const float& min_score);

    MapCell ConvertToScoreMapCell(const float& x, const float& y) const;
    MapPoint ScoreMapCellConvertToPose(const MapCell& score_map_cell) const;
    FloatValueGridMap2D::SPtr GenerateNewScoreMap() const;

    MapPoint init_pose_;
    // This bound is in map frame.
    DynamicMapPointBound search_linear_bound_;
    float linear_search_step_;
    // Search for init pose degree ± degree_range_.
    float degree_range_;
    float degree_step_;
    float min_score_;
    uint16_t num_scans_;
  };

  class GradientSearchParameter : public NormalSearchParameter {
   public:
    GradientSearchParameter() = delete;
    GradientSearchParameter(
        const MapPoint& init_pose,
        const DynamicMapPointBound& search_linear_bound_in_map_frame,
        const float& linear_search_step, const float& degree_range,
        const float& degree_step, const uint8_t& max_iteration_num,
        const float& min_score);

    uint8_t max_iteration_num_;
  };

  template <typename T>
  class RotateScan {
   public:
    RotateScan() = delete;
    RotateScan(const T& scan, const float& degree, const float& offset_degree)
        : scan_(scan), degree_(degree), offset_degree_(offset_degree){};

    // These scan data are in chassis frame.
    T scan_;
    float degree_;
    float offset_degree_;
  };

  using BABRotateScan = RotateScan<VMapCells>;
  using BABRotateScans = std::vector<BABRotateScan>;
  using NormalRotateScan = RotateScan<VMapPoints>;
  using NormalRotateScans = std::vector<NormalRotateScan>;
  using GradientRotateScan = RotateScan<VMapPoints>;
  using GradientRotateScans = std::vector<GradientRotateScan>;

  class BABSearchCandidate {
   public:
    BABSearchCandidate() = delete;
    BABSearchCandidate(const MapCell& cell_pose, const MapCell& offset_cell,
                       const uint16_t& scan_index, const float& degree,
                       const float& offset_degree, const float& score);

    bool operator<(const BABSearchCandidate& other) const {
      return score_ < other.score_;
    }
    bool operator>(const BABSearchCandidate& other) const {
      return score_ > other.score_;
    }

    std::string DebugString() const {
      return "[" + cell_pose_.DebugString() +
             " offset cell: " + offset_cell_.DebugString() +
             " degree: " + FloatToString(degree_, 1) +
             " offset degree: " + FloatToString(offset_degree_, 1) +
             ", score: " + FloatToString(score_, 4) + "]";
    };

    MapCell cell_pose_;
    MapCell offset_cell_;
    uint16_t scan_index_;
    float degree_;
    float offset_degree_;
    float score_;
  };
  using BABSearchCandidates = std::vector<BABSearchCandidate>;

  class NormalSearchCandidate {
   public:
    NormalSearchCandidate() = delete;
    NormalSearchCandidate(const MapPoint& pose, const MapPoint& offset_point,
                          const uint16_t& scan_index, const float& score);

    bool operator<(const NormalSearchCandidate& other) const {
      return score_ < other.score_;
    }
    bool operator>(const NormalSearchCandidate& other) const {
      return score_ > other.score_;
    }

    std::string DebugString() const {
      return "[" + pose_.DebugString() +
             " offset point: " + offset_point_.DebugString() +
             ", score: " + FloatToString(score_, 4) + "]";
    };

    MapPoint pose_;
    MapPoint offset_point_;
    uint16_t scan_index_;
    float score_;
  };
  using NormalSearchCandidates = std::vector<NormalSearchCandidate>;

  class MatchResult {
   public:
    MatchResult() : valid_(false){};
    ~MatchResult() = default;

    void SetMatchResult(const MapPoint& pose, const float& score,
                        const float& match_time);
    bool IsValid() const { return valid_.load(); }

    std::string DebugString() const {
      return "[" + pose_.DebugString() +
             ", score: " + FloatToString(score_, 8) + ", match_time " +
             FloatToString(match_time_, 4) + "s]";
    };

    MapPoint pose_;
    float score_;
    float match_time_;

   private:
    atomic_bool valid_;
  };

  /*
   * @brief  This function is for matching with parameter.
   * @param  map Highest resolution map.
   * @param  point_cloud Pointcloud in chassis frame.
   * @param  parameter Match parameter.
   * @param  match_result Result container.
   * @param  enable_weight True for enable weighting score.
   * @return True for match pose is found.
   */
  bool NormalSearch(const SlamValueGridMap2D::SPtr& map,
                    const PointCloud::SPtr& point_cloud_in_chassis_frame,
                    const NormalSearchParameter& parameter,
                    MatchResult& match_result,
                    const bool& enable_weight = true);

  /*
   * @brief  This function is for matching with parameter.
   * @param  map Highest resolution map.
   * @param  point_cloud Pointcloud in chassis frame.
   * @param  parameter Match parameter.
   * @param  match_result Result container.
   * @param  enable_weight True for enable weighting score.
   * @return True for match pose is found.
   */
  bool NormalSearch(const ProbabilityIndexGridMap2D::SPtr& map,
                    const PointCloud::SPtr& point_cloud_in_chassis_frame,
                    const NormalSearchParameter& parameter,
                    MatchResult& match_result,
                    const bool& enable_weight = true);

  /*
   * @brief  This function is for matching with parameter.
   * @param  map Highest resolution map.
   * @param  point_cloud Pointcloud in chassis frame.
   * @param  parameter Match parameter.
   * @param  match_result Result container.
   * @param  enable_weight True for enable weighting score.
   * @return True for match pose is found.
   */
  bool GradientSearch(
      const ProbabilityIndexGridMap2D::SPtr& map,
      const PointCloud::SPtr& point_cloud_in_chassis_frame,
      const GradientSearchParameter& parameter, MatchResult& match_result,
      const bool& enable_weight = true,
      const FloatValueGridStaticMap2D::SPtr cached_probability_map = nullptr);

  /*
   * @brief  This function is for matching with branch and bound method.
   * @param  map Highest resolution map.
   * @param  point_cloud Pointcloud in chassis frame.
   * @param  parameter Match parameter.
   * @param  match_result Result container.
   * @return True for match pose is found.
   */
  bool BABSearch(const SlamValueGridMap2D::SPtr& map,
                 const PointCloud::SPtr& point_cloud_in_chassis_frame,
                 const BABSearchParameter& parameter,
                 MatchResult& match_result);

  /*
   * @brief  This function is for matching with branch and bound method with
   * boost algorithm.
   * @param  map Highest resolution map.
   * @param  point_cloud Pointcloud in chassis frame.
   * @param  parameter Match parameter.
   * @param  match_result Result container.
   * @return True for match pose is found.
   */
  bool BoostBABSearch(const SlamValueGridMap2D::SPtr& map,
                      const PointCloud::SPtr& point_cloud_in_chassis_frame,
                      const BABSearchParameter& parameter,
                      MatchResult& match_result);

  void StopSearch();

  void DebugPointCloudInMap(const SlamValueGridMap2D::SPtr& map,
                            const MapCell& cell_pose,
                            const BABRotateScan& scan);
  void DebugPointCloudInMap(const SlamValueGridMap2D::SPtr& map,
                            const MapPoint& pose, const NormalRotateScan& scan);
  void DebugPointCloudInMap(const ProbabilityIndexGridMap2D::SPtr& map,
                            const MapPoint& pose, const NormalRotateScan& scan);

 protected:
  BABRotateScans GenerateRotatedPointCloudCellsInMapFrame(
      const PointCloud::SPtr& point_cloud_in_chassis_frame,
      const SlamValueGridMap2D::SPtr& map, const BABSearchParameter& parameter);

  NormalRotateScans GenerateRotatedPointCloudPointsInMapFrame(
      const PointCloud::SPtr& point_cloud_in_chassis_frame,
      const NormalSearchParameter& parameter);

  BABSearchCandidates GenerateBABCandidates(
      const SlamValueGridMap2D::SPtr& map,
      const BABRotateScans& rotated_scans_in_map_frame,
      const BABSearchParameter& parameter);

  NormalSearchCandidates GenerateNormalCandidates(
      const SlamValueGridMap2D::SPtr& map,
      const NormalRotateScans& rotated_scans_in_map_frame,
      const NormalSearchParameter& parameter);

  NormalSearchCandidates GenerateNormalCandidates(
      const ProbabilityIndexGridMap2D::SPtr& map,
      const NormalRotateScans& rotated_scans_in_map_frame,
      const NormalSearchParameter& parameter);

  void MatchAndSortCandidates(const SlamValueGridMap2D::SPtr& map,
                              const BABRotateScans& rotated_scans_in_map_frame,
                              BABSearchCandidates& candidates);

  void MatchAndSortCandidates(
      const SlamValueGridMap2D::SPtr& map,
      const NormalRotateScans& rotated_scans_in_map_frame,
      NormalSearchCandidates& candidates, const bool& enable_weight);

  void MatchAndSortCandidates(
      const ProbabilityIndexGridMap2D::SPtr& map,
      const NormalRotateScans& rotated_scans_in_map_frame,
      NormalSearchCandidates& candidates, const bool& enable_weight);

  MapCell MatchForGradient(
      const ProbabilityIndexGridMap2D::SPtr& map,
      const FloatValueGridMap2D::SPtr& score_map,
      const NormalRotateScan& rotated_scans_in_map_frame,
      const MapCell& init_score_map_cell,
      const GradientSearchParameter& parameter, const bool& use_8_direction,
      const bool& enable_weight,
      const FloatValueGridStaticMap2D::SPtr cached_probability_map = nullptr);

  BABSearchCandidate BranchAndBound(
      const MultiResolutionSlamValueGridMap2D::SPtr& maps,
      const BABRotateScans rotated_scans_in_map_frame,
      const BABSearchParameter& parameter,
      const BABSearchCandidates& candidates, const uint8_t& search_level,
      const float& min_score);

  // ==========  Try to boost with multi-thread ============
  void BoostMatchAndSortCandidates(
      const SlamValueGridMap2D::SPtr& map,
      const BABRotateScans& rotated_scans_in_map_frame,
      BABSearchCandidates& candidates);

  class SharedScoreBoard {
   public:
    SharedScoreBoard() = delete;
    SharedScoreBoard(const float& min_score);

    void UpdateScore(const float& new_min_score);
    float GetCurrentScore() const;

   private:
    ReadWriteLock::SPtr access_;
    float current_min_score_;
  };

  BABSearchCandidate BoostBranchAndBound(
      const MultiResolutionSlamValueGridMap2D::SPtr& maps,
      const BABRotateScans& rotated_scans_in_map_frame,
      const BABSearchParameter& parameter,
      const BABSearchCandidates& candidates, const uint8_t& search_level,
      const float& min_score, SharedScoreBoard& score_board);

  // ==========  Try to boost with multi-thread end ============

  float WeighingScore(const float& score, const float& offset_length,
                      const float& offset_degree,
                      const float& translation_delta_cost_weight,
                      const float& rotation_degree_delta_cost_weight) const;
  float WeighingScore(const NormalSearchCandidate& candidate,
                      const float& translation_delta_cost_weight,
                      const float& rotation_degree_delta_cost_weight) const;

  void GenerateScoreGraph(const NormalSearchCandidates& result_candidates,
                          const NormalSearchParameter& parameter) const;

  float translation_delta_cost_weight_;
  float rotation_degree_delta_cost_weight_;

  FloatValueGridMap2D::SPtr score_map_;

  atomic_bool searching_;
  atomic_bool stop_cmd_;
};

class AsyncPointCloudMatcher {
 public:
  AsyncPointCloudMatcher() = delete;
  AsyncPointCloudMatcher(
      const std::string thread_name, const SlamValueGridMap2D::SPtr& map,
      const PointCloud::SPtr& point_cloud_in_chassis_frame,
      const PointCloudMatcher::BABSearchParameter& parameter);
  ~AsyncPointCloudMatcher();

  using SPtr = std::shared_ptr<AsyncPointCloudMatcher>;

  bool IsMatching();
  bool GetMatchResult(PointCloudMatcher::MatchResult& match_result);
  PointCloudMatcher::BABSearchParameter GetParameter() const {
    return parameter_;
  }

 private:
  void AsyncMatch(const ZimaThreadWrapper::ThreadParam& param,
                  const SlamValueGridMap2D::SPtr& map,
                  const PointCloud::SPtr& point_cloud_in_chassis_frame,
                  const PointCloudMatcher::BABSearchParameter& parameter);

  ReadWriteLock::SPtr access_;
  PointCloudMatcher::SPtr matcher_;
  PointCloudMatcher::MatchResult match_result_;
  ZimaThreadWrapper::ThreadParam thread_param_;
  PointCloudMatcher::BABSearchParameter parameter_;
};

}  // namespace zima

#endif  // ZIMA_POINT_CLOUD_MATCHER_H
