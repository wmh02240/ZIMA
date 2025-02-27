/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_PROBABILITY_MAP_H
#define ZIMA_PROBABILITY_MAP_H

#include "zima/common/debug.h"
#include "zima/grid_map/map_2d.h"
#include "zima/proto/probability_map.pb.h"

namespace zima {

using ProbabilityIndex = int16_t;
// Learn from cartographer.
// ProbabilityIndex value range:
//     0 ~ (probability count - 1) : Index for probability elements container.
//     (probability count) ~ +inf : Consider as unknown, probability is 0.5.
// If index is below 0, means it is updated, and use abs(index) to fit its
// meaning as above.
class ProbabilityIndexGridMap2D : public DynamicMap2D<ProbabilityIndex> {
 public:
  class Config {
   public:
    Config();
    explicit Config(const JsonSPtr& json);
    ~Config() = default;

    bool ParseFromJson(const JsonSPtr& json);

    bool config_valid_;

    static const std::string kConfigKey_;

    static const std::string kHitProbabilityOnObstacleKey_;
    float hit_probability_on_obstacle_;
    float hit_probability_on_space_;
    static const std::string kMissProbabilityOnObstacleKey_;
    float miss_probability_on_obstacle_;
    float miss_probability_on_space_;

    static const std::string kMaxProbabilityForObstacleInCellKey_;
    float max_probability_for_obstacle_in_cell_;
    static const std::string kMinProbabilityForObstacleInCellKey_;
    float min_probability_for_obstacle_in_cell_;
    static const std::string kProbabilityCountKey_;
    uint probability_count_;
  };

  class ProbabilityElement {
   public:
    ProbabilityElement() = delete;
    ProbabilityElement(const float& obstacle_probability,
                       const ProbabilityIndex& next_hit_index,
                       const ProbabilityIndex& next_miss_index)
        : obstacle_probability_(obstacle_probability),
          next_hit_index_(next_hit_index),
          next_miss_index_(next_miss_index) {}

    std::string DebugString() const {
      return "Probability: " + FloatToString(obstacle_probability_, 3) +
             ", next hit index: " + std::to_string(next_hit_index_) +
             ", next miss index: " + std::to_string(next_miss_index_);
    }

    ~ProbabilityElement() = default;

    float GetProbability() const { return obstacle_probability_; }
    ProbabilityIndex GetNextHitIndex() const { return next_hit_index_; }
    ProbabilityIndex GetNextMissIndex() const { return next_miss_index_; }

   private:
    float obstacle_probability_;
    ProbabilityIndex next_hit_index_;
    ProbabilityIndex next_miss_index_;
  };

  using ProbabilityElements = std::vector<ProbabilityElement>;

  ProbabilityIndexGridMap2D(const std::string& name,
                            const uint16_t& cells_range_x,
                            const uint16_t& cells_range_y,
                            const float& resolution, const Config& config);

  ProbabilityIndexGridMap2D(const ProbabilityIndexGridMap2D& map);

  ProbabilityIndexGridMap2D(const ProbabilityIndexGridMap2D& map,
                            const std::string& name,
                            const bool& reserve = false);

  ProbabilityIndexGridMap2D& operator=(const ProbabilityIndexGridMap2D& map) = delete;

  using SPtr = std::shared_ptr<ProbabilityIndexGridMap2D>;

  bool IsValid() const {
    ReadLocker lock(access_);
    return map_valid_;
  }

  ProbabilityElements GetProbabilityElements() const {
    ReadLocker lock(access_);
    return probability_elements_;
  }

  bool GetProbability(const int& x, const int& y, float& probability) const;
  bool IsUnknown(const int& x, const int& y) const;

  bool PrepareForUpdate();
  bool UpdateForHit(const int& x, const int& y);
  bool UpdateForMiss(const int& x, const int& y);

  float GetHitProbabilityOnObstacle() const;
  float GetMissProbabilityOnObstacle() const;
  float GetMinProbabilityForObstacleInCell() const;
  float GetMaxProbabilityForObstacleInCell() const;
  float GetMediumProbabilityForObstacleInCell() const;
  uint GetProbabilityCount() const;

  bool ResetProbabilityElementVector(
      const float& hit_probability_on_obstacle,
      const float& miss_probability_on_obstacle,
      const float& max_probability_for_obstacle_in_cell,
      const float& min_probability_for_obstacle_in_cell,
      const uint& probability_count);

  std::vector<std::string> DebugString(
      const DynamicMapCellBound& bound,
      OverridePrintCellFunc func = nullptr) const override;
  std::vector<std::string> DebugString(
      OverridePrintCellFunc func = nullptr) const override;

 private:
  uint32_t ProbabilityIndexToProbabilityElementsIndex(
      const ProbabilityIndex& index) const;
  float ProbabilityIndexToProbability(const ProbabilityIndex& index) const;
  ProbabilityIndex ProbabilityToProbabilityIndex(
      const float& probability) const;
  float OddsToProbability(const float& odds) const;
  float ProbabilityToOdds(const float& probability) const;

  void InitializeProbabilityElementVector();

  bool map_valid_;

  float hit_probability_on_obstacle_;
  float hit_probability_on_space_;
  float miss_probability_on_obstacle_;
  float miss_probability_on_space_;

  float max_probability_for_obstacle_in_cell_;
  float min_probability_for_obstacle_in_cell_;
  uint probability_count_;

  ProbabilityElements probability_elements_;
};

class ProbabilityIndexGridMap2DSerializer {
 public:
  ProbabilityIndexGridMap2DSerializer() = delete;

  static ZimaProto::ProbabilityMap::PProbabilityIndexGridMap2D ToProto(
      const ProbabilityIndexGridMap2D::SPtr& map);
  static bool FromProto(
      ProbabilityIndexGridMap2D::SPtr& map,
      const ZimaProto::ProbabilityMap::PProbabilityIndexGridMap2D& proto);
};

}  // namespace zima

#endif  // ZIMA_PROBABILITY_MAP_H
