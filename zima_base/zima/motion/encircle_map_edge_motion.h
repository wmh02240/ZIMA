/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_ENCIRCLE_MAP_EDGE_MOTION_H
#define ZIMA_ENCIRCLE_MAP_EDGE_MOTION_H

#include "zima/common/lock.h"
#include "zima/motion/motion.h"

namespace zima {

class EncircleMapEdgeMotion : public MotionBase {
 public:
  class Config : public MotionConfigBase {
   public:
    Config() = delete;
    Config(const float& wheel_max_speed, const float& map_resolution,
           const JsonSPtr& json = nullptr);
    ~Config() = default;

    using SPtr = std::shared_ptr<Config>;

    bool ParseFromJson(const float& wheel_max_speed,
                       const float& map_resolution, const JsonSPtr& json);

    bool config_valid_;

    static const std::string kConfigKey_;

    static const std::string kTargetMapEdgeDistanceKey_;
    float target_map_edge_distance_;
    static const std::string kMaxValidMapEdgeDistanceKey_;
    float max_valid_map_edge_distance_;
    static const std::string kTurnCircleRadiusKey_;
    float turn_circle_radius_;
    static const std::string kDelayDistanceKey_;
    float delay_distance_;
  };

  EncircleMapEdgeMotion() = delete;
  EncircleMapEdgeMotion(const float& half_track_length,
                        const float& max_wheel_speed,
                        const float& min_wheel_speed,
                        const bool& map_edge_on_left, const Config& config);
  ~EncircleMapEdgeMotion();

  using SPtr = std::shared_ptr<EncircleMapEdgeMotion>;

  void SetCurrentMapEdgeDistance(const MapPoint& pose,
                                 const float& current_map_edge_distance);

  void CheckState(const MapPoint& world_pose, const MapPoint& odom_pose,
                  const float& current_left_wheel_speed,
                  const float& current_right_wheel_speed) override;
  void SpeedControl(const MapPoint& world_pose, const MapPoint& odom_pose,
                    const float& current_left_wheel_speed,
                    const float& current_right_wheel_speed,
                    float& left_wheel_speed, float& right_wheel_speed) override;

 protected:
  void DistanceBaseSpeedControl(const MapPoint& world_pose,
                                const MapPoint& odom_pose,
                                const float& current_speed,
                                float& left_wheel_speed,
                                float& right_wheel_speed);

  float current_map_edge_distance_;
  float last_map_edge_distance_;
  bool map_edge_on_left_;
  float target_map_edge_distance_;
  float max_valid_map_edge_distance_;
  float turn_circle_radius_;
  MapPoint lost_map_edge_odom_pose_;
  float delay_distance_;
};

}  // namespace zima
#endif  // ZIMA_ENCIRCLE_MAP_EDGE_MOTION_H
