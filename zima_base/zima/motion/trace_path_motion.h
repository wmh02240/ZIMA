/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_TRACE_PATH_MOTION_H
#define ZIMA_TRACE_PATH_MOTION_H

#include "zima/common/config.h"
#include "zima/motion/motion.h"

namespace zima {

class TracePathMotion : public MotionBase {
 public:
  class Config : public MotionConfigBase {
   public:
    Config() = delete;
    Config(const float& wheel_max_speed, const float& chassis_radius,
           const JsonSPtr& json = nullptr);
    ~Config() = default;

    using SPtr = std::shared_ptr<Config>;

    bool ParseFromJson(const float& wheel_max_speed,
                       const float& chassis_radius, const JsonSPtr& json);

    bool config_valid_;

    static const std::string kConfigKey_;

    static const std::string kMinTurnCircleRadiusKey_;
    float min_turn_circle_radius_;
    static const std::string kInterpolationDistanceKey_;
    float interpolation_distance_;
    static const std::string kMaxNearDistanceKey_;
    float max_near_distance_;
    static const std::string kMaxReachDistanceKey_;
    float max_reach_distance_;
    static const std::string kStopForRoomValueDistanceKey_;
    float stop_for_room_value_distance_;
    static const std::string kStopForUserBlockDistanceKey_;
    float stop_for_user_block_distance_;
    static const std::string kStopForCleaningAreaEdgeDistanceKey_;
    float stop_for_cleaning_area_edge_distance_;
    static const std::string kStopForLidarObsDistanceKey_;
    float stop_for_lidar_obs_distance_;
    static const std::string kStopForLidarCompensateKey_;
    float stop_for_lidar_compensate_;
  };

  TracePathMotion() = delete;
  TracePathMotion(const float& half_track_length, const float& max_wheel_speed,
                  const float& min_wheel_speed, const MapPointPath& path,
                  const Config& config);
  // TODO(Austin): What if max/min wheel speed conflict with
  // min_turn_circle_radius...
  ~TracePathMotion();

  using SPtr = std::shared_ptr<TracePathMotion>;

  void CheckState(const MapPoint& world_pose, const MapPoint& odom_pose,
                  const float& current_left_wheel_speed,
                  const float& current_right_wheel_speed) override;
  void SpeedControl(const MapPoint& world_pose, const MapPoint& odom_pose,
                    const float& current_left_wheel_speed,
                    const float& current_right_wheel_speed,
                    float& left_wheel_speed, float& right_wheel_speed) override;

  uint32_t GetRestPathPointCount() const;
  MapPointPath GetRestPath() const;

  bool ExtendPath(const MapPointPath& path);

 protected:

  using CheckReachFunction = std::function<bool(const MapPoint& world_pose)>;

  bool tracing_last_point_;
  MapPointPath path_;
  MapPointPathIter target_;
  float interpolation_distance_;
  // MapPointPath interpolation_path_;
  // MapPointPathIter interpolation_target_;
  float min_turn_circle_radius_;
  float max_near_distance_;
  float max_reach_distance_;

  CheckReachFunction check_reach_function_;
  MapPoint::SPtr check_reach_target_coordinate_;
};

}  // namespace zima

#endif  // ZIMA_TRACE_PATH_MOTION_H
