/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_ENCIRCLE_OBSTACLE_MOTION_H
#define ZIMA_ENCIRCLE_OBSTACLE_MOTION_H

#include "zima/common/lock.h"
#include "zima/motion/motion.h"

namespace zima {

class EncircleObstacleMotion : public MotionBase {
 public:

  class Config : public MotionConfigBase {
   public:
    Config() = delete;
    Config(const float& wheel_max_speed, const float& chassis_radius,
           const MapPoint& wall_sensor_tf, const JsonSPtr& json = nullptr);
    ~Config() = default;

    using SPtr = std::shared_ptr<Config>;

    bool ParseFromJson(const float& wheel_max_speed,
                       const float& chassis_radius,
                       const MapPoint& wall_sensor_tf, const JsonSPtr& json);

    bool config_valid_;

    static const std::string kConfigKey_;

    static const std::string kTargetObstacleDistanceKey_;
    float target_obstacle_distance_;
    static const std::string kMaxValidObstacleDistanceKey_;
    float max_valid_obstacle_distance_;
    static const std::string kTurnCircleRadiusKey_;
    float turn_circle_radius_;
    static const std::string kDelayDistanceKey_;
    float delay_distance_;
    static const std::string kReachTempTargetDistanceKey_;
    float reach_temp_target_distance_;
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

  EncircleObstacleMotion() = delete;
  EncircleObstacleMotion(const float& half_track_length,
                         const float& max_wheel_speed,
                         const float& min_wheel_speed,
                         const bool& obstacle_on_left, const Config& config);
  ~EncircleObstacleMotion();

  using SPtr = std::shared_ptr<EncircleObstacleMotion>;

  void SetCurrentObsDistance(const MapPoint& pose,
                             const float& current_obstacle_distance);

  void CheckState(const MapPoint& world_pose, const MapPoint& odom_pose,
                  const float& current_left_wheel_speed,
                  const float& current_right_wheel_speed) override;
  void SpeedControl(const MapPoint& world_pose, const MapPoint& odom_pose,
                    const float& current_left_wheel_speed,
                    const float& current_right_wheel_speed,
                    float& left_wheel_speed, float& right_wheel_speed) override;

  void UpdateDynamicPath(const MapPointPath& path_points_in_odom);

 protected:
  void DistanceBaseSpeedControl(const MapPoint& world_pose,
                                const MapPoint& odom_pose,
                                const float& current_speed,
                                float& left_wheel_speed,
                                float& right_wheel_speed);
  void PathBaseSpeedControl(const MapPoint& world_pose,
                            const MapPoint& odom_pose,
                            const float& current_speed, float& left_wheel_speed,
                            float& right_wheel_speed);

  float current_obstacle_distance_;
  float last_obstacle_distance_;
  bool obstacle_on_left_;
  float target_obstacle_distance_;
  float max_valid_obstacle_distance_;
  float turn_circle_radius_;
  MapPoint lost_obstacle_odom_pose_;
  float delay_distance_;

  ReadWriteLock::SPtr dynamic_path_lock_;
  MapPointPath dynamic_path_in_odom_;
  float reach_temp_target_distance_;
};

}  // namespace zima
#endif  // ZIMA_ENCIRCLE_OBSTACLE_MOTION_H
