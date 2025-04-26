/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_MOVE_FORWARD_MOTION_H
#define ZIMA_MOVE_FORWARD_MOTION_H

#include "zima/motion/motion.h"

namespace zima {

class MoveForwardMotion : public MotionBase {
 public:

  class Config : public MotionConfigBase {
   public:
    Config() = delete;
    Config(const float& move_distance, const float& move_time,
           const float& wheel_max_speed, const float& robot_radius,
           const JsonSPtr& json = nullptr);
    ~Config() = default;

    using SPtr = std::shared_ptr<Config>;

    bool ParseFromJson(const float& move_distance, const float& move_time,
                       const float& wheel_max_speed, const float& robot_radius,
                       const JsonSPtr& json);

    bool config_valid_;

    static const std::string kConfigKey_;

    static const std::string kMinTurnCircleRadiusKey_;
    float min_turn_circle_radius_;
  };

  MoveForwardMotion() = delete;
  MoveForwardMotion(const float& half_track_length,
                    const float& max_wheel_speed, const float& min_wheel_speed,
                    const float& move_distance, const float& move_time,
                    const MapPoint& init_odom_pose,
                    const Config& config,
                    const bool& stop_when_reach = true);
  ~MoveForwardMotion();

  using SPtr = std::shared_ptr<MoveForwardMotion>;

  void CheckState(const MapPoint& world_pose, const MapPoint& odom_pose,
                  const float& current_left_wheel_speed,
                  const float& current_right_wheel_speed) override;
  void SpeedControl(const MapPoint& world_pose, const MapPoint& odom_pose,
                    const float& current_left_wheel_speed,
                    const float& current_right_wheel_speed,
                    float& left_wheel_speed, float& right_wheel_speed) override;

 private:
  bool IsDistanceReach(const MapPoint& odom_pose);
  bool IsTimeReach();
  float move_distance_;
  float move_time_;
  float min_turn_circle_radius_;
  MapPoint init_odom_pose_;
  bool stop_when_reach_;
};

}  // namespace zima
#endif  // ZIMA_MOVE_FORWARD_MOTION_H
