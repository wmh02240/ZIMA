/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_ROTATE_MOTION_H
#define ZIMA_ROTATE_MOTION_H

#include "zima/motion/motion.h"

namespace zima {

class RotateMotion : public MotionBase {
 public:
  class Config : public MotionConfigBase {
   public:
    Config() = delete;
    Config(const float& track_length, const float& wheel_max_speed,
           const JsonSPtr& json = nullptr);
    ~Config() = default;

    using SPtr = std::shared_ptr<Config>;

    bool ParseFromJson(const float& track_length, const float& wheel_max_speed,
                       const JsonSPtr& json);

    bool config_valid_;

    static const std::string kConfigKey_;

    static const std::string kAccuracyAngleKey_;
    float accuracy_angle_;
    static const std::string kMinSpeedKey_;
    float min_speed_;
  };

  RotateMotion() = delete;
  RotateMotion(const float& half_track_length, const float& max_wheel_speed,
               const float& min_wheel_speed, const MapPoint& start_odom_pose,
               const float& rotate_angle, const Config& config,
               const bool& require_preciseness = true,
               const bool& stop_when_reach = true);
  ~RotateMotion();

  using SPtr = std::shared_ptr<RotateMotion>;

  void CheckState(const MapPoint& world_pose, const MapPoint& odom_pose,
                  const float& current_left_wheel_speed,
                  const float& current_right_wheel_speed) override;
  void SpeedControl(const MapPoint& world_pose, const MapPoint& odom_pose,
                    const float& current_left_wheel_speed,
                    const float& current_right_wheel_speed,
                    float& left_wheel_speed, float& right_wheel_speed) override;

 protected:
  bool IsTargetReached(const float& angle_diff);

  MapPoint target_odom_pose_;
  float rotate_angle_;
  float accuracy_angle_;
  float min_speed_;

  MapPoint last_odom_pose_;
  float sum_rotate_angle_;

  bool require_preciseness_;
  bool stop_when_reach_;
};

}  // namespace zima
#endif  // ZIMA_ROTATE_MOTION_H
