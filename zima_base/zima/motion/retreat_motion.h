/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_RETREAT_MOTION_H
#define ZIMA_RETREAT_MOTION_H

#include "zima/motion/motion.h"

namespace zima {

class RetreatMotion : public MotionBase {
 public:
  class Config : public MotionConfigBase {
   public:
    Config() = delete;
    Config(const float& wheel_max_speed, const float& retreat_distance,
           const JsonSPtr& json = nullptr);
    ~Config() = default;

    using SPtr = std::shared_ptr<Config>;

    bool ParseFromJson(const float& wheel_max_speed,
                       const float& retreat_distance, const JsonSPtr& json);

    bool config_valid_;

    static const std::string kConfigKey_;

    // static const std::string kRetreatDistanceKey_;
    // float retreat_distance_;
    static const std::string kMinSpeedKey_;
    float min_speed_;
  };

  RetreatMotion() = delete;
  RetreatMotion(const float& half_track_length, const float& max_wheel_speed,
                const float& min_wheel_speed, const MapPoint& start_odom_pose,
                const float& retreat_distance, const Config& config,
                const bool& require_preciseness = true,
                const bool& stop_when_reach = true);
  ~RetreatMotion();

  using SPtr = std::shared_ptr<RetreatMotion>;

  void CheckState(const MapPoint& world_pose, const MapPoint& odom_pose,
                  const float& current_left_wheel_speed,
                  const float& current_right_wheel_speed) override;
  void SpeedControl(const MapPoint& world_pose, const MapPoint& odom_pose,
                    const float& current_left_wheel_speed,
                    const float& current_right_wheel_speed,
                    float& left_wheel_speed, float& right_wheel_speed) override;

 protected:
  float RemainDistance(const MapPoint& odom_pose);
  bool IsTargetReached(const MapPoint& odom_pose);

  MapPoint start_odom_pose_;
  float retreat_distance_;
  float min_speed_;

  bool require_preciseness_;
  bool stop_when_reach_;
};

}  // namespace zima
#endif  // ZIMA_RETREAT_MOTION_H
