/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_MOTION_H
#define ZIMA_MOTION_H

#include "zima/common/config.h"
#include "zima/common/debug.h"
#include "zima/common/lock.h"
#include "zima/common/point_cell.h"

namespace zima {

class MotionBase : public DebugBase {
 public:
  MotionBase() = delete;
  MotionBase(const float& half_track_length, const float& max_wheel_speed,
             const float& min_wheel_speed, const float& target_speed,
             const float& speed_up_step, const float& timeout_sec = 0);
  ~MotionBase() = default;

  class MotionConfigBase {
   public:
    MotionConfigBase() = delete;
    MotionConfigBase(const JsonSPtr& json = nullptr);
    ~MotionConfigBase() = default;

    bool ParseFromJson(const JsonSPtr& json);

    static const std::string kCycleRateKey_;
    float cycle_rate_;
    static const std::string kTargetSpeedKey_;
    float target_speed_;
    static const std::string kSpeedUpStepKey_;
    float speed_up_step_;
    static const std::string kTimeoutKey_;
    float timeout_;
  };

  enum State {
    kReady,
    kRunning,
    kNearTarget,
    kReachTarget,
    kFinish,
    kTimeout,
    kStop,
    kException,
    kError,
  };

  State GetState() const;

  virtual void SlowdownTo(const float& slow_down_speed,
                          const float& slow_down_step);

 protected:
  virtual void CheckState(const MapPoint& world_pose, const MapPoint& odom_pose,
                          const float& current_left_wheel_speed,
                          const float& current_right_wheel_speed) = 0;
  virtual void SpeedControl(const MapPoint& world_pose,
                            const MapPoint& odom_pose,
                            const float& current_left_wheel_speed,
                            const float& current_right_wheel_speed,
                            float& left_wheel_speed,
                            float& right_wheel_speed) = 0;

  bool Runable();
  void LimitSpeed(float& left_wheel_speed, float& right_wheel_speed);
  void LimitSpeed(float& left_wheel_speed, float& right_wheel_speed,
                  const float& limit_speed_min, const float& limit_speed_max);
  void Stop();

  std::string name_;
  State state_;
  ReadWriteLock::SPtr access_;

  float half_track_length_;
  float max_wheel_speed_;
  float min_wheel_speed_;
  float timeout_sec_;
  double start_time_;

  float target_speed_;
  float speed_up_step_;
  std::shared_ptr<float> last_speed_;
  bool handle_slow_down_;
  float slow_down_speed_;
  float slow_down_step_;

  bool reached_target_log_displayed_;
};

}  // namespace zima

#endif  // ZIMA_MOTION_H
