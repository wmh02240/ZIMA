/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/motion/move_forward_motion.h"

#include "zima/common/time.h"
#include "zima/logger/logger.h"

namespace zima {

MoveForwardMotion::Config::Config(const float& move_distance,
                                  const float& move_time,
                                  const float& wheel_max_speed,
                                  const float& chassis_radius,
                                  const JsonSPtr& json)
    : MotionConfigBase(json) {
  // Override if json config is provided.
  if (json != nullptr) {
    // ZINFO << "Load config from json.";
    config_valid_ = ParseFromJson(move_distance, move_time, wheel_max_speed,
                                  chassis_radius, json);
  } else {
    JsonSPtr config;
    if (GlobalJsonConfig::Instance()->GetGlobalConfigObject(kConfigKey_,
                                                            config)) {
      // ZINFO << "Load config from global json config.";
      config_valid_ = ParseFromJson(move_distance, move_time, wheel_max_speed,
                                    chassis_radius, config);
    } else {
      config_valid_ = false;
    }
  }
}

bool MoveForwardMotion::Config::ParseFromJson(const float& move_distance,
                                              const float& move_time,
                                              const float& wheel_max_speed,
                                              const float& chassis_radius,
                                              const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }

  if (!MotionConfigBase::ParseFromJson(json)) {
    return false;
  }

  if (FloatEqual(target_speed_, 0)) {
    target_speed_ = wheel_max_speed * 1 / 2;
  }
  if (FloatEqual(speed_up_step_, 0)) {
    speed_up_step_ = 2 * target_speed_ / cycle_rate_;
  }

  if (timeout_ < 0.01) {
    timeout_ = Maximum(move_time * 1.5, 10.0 * move_distance / target_speed_);
  }

  if (!JsonHelper::GetFloat(*json, kMinTurnCircleRadiusKey_,
                            min_turn_circle_radius_)) {
    min_turn_circle_radius_ = chassis_radius / 10;
  }

  return true;
}

MoveForwardMotion::MoveForwardMotion(
    const float& half_track_length, const float& max_wheel_speed,
    const float& min_wheel_speed, const float& move_distance,
    const float& move_time, const MapPoint& init_world_pose,
    const Config& config, const bool& stop_when_reach)
    : MotionBase(half_track_length, max_wheel_speed, min_wheel_speed,
                 config.target_speed_, config.speed_up_step_, config.timeout_),
      move_distance_(move_distance),
      move_time_(move_time),
      min_turn_circle_radius_(config.min_turn_circle_radius_),
      init_odom_pose_(init_world_pose),
      stop_when_reach_(stop_when_reach) {
  name_ = typeid(MoveForwardMotion).name();
  if (state_ == State::kError) {
    ZWARN << "State error.";
    return;
  }
  if (move_time_ <= 0) {
    ZWARN << "Move time: " << move_time_ << " invalid.";
    state_ = State::kError;
    return;
  }
  if (move_time_ > timeout_sec_) {
    ZWARN << "Move time: " << move_time_
          << " is more than timeout sec: " << timeout_sec_;
    state_ = State::kError;
    return;
  }
  if (move_distance_ <= 0) {
    ZWARN << "Move distance: " << move_distance_ << " invalid.";
    state_ = State::kError;
    return;
  }
  if (min_turn_circle_radius_ < 0) {
    ZWARN << "min_turn_circle_radius_ " << min_turn_circle_radius_;
    state_ = State::kError;
    return;
  }
  // ZINFO << "For " << move_time_ << "s, " << move_distance_ << "m.";
}

MoveForwardMotion::~MoveForwardMotion() {
  // ZINFO;
}

void MoveForwardMotion::CheckState(const MapPoint& world_pose,
                                   const MapPoint& odom_pose,
                                   const float& current_left_wheel_speed,
                                   const float& current_right_wheel_speed) {
  WriteLocker lock(access_);
  if (IsDistanceReach(odom_pose)) {
    if (!reached_target_log_displayed_) {
      // ZINFO << "Reach target distance: " << move_distance_;
      reached_target_log_displayed_ = true;
    }
    if (stop_when_reach_ && (fabs(current_left_wheel_speed) > 0.01 ||
                             fabs(current_right_wheel_speed) > 0.01)) {
      return;
    }

    state_ = kReachTarget;
    return;
  }

  if (IsTimeReach()) {
    if (!reached_target_log_displayed_) {
      // ZINFO << "Run for target time: " << move_time_;
      reached_target_log_displayed_ = true;
    }
    if (stop_when_reach_ && (fabs(current_left_wheel_speed) > 0.01 ||
                             fabs(current_right_wheel_speed) > 0.01)) {
      return;
    }
    state_ = kReachTarget;
    return;
  }
}

void MoveForwardMotion::SpeedControl(const MapPoint& world_pose,
                                     const MapPoint& odom_pose,
                                     const float& current_left_wheel_speed,
                                     const float& current_right_wheel_speed,
                                     float& left_wheel_speed,
                                     float& right_wheel_speed) {
  WriteLocker lock(access_);
  if (!Runable()) {
    left_wheel_speed = 0;
    right_wheel_speed = 0;
    return;
  }

  // TODO(Austin): Here we can use PID in the future.
  // float current_speed =
  //     (current_left_wheel_speed + current_right_wheel_speed) / 2;
  if (last_speed_ == nullptr) {
    last_speed_ = std::make_shared<float>(
        (current_left_wheel_speed + current_right_wheel_speed) / 2);
  }
  auto current_speed = *last_speed_;

  if (stop_when_reach_ && (IsDistanceReach(odom_pose) || IsTimeReach())) {
    current_speed = 0;
  } else if (handle_slow_down_ && current_speed > slow_down_speed_) {
    current_speed -= slow_down_step_;
  } else {
    if (current_speed < target_speed_) {
      current_speed += speed_up_step_;
    } else if (current_speed > target_speed_ + speed_up_step_) {
      ZGWARN << "Init speed too high: " << current_speed;
      current_speed -= speed_up_step_;
    }
  }
  handle_slow_down_ = false;

  // Go straight by angle for a distance.
  auto degree = odom_pose.AngleDiff(init_odom_pose_);
  float turn_circle_radius = min_turn_circle_radius_;
  if (InRange(static_cast<double>(degree), -45.0, 45.0)) {
    // When degree is 90, radius should reach the min.
    // Smaller the degree, larger the radius.
    turn_circle_radius =
        min_turn_circle_radius_ / fabs(sin(DegreesToRadians(degree * 2)));
    // ZINFO << "Current radius:" << turn_circle_radius;
  }
  if (degree > 0) {
    // Handle for left.
    left_wheel_speed = current_speed - (current_speed * half_track_length_) /
                                           (2 * turn_circle_radius);
    right_wheel_speed = current_speed + (current_speed * half_track_length_) /
                                            (2 * turn_circle_radius);
  } else {
    // Handle for right.
    left_wheel_speed = current_speed + (current_speed * half_track_length_) /
                                           (2 * turn_circle_radius);
    right_wheel_speed = current_speed - (current_speed * half_track_length_) /
                                            (2 * turn_circle_radius);
  }

  *last_speed_ = (left_wheel_speed + right_wheel_speed) / 2;

  LimitSpeed(left_wheel_speed, right_wheel_speed);

  // ZINFO << name_ << "(" << left_wheel_speed << ", " << right_wheel_speed <<
  // ")";
  return;
}

bool MoveForwardMotion::IsDistanceReach(const MapPoint& odom_pose) {
  return odom_pose.Distance(init_odom_pose_) > move_distance_;
}

bool MoveForwardMotion::IsTimeReach() {
  return Time::Now() - start_time_ > move_time_;
}

}  // namespace zima
