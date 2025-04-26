/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/motion/retreat_motion.h"

#include "zima/logger/logger.h"

namespace zima {

RetreatMotion::Config::Config(const float& wheel_max_speed,
                              const float& retreat_distance,
                              const JsonSPtr& json)
    : MotionConfigBase(json) {
  // Override if json config is provided.
  if (json != nullptr) {
    // ZINFO << "Load config from json.";
    config_valid_ = ParseFromJson(wheel_max_speed, retreat_distance, json);
  } else {
    JsonSPtr config;
    if (GlobalJsonConfig::Instance()->GetGlobalConfigObject(kConfigKey_,
                                                            config)) {
      // ZINFO << "Load config from global json config.";
      config_valid_ = ParseFromJson(wheel_max_speed, retreat_distance, config);
    } else {
      config_valid_ = false;
    }
  }
}

bool RetreatMotion::Config::ParseFromJson(const float& wheel_max_speed,
                                          const float& retreat_distance,
                                          const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }

  if (!MotionConfigBase::ParseFromJson(json)) {
    return false;
  }

  if (FloatEqual(target_speed_, 0)) {
    target_speed_ = -wheel_max_speed * 2 / 3;
  }
  if (FloatEqual(speed_up_step_, 0)) {
    speed_up_step_ = target_speed_ / 5;
  }

  if (timeout_ < 0.01) {
    timeout_ = Maximum(20 * fabs(retreat_distance / target_speed_), 2.0f);
  }

  if (!JsonHelper::GetFloat(*json, kMinSpeedKey_, min_speed_)) {
    min_speed_ = -0.02;
  }

  return true;
}

RetreatMotion::RetreatMotion(
    const float& half_track_length, const float& max_wheel_speed,
    const float& min_wheel_speed, const MapPoint& start_odom_pose,
    const float& retreat_distance, const Config& config,
    const bool& require_preciseness, const bool& stop_when_reach)
    : MotionBase(half_track_length, max_wheel_speed, min_wheel_speed,
                 config.target_speed_, config.speed_up_step_, config.timeout_),
      start_odom_pose_(start_odom_pose),
      retreat_distance_(retreat_distance),
      min_speed_(config.min_speed_),
      require_preciseness_(require_preciseness),
      stop_when_reach_(stop_when_reach) {
  name_ = typeid(RetreatMotion).name();
  if (state_ == State::kError) {
    ZWARN << "State error.";
    return;
  }
  if (target_speed_ > 0) {
    ZWARN << "Target speed " << FloatToString(target_speed_, 2);
    state_ = State::kError;
    return;
  }
  if (retreat_distance_ < 0) {
    ZWARN << "Retreat distance " << FloatToString(retreat_distance_, 3);
    state_ = State::kError;
    return;
  }
  // ZINFO << "Start pose " << start_odom_pose_
  //       << ", distance: " << FloatToString(retreat_distance_, 4)
  //       << ", speed: " << FloatToString(target_speed_, 2)
  //       << ", speed step: " << FloatToString(speed_up_step_, 2);
}

RetreatMotion::~RetreatMotion() {
  // ZINFO;
}

void RetreatMotion::CheckState(const MapPoint& world_pose,
                               const MapPoint& odom_pose,
                               const float& current_left_wheel_speed,
                               const float& current_right_wheel_speed) {
  WriteLocker lock(access_);
  if (IsTargetReached(odom_pose)) {
    if (!reached_target_log_displayed_) {
      // ZINFO << "Reach distance : " << FloatToString(retreat_distance_, 4);
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

void RetreatMotion::SpeedControl(const MapPoint& world_pose,
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

  // float current_speed =
  //     (current_left_wheel_speed + current_right_wheel_speed) / 2;
  if (last_speed_ == nullptr) {
    last_speed_ = std::make_shared<float>(
        (current_left_wheel_speed + current_right_wheel_speed) / 2);
  }
  auto current_speed = *last_speed_;

  if (current_speed > 0) {
    current_speed = 0;
  }
  if (stop_when_reach_ && IsTargetReached(odom_pose)) {
    current_speed = 0;
  } else {
    if (!require_preciseness_) {
      // Method 1
      if (fabs(current_speed) < fabs(target_speed_)) {
        // current_speed += speed_up_step_;
        current_speed = target_speed_;
      } else if (fabs(current_speed) >
                 fabs(target_speed_) + fabs(speed_up_step_)) {
        ZGWARN << "Init speed too high: " << FloatToString(current_speed, 2);
        current_speed -= speed_up_step_;
      }
    } else {
      // Method 2
      const float kAdjustSpeedDistanceMin = 0.01;
      const float kAdjustSpeedDistanceMax = 0.05;
      const float kAdjustSpeedDistanceRange =
          kAdjustSpeedDistanceMax - kAdjustSpeedDistanceMin;
      float distance_left = RemainDistance(odom_pose);

      float speed_limit;
      if (distance_left > kAdjustSpeedDistanceMax) {
        speed_limit = target_speed_;
      } else if (distance_left < kAdjustSpeedDistanceMin) {
        speed_limit = min_speed_;
      } else {
        speed_limit =
            min_speed_ + (target_speed_ - min_speed_) *
                             ((distance_left - kAdjustSpeedDistanceMin) /
                              kAdjustSpeedDistanceRange);
      }
      if (fabs(current_speed) > fabs(speed_limit) + fabs(speed_up_step_)) {
        current_speed -= speed_up_step_;
        if (current_speed >= 0) {
          current_speed = speed_limit;
        }
      } else if (fabs(current_speed) < fabs(speed_limit)) {
        // current_speed += speed_up_step_;
        current_speed = speed_limit;
      }

      // if (distance_left < kAdjustSpeedDistanceMin) {
      //   ZINFO << "Remain distance " << FloatToString(distance_left, 3)
      //         << ", near reach speed: " << FloatToString(current_speed, 3);
      // }
    }
  }

  left_wheel_speed = current_speed;
  right_wheel_speed = current_speed;

  *last_speed_ = (left_wheel_speed + right_wheel_speed) / 2;

  LimitSpeed(left_wheel_speed, right_wheel_speed);

  // ZINFO << name_ << "(" << left_wheel_speed << ", " << right_wheel_speed <<
  // ")";
  return;
}

float RetreatMotion::RemainDistance(const MapPoint& odom_pose) {
  return retreat_distance_ - odom_pose.Distance(start_odom_pose_);
}

bool RetreatMotion::IsTargetReached(const MapPoint& odom_pose) {
  return RemainDistance(odom_pose) < 0;
}

}  // namespace zima
