/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/motion/rotate_motion.h"

#include "zima/logger/logger.h"

namespace zima {

RotateMotion::Config::Config(const float& track_length,
                             const float& wheel_max_speed, const JsonSPtr& json)
    : MotionConfigBase(json) {
  // Override if json config is provided.
  if (json != nullptr) {
    // ZINFO << "Load config from json.";
    config_valid_ = ParseFromJson(track_length, wheel_max_speed, json);
  } else {
    JsonSPtr config;
    if (GlobalJsonConfig::Instance()->GetGlobalConfigObject(kConfigKey_,
                                                            config)) {
      // ZINFO << "Load config from global json config.";
      config_valid_ = ParseFromJson(track_length, wheel_max_speed, config);
    } else {
      config_valid_ = false;
    }
  }
}

bool RotateMotion::Config::ParseFromJson(const float& track_length,
                                         const float& wheel_max_speed,
                                         const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }

  if (!MotionConfigBase::ParseFromJson(json)) {
    return false;
  }

  if (FloatEqual(target_speed_, 0)) {
    target_speed_ = wheel_max_speed * 2 / 3;
  }

  if (FloatEqual(speed_up_step_, 0)) {
    speed_up_step_ = target_speed_ / 10;
  }

  if (timeout_ < 0.01) {
    timeout_ = Maximum(10 * track_length * M_PI / target_speed_, 2.0);
  }

  if (!JsonHelper::GetFloat(*json, kAccuracyAngleKey_, accuracy_angle_)) {
    accuracy_angle_ = 1;
  }
  if (!JsonHelper::GetFloat(*json, kMinSpeedKey_, min_speed_)) {
    min_speed_ = 0.02;
  }

  return true;
}

RotateMotion::RotateMotion(const float& half_track_length,
                           const float& max_wheel_speed,
                           const float& min_wheel_speed,
                           const MapPoint& start_odom_pose,
                           const float& rotate_angle, const Config& config,
                           const bool& require_preciseness,
                           const bool& stop_when_reach)
    : MotionBase(half_track_length, max_wheel_speed, min_wheel_speed,
                 config.target_speed_, config.speed_up_step_, config.timeout_),
      rotate_angle_(rotate_angle),
      accuracy_angle_(config.accuracy_angle_),
      min_speed_(config.min_speed_),
      last_odom_pose_(start_odom_pose),
      sum_rotate_angle_(0),
      require_preciseness_(require_preciseness),
      stop_when_reach_(stop_when_reach) {
  name_ = typeid(RotateMotion).name();
  if (state_ == State::kError) {
    ZWARN << "State error.";
    return;
  }
  if (accuracy_angle_ > fabs(rotate_angle_)) {
    ZGWARN << "Reach target, no need to rotate, accuracy:(" << accuracy_angle_
          << "), rotate angle: (" << rotate_angle_ << ")";
    state_ = State::kReachTarget;
    return;
  }
  target_odom_pose_ = start_odom_pose;
  target_odom_pose_.SetDegree(
      NormalizeDegree(target_odom_pose_.Degree() + rotate_angle_));
  ZGINFO << "Start pose " << start_odom_pose
         << " target pose: " << target_odom_pose_
         << " rotate angle: " << rotate_angle_ << ", speed: " << target_speed_
         << ", speed step: " << FloatToString(speed_up_step_, 4)
         << (require_preciseness_ ? ", require preciseness." : ".");
}

RotateMotion::~RotateMotion() {
  // ZINFO;
}

void RotateMotion::CheckState(const MapPoint& world_pose,
                              const MapPoint& odom_pose,
                              const float& current_left_wheel_speed,
                              const float& current_right_wheel_speed) {
  WriteLocker lock(access_);
  auto angle_diff = odom_pose.AngleDiff(target_odom_pose_);
  sum_rotate_angle_ += last_odom_pose_.AngleDiff(odom_pose);
  last_odom_pose_ = odom_pose;
  // ZINFO << "AngleDiff:" << angle_diff << ", sum: " << sum_rotate_angle_;
  // ZINFO << "speed left: " << current_left_wheel_speed
  //       << " right:" << current_right_wheel_speed;
  if (IsTargetReached(angle_diff)) {
    if (!reached_target_log_displayed_) {
      ZGINFO << "Finish rotate for: " << rotate_angle_
             << " degree, current: " << FloatToString(odom_pose.Degree(), 1)
             << ", sum degree: " << FloatToString(sum_rotate_angle_, 1);
      reached_target_log_displayed_ = true;
    }
    if (stop_when_reach_ && (fabs(current_left_wheel_speed) > 0.01 ||
                             fabs(current_right_wheel_speed) > 0.01)) {
      // ZINFO << "Current speed left("
      //       << FloatToString(current_left_wheel_speed, 5) << ") right("
      //       << FloatToString(current_right_wheel_speed, 5) << ")";
      return;
    }
    state_ = kReachTarget;
    return;
  }
}

void RotateMotion::SpeedControl(const MapPoint& world_pose,
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

  auto angle_diff = odom_pose.AngleDiff(target_odom_pose_);
  // float current_speed =
  //     (fabs(current_left_wheel_speed) + fabs(current_right_wheel_speed)) / 2;
  if (last_speed_ == nullptr) {
    last_speed_ = std::make_shared<float>(
        (fabs(current_left_wheel_speed) + fabs(current_right_wheel_speed)) / 2);
  }
  auto current_speed = *last_speed_;

  // ZINFO << "speed: " << current_speed << " target:" << target_speed_;
  // ZINFO << "speed left: " << current_left_wheel_speed
  //       << " right:" << current_right_wheel_speed;
  if (stop_when_reach_ && IsTargetReached(angle_diff)) {
    current_speed = 0;
  } else {
    if (!require_preciseness_) {
      // Method 1
      if (current_speed < target_speed_) {
        current_speed += speed_up_step_;
      } else if (current_speed > target_speed_ + speed_up_step_) {
        ZGWARN << "Init speed too high: " << FloatToString(current_speed, 2);
        current_speed -= speed_up_step_;
      }
    } else {
      // Method 2
      const float kAdjustSpeedDegreeMin = 5;
      const float kAdjustSpeedDegreeMax = 30;
      const float kAdjustSpeedDegreeRange =
          kAdjustSpeedDegreeMax - kAdjustSpeedDegreeMin;
      float degree_left = fabs(rotate_angle_) - fabs(sum_rotate_angle_);
      // ZINFO << "Degree left " << degree_left;

      float speed_limit;
      if (degree_left > kAdjustSpeedDegreeMax) {
        speed_limit = target_speed_;
      } else if (degree_left < kAdjustSpeedDegreeMin) {
        speed_limit = min_speed_;
      } else {
        speed_limit = min_speed_ + (target_speed_ - min_speed_) *
                                       ((degree_left - kAdjustSpeedDegreeMin) /
                                        kAdjustSpeedDegreeRange);
      }
      // ZINFO << "Speed limit " << speed_limit;
      if (current_speed > speed_limit + speed_up_step_) {
        current_speed -= speed_up_step_;
        if (current_speed <= 0) {
          current_speed = speed_limit;
        }
      } else if (current_speed < speed_limit) {
        // current_speed += speed_up_step_;
        current_speed = speed_limit;
      }

      // if (InRange(angle_diff, -kAdjustSpeedDegreeMin * accuracy_angle_,
      //             kAdjustSpeedDegreeMin * accuracy_angle_)) {
      //   ZINFO << "Near reach speed: " << FloatToString(current_speed, 3);
      // }
    }
  }
  // ZINFO << "speed: " << current_speed;

  if (angle_diff > 0) {
    left_wheel_speed = -current_speed;
    right_wheel_speed = current_speed;
  } else {
    left_wheel_speed = current_speed;
    right_wheel_speed = -current_speed;
  }

  *last_speed_ = (fabs(left_wheel_speed) + fabs(right_wheel_speed)) / 2;

  LimitSpeed(left_wheel_speed, right_wheel_speed);

  // ZINFO << name_ << "(" << left_wheel_speed << ", " << right_wheel_speed <<
  // ")";
  return;
}

bool RotateMotion::IsTargetReached(const float& angle_diff) {
  if (InRange(angle_diff, -accuracy_angle_, accuracy_angle_) ||
      fabs(sum_rotate_angle_) > fabs(rotate_angle_)) {
    return true;
  }
  return false;
}

}  // namespace zima
