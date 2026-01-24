/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/motion/motion.h"

#include "zima/common/time.h"
#include "zima/logger/logger.h"

namespace zima {

MotionBase::MotionConfigBase::MotionConfigBase(const JsonSPtr& json) {
  cycle_rate_ = 50;
  target_speed_ = 0;
  speed_up_step_ = 0;
  timeout_ = 0.0001;

  if (json != nullptr) {
    ParseFromJson(json);
  }
};

bool MotionBase::MotionConfigBase::ParseFromJson(const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }

  // Override.
  JsonHelper::GetFloat(*json, kCycleRateKey_, cycle_rate_);
  JsonHelper::GetFloat(*json, kTargetSpeedKey_, target_speed_);
  JsonHelper::GetFloat(*json, kSpeedUpStepKey_, speed_up_step_);
  JsonHelper::GetFloat(*json, kTimeoutKey_, timeout_);

  return true;
}

MotionBase::MotionBase(const float& half_track_length,
                       const float& max_wheel_speed,
                       const float& min_wheel_speed, const float& target_speed,
                       const float& speed_up_step, const float& timeout_sec)
    : access_(std::make_shared<ReadWriteLock>()),
      half_track_length_(half_track_length),
      max_wheel_speed_(max_wheel_speed),
      min_wheel_speed_(min_wheel_speed),
      timeout_sec_(timeout_sec),
      target_speed_(target_speed),
      speed_up_step_(speed_up_step),
      last_speed_(nullptr),
      handle_slow_down_(false),
      reached_target_log_displayed_(false) {
  name_ = "";
  start_time_ = Time::Now();
  // ZINFO << "Start time at: " << std::to_string(start_time_);
  if (half_track_length_ < 0) {
    ZWARN << "Invalid track length: " << FloatToString(half_track_length, 4);
    state_ = State::kError;
    return;
  } else if (max_wheel_speed_ < min_wheel_speed_) {
    ZWARN << "Invalid speed setting: max(" << FloatToString(max_wheel_speed_, 4)
          << "), min(" << FloatToString(min_wheel_speed_, 4) << ")";
    state_ = State::kError;
    return;
  } else if (timeout_sec_ < 0) {
    ZWARN << "Invalid timeout setting: " << FloatToString(timeout_sec_, 4);
    state_ = State::kError;
    return;
  } else if (FloatEqual(speed_up_step_, 0)) {
    ZWARN << "Invalid speed up step: " << FloatToString(speed_up_step_, 6);
    state_ = State::kError;
    return;
  }

  state_ = State::kReady;
}

MotionBase::State MotionBase::GetState() const {
  ReadLocker lock(access_);
  return state_;
};

void MotionBase::SlowdownTo(const float& slow_down_speed,
                            const float& slow_down_step) {
  WriteLocker lock(access_);
  if (slow_down_speed < 0) {
    ZWARN << "Slow down speed " << FloatToString(slow_down_speed, 2) << " < 0";
    return;
  } else if (slow_down_speed > target_speed_) {
    ZWARN << "Slow down speed " << FloatToString(slow_down_speed, 2)
          << " > target speed " << target_speed_;
    return;
  }

  if (slow_down_step < 0) {
    ZWARN << "Slow down step " << FloatToString(slow_down_step, 2) << " < 0";
    return;
  }

  // Only valid for once cycle.
  handle_slow_down_ = true;
  slow_down_speed_ = slow_down_speed;
  slow_down_step_ = slow_down_step;
  // ZINFO << "Slow down to " << FloatToString(slow_down_speed_, 3) << "("
  //       << FloatToString(-slow_down_step_, 3) << ")";
}

bool MotionBase::Runable() {
  switch (state_) {
    case kStop:
    case kTimeout:
    case kReachTarget: {
      return false;
    }
    case kException:
    case kError: {
      ZWARN << "Should not run this function under state " << state_;
      return false;
    }
    case kReady: {
      state_ = kRunning;
      break;
    }
    default: {  // kRunning
      // ZINFO << "Now: " << std::to_string(Time::Now())
      //       << " start at: " << std::to_string(start_time_)
      //       << " Run for: " << std::to_string(Time::Now() - start_time_) <<
      //       "s";
      if (!DoubleEqual(timeout_sec_, 0) &&
          Time::Now() - start_time_ > timeout_sec_) {
        ZWARN << "Timeout(" << timeout_sec_ << "s).";
        state_ = kTimeout;
        return false;
      }
      break;
    }
  }

  return true;
}

void MotionBase::LimitSpeed(float& left_wheel_speed, float& right_wheel_speed) {
  LimitSpeed(left_wheel_speed, right_wheel_speed, min_wheel_speed_,
             max_wheel_speed_);
}

void MotionBase::LimitSpeed(float& left_wheel_speed, float& right_wheel_speed,
                            const float& limit_speed_min,
                            const float& limit_speed_max) {
  // Narrow speed to limit but maintain turn circle radius.
  while (left_wheel_speed > limit_speed_max ||
         left_wheel_speed < limit_speed_min ||
         right_wheel_speed > limit_speed_max ||
         right_wheel_speed < limit_speed_min) {
    double proportion;
    if (left_wheel_speed > limit_speed_max) {
      proportion = limit_speed_max / left_wheel_speed;
    } else if (left_wheel_speed < limit_speed_min) {
      proportion = limit_speed_min / left_wheel_speed;
    } else if (right_wheel_speed > limit_speed_max) {
      proportion = limit_speed_max / right_wheel_speed;
    } else if (right_wheel_speed < limit_speed_min) {
      proportion = limit_speed_min / right_wheel_speed;
    }
    left_wheel_speed *= proportion;
    right_wheel_speed *= proportion;
    // ZINFO << "Narrow speed to (" << left_wheel_speed << ", "
    //       << right_wheel_speed << ")";
  }
}

void MotionBase::Stop() { state_ = State::kStop; }

}  // namespace zima
