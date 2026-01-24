/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/motion/encircle_map_edge_motion.h"

#include "zima/logger/logger.h"

namespace zima {

EncircleMapEdgeMotion::Config::Config(const float& wheel_max_speed,
                                      const float& map_resolution,
                                      const JsonSPtr& json)
    : MotionConfigBase(json) {
  // Override if json config is provided.
  if (json != nullptr) {
    // ZINFO << "Load config from json.";
    config_valid_ = ParseFromJson(wheel_max_speed, map_resolution, json);
  } else {
    JsonSPtr config;
    if (GlobalJsonConfig::Instance()->GetGlobalConfigObject(kConfigKey_,
                                                            config)) {
      // ZINFO << "Load config from global json config.";
      config_valid_ = ParseFromJson(wheel_max_speed, map_resolution, config);
    } else {
      config_valid_ = false;
    }
  }
}

bool EncircleMapEdgeMotion::Config::ParseFromJson(const float& wheel_max_speed,
                                                  const float& map_resolution,
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
    speed_up_step_ = 2 * target_speed_ / cycle_rate_;
  }
  if (timeout_ < 0.01) {
    timeout_ = 0;
  }

  if (!JsonHelper::GetFloat(*json, kTargetMapEdgeDistanceKey_,
                            target_map_edge_distance_)) {
    target_map_edge_distance_ = 0.01;
  }
  if (!JsonHelper::GetFloat(*json, kMaxValidMapEdgeDistanceKey_,
                            max_valid_map_edge_distance_)) {
    max_valid_map_edge_distance_ = map_resolution * 4;
  }
  if (!JsonHelper::GetFloat(*json, kTurnCircleRadiusKey_,
                            turn_circle_radius_)) {
    turn_circle_radius_ = map_resolution / 2;
  }
  if (!JsonHelper::GetFloat(*json, kDelayDistanceKey_, delay_distance_)) {
    delay_distance_ = 0;
  }

  return true;
}

EncircleMapEdgeMotion::EncircleMapEdgeMotion(const float& half_track_length,
                                             const float& max_wheel_speed,
                                             const float& min_wheel_speed,
                                             const bool& map_edge_on_left,
                                             const Config& config)
    : MotionBase(half_track_length, max_wheel_speed, min_wheel_speed,
                 config.target_speed_, config.speed_up_step_, config.timeout_),
      last_map_edge_distance_(0),
      map_edge_on_left_(map_edge_on_left),
      target_map_edge_distance_(config.target_map_edge_distance_),
      max_valid_map_edge_distance_(config.max_valid_map_edge_distance_),
      turn_circle_radius_(config.turn_circle_radius_),
      delay_distance_(config.delay_distance_) {
  name_ = typeid(EncircleMapEdgeMotion).name();
  if (state_ == State::kError) {
    ZWARN << "State error.";
    return;
  }
  if (target_map_edge_distance_ > max_valid_map_edge_distance_) {
    ZWARN << "Target distance:(" << target_map_edge_distance_
          << ") > min invalid distance: (" << max_valid_map_edge_distance_
          << ")";
    state_ = State::kError;
    return;
  }
  if (turn_circle_radius_ < 0) {
    ZWARN << "Turn circle radius invalid:(" << turn_circle_radius_;
    state_ = State::kError;
    return;
  }
  if (delay_distance_ < 0) {
    ZWARN << "Delay distance invalid:(" << delay_distance_;
    state_ = State::kError;
    return;
  }
  ZINFO << "On left(" << map_edge_on_left_ << "), target:("
        << target_map_edge_distance_ << "), min invalid: ("
        << max_valid_map_edge_distance_ << "), circle radius: ("
        << turn_circle_radius_ << ") delay: (" << delay_distance_ << ")";
}

EncircleMapEdgeMotion::~EncircleMapEdgeMotion() {
  // ZINFO;
}

void EncircleMapEdgeMotion::SetCurrentMapEdgeDistance(
    const MapPoint& pose, const float& current_map_edge_distance) {
  WriteLocker lock(access_);
  if (!DoubleEqual(last_map_edge_distance_, 0) &&
      last_map_edge_distance_ < max_valid_map_edge_distance_ &&
      current_map_edge_distance > max_valid_map_edge_distance_) {
    lost_map_edge_odom_pose_ = pose;
    ZINFO << "Lost map edge at: " << lost_map_edge_odom_pose_
          << "(distance: " << current_map_edge_distance << ")";
  }
  current_map_edge_distance_ = current_map_edge_distance;
  last_map_edge_distance_ = current_map_edge_distance_;
}

void EncircleMapEdgeMotion::CheckState(const MapPoint& world_pose,
                                       const MapPoint& odom_pose,
                                       const float& current_left_wheel_speed,
                                       const float& current_right_wheel_speed) {
  // No internal checking logic.
}

void EncircleMapEdgeMotion::SpeedControl(const MapPoint& world_pose,
                                         const MapPoint& odom_pose,
                                         const float& current_left_wheel_speed,
                                         const float& current_right_wheel_speed,
                                         float& left_wheel_speed,
                                         float& right_wheel_speed) {
  WriteLocker lock(access_);
  if (!Runable()) {
    left_wheel_speed = 0;
    right_wheel_speed = 0;
    ZERROR;
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

  if (handle_slow_down_ && current_speed > slow_down_speed_) {
    // ZWARN;
    current_speed -= slow_down_step_;
  } else {
    // ZWARN << "Current speed:" << current_speed << ", target: " <<
    // target_speed_
    //       << ", step: " << speed_up_step_;
    if (current_speed < target_speed_) {
      current_speed += speed_up_step_;
    } else if (current_speed > target_speed_ + speed_up_step_) {
      ZGWARN << "Init speed too high: " << current_speed;
      current_speed -= speed_up_step_;
    }
  }

  // ZWARN << "Current speed:" << current_speed;
  // ZERROR << "Current distance:" << current_map_edge_distance_;

  // This motion will not reach target at all.
  float distance_base_left_wheel_speed, distance_base_right_wheel_speed;

  DistanceBaseSpeedControl(world_pose, odom_pose, current_speed,
                           distance_base_left_wheel_speed,
                           distance_base_right_wheel_speed);
  left_wheel_speed = distance_base_left_wheel_speed;
  right_wheel_speed = distance_base_right_wheel_speed;

  *last_speed_ = (left_wheel_speed + right_wheel_speed) / 2;

  if (handle_slow_down_ &&
      (fabs(current_left_wheel_speed) > slow_down_speed_ ||
       fabs(current_right_wheel_speed) > slow_down_speed_)) {
    auto max = std::max(fabs(current_left_wheel_speed),
                        fabs(current_right_wheel_speed));
    LimitSpeed(left_wheel_speed, right_wheel_speed, -max, max);
  } else {
    LimitSpeed(left_wheel_speed, right_wheel_speed);
  }

  handle_slow_down_ = false;
  // ZINFO << name_ << "(" << left_wheel_speed << ", " << right_wheel_speed <<
  // ")";
  return;
}

void EncircleMapEdgeMotion::DistanceBaseSpeedControl(const MapPoint& world_pose,
                                                     const MapPoint& odom_pose,
                                                     const float& current_speed,
                                                     float& left_wheel_speed,
                                                     float& right_wheel_speed) {
  // This motion will not reach target at all.
  if (current_map_edge_distance_ > max_valid_map_edge_distance_) {
    // Handle for lost map edge distance.
    // ZWARN << "Handle for lost map edge distance.";
    if (odom_pose.Distance(lost_map_edge_odom_pose_) > delay_distance_) {
      // ZWARN << "Current speed:" << current_speed;
      // Turn for a circle.
      if (map_edge_on_left_) {
        left_wheel_speed =
            current_speed -
            (current_speed * half_track_length_) / (2 * turn_circle_radius_);
        right_wheel_speed =
            current_speed +
            (current_speed * half_track_length_) / (2 * turn_circle_radius_);
      } else {
        left_wheel_speed =
            current_speed +
            (current_speed * half_track_length_) / (2 * turn_circle_radius_);
        right_wheel_speed =
            current_speed -
            (current_speed * half_track_length_) / (2 * turn_circle_radius_);
      }
    } else {
      // ZWARN << "Current speed:" << current_speed;
      // Go straight for a distance.
      left_wheel_speed = current_speed;
      right_wheel_speed = current_speed;
    }
  } else {
    // ZWARN << "Current speed:" << current_speed;
    // Carefully keep distance with map edge.
    auto delta_distance =
        current_map_edge_distance_ - target_map_edge_distance_;
    float degree =
        RadiansToDegrees(atan2(delta_distance, turn_circle_radius_ * 2));
    // ZWARN << "Delta distance: " << delta_distance << ", degree: " << degree;
    float adjust_circle_radius = turn_circle_radius_;
    if (InRange(static_cast<double>(degree), -15.0, 15.0)) {
      // When degree is 90, radius should reach the min.
      // Smaller the degree, larger the radius.
      adjust_circle_radius =
          turn_circle_radius_ / fabs(sin(DegreesToRadians(degree * 6)));
      // ZINFO << "Current radius:" << turn_circle_radius;
    }

    // ZERROR << "adjust_circle_radius: " << adjust_circle_radius;

    if (delta_distance > 0) {
      if (map_edge_on_left_) {
        left_wheel_speed =
            current_speed -
            (current_speed * half_track_length_) / (2 * adjust_circle_radius);
        right_wheel_speed =
            current_speed +
            (current_speed * half_track_length_) / (2 * adjust_circle_radius);
      } else {
        left_wheel_speed =
            current_speed +
            (current_speed * half_track_length_) / (2 * adjust_circle_radius);
        right_wheel_speed =
            current_speed -
            (current_speed * half_track_length_) / (2 * adjust_circle_radius);
      }
    } else {
      if (map_edge_on_left_) {
        left_wheel_speed =
            current_speed +
            (current_speed * half_track_length_) / (1.5 * adjust_circle_radius);
        right_wheel_speed =
            current_speed -
            (current_speed * half_track_length_) / (1.5 * adjust_circle_radius);
      } else {
        left_wheel_speed =
            current_speed -
            (current_speed * half_track_length_) / (1.5 * adjust_circle_radius);
        right_wheel_speed =
            current_speed +
            (current_speed * half_track_length_) / (1.5 * adjust_circle_radius);
      }
    }
    // ZWARN << "Left speed:" << left_wheel_speed
    //       << " right: " << right_wheel_speed;
  }
}

}  // namespace zima
