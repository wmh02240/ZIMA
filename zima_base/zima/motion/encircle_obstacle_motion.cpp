/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/motion/encircle_obstacle_motion.h"

#include "zima/logger/logger.h"

namespace zima {

EncircleObstacleMotion::Config::Config(const float& wheel_max_speed,
                                       const float& chassis_radius,
                                       const MapPoint& wall_sensor_tf,
                                       const JsonSPtr& json)
    : MotionConfigBase(json) {
  // Override if json config is provided.
  if (json != nullptr) {
    // ZINFO << "Load config from json.";
    config_valid_ =
        ParseFromJson(wheel_max_speed, chassis_radius, wall_sensor_tf, json);
  } else {
    JsonSPtr config;
    if (GlobalJsonConfig::Instance()->GetGlobalConfigObject(kConfigKey_,
                                                            config)) {
      // ZINFO << "Load config from global json config.";
      config_valid_ = ParseFromJson(wheel_max_speed, chassis_radius,
                                    wall_sensor_tf, config);
    } else {
      config_valid_ = false;
    }
  }
}

bool EncircleObstacleMotion::Config::ParseFromJson(
    const float& wheel_max_speed, const float& chassis_radius,
    const MapPoint& wall_sensor_tf, const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }

  if (!MotionConfigBase::ParseFromJson(json)) {
    return false;
  }

  if (FloatEqual(target_speed_, 0)) {
    target_speed_ = wheel_max_speed / 2;
  }
  if (FloatEqual(speed_up_step_, 0)) {
    speed_up_step_ = 2 * target_speed_ / cycle_rate_;
  }
  if (timeout_ < 0.01) {
    timeout_ = 0;
  }

  if (!JsonHelper::GetFloat(*json, kTargetObstacleDistanceKey_,
                            target_obstacle_distance_)) {
    target_obstacle_distance_ = 0.02;
  }
  if (!JsonHelper::GetFloat(*json, kMaxValidObstacleDistanceKey_,
                            max_valid_obstacle_distance_)) {
    max_valid_obstacle_distance_ = target_obstacle_distance_ * 3;
  }
  if (!JsonHelper::GetFloat(*json, kTurnCircleRadiusKey_,
                            turn_circle_radius_)) {
    turn_circle_radius_ = chassis_radius / 2;
  }
  if (!JsonHelper::GetFloat(*json, kDelayDistanceKey_, delay_distance_)) {
    delay_distance_ = wall_sensor_tf.Length() * cos(wall_sensor_tf.Radian());
  }
  if (!JsonHelper::GetFloat(*json, kReachTempTargetDistanceKey_,
                            reach_temp_target_distance_)) {
    reach_temp_target_distance_ = turn_circle_radius_;
  }
  if (!JsonHelper::GetFloat(*json, kStopForRoomValueDistanceKey_,
                            stop_for_room_value_distance_)) {
    stop_for_room_value_distance_ = 0.02;
  }
  if (!JsonHelper::GetFloat(*json, kStopForUserBlockDistanceKey_,
                            stop_for_user_block_distance_)) {
    stop_for_user_block_distance_ = chassis_radius + 0.03;
  }
  if (!JsonHelper::GetFloat(*json, kStopForCleaningAreaEdgeDistanceKey_,
                            stop_for_cleaning_area_edge_distance_)) {
    stop_for_cleaning_area_edge_distance_ = chassis_radius * 1.2;
  }
  if (!JsonHelper::GetFloat(*json, kStopForLidarObsDistanceKey_,
                            stop_for_lidar_obs_distance_)) {
    stop_for_lidar_obs_distance_ = 0.00;
  }
  if (!JsonHelper::GetFloat(*json, kStopForLidarCompensateKey_,
                            stop_for_lidar_compensate_)) {
    stop_for_lidar_compensate_ = 0;
  }

  return true;
}

EncircleObstacleMotion::EncircleObstacleMotion(const float& half_track_length,
                                               const float& max_wheel_speed,
                                               const float& min_wheel_speed,
                                               const bool& obstacle_on_left,
                                               const Config& config)
    : MotionBase(half_track_length, max_wheel_speed, min_wheel_speed,
                 config.target_speed_, config.speed_up_step_, config.timeout_),
      last_obstacle_distance_(0),
      obstacle_on_left_(obstacle_on_left),
      target_obstacle_distance_(config.target_obstacle_distance_),
      max_valid_obstacle_distance_(config.max_valid_obstacle_distance_),
      turn_circle_radius_(config.turn_circle_radius_),
      delay_distance_(config.delay_distance_),
      dynamic_path_lock_(make_shared<ReadWriteLock>()),
      dynamic_path_in_odom_({}),
      reach_temp_target_distance_(config.reach_temp_target_distance_) {
  name_ = typeid(EncircleObstacleMotion).name();
  if (state_ == State::kError) {
    ZWARN << "State error.";
    return;
  }
  if (target_obstacle_distance_ > max_valid_obstacle_distance_) {
    ZWARN << "Target distance:(" << target_obstacle_distance_
          << ") > min invalid distance: (" << max_valid_obstacle_distance_
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
  // ZINFO << "On left(" << obstacle_on_left_ << "), target:("
  //       << target_obstacle_distance_ << "), min invalid: ("
  //       << max_valid_obstacle_distance_ << "), circle radius: ("
  //       << turn_circle_radius_ << ") delay: (" << delay_distance_ << ")";
}

EncircleObstacleMotion::~EncircleObstacleMotion() {
  // ZINFO;
}

void EncircleObstacleMotion::SetCurrentObsDistance(
    const MapPoint& pose, const float& current_obstacle_distance) {
  WriteLocker lock(access_);
  if (!DoubleEqual(last_obstacle_distance_, 0) &&
      last_obstacle_distance_ < max_valid_obstacle_distance_ &&
      current_obstacle_distance > max_valid_obstacle_distance_) {
    lost_obstacle_odom_pose_ = pose;
    ZGINFO << "Lost obstacle at: " << lost_obstacle_odom_pose_
           << "(distance: " << current_obstacle_distance << ")";
  }
  // ZINFO << "Set obs distance: " << FloatToString(current_obstacle_distance,
  // 2);
  current_obstacle_distance_ = current_obstacle_distance;
  last_obstacle_distance_ = current_obstacle_distance_;
}

void EncircleObstacleMotion::CheckState(
    const MapPoint& world_pose, const MapPoint& odom_pose,
    const float& current_left_wheel_speed,
    const float& current_right_wheel_speed) {
  // No internal checking logic.
}

void EncircleObstacleMotion::SpeedControl(
    const MapPoint& world_pose, const MapPoint& odom_pose,
    const float& current_left_wheel_speed,
    const float& current_right_wheel_speed, float& left_wheel_speed,
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

  if (handle_slow_down_ && current_speed > slow_down_speed_) {
    current_speed -= slow_down_step_;
  } else {
    // ZWARN << "Current speed:" << current_speed << ", target: " <<
    // target_speed_
    //       << ", step: " << speed_up_step_;
    if (current_speed < target_speed_) {
      current_speed += speed_up_step_;
    } else if (current_speed > target_speed_ + speed_up_step_) {
      // ZGWARN << "Init speed too high: " << current_speed;
      current_speed -= speed_up_step_;
    }
  }

  // ZWARN << "Current speed:" << current_speed;
  // ZERROR << "Current distance:" << current_obstacle_distance_;

  // This motion will not reach target at all.
  float path_base_left_wheel_speed, path_base_right_wheel_speed,
      distance_base_left_wheel_speed, distance_base_right_wheel_speed;

  PathBaseSpeedControl(world_pose, odom_pose, current_speed,
                       path_base_left_wheel_speed, path_base_right_wheel_speed);
  DistanceBaseSpeedControl(world_pose, odom_pose, current_speed,
                           distance_base_left_wheel_speed,
                           distance_base_right_wheel_speed);
  if ((obstacle_on_left_ &&
       path_base_left_wheel_speed - path_base_right_wheel_speed >
           distance_base_left_wheel_speed - distance_base_right_wheel_speed) ||
      (!obstacle_on_left_ &&
       path_base_right_wheel_speed - path_base_left_wheel_speed >
           distance_base_right_wheel_speed - distance_base_left_wheel_speed)) {
    left_wheel_speed = path_base_left_wheel_speed;
    right_wheel_speed = path_base_right_wheel_speed;
  } else {
    left_wheel_speed = distance_base_left_wheel_speed;
    right_wheel_speed = distance_base_right_wheel_speed;
  }

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

void EncircleObstacleMotion::DistanceBaseSpeedControl(
    const MapPoint& world_pose, const MapPoint& odom_pose,
    const float& current_speed, float& left_wheel_speed,
    float& right_wheel_speed) {
  // This motion will not reach target at all.
  if (current_obstacle_distance_ > max_valid_obstacle_distance_) {
    // Handle for lost obs distance.
    // ZWARN << "Handle for lost obs distance.";
    if (odom_pose.Distance(lost_obstacle_odom_pose_) > delay_distance_) {
      // ZWARN << "Current speed:" << current_speed;
      // Turn for a circle.
      if (obstacle_on_left_) {
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
    // ZWARN << "Left speed:" << current_left_wheel_speed
    //       << " right: " << current_right_wheel_speed;
    // Carefully keep distance with obstacle.
    auto delta_distance =
        current_obstacle_distance_ - target_obstacle_distance_;
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
      if (obstacle_on_left_) {
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
      if (obstacle_on_left_) {
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
  }
}

void EncircleObstacleMotion::PathBaseSpeedControl(const MapPoint& world_pose,
                                                  const MapPoint& odom_pose,
                                                  const float& current_speed,
                                                  float& left_wheel_speed,
                                                  float& right_wheel_speed) {
  WriteLocker lock(dynamic_path_lock_);
  while (!dynamic_path_in_odom_.empty()) {
    if (odom_pose.Distance(dynamic_path_in_odom_.front()) <
        reach_temp_target_distance_) {
      dynamic_path_in_odom_.pop_front();
      // ZINFO << dynamic_path_in_odom_.size() << " points left.";
      continue;
    }
    // Calculate angle diff from robot heading to robot-target direction.
    // ZINFO << "Pose:" << pose;
    float degree = odom_pose.AngleDiff(
        MapPoint::GetVector(odom_pose, dynamic_path_in_odom_.front()));
    // ZINFO << "Current degree:" << degree;
    float turn_circle_radius = turn_circle_radius_;
    if (InRange(static_cast<double>(degree), -45.0, 45.0)) {
      // When degree is 90, radius should reach the min.
      // Smaller the degree, larger the radius.
      turn_circle_radius =
          turn_circle_radius_ / fabs(sin(DegreesToRadians(degree * 2)));
      // ZINFO << "Current radius:" << turn_circle_radius;
    }
    // ZINFO << "Current radius:" << turn_circle_radius;
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
    break;
  }

  if (dynamic_path_in_odom_.empty()) {
    float adjust_circle_radius = turn_circle_radius_;
    if (obstacle_on_left_) {
      left_wheel_speed = current_speed - (current_speed * half_track_length_) /
                                             (2 * adjust_circle_radius);
      right_wheel_speed = current_speed + (current_speed * half_track_length_) /
                                              (2 * adjust_circle_radius);
    } else {
      left_wheel_speed = current_speed + (current_speed * half_track_length_) /
                                             (2 * adjust_circle_radius);
      right_wheel_speed = current_speed - (current_speed * half_track_length_) /
                                              (2 * adjust_circle_radius);
    }
  }
  // ZINFO << name_ << "(" << left_wheel_speed << ", " << right_wheel_speed <<
  // ")";
}

void EncircleObstacleMotion::UpdateDynamicPath(
    const MapPointPath& dynamic_path_in_odom) {
  WriteLocker lock(dynamic_path_lock_);
  dynamic_path_in_odom_ = dynamic_path_in_odom;
  // ZINFO << "Update for " << dynamic_path_in_odom_.size() << " points.";
}

}  // namespace zima
