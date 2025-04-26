/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/motion/trace_path_motion.h"

#include "zima/common/util.h"
#include "zima/common/transform.h"
#include "zima/logger/logger.h"

namespace zima {

TracePathMotion::Config::Config(const float& wheel_max_speed,
                                const float& chassis_radius,
                                const JsonSPtr& json)
    : MotionConfigBase(json) {
  // Override if json config is provided.
  if (json != nullptr) {
    // ZINFO << "Load config from json.";
    config_valid_ = ParseFromJson(wheel_max_speed, chassis_radius, json);
  } else {
    JsonSPtr config;
    if (GlobalJsonConfig::Instance()->GetGlobalConfigObject(kConfigKey_,
                                                            config)) {
      // ZINFO << "Load config from global json config.";
      config_valid_ = ParseFromJson(wheel_max_speed, chassis_radius, config);
    } else {
      config_valid_ = false;
    }
  }
}

bool TracePathMotion::Config::ParseFromJson(const float& wheel_max_speed,
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
    target_speed_ = wheel_max_speed;
  }
  if (FloatEqual(speed_up_step_, 0)) {
    speed_up_step_ = target_speed_ / cycle_rate_;
  }
  if (timeout_ < 0.01) {
    timeout_ = 120;
  }

  if (!JsonHelper::GetFloat(*json, kMinTurnCircleRadiusKey_,
                            min_turn_circle_radius_)) {
    min_turn_circle_radius_ = chassis_radius / 3;
  }
  if (!JsonHelper::GetFloat(*json, kInterpolationDistanceKey_,
                            interpolation_distance_)) {
    interpolation_distance_ = 0.02;
  }
  if (!JsonHelper::GetFloat(*json, kMaxNearDistanceKey_, max_near_distance_)) {
    max_near_distance_ = chassis_radius;
  }
  if (!JsonHelper::GetFloat(*json, kMaxReachDistanceKey_,
                            max_reach_distance_)) {
    max_reach_distance_ = 0.01;
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
    stop_for_lidar_obs_distance_ = 0.01;
  }
  if (!JsonHelper::GetFloat(*json, kStopForLidarCompensateKey_,
                            stop_for_lidar_compensate_)) {
    stop_for_lidar_compensate_ = 0;
  }
  return true;
}

TracePathMotion::TracePathMotion(const float& half_track_length,
                                 const float& max_wheel_speed,
                                 const float& min_wheel_speed,
                                 const MapPointPath& path, const Config& config)
    : MotionBase(half_track_length, max_wheel_speed, min_wheel_speed,
                 config.target_speed_, config.speed_up_step_, config.timeout_),
      tracing_last_point_(false),
      path_(path),
      interpolation_distance_(config.interpolation_distance_),
      // interpolation_path_({}),
      min_turn_circle_radius_(config.min_turn_circle_radius_),
      max_near_distance_(config.max_near_distance_),
      max_reach_distance_(config.max_reach_distance_),
      check_reach_target_coordinate_(nullptr) {
  name_ = typeid(TracePathMotion).name();
  if (state_ == State::kError) {
    ZWARN << "State error.";
    return;
  }
  if (target_speed_ < 0) {
    ZWARN << "Target speed " << FloatToString(target_speed_, 2);
    state_ = State::kError;
    return;
  }
  if (path_.size() < 2) {
    ZWARN << "Path size " << path_.size();
    state_ = State::kError;
    return;
  }
  if (interpolation_distance_ < 0.005) {
    ZWARN << "Interpolation distance "
          << FloatToString(interpolation_distance_, 4);
    state_ = State::kError;
    return;
  }
  if (min_turn_circle_radius_ < 0) {
    ZWARN << "min_turn_circle_radius_ "
          << FloatToString(min_turn_circle_radius_, 3);
    state_ = State::kError;
    return;
  }

  // TODO(Austin): Check for min_turn_circle_radius vs min/max_wheel_speed
  // if (...)

  // ZINFO << MapPoint::DebugString(path_);
  target_ = path_.begin() + 1;
  // interpolation_path_ = GenerateInterpolationPoints(
  //     *path_.begin(), *(path_.begin() + 1), interpolation_distance_);
  // interpolation_target_ = interpolation_path_.begin();

  handle_slow_down_ = false;
  slow_down_speed_ = 0;
  slow_down_step_ = 0;

  if (path_.size() > 2 || path_.front() != path_.back()) {
    auto point_at_target_vector =
        MapPoint::GetVector(*(path_.end() - 2), *(path_.end() - 1));
    // Use vector angle as x axis.
    check_reach_target_coordinate_ = std::make_shared<MapPoint>(
        path_.back().X(), path_.back().Y(), point_at_target_vector.Degree());
  }

  check_reach_function_ = [&](const MapPoint& world_pose) -> bool {
    if (check_reach_target_coordinate_ == nullptr) {
      return world_pose.Distance(*target_) < max_reach_distance_;
    } else {
      double x, y;
      Transform::CoordinateTransformationAB(
          world_pose.X(), world_pose.Y(), check_reach_target_coordinate_->X(),
          check_reach_target_coordinate_->Y(),
          DegreesToRadians(check_reach_target_coordinate_->Degree()), x, y);
      if (x > -max_reach_distance_) {
        // ZERROR;
        return true;
      }
      return false;
    }
  };

  // ZINFO << "Trace " << path_.size() << " targets, speed("
  //       << FloatToString(target_speed_, 2) << ")";
}

TracePathMotion::~TracePathMotion() {
  // ZINFO;
}

void TracePathMotion::CheckState(const MapPoint& world_pose,
                                 const MapPoint& odom_pose,
                                 const float& current_left_wheel_speed,
                                 const float& current_right_wheel_speed) {
  WriteLocker lock(access_);

  tracing_last_point_ = false;
  if (target_ == path_.end() - 1) {
    // Handle for last point.
    // ZINFO << "Distance: " << world_pose.Distance(*target_);
    tracing_last_point_ = true;
    if (check_reach_function_(world_pose)) {
      if (!reached_target_log_displayed_) {
        ZGINFO << "Reach target: " << *target_;
        reached_target_log_displayed_ = true;
      }
      state_ = kReachTarget;
      return;
    }
  } else {
    if (world_pose.Distance(*target_) < max_near_distance_) {
      while (target_ != path_.end() - 1) {
        // auto start_point = *target_;
        target_++;
        if (world_pose.Distance(*target_) > max_near_distance_) {
          // ZINFO << "Curr: " << world_pose << " change target as " << *target_;
          // interpolation_path_ = GenerateInterpolationPoints(
          //     start_point, *target_, interpolation_distance_);
          // ZINFO << "Interpolated to " << interpolation_path_.size()
          //       << " points.";
          // interpolation_target_ = interpolation_path_.begin();
          break;
        }
      }
    }
    // if (world_pose.Distance(*interpolation_target_) < max_near_distance_) {
    //   while (interpolation_target_ != interpolation_path_.end() - 1) {
    //     interpolation_target_++;
    //     if (world_pose.Distance(*interpolation_target_) > max_near_distance_) {
    //       // ZINFO << "Curr: " << world_pose << " change interpolation target as "
    //       //       << *interpolation_target_;
    //       break;
    //     }
    //   }
    // }
  }
}

void TracePathMotion::SpeedControl(const MapPoint& world_pose,
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

  // Calculate speed.
  // TODO(Austin): Here we can use PID in the future.
  // float current_speed =
  //     (current_left_wheel_speed + current_right_wheel_speed) / 2;
  if (last_speed_ == nullptr) {
    last_speed_ = std::make_shared<float>(
        (current_left_wheel_speed + current_right_wheel_speed) / 2);
  }
  auto current_speed = *last_speed_;
  auto target = target_;
  // auto target = interpolation_target_;
  // ZISODBG << "Current speed:" << current_left_wheel_speed << ", "
  //        << current_right_wheel_speed;
  auto degree_diff = target->AngleDiff(world_pose);
  if (handle_slow_down_ && current_speed > slow_down_speed_) {
    current_speed -= slow_down_step_;
    // ZINFO << "Slowing down " << FloatToString(current_speed, 3);
  } else if (tracing_last_point_ && current_speed > target_speed_ / 2) {
    // ZISODBG << "Slow down before last target";
    current_speed -= slow_down_step_;
  } else if (fabs(degree_diff) > 5 && current_speed > target_speed_ / 2) {
    // ZISODBG << "Slow down before turn, curr " << world_pose.Degree()
    //        << ", target " << target->Degree();
    current_speed -= slow_down_step_;
  } else {
    // ZISODBG << "Current speed:" << current_speed
    //       << " target speed:" << target_speed_ << " step: " <<
    //       speed_up_step_;
    if (current_speed < target_speed_) {
      current_speed += speed_up_step_;
    } else if (current_speed > target_speed_ + speed_up_step_) {
      ZGWARN << "Init speed too high: " << FloatToString(current_speed, 2);
      current_speed -= speed_up_step_;
    }
  }
  // ZISODBG << "Current speed:" << current_speed
  //       << " target speed:" << target_speed_;

  // Calculate angle diff from robot heading to robot-target direction.
  // ZINFO << "Pose:" << pose;
  float degree = world_pose.AngleDiff(MapPoint::GetVector(world_pose, *target));
  // ZINFO << "Current degree: " << ZCOLOR_GREEN << FloatToString(degree, 1)
  //       << ZCOLOR_NONE;
  float turn_circle_radius = min_turn_circle_radius_;
  if (InRange(static_cast<double>(degree), -8.0, 8.0)) {
    turn_circle_radius =
        min_turn_circle_radius_ / fabs(sin(DegreesToRadians(degree / 3)));
  } else if (InRange(static_cast<double>(degree), -45.0, 45.0)) {
    // When degree is 90, radius should reach the min.
    // Smaller the degree, larger the radius.
    turn_circle_radius =
        min_turn_circle_radius_ / fabs(sin(DegreesToRadians(degree * 2)));
    // ZINFO << "Current radius:" << turn_circle_radius;
  }
  // ZISODBG << "Current radius:" << turn_circle_radius;
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
  // ZINFO << "(" << FloatToString(left_wheel_speed, 2) << ", "
  //       << FloatToString(right_wheel_speed, 2) << ")";

  return;
}

uint32_t TracePathMotion::GetRestPathPointCount() const {
  ReadLocker lock(access_);
  auto distance = std::distance(target_, path_.cend());
  if (distance < 0) {
    ZERROR << "Distance below zero.";
    return UINT32_MAX;
  }
  return distance;
}

MapPointPath TracePathMotion::GetRestPath() const {
  ReadLocker lock(access_);
  MapPointPath rest_path{};
  rest_path.insert(rest_path.end(), target_, path_.cend());
  return rest_path;
}

bool TracePathMotion::ExtendPath(const MapPointPath& path) {
  WriteLocker lock(access_);
  auto distance = std::distance(path_.cbegin(), target_);
  ZGINFO << "Target(" << distance << ") pointing at " << target_->DebugString()
         << "Update " << path.size() << " path points.";
  path_.insert(path_.end(), path.cbegin(), path.cend());
  target_ = path_.begin() + distance;
  ZGINFO << MapPoint::DebugString(path_);
  ZGINFO << "Target(" << distance << ") now pointing at "
         << target_->DebugString();
  // interpolation_path_ =
  //     GenerateInterpolationPoints(*target_, *target_,
  //     interpolation_distance_);
  // ZINFO << "Interpolated to " << interpolation_path_.size() << " points.";
  // interpolation_target_ = interpolation_path_.begin();
  return true;
}

}  // namespace zima
