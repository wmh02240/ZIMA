/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/movement/trace_path_movement.h"

#include "zima/common/util.h"

namespace zima {

TracePathMovement::Config::Config(const Chassis::SPtr& chassis,
                                  const NavMap::SPtr& map,
                                  const JsonSPtr& json) {
  // Override if json config is provided.
  if (json != nullptr) {
    // ZINFO << "Load config from json.";
    config_valid_ = ParseFromJson(
        chassis->GetRadius(), chassis->GetTrackLength(),
        chassis->GetWheel(chassis->kLeftWheel_)->MaxSpeed(), json);
  } else {
    JsonSPtr config;
    if (GlobalJsonConfig::Instance()->GetGlobalConfigObject(kConfigKey_,
                                                            config)) {
      // ZINFO << "Load config from global json config.";
      config_valid_ = ParseFromJson(
          chassis->GetRadius(), chassis->GetTrackLength(),
          chassis->GetWheel(chassis->kLeftWheel_)->MaxSpeed(), config);
    } else {
      config_valid_ = false;
    }
  }
}

bool TracePathMovement::Config::ParseFromJson(const float& chassis_radius,
                                              const float& track_length,
                                              const float& wheel_max_speed,
                                              const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }

  // Override.
  if (!JsonHelper::GetFloat(*json, kRetreatDistanceKey_, retreat_distance_)) {
    retreat_distance_ = 0.005;
  }
  JsonSPtr _tp_json_config(new Json());
  if (JsonHelper::GetObject(*json, TracePathMotion::Config::kConfigKey_,
                            *_tp_json_config)) {
    // ZINFO << "Override config from json.";
    trace_path_motion_config_.reset(new TracePathMotion::Config(
        wheel_max_speed, chassis_radius, _tp_json_config));
    if (!trace_path_motion_config_->config_valid_) {
      ZERROR << "Config " << TracePathMotion::Config::kConfigKey_ << " in "
             << kConfigKey_ << " invalid.";
      return false;
    }
  }
  JsonSPtr _retreat_json_config(new Json());
  if (JsonHelper::GetObject(*json, RetreatMotion::Config::kConfigKey_,
                            *_retreat_json_config)) {
    // ZINFO << "Override config from json.";
    retreat_motion_config_.reset(new RetreatMotion::Config(
        retreat_distance_, wheel_max_speed, _retreat_json_config));
    if (!retreat_motion_config_->config_valid_) {
      ZERROR << "Config " << RetreatMotion::Config::kConfigKey_ << " in "
             << kConfigKey_ << " invalid.";
      return false;
    }
  }
  JsonSPtr _rotate_json_config(new Json());
  if (JsonHelper::GetObject(*json, RotateMotion::Config::kConfigKey_,
                            *_rotate_json_config)) {
    // ZINFO << "Override config from json.";
    rotate_motion_config_.reset(new RotateMotion::Config(
        track_length, wheel_max_speed, _rotate_json_config));
    if (!rotate_motion_config_->config_valid_) {
      ZERROR << "Config " << RotateMotion::Config::kConfigKey_ << " in "
             << kConfigKey_ << " invalid.";
      return false;
    }
  }

  if (!JsonHelper::GetFloat(*json, kTracePathObstacleSlowDownDistanceKey_,
                            trace_path_obstacle_slow_down_distance_)) {
    trace_path_obstacle_slow_down_distance_ = 0.01;
  }
  if (!JsonHelper::GetFloat(*json, kTracePathObstacleSlowDownSpeedKey_,
                            trace_path_obstacle_slow_down_speed_)) {
    trace_path_obstacle_slow_down_speed_ =
        trace_path_motion_config_->target_speed_ / 3;
  }
  if (!JsonHelper::GetFloat(*json, kTracePathObstacleSlowDownSpeedStepKey_,
                            trace_path_obstacle_slow_down_speed_step_)) {
    trace_path_obstacle_slow_down_speed_step_ =
        trace_path_motion_config_->speed_up_step_ * 3;
  }

  return true;
}

using State = MotionBase::State;

TracePathMovement::TracePathMovement(
    const Chassis::SPtr& chassis, const MapPointPath& path,
    const NavMap::SPtr& map, const CharGridMap2D::DataType& current_room_index,
    const bool& enable_user_select_area_checking,
    const bool& enable_room_value_checking, const bool& trace_for_edge_path)
    : config_(chassis, map),
      stage_(kTurnForTarget),
      stop_reason_(kStopForException),
      current_room_index_(current_room_index),
      path_(path),
      enable_user_select_area_checking_(enable_user_select_area_checking),
      enable_room_value_checking_(enable_room_value_checking),
      trace_for_edge_path_(trace_for_edge_path) {
  name_ = typeid(TracePathMovement).name();
  chassis_has_center_bumper_ =
      chassis->IsDeviceRegistered(chassis->kCenterBumper_);

  if (!config_.config_valid_) {
    ZERROR << "Config invalid.";
    stage_= kStop;
    state_ = State::kError;
    return;
  }
  ZGINFO << "Trace for " << path_.size() << " path points.";
}

TracePathMovement::~TracePathMovement() {}

void TracePathMovement::Run(const Chassis::SPtr& chassis,
                            const MapPoint& world_pose,
                            const MapPoint& odom_pose,
                            const NavMap::SPtr& map) {
  WriteLocker lock(access_);
  float left_wheel_speed = 0;
  float right_wheel_speed = 0;

  if (steps_recorder_.AddPathPoint(StepPoint(world_pose))) {
    MapCell pose_cell;
    if (map->GetFootStepLayer()->WorldToMap(world_pose, pose_cell)) {
      // ZINFO << pose_cell.DebugString() << world_pose.DebugString();
    }
  }

  bool finish_run = false;

  while (!finish_run) {
    finish_run = true;

    switch (stage_) {
      case kFinish:
      case kStop: {
        break;
      }
      case kTurnForTarget: {
        if (p_rotate_motion_ == nullptr) {
          auto first_point = *path_.begin();
          auto second_point = *(path_.begin() + 1);
          auto pose_vector = MapPoint::GetVector(first_point, second_point);
          auto degree = world_pose.AngleDiff(pose_vector);
          if (!SwitchToRotateMotion(chassis, odom_pose, degree)) {
            break;
          }
        }

        // Abnormal state checking.
        auto state = p_rotate_motion_->GetState();
        // ZINFO << "Turn state:" << state;
        if (state == State::kError || state == State::kTimeout ||
            state == State::kException) {
          state_ = state;
          stop_reason_ = kStopForException;
          SwitchStageAndClearMotion(kStop);
          break;
        }

        p_rotate_motion_->CheckState(
            world_pose, odom_pose,
            chassis->GetWheel(chassis->kLeftWheel_)->CurrentSpeed(),
            chassis->GetWheel(chassis->kRightWheel_)->CurrentSpeed());
        state = p_rotate_motion_->GetState();
        if (state == State::kReachTarget) {
          SwitchToTracePathMotion(chassis, map, path_);
          finish_run = false;
          break;
        }

        // Event checking.
        if (HandleBumper(chassis, world_pose, map)) {
          SwitchToRetreatMotion(chassis, odom_pose);
          finish_run = false;
          break;
        }

        state_ = state;
        p_rotate_motion_->SpeedControl(
            world_pose, odom_pose,
            chassis->GetWheel(chassis->kLeftWheel_)->CurrentSpeed(),
            chassis->GetWheel(chassis->kRightWheel_)->CurrentSpeed(),
            left_wheel_speed, right_wheel_speed);
        break;
      }
      case kTracePath: {
        if (p_trace_path_motion_ == nullptr) {
          if (!SwitchToTracePathMotion(chassis, map, path_)) {
            break;
          }
        }
        auto state = p_trace_path_motion_->GetState();
        // ZINFO << "TP state:" << state;
        if (state == State::kError || state == State::kTimeout ||
            state == State::kStop || state == State::kException) {
          state_ = state;
          stop_reason_ = kStopForException;
          SwitchStageAndClearMotion(kStop);
          break;
        }

        p_trace_path_motion_->CheckState(
            world_pose, odom_pose,
            chassis->GetWheel(chassis->kLeftWheel_)->CurrentSpeed(),
            chassis->GetWheel(chassis->kRightWheel_)->CurrentSpeed());
        state = p_trace_path_motion_->GetState();

        if (state == State::kReachTarget) {
          SwitchStageAndClearMotion(kFinish);
          state_ = state;
          if (steps_recorder_.AddPathPoint(StepPoint(path_.back()))) {
            MapCell pose_cell;
            if (map->GetFootStepLayer()->WorldToMap(world_pose, pose_cell)) {
              ZGINFO << pose_cell.DebugString() << world_pose.DebugString();
            }
          }
          break;
        }

        // Event checking.
        if (HandleBumper(chassis, world_pose, map)) {
          SwitchToRetreatMotion(chassis, odom_pose);
          finish_run = false;
          break;
        }
        if (HandleMapValue(chassis, world_pose, map)) {
          // Stop reason is already format in HandleMapValue().
          state_ = State::kStop;
          SwitchStageAndClearMotion(kStop);
          break;
        }
        auto nearest_point = GetChassisFrontNearestPointCloudObs(chassis);
        const float kSlowDownDistance =
            chassis->GetRadius() +
            config_.trace_path_obstacle_slow_down_distance_;
        if (nearest_point.Distance() < kSlowDownDistance) {
          ZGINFO << "nearest_point.Distance()"
                 << FloatToString(nearest_point.Distance(), 3)
                 << ", kSlowDownDistance "
                 << FloatToString(kSlowDownDistance, 3);
          p_trace_path_motion_->SlowdownTo(
              config_.trace_path_obstacle_slow_down_speed_,
              config_.trace_path_obstacle_slow_down_speed_step_);
        }

        // auto rest_path_point_count =
        //     p_trace_path_motion_->GetRestPathPointCount();
        // if (rest_path_point_count < 1) {
        //   p_trace_path_motion_->SlowdownTo(
        //       config_.trace_path_obstacle_slow_down_speed_,
        //       config_.trace_path_obstacle_slow_down_speed_step_);
        // }

        // if (world_pose.Distance(path_.back()) < map->GetResolution() * 2 / 3) {
        if (world_pose.Distance(path_.back()) < map->GetResolution()) {
          p_trace_path_motion_->SlowdownTo(
              config_.trace_path_obstacle_slow_down_speed_,
              config_.trace_path_obstacle_slow_down_speed_step_);
        }

        if (HandleMainLidar(chassis, world_pose, map, nearest_point)) {
          stop_reason_ = kStopForObstacle;
          state_ = State::kStop;
          SwitchStageAndClearMotion(kStop);
          break;
        }

        auto path_situation = steps_recorder_.CheckPath();
        // ZISODBG << "path_situation: " << path_situation;
        if (path_situation == StepsRecorder::SituationCode::NORMAL_CLOSE_LOOP) {
          ZGINFO << "Loop close.";
          SwitchStageAndClearMotion(kFinish);
          state_ = State::kStop;
          break;
        }

        state_ = state;
        p_trace_path_motion_->SpeedControl(
            world_pose, odom_pose,
            chassis->GetWheel(chassis->kLeftWheel_)->CurrentSpeed(),
            chassis->GetWheel(chassis->kRightWheel_)->CurrentSpeed(),
            left_wheel_speed, right_wheel_speed);
        break;
      }
      case kRetreat: {
        if (p_retreat_motion_ == nullptr) {
          if (!SwitchToRetreatMotion(chassis, odom_pose)) {
            chassis->ClearBumperEvent();
            break;
          }
        }
        auto state = p_retreat_motion_->GetState();
        if (state == State::kError || state == State::kTimeout ||
            state == State::kStop || state == State::kException) {
          state_ = state;
          stop_reason_ = kStopForException;
          SwitchStageAndClearMotion(kStop);
          chassis->ClearBumperEvent();
          break;
        }

        p_retreat_motion_->CheckState(
            world_pose, odom_pose,
            chassis->GetWheel(chassis->kLeftWheel_)->CurrentSpeed(),
            chassis->GetWheel(chassis->kRightWheel_)->CurrentSpeed());
        state = p_retreat_motion_->GetState();

        if (state == State::kReachTarget) {
          stop_reason_ = kStopForObstacle;
          SwitchStageAndClearMotion(kStop);
          state_ = State::kStop;
          chassis->ClearBumperEvent();
          break;
        }
        state_ = state;
        p_retreat_motion_->SpeedControl(
            world_pose, odom_pose,
            chassis->GetWheel(chassis->kLeftWheel_)->CurrentSpeed(),
            chassis->GetWheel(chassis->kRightWheel_)->CurrentSpeed(),
            left_wheel_speed, right_wheel_speed);
        break;
      }
      default: {
        ZERROR << "Should never run here.";
      }
    }
  }

  if (state_ == State::kStop) {
    left_wheel_speed = 0;
    right_wheel_speed = 0;
  }
  chassis->GetWheel(chassis->kLeftWheel_)->SetTargetSpeed(left_wheel_speed);
  chassis->GetWheel(chassis->kRightWheel_)->SetTargetSpeed(right_wheel_speed);
}

float TracePathMovement::GetObstacleDegree() const {
  ReadLocker lock(access_);
  return obstacle_degree_;
}
float TracePathMovement::GetMapObstacleDegree() const {
  ReadLocker lock(access_);
  return map_obstacle_degree_;
}

TracePathMovement::StopReason TracePathMovement::GetStopReason() const {
  ReadLocker lock(access_);
  return stop_reason_;
}

void TracePathMovement::SwitchStageAndClearMotion(const Stage& stage) {
  // ZINFO << "Switch to stage: " << stage;
  stage_ = stage;
  p_trace_path_motion_.reset();
  p_retreat_motion_.reset();
  p_rotate_motion_.reset();
}

bool TracePathMovement::SwitchToRotateMotion(const Chassis::SPtr& chassis,
                                             const MapPoint& odom_pose,
                                             const float& rotate_degree) {
  SwitchStageAndClearMotion(kTurnForTarget);
  p_rotate_motion_.reset(new RotateMotion(
      chassis->GetTrackLength() / 2,
      chassis->GetWheel(chassis->kLeftWheel_)->MaxSpeed(),
      chassis->GetWheel(chassis->kLeftWheel_)->MinSpeed(), odom_pose,
      rotate_degree, *config_.rotate_motion_config_));

  if (p_rotate_motion_->GetState() == State::kError) {
    state_ = State::kError;
    stop_reason_ = kStopForException;
    SwitchStageAndClearMotion(kStop);
    ZERROR;
    return false;
  }
  return true;
}

bool TracePathMovement::SwitchToRetreatMotion(const Chassis::SPtr& chassis,
                                              const MapPoint& odom_pose) {
  SwitchStageAndClearMotion(kRetreat);
  p_retreat_motion_.reset(new RetreatMotion(
      chassis->GetTrackLength() / 2,
      chassis->GetWheel(chassis->kLeftWheel_)->MaxSpeed(),
      chassis->GetWheel(chassis->kLeftWheel_)->MinSpeed(), odom_pose,
      config_.retreat_distance_, *config_.retreat_motion_config_));

  if (p_retreat_motion_->GetState() == State::kError) {
    state_ = State::kError;
    stop_reason_ = kStopForException;
    SwitchStageAndClearMotion(kStop);
    ZERROR;
    return false;
  }
  return true;
}

bool TracePathMovement::SwitchToTracePathMotion(const Chassis::SPtr& chassis,
                                                const NavMap::SPtr& map,
                                                const MapPointPath& path) {
  SwitchStageAndClearMotion(kTracePath);
  p_trace_path_motion_.reset(
      new TracePathMotion(chassis->GetTrackLength() / 2,
                          chassis->GetWheel(chassis->kLeftWheel_)->MaxSpeed(),
                          chassis->GetWheel(chassis->kLeftWheel_)->MinSpeed(),
                          path, *config_.trace_path_motion_config_));

  if (p_trace_path_motion_->GetState() == State::kError) {
    state_ = State::kError;
    stop_reason_ = kStopForException;
    SwitchStageAndClearMotion(kStop);
    ZERROR;
    return false;
  }
  // p_trace_path_motion_->EnableDebugLog();
  return true;
}

PointCloud::Point TracePathMovement::GetChassisFrontNearestPointCloudObs(
    const Chassis::SPtr& chassis) {
  auto current_point_cloud =
      chassis->GetLidar(chassis->kLidar_)->GetPointCloudInChassisFrame();
  static auto half_degree = RadiansToDegrees(atan2(
      NavMap::GetResolution() * 1.5,
      chassis->GetRadius() - chassis->GetLidar(chassis->kLidar_)->GetTf().X()));
  double timestamp = 0;
  float min_distance = std::numeric_limits<float>::max();
  float min_distance_degree = 0;
  float intensity = 0;
  ReadLocker lock(current_point_cloud->GetLock());
  if (current_point_cloud->Empty()) {
    return PointCloud::Point(0, min_distance, min_distance_degree, intensity);
  }
  current_point_cloud->ProcessRangedPoints(
      -half_degree, half_degree, [&](const PointCloud::Point& point) {
        if (fabs(point.Y()) < chassis->GetRadius() * 1.1 &&
            point.Distance() < min_distance) {
          timestamp = point.TimeStamp();
          min_distance = point.Distance();
          min_distance_degree = point.Degree();
          intensity = point.Intensity();
        }
      });
  lock.Unlock();
  // if (min_distance < 1.0) {
  //   ZWARN << "Main lidar dis: " << FloatToString(min_distance, 2)
  //         << " degree: " << FloatToString(min_distance_degree, 1);
  // }
  return PointCloud::Point(timestamp, min_distance, min_distance_degree,
                           intensity);
}

bool TracePathMovement::HandleMainLidar(
    const Chassis::SPtr& chassis, const MapPoint& world_pose,
    const NavMap::SPtr& map, const PointCloud::Point& nearest_point) {
  if (fabs(nearest_point.Y()) <= chassis->GetRadius() &&
      nearest_point.X() <
          chassis->GetRadius() *
                  cos(asin(nearest_point.Y() / chassis->GetRadius())) +
              config_.trace_path_motion_config_->stop_for_lidar_obs_distance_ +
              config_.trace_path_motion_config_->stop_for_lidar_compensate_) {
    ZGWARN << "Main lidar trigger, point: " << nearest_point.DebugString();
    MarkerPoints marker_points;
    marker_points.emplace_back(MarkerPoint(
        MapPoint(nearest_point.X(), nearest_point.Y()), map->kWall_));
    obstacle_degree_ = nearest_point.Degree();

    if (steps_recorder_.AddPathPoint(StepPoint(world_pose, marker_points), true,
                                     true)) {
      MapCell pose_cell;
      if (map->GetFootStepLayer()->WorldToMap(world_pose, pose_cell)) {
        ZGINFO << pose_cell.DebugString() << world_pose.DebugString();
      }
    }

    return true;
  }
  return false;
}

bool TracePathMovement::HandleBumper(const Chassis::SPtr& chassis,
                                     const MapPoint& world_pose,
                                     const NavMap::SPtr& map) {
  std::vector<std::string> triggered_bumpers;
  if (chassis->GetBumperEvent(triggered_bumpers)) {
    ZGWARN << "Bumper trigger.";
    MarkerPoints marker_points;
    if (!chassis_has_center_bumper_) {
      // Saperate 3 situation.
      bool left_bumper = false;
      bool right_bumper = false;
      for (auto&& bumper : triggered_bumpers) {
        if (bumper == chassis->kLeftBumper_) {
          left_bumper = true;
        } else if (bumper == chassis->kRightBumper_) {
          right_bumper = true;
        }
      }
      if (left_bumper && right_bumper) {
        marker_points.emplace_back(
            MarkerPoint(MapPoint(chassis->GetBumper(chassis->kLeftBumper_)
                                     ->GetTracePathMovementMarkPoint()
                                     .X(),
                                 0),
                        map->kBumper_));
        obstacle_degree_ = 0;
      } else if (left_bumper) {
        marker_points.emplace_back(MarkerPoint(
            chassis->GetBumper(chassis->kLeftBumper_)->GetTracePathMovementMarkPoint(),
            map->kBumper_));
        obstacle_degree_ =
            chassis->GetBumper(chassis->kLeftBumper_)->GetTf().Degree();
      } else if (right_bumper) {
        marker_points.emplace_back(MarkerPoint(
            chassis->GetBumper(chassis->kRightBumper_)->GetTracePathMovementMarkPoint(),
            map->kBumper_));
        obstacle_degree_ =
            chassis->GetBumper(chassis->kRightBumper_)->GetTf().Degree();
      }

      if (steps_recorder_.AddPathPoint(StepPoint(world_pose, marker_points),
                                       true, true)) {
        MapCell pose_cell;
        if (map->GetFootStepLayer()->WorldToMap(world_pose, pose_cell)) {
          ZGINFO << pose_cell.DebugString() << world_pose.DebugString();
        }
      }
    } else {
      // Saperate 3 situation.
      bool left_bumper = false;
      bool center_bumper = false;
      bool right_bumper = false;
      for (auto&& bumper : triggered_bumpers) {
        if (bumper == chassis->kLeftBumper_) {
          left_bumper = true;
        } else if (bumper == chassis->kCenterBumper_) {
          center_bumper = true;
        } else if (bumper == chassis->kRightBumper_) {
          right_bumper = true;
        }
      }
      if (center_bumper) {
        marker_points.emplace_back(
            MarkerPoint(chassis->GetBumper(chassis->kCenterBumper_)
                            ->GetTracePathMovementMarkPoint(),
                        map->kBumper_));
        obstacle_degree_ = 0;
      } else if (left_bumper) {
        marker_points.emplace_back(MarkerPoint(
            chassis->GetBumper(chassis->kLeftBumper_)->GetTracePathMovementMarkPoint(),
            map->kBumper_));
        obstacle_degree_ =
            chassis->GetBumper(chassis->kLeftBumper_)->GetTf().Degree();
      } else if (right_bumper) {
        marker_points.emplace_back(MarkerPoint(
            chassis->GetBumper(chassis->kRightBumper_)->GetTracePathMovementMarkPoint(),
            map->kBumper_));
        obstacle_degree_ =
            chassis->GetBumper(chassis->kRightBumper_)->GetTf().Degree();
      }

      if (steps_recorder_.AddPathPoint(StepPoint(world_pose, marker_points),
                                       true, true)) {
        MapCell pose_cell;
        if (map->GetFootStepLayer()->WorldToMap(world_pose, pose_cell)) {
          ZGINFO << pose_cell.DebugString() << world_pose.DebugString();
        }
      }
    }
    return true;
  }
  return false;
}

bool TracePathMovement::HandleMapValue(const Chassis::SPtr& chassis,
                                       const MapPoint& world_pose,
                                       const NavMap::SPtr& map) {
  // Handle for room edge.
  bool reach_room_edge = false;
  if (enable_room_value_checking_ &&
      current_room_index_ != NavMap::kUnknown_) {
    auto room_map = map->GetRoomLayer();
    ReadLocker read_lock(room_map->GetLock());
    int map_x, map_y;
    auto value = room_map->GetDefaultValue();
    MapPoints check_points;
    auto stop_distance =
        config_.trace_path_motion_config_->stop_for_room_value_distance_;
    check_points.emplace_back(
        MapPoint(stop_distance * cos(DegreesToRadians(10)),
                 stop_distance * sin(DegreesToRadians(10))));
    check_points.emplace_back(
        MapPoint(stop_distance * cos(DegreesToRadians(-10)),
                 stop_distance * sin(DegreesToRadians(-10))));

    for (auto&& point : check_points) {
      double point_x_in_map, point_y_in_map;
      Transform::CoordinateTransformationBA(
          point.X(), point.Y(), world_pose.X(), world_pose.Y(),
          world_pose.Radian(), point_x_in_map, point_y_in_map);
      if (room_map->WorldToMap(point_x_in_map, point_y_in_map, map_x, map_y)) {
        room_map->GetValue(map_x, map_y, value);
        if (value != current_room_index_ && value != NavMap::kUnknown_) {
          ZGINFO << "Reach room edge, current index: " << current_room_index_
                 << ", front index: " << value;
          reach_room_edge = true;
          map_obstacle_degree_ = RadiansToDegrees(atan2(point.Y(), point.X()));
          break;
        }
      }
    }
  }

  if (reach_room_edge) {
    stop_reason_ = kStopForRoomEdge;
    return true;
  }

  // Handle for user block edge.
  bool reach_user_block_edge = false;
  {
    auto user_block_map = map->GetUserBlockLayer();
    ReadLocker read_lock(user_block_map->GetLock());
    int map_x, map_y;
    auto value = user_block_map->GetDefaultValue();
    MapPoints check_points;
    auto stop_distance =
        config_.trace_path_motion_config_->stop_for_user_block_distance_;
    check_points.emplace_back(
        MapPoint(stop_distance * cos(DegreesToRadians(10)),
                 stop_distance * sin(DegreesToRadians(10))));
    check_points.emplace_back(
        MapPoint(stop_distance * cos(DegreesToRadians(-10)),
                 stop_distance * sin(DegreesToRadians(-10))));

    for (auto&& point : check_points) {
      double point_x_in_map, point_y_in_map;
      Transform::CoordinateTransformationBA(
          point.X(), point.Y(), world_pose.X(), world_pose.Y(),
          world_pose.Radian(), point_x_in_map, point_y_in_map);
      if (user_block_map->WorldToMap(point_x_in_map, point_y_in_map, map_x,
                                     map_y)) {
        user_block_map->GetValue(map_x, map_y, value);
        if (value == NavMap::kVirtualWall_ ||
            value == NavMap::kStrictBlockArea_) {
          ZGINFO << "Reach virtual wall or strict block, front value: "
                 << value;
          reach_user_block_edge = true;
          map_obstacle_degree_ = RadiansToDegrees(atan2(point.Y(), point.X()));
          break;
        }
      }
    }
  }

  if (reach_user_block_edge) {
    stop_reason_ = kStopForForbidArea;
    return true;
  }

  // Handle for user select area.
  bool reach_user_select_area_edge = false;
  if (enable_user_select_area_checking_) {
    auto user_select_area_map = map->GetUserSelectAreaLayer();
    ReadLocker read_lock(user_select_area_map->GetLock());
    int map_x, map_y;
    auto value = user_select_area_map->GetDefaultValue();
    MapPoints check_points;
    auto stop_distance =
        config_.trace_path_motion_config_->stop_for_room_value_distance_;

    check_points.emplace_back(
        MapPoint(stop_distance * cos(DegreesToRadians(10)),
                 stop_distance * sin(DegreesToRadians(10))));
    check_points.emplace_back(
        MapPoint(stop_distance * cos(DegreesToRadians(-10)),
                 stop_distance * sin(DegreesToRadians(-10))));

    for (auto&& point : check_points) {
      double point_x_in_map, point_y_in_map;
      Transform::CoordinateTransformationBA(
          point.X(), point.Y(), world_pose.X(), world_pose.Y(),
          world_pose.Radian(), point_x_in_map, point_y_in_map);
      if (user_select_area_map->WorldToMap(point_x_in_map, point_y_in_map,
                                           map_x, map_y)) {
        user_select_area_map->GetValue(map_x, map_y, value);
        if (value != NavMap::kUserSelected_) {
          ZGINFO << "Reach user select area edge, front value: " << value;
          reach_user_select_area_edge = true;
          map_obstacle_degree_ = RadiansToDegrees(atan2(point.Y(), point.X()));
          break;
        }
      }
    }
  }

  if (reach_user_select_area_edge) {
    stop_reason_ = kStopForUserUnSelectArea;
    return true;
  }

  // Handle for footstep map edge.
  bool reach_cleaning_area_edge = false;
  {
    auto footstep_map = map->GetFootStepLayer();
    ReadLocker read_lock(footstep_map->GetLock());
    int map_x, map_y;
    MapPoints check_points;
    auto stop_distance = config_.trace_path_motion_config_
                             ->stop_for_cleaning_area_edge_distance_;
    check_points.emplace_back(
        MapPoint(stop_distance * cos(DegreesToRadians(10)),
                 stop_distance * sin(DegreesToRadians(10))));
    check_points.emplace_back(
        MapPoint(stop_distance * cos(DegreesToRadians(-10)),
                 stop_distance * sin(DegreesToRadians(-10))));

    footstep_map->WorldToMap(world_pose.X(), world_pose.Y(), map_x, map_y);
    auto footstep_map_max_clean_bound =
        map->GetMaxCleanBound(std::make_shared<MapCell>(map_x, map_y));
    for (auto&& point : check_points) {
      double point_x_in_map, point_y_in_map;
      Transform::CoordinateTransformationBA(
          point.X(), point.Y(), world_pose.X(), world_pose.Y(),
          world_pose.Radian(), point_x_in_map, point_y_in_map);
      if (footstep_map->WorldToMap(point_x_in_map, point_y_in_map, map_x,
                                     map_y)) {
        if (!footstep_map_max_clean_bound.Contain(MapCell(map_x, map_y))) {
          ZINFO << "Reach cleaning area edge.";
          ZGINFO << footstep_map_max_clean_bound.DebugString() << ", current "
                 << world_pose.DebugString() << ", edge at "
                 << MapCell(map_x, map_y).DebugString() << ", "
                 << MapPoint(point_x_in_map, point_y_in_map).DebugString();
          reach_cleaning_area_edge = true;
          map_obstacle_degree_ = RadiansToDegrees(atan2(point.Y(), point.X()));
          break;
        }
      }
    }
  }

  if (reach_cleaning_area_edge) {
    stop_reason_ = kStopForCleaningAreaEdge;
    return true;
  }

  return false;
}

uint32_t TracePathMovement::GetRestPathPointCount() const {
  ReadLocker lock(access_);
  if (stage_ == kTracePath && p_trace_path_motion_ != nullptr) {
    return p_trace_path_motion_->GetRestPathPointCount();
  }
  return MovementBase::GetRestPathPointCount();
}

MapPointPath TracePathMovement::GetRestPath() const {
  ReadLocker lock(access_);
  if (stage_ == kTracePath && p_trace_path_motion_ != nullptr) {
    return p_trace_path_motion_->GetRestPath();
  }
  return MovementBase::GetRestPath();
}

bool TracePathMovement::ExtendPath(const MapPointPath& path) {
  WriteLocker lock(access_);
  if (stage_ == kTracePath && p_trace_path_motion_ != nullptr) {
    return p_trace_path_motion_->ExtendPath(path);
  }
  return MovementBase::ExtendPath(path);
}

}  // namespace zima
