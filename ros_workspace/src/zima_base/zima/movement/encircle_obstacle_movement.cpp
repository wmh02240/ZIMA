/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/movement/encircle_obstacle_movement.h"

#include <algorithm>

#include "zima/common/util.h"

namespace zima {

EncircleObstacleMovement::Config::Config(const Chassis::SPtr& chassis,
                                         const float& map_resolution,
                                         const JsonSPtr& json)
    : left_point_cloud_select_range_(
          0, chassis->GetRadius(), chassis->GetRadius() / 2,
          chassis->GetRadius() + 2 * map_resolution),
      right_point_cloud_select_range_(
          0, chassis->GetRadius(),
          -1 * left_point_cloud_select_range_.GetMax().Y(),
          -1 * left_point_cloud_select_range_.GetMin().Y()) {
  auto wall_sensor_tf =
      chassis->EncircleObstacleOnLeft()
          ? chassis->GetWallSensor(chassis->kLeftWallSensor_)->GetTf()
          : chassis->GetWallSensor(chassis->kRightWallSensor_)->GetTf();
  if (json != nullptr) {
    // ZINFO << "Load config from json.";
    config_valid_ =
        ParseFromJson(chassis->GetTrackLength(), chassis->GetRadius(),
                      chassis->GetWheel(chassis->kLeftWheel_)->MaxSpeed(),
                      wall_sensor_tf, json);
  } else {
    JsonSPtr config;
    if (GlobalJsonConfig::Instance()->GetGlobalConfigObject(kConfigKey_,
                                                            config)) {
      // ZINFO << "Load config from global json config.";
      config_valid_ =
          ParseFromJson(chassis->GetTrackLength(), chassis->GetRadius(),
                        chassis->GetWheel(chassis->kLeftWheel_)->MaxSpeed(),
                        wall_sensor_tf, config);
    } else {
      config_valid_ = false;
    }
  }
}

bool EncircleObstacleMovement::Config::ParseFromJson(
    const float& track_length, const float& chassis_radius,
    const float& wheel_max_speed, const MapPoint& wall_sensor_tf,
    const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }

  // Override.
  if (!JsonHelper::GetFloat(*json, kRetreatDistanceKey_, retreat_distance_)) {
    retreat_distance_ = 0.005;
  }
  if (!JsonHelper::GetFloat(*json, kMoveForwardDistanceKey_,
                            move_forward_distance_)) {
    move_forward_distance_ = 0.05;
  }
  if (!JsonHelper::GetFloat(*json, kMoveForwardTimeKey_, move_forward_time_)) {
    move_forward_time_ = 0.5;
  }
  JsonSPtr _eo_json_config(new Json());
  if (JsonHelper::GetObject(*json, EncircleObstacleMotion::Config::kConfigKey_,
                            *_eo_json_config)) {
    // ZINFO << "Override config from json.";
    encircle_obstacle_motion_config_.reset(new EncircleObstacleMotion::Config(
        wheel_max_speed, chassis_radius, wall_sensor_tf, _eo_json_config));
    if (!encircle_obstacle_motion_config_->config_valid_) {
      ZERROR << "Config " << EncircleObstacleMotion::Config::kConfigKey_ << " in "
             << kConfigKey_ << " invalid.";
      return false;
    }
  }
  JsonSPtr _move_forward_json_config(new Json());
  if (JsonHelper::GetObject(*json, MoveForwardMotion::Config::kConfigKey_,
                            *_move_forward_json_config)) {
    // ZINFO << "Override config from json: " << _move_forward_json_config->dump();
    move_forward_motion_config_.reset(new MoveForwardMotion::Config(
        move_forward_distance_, move_forward_time_, wheel_max_speed,
        chassis_radius, _move_forward_json_config));
    if (!move_forward_motion_config_->config_valid_) {
      ZERROR << "Config " << MoveForwardMotion::Config::kConfigKey_ << " in "
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

  if (!JsonHelper::GetFloat(*json, kRotateLidarCalDegreeCompensateKey_,
                            rotate_lidar_cal_degree_compensate_)) {
    rotate_lidar_cal_degree_compensate_ = 8;
  }

  return true;
}

using State = MotionBase::State;

EncircleObstacleMovement::EncircleObstacleMovement(
    const Chassis::SPtr& chassis, const float& map_resolution,
    const DynamicMapPointBound& limit_bound, const MapPoint& start_world_pose,
    const MapPoint& start_odom_pose, const bool& on_left,
    const float& init_rotate_degree,
    const CharGridMap2D::DataType& current_room_index,
    const bool& turn_with_less_degree)
    : EncircleMovementBase(limit_bound, map_resolution),
      config_(chassis, map_resolution),
      stage_(kMoveForward),
      on_left_(on_left),
      start_world_pose_(start_world_pose),
      current_room_index_(current_room_index),
      stop_reason_(kStopForException),
      turn_with_less_degree_(turn_with_less_degree),
      last_process_point_cloud_seq_(0) {
  name_ = typeid(EncircleObstacleMovement).name();
  if (!FloatEqual(init_rotate_degree, 0)) {
    SwitchToRotateMotion(chassis, start_odom_pose, init_rotate_degree);
  }

  chassis_has_center_bumper_ =
      chassis->IsDeviceRegistered(chassis->kCenterBumper_);

  if (!config_.config_valid_) {
    ZERROR << "Config invalid.";
    stage_= kStop;
    state_ = State::kError;
    return;
  }

  ZGINFO << "Encircle obstacle for bound:" << limit_bound_.DebugString()
         << ", on left(" << on_left_ << "), turn with less degree("
         << turn_with_less_degree_ << ")";
}

EncircleObstacleMovement::~EncircleObstacleMovement() {}

void EncircleObstacleMovement::Run(const Chassis::SPtr& chassis,
                                   const MapPoint& world_pose,
                                   const MapPoint& odom_pose,
                                   const NavMap::SPtr& map) {
  WriteLocker movement_lock(access_);
  float left_wheel_speed = 0;
  float right_wheel_speed = 0;

  auto point_cloud_in_chassis_frame =
      chassis->GetLidar(chassis->kLidar_)->GetPointCloudInChassisFrame();
  ReadLocker lock(point_cloud_in_chassis_frame->GetLock());
  auto seq = point_cloud_in_chassis_frame->GetSeq();
  bool point_cloud_empty = point_cloud_in_chassis_frame->Empty();
  lock.Unlock();
  auto point_cloud_updated = last_process_point_cloud_seq_ != seq;
  if (point_cloud_updated) {
    last_process_point_cloud_seq_ = seq;
  }

  MarkerPoints marker_points;
  auto distance = chassis
                      ->GetWallSensor(on_left_ ? chassis->kLeftWallSensor_
                                               : chassis->kRightWallSensor_)
                      ->GetDistance();
  // ZWARN << "wall sensor:" << FloatToString(distance, 2);
  if (distance < config_.encircle_obstacle_motion_config_->max_valid_obstacle_distance_) {
    marker_points.emplace_back(
        MarkerPoint(chassis
                        ->GetWallSensor(on_left_ ? chassis->kLeftWallSensor_
                                                 : chassis->kRightWallSensor_)
                        ->GetMarkPoint(),
                    map->kWall_));
    // ZINFO << "Add marker point:" << marker_points.back().DebugString();
  }

  if (!point_cloud_empty) {
    auto distance =
        GetWallDistanceFromPointCloud(chassis, point_cloud_in_chassis_frame) -
        chassis->GetRadius();
    // ZWARN << "lidar distance:" << FloatToString(distance, 2);
    if (distance < config_.encircle_obstacle_motion_config_
                       ->max_valid_obstacle_distance_) {
      marker_points.emplace_back(
          MarkerPoint(chassis
                          ->GetWallSensor(on_left_ ? chassis->kLeftWallSensor_
                                                   : chassis->kRightWallSensor_)
                          ->GetMarkPoint(),
                      map->kWall_));
    }
  }

  if (steps_recorder_.AddPathPoint(StepPoint(world_pose, marker_points))) {
    MapCell pose_cell;
    if (map->GetFootStepLayer()->WorldToMap(world_pose, pose_cell)) {
      // ZINFO << pose_cell.DebugString() << world_pose.DebugString();
    }
  }

  bool finish_run = false;

  auto footstep_map = map->GetFootStepLayer();

  while (!finish_run) {
    finish_run = true;

    switch (stage_) {
      case kFinish:
      case kStop: {
        break;
      }
      case kMoveForward: {
        if (p_move_forward_motion_ == nullptr) {
          if (!SwitchToMoveForwardMotion(chassis, world_pose,
                                         config_.move_forward_distance_,
                                         config_.move_forward_time_)) {
            break;
          }
        }

        // Abnormal state checking.
        auto state = p_move_forward_motion_->GetState();
        // ZINFO << "Move forward state:" << state;
        if (state == State::kError || state == State::kTimeout ||
            state == State::kException) {
          state_ = state;
          stop_reason_ = kStopForException;
          SwitchStageAndClearMotion(kStop);
          break;
        }

        p_move_forward_motion_->CheckState(
            world_pose, odom_pose,
            chassis->GetWheel(chassis->kLeftWheel_)->CurrentSpeed(),
            chassis->GetWheel(chassis->kRightWheel_)->CurrentSpeed());
        state = p_move_forward_motion_->GetState();
        if (state == State::kReachTarget) {
          SwitchToEncircleObstacleMotion(chassis);
          finish_run = false;
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
          if (stop_reason_ == StopReason::kStopForCleaningAreaEdge) {
            SwitchToRetreatMotion(chassis, odom_pose);
            finish_run = false;
          }
          ZGERROR;
          state_ = State::kStop;
          SwitchStageAndClearMotion(kStop);
          break;
        }

        auto nearest_point = GetChassisFrontNearestPointCloudObs(chassis);
        if (HandleMainLidar(chassis, world_pose, map, nearest_point)) {
          auto rotate_degree = CalculateRotateDegree(odom_pose, chassis);
          SwitchToRotateMotion(chassis, odom_pose, rotate_degree);
          finish_run = false;
          break;
        }

        state_ = state;
        p_move_forward_motion_->SpeedControl(
            world_pose, odom_pose,
            chassis->GetWheel(chassis->kLeftWheel_)->CurrentSpeed(),
            chassis->GetWheel(chassis->kRightWheel_)->CurrentSpeed(),
            left_wheel_speed, right_wheel_speed);

        break;
      }
      case kTurnForObs: {
        if (p_rotate_motion_ == nullptr) {
          ZERROR;
          if (!SwitchToRotateMotion(chassis, odom_pose, 0)) {
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
          SwitchToMoveForwardMotion(chassis, odom_pose,
                                    config_.move_forward_distance_,
                                    config_.move_forward_time_);
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
      case kEncircleObs: {
        if (p_encircle_obstacle_motion_ == nullptr) {
          if (!SwitchToEncircleObstacleMotion(chassis)) {
            break;
          }
        }
        auto state = p_encircle_obstacle_motion_->GetState();
        // ZINFO << "EO state:" << state;
        if (state == State::kError || state == State::kTimeout ||
            state == State::kStop || state == State::kException) {
          state_ = state;
          stop_reason_ = kStopForException;
          SwitchStageAndClearMotion(kStop);
          break;
        }

        p_encircle_obstacle_motion_->CheckState(
            world_pose, odom_pose,
            chassis->GetWheel(chassis->kLeftWheel_)->CurrentSpeed(),
            chassis->GetWheel(chassis->kRightWheel_)->CurrentSpeed());
        state = p_encircle_obstacle_motion_->GetState();

        auto path_situation = steps_recorder_.CheckPath();
        // ZGERROR << "path_situation: " << path_situation;
        if (path_situation == StepsRecorder::SituationCode::NORMAL_CLOSE_LOOP) {
          SwitchStageAndClearMotion(kFinish);
          stop_reason_ = StopReason::kStopForLoopDetected;
          state_ = State::kFinish;
          break;
        } else if (world_pose.Distance(start_world_pose_) >
                       map->GetResolution() &&
                   OutOfBound(world_pose, map, marker_points)) {
          ZGINFO << "Out of bound: " << world_pose;
          SwitchStageAndClearMotion(kFinish);
          state_ = State::kFinish;
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
        if (HandleMainLidar(chassis, world_pose, map, nearest_point)) {
          auto rotate_degree = CalculateRotateDegree(odom_pose, chassis);
          SwitchToRotateMotion(chassis, odom_pose, rotate_degree);
          finish_run = false;
          break;
        }

        // if (world_pose.Distance(start_world_pose_) > chassis->GetRadius()) {
        if (world_pose.Distance(start_world_pose_) > map->GetResolution() * 5) {
          step_on_past_path_.store(false);
          auto sensor_map = map->GetSensorLayer();
          ReadLocker sensor_lock(sensor_map->GetLock());
          bool already_mark_wall = false;
          for (auto&& wall_point : marker_points) {
            MapCell wall_cell;
            double x, y;
            Transform::CoordinateTransformationBA(
                wall_point.Point().X(), wall_point.Point().Y(), world_pose.X(),
                world_pose.Y(), world_pose.Radian(), x, y);
            MapPoint wall_point_in_world(x, y);

            if (sensor_map->WorldToMap(wall_point_in_world, wall_cell)) {
              CharGridMap2D::DataType value;
              // ZINFO << "Check wall for " << wall_cell;
              if (sensor_map->GetValue(wall_cell.X(), wall_cell.Y(), value) &&
                  value != sensor_map->GetDefaultValue()) {
                already_mark_wall = true;
                break;
              }
            }
          }

          if (already_mark_wall) {
            ReadLocker footstep_lock(footstep_map->GetLock());
            MapCell pose_cell;

            if (footstep_map->WorldToMap(world_pose, pose_cell)) {
              CharGridMap2D::DataType value;
              // ZINFO << "Check pose for " << pose_cell;
              if (footstep_map->GetValue(pose_cell.X(), pose_cell.Y(),
                                          value) &&
                  value == NavMap::kFootStep_) {
                ZGINFO << "Step on past path.";
                step_on_past_path_.store(true);
              }
            }
          }
        }

        state_ = state;
        // Calculate for distance in y.
        auto distance =
            chassis
                ->GetWallSensor(on_left_ ? chassis->kLeftWallSensor_
                                         : chassis->kRightWallSensor_)
                ->GetDistance();
        // ZWARN << "distance: " << FloatToString(distance, 2);
        const float kMaxValidDistance_ = 1.0;
        if (distance > kMaxValidDistance_) {
          p_encircle_obstacle_motion_->SetCurrentObsDistance(odom_pose,
                                                             distance);
        } else {
          auto y_distance = fabs(
              (distance + chassis->GetRadius()) *
              sin(chassis
                      ->GetWallSensor(on_left_ ? chassis->kLeftWallSensor_
                                               : chassis->kRightWallSensor_)
                      ->GetTf()
                      .Radian()));

          // ZINFO << "Sensor: "
          //       << chassis->GetWallSensor(on_left_ ?
          //       chassis->kLeftWallSensor_ : chassis->kRightWallSensor_)
          //              ->GetDistance()
          //       << ", y: " << y_distance - chassis->GetRadius();
          p_encircle_obstacle_motion_->SetCurrentObsDistance(
              odom_pose, y_distance - chassis->GetRadius());
        }

        if (point_cloud_updated) {
          p_encircle_obstacle_motion_->UpdateDynamicPath(GetPathFromPointCloud(
              chassis, odom_pose, point_cloud_in_chassis_frame,
              config_.encircle_obstacle_motion_config_
                       ->target_obstacle_distance_));
        }

        if (NearBound(world_pose)) {
          p_encircle_obstacle_motion_->SlowdownTo(
              config_.encircle_obstacle_motion_config_
                       ->target_speed_ / 2,
              config_.encircle_obstacle_motion_config_
                       ->speed_up_step_);
        }
        p_encircle_obstacle_motion_->SpeedControl(
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
          auto rotate_degree = CalculateRotateDegree(odom_pose, chassis);
          SwitchToRotateMotion(chassis, odom_pose, rotate_degree);
          finish_run = false;
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
        ZERROR << "Should never run here." << state_;
      }
    }
  }

  // if (FloatEqual(left_wheel_speed, 0) && FloatEqual(right_wheel_speed, 0)) {
  //   ZWARN << "Stopping.";
  // }

  chassis->GetWheel(chassis->kLeftWheel_)->SetTargetSpeed(left_wheel_speed);
  chassis->GetWheel(chassis->kRightWheel_)->SetTargetSpeed(right_wheel_speed);
}

float EncircleObstacleMovement::GetMapObstacleDegree() const {
  ReadLocker lock(access_);
  return map_obstacle_degree_;
}

EncircleObstacleMovement::StopReason EncircleObstacleMovement::GetStopReason()
    const {
  ReadLocker lock(access_);
  return stop_reason_;
}

void EncircleObstacleMovement::SwitchStageAndClearMotion(const Stage& stage) {
  // ZINFO << "Switch to stage: " << stage;
  stage_ = stage;
  p_encircle_obstacle_motion_.reset();
  p_move_forward_motion_.reset();
  p_retreat_motion_.reset();
  p_rotate_motion_.reset();
}

bool EncircleObstacleMovement::SwitchToRotateMotion(
    const Chassis::SPtr& chassis, const MapPoint& odom_pose,
    const float& rotate_degree) {
  SwitchStageAndClearMotion(kTurnForObs);
  p_rotate_motion_.reset(new RotateMotion(
      chassis->GetTrackLength() / 2,
      chassis->GetWheel(chassis->kLeftWheel_)->MaxSpeed(),
      chassis->GetWheel(chassis->kLeftWheel_)->MinSpeed(), odom_pose,
      rotate_degree, *config_.rotate_motion_config_, false, false));

  if (p_rotate_motion_->GetState() == State::kError) {
    state_ = State::kError;
    SwitchStageAndClearMotion(kStop);
    ZERROR;
    return false;
  }
  return true;
}

bool EncircleObstacleMovement::SwitchToRetreatMotion(
    const Chassis::SPtr& chassis, const MapPoint& odom_pose) {
  SwitchStageAndClearMotion(kRetreat);
  p_retreat_motion_.reset(
      new RetreatMotion(chassis->GetTrackLength() / 2,
                        chassis->GetWheel(chassis->kLeftWheel_)->MaxSpeed(),
                        chassis->GetWheel(chassis->kLeftWheel_)->MinSpeed(),
                        odom_pose, config_.retreat_distance_,
                        *config_.retreat_motion_config_, false, false));

  if (p_retreat_motion_->GetState() == State::kError) {
    state_ = State::kError;
    SwitchStageAndClearMotion(kStop);
    ZERROR;
    return false;
  }
  return true;
}

bool EncircleObstacleMovement::SwitchToEncircleObstacleMotion(
    const Chassis::SPtr& chassis) {
  SwitchStageAndClearMotion(kEncircleObs);
  p_encircle_obstacle_motion_.reset(new EncircleObstacleMotion(
      chassis->GetTrackLength() / 2,
      chassis->GetWheel(chassis->kLeftWheel_)->MaxSpeed(),
      chassis->GetWheel(chassis->kLeftWheel_)->MinSpeed(), on_left_,
      *config_.encircle_obstacle_motion_config_));

  if (p_encircle_obstacle_motion_->GetState() == State::kError) {
    state_ = State::kError;
    SwitchStageAndClearMotion(kStop);
    ZERROR;
    return false;
  }
  return true;
}

bool EncircleObstacleMovement::SwitchToMoveForwardMotion(
    const Chassis::SPtr& chassis, const MapPoint& world_pose,
    const float& move_distance, const float& move_time) {
  SwitchStageAndClearMotion(kMoveForward);
  p_move_forward_motion_.reset(new MoveForwardMotion(
      chassis->GetTrackLength() / 2,
      chassis->GetWheel(chassis->kLeftWheel_)->MaxSpeed(),
      chassis->GetWheel(chassis->kLeftWheel_)->MinSpeed(),
      config_.move_forward_distance_, config_.move_forward_time_,
      world_pose, *config_.move_forward_motion_config_, false));

  // ZINFO;
  if (p_move_forward_motion_->GetState() == State::kError) {
    state_ = State::kError;
    SwitchStageAndClearMotion(kStop);
    ZERROR;
    return false;
  }
  // ZINFO;
  return true;
}

MapPointPath EncircleObstacleMovement::GetPathFromPointCloud(
    const Chassis::SPtr& chassis, const MapPoint& odom_pose,
    const PointCloud::SPtr& point_cloud_in_chassis_frame,
    const float& distance) {
  MapPointPath path;
  ReadLocker lock(point_cloud_in_chassis_frame->GetLock());
  if (on_left_) {
    const float range_start = 90;
    const float range_end = 45;
    point_cloud_in_chassis_frame->ProcessRangedPoints(
        range_start, range_end, [&](const PointCloud::Point& point) {
          // ZINFO << point.DebugString();
          auto check_point = point.ToMapPoint();
          if (config_.left_point_cloud_select_range_.Contain(check_point)) {
            path.emplace_back(check_point +
                              MapPoint(config_.encircle_obstacle_motion_config_
                                               ->reach_temp_target_distance_ /
                                           2,
                                       -(chassis->GetRadius() + distance)));
          }
        });
    // ZINFO << path.size();
  } else {
    const float range_start = -90;
    const float range_end = -45;
    point_cloud_in_chassis_frame->ProcessRangedPoints(
        range_start, range_end, [&](const PointCloud::Point& point) {
          // ZINFO << point.DebugString();
          auto check_point = point.ToMapPoint();
          if (config_.right_point_cloud_select_range_.Contain(check_point)) {
            path.emplace_back(check_point +
                              MapPoint(config_.encircle_obstacle_motion_config_
                                               ->reach_temp_target_distance_ /
                                           2,
                                       chassis->GetRadius() + distance));
          }
        });
    // ZINFO << path.size();
  }
  lock.Unlock();

  // ZINFO << odom_pose.DebugString();
  for_each(path.begin(), path.end(), [&](MapPoint& path_point) -> void {
    double x, y;
    Transform::CoordinateTransformationBA(path_point.X(), path_point.Y(),
                                          odom_pose.X(), odom_pose.Y(),
                                          odom_pose.Radian(), x, y);
    path_point.SetX(x);
    path_point.SetY(y);
    // ZINFO << path_point.DebugString();
  });

  return path;
}

float EncircleObstacleMovement::GetWallDistanceFromPointCloud(
    const Chassis::SPtr& chassis,
    const PointCloud::SPtr& point_cloud_in_chassis_frame) {
  ReadLocker lock(point_cloud_in_chassis_frame->GetLock());

  float distance = std::numeric_limits<float>::max();
  if (on_left_) {
    const float range_start =
        chassis->GetWallSensor(chassis->kLeftWallSensor_)->GetTf().Degree() + 5;
    const float range_end = range_start - 20;

    point_cloud_in_chassis_frame->ProcessRangedPoints(
        range_start, range_end, [&](const PointCloud::Point& point) {
          // ZINFO << point.DebugString();
          distance = std::min(distance, point.Distance());
        });
  } else {
    const float range_start =
        chassis->GetWallSensor(chassis->kRightWallSensor_)->GetTf().Degree() -
        5;
    const float range_end = range_start + 20;
    point_cloud_in_chassis_frame->ProcessRangedPoints(
        range_start, range_end, [&](const PointCloud::Point& point) {
          // ZINFO << point.DebugString();
          distance = std::min(distance, point.Distance());
        });
    // ZINFO << path.size();
  }
  lock.Unlock();

  return distance;
}

bool EncircleObstacleMovement::HandleMainLidar(
    const Chassis::SPtr& chassis, const MapPoint& world_pose,
    const NavMap::SPtr& map, const PointCloud::Point& nearest_point) {
  if (nearest_point.Distance() < chassis->GetRadius() +
                                     config_.encircle_obstacle_motion_config_
                                         ->stop_for_lidar_obs_distance_ +
                                     config_.encircle_obstacle_motion_config_
                                         ->stop_for_lidar_compensate_) {
    ZGWARN << "Main lidar trigger, dis: "
           << FloatToString(nearest_point.Distance(), 2)
           << " degree: " << FloatToString(nearest_point.Degree(), 1);
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

bool EncircleObstacleMovement::HandleBumper(const Chassis::SPtr& chassis,
                                            const MapPoint& world_pose,
                                            const NavMap::SPtr& map) {
  std::vector<std::string> triggered_bumpers;
  if (chassis->GetBumperEvent(triggered_bumpers)) {
    // ZWARN << "Bumper trigger.";
    MarkerPoints marker_points;
    if (!chassis_has_center_bumper_) {
      // Saperate 3 situation.
      bool left_bumper_triggered = false;
      bool right_bumper_triggered = false;
      for (auto&& bumper : triggered_bumpers) {
        if (bumper == chassis->kLeftBumper_) {
          left_bumper_triggered = true;
        } else if (bumper == chassis->kRightBumper_) {
          right_bumper_triggered = true;
        }
      }
      if (left_bumper_triggered && right_bumper_triggered) {
        marker_points.emplace_back(
            MarkerPoint(MapPoint(chassis->GetBumper(chassis->kLeftBumper_)
                                     ->GetTracePathMovementMarkPoint()
                                     .X(),
                                 0),
                        map->kBumper_));
      } else if (left_bumper_triggered) {
        marker_points.emplace_back(
            MarkerPoint(chassis->GetBumper(chassis->kLeftBumper_)
                            ->GetEncircleObstacleMovementMarkPoint(on_left_),
                        map->kBumper_));
      } else if (right_bumper_triggered) {
        marker_points.emplace_back(
            MarkerPoint(chassis->GetBumper(chassis->kRightBumper_)
                            ->GetEncircleObstacleMovementMarkPoint(on_left_),
                        map->kBumper_));
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
      bool left_bumper_triggered = false;
      bool center_bumper_triggered = false;
      bool right_bumper_triggered = false;
      for (auto&& bumper : triggered_bumpers) {
        if (bumper == chassis->kLeftBumper_) {
          left_bumper_triggered = true;
        } else if (bumper == chassis->kCenterBumper_) {
          center_bumper_triggered = true;
        } else if (bumper == chassis->kRightBumper_) {
          right_bumper_triggered = true;
        }
      }
      if (on_left_) {
        if (right_bumper_triggered) {
          marker_points.emplace_back(
              MarkerPoint(chassis->GetBumper(chassis->kRightBumper_)
                              ->GetEncircleObstacleMovementMarkPoint(on_left_),
                          map->kBumper_));
          obstacle_degree_ =
              chassis->GetBumper(chassis->kRightBumper_)->GetTf().Degree();
        } else if (center_bumper_triggered) {
          marker_points.emplace_back(
              MarkerPoint(chassis->GetBumper(chassis->kCenterBumper_)
                              ->GetEncircleObstacleMovementMarkPoint(on_left_),
                          map->kBumper_));
          obstacle_degree_ =
              chassis->GetBumper(chassis->kCenterBumper_)->GetTf().Degree();
        } else if (left_bumper_triggered) {
          marker_points.emplace_back(
              MarkerPoint(chassis->GetBumper(chassis->kLeftBumper_)
                              ->GetEncircleObstacleMovementMarkPoint(on_left_),
                          map->kBumper_));
          obstacle_degree_ =
              chassis->GetBumper(chassis->kLeftBumper_)->GetTf().Degree();
        }
      } else {
        if (left_bumper_triggered) {
          marker_points.emplace_back(
              MarkerPoint(chassis->GetBumper(chassis->kLeftBumper_)
                              ->GetEncircleObstacleMovementMarkPoint(on_left_),
                          map->kBumper_));
          obstacle_degree_ =
              chassis->GetBumper(chassis->kLeftBumper_)->GetTf().Degree();
        } else if (center_bumper_triggered) {
          marker_points.emplace_back(
              MarkerPoint(chassis->GetBumper(chassis->kCenterBumper_)
                              ->GetEncircleObstacleMovementMarkPoint(on_left_),
                          map->kBumper_));
          obstacle_degree_ =
              chassis->GetBumper(chassis->kCenterBumper_)->GetTf().Degree();
        } else if (right_bumper_triggered) {
          marker_points.emplace_back(
              MarkerPoint(chassis->GetBumper(chassis->kRightBumper_)
                              ->GetEncircleObstacleMovementMarkPoint(on_left_),
                          map->kBumper_));
          obstacle_degree_ =
              chassis->GetBumper(chassis->kRightBumper_)->GetTf().Degree();
        }
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

bool EncircleObstacleMovement::HandleMapValue(const Chassis::SPtr& chassis,
                                              const MapPoint& world_pose,
                                              const NavMap::SPtr& map) {
  // Handle for room edge.
  bool reach_room_edge = false;
  // ZINFO << "Current index: " << current_room_index_;
  if (current_room_index_ != NavMap::kUnknown_) {
    auto room_map = map->GetRoomLayer();
    ReadLocker read_lock(room_map->GetLock());
    int map_x, map_y;
    auto value = room_map->GetDefaultValue();
    MapPoints check_points;
    check_points.emplace_back(MapPoint(config_.encircle_obstacle_motion_config_
                                               ->stop_for_room_value_distance_ *
                                           cos(DegreesToRadians(0)),
                                       config_.encircle_obstacle_motion_config_
                                               ->stop_for_room_value_distance_ *
                                           sin(DegreesToRadians(0))));

    for (auto&& point : check_points) {
      double point_x_in_map, point_y_in_map;
      Transform::CoordinateTransformationBA(
          point.X(), point.Y(), world_pose.X(), world_pose.Y(),
          world_pose.Radian(), point_x_in_map, point_y_in_map);
      if (room_map->WorldToMap(point_x_in_map, point_y_in_map, map_x, map_y)) {
        room_map->GetValue(map_x, map_y, value);
        // ZINFO << "Current index: " << current_room_index_
        //       << ", front index: " << value;
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
    check_points.emplace_back(MapPoint(config_.encircle_obstacle_motion_config_
                                               ->stop_for_user_block_distance_ *
                                           cos(DegreesToRadians(0)),
                                       config_.encircle_obstacle_motion_config_
                                               ->stop_for_user_block_distance_ *
                                           sin(DegreesToRadians(0))));

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
    ZERROR;
    stop_reason_ = kStopForForbidArea;
    return true;
  }

  // Handle for user select area.
  bool reach_user_select_area_edge = false;
  {
    auto user_select_area_map = map->GetUserSelectAreaLayer();
    ReadLocker read_lock(user_select_area_map->GetLock());
    int map_x, map_y;
    auto value = user_select_area_map->GetDefaultValue();
    MapPoints check_points;
    check_points.emplace_back(MapPoint(config_.encircle_obstacle_motion_config_
                                               ->stop_for_room_value_distance_ *
                                           cos(DegreesToRadians(0)),
                                       config_.encircle_obstacle_motion_config_
                                               ->stop_for_room_value_distance_ *
                                           sin(DegreesToRadians(0))));

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
    auto stop_distance = config_.encircle_obstacle_motion_config_
                             ->stop_for_cleaning_area_edge_distance_;
    check_points.emplace_back(
        MapPoint(stop_distance * cos(DegreesToRadians(0)),
                 stop_distance * sin(DegreesToRadians(0))));

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
          ZGINFO << "Reach cleaning area edge.";
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

PointCloud::Point EncircleObstacleMovement::GetChassisFrontNearestPointCloudObs(
    const Chassis::SPtr& chassis) {
  auto current_point_cloud =
      chassis->GetLidar(chassis->kLidar_)->GetPointCloudInChassisFrame();
  // static auto half_degree = RadiansToDegrees(atan2(
  //     NavMap::GetResolution() * 1.5,
  //     chassis->GetRadius() - chassis->GetLidar(chassis->kLidar_)->GetTf().X()));
  static auto half_degree = 80;
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
  // if (min_distance < 0.5) {
  //   ZWARN << "Main lidar dis: " << FloatToString(min_distance, 2)
  //         << " degree: " << FloatToString(min_distance_degree, 1);
  // }
  return PointCloud::Point(timestamp, min_distance, min_distance_degree,
                           intensity);
}

float EncircleObstacleMovement::CalculateRotateDegree(
    const MapPoint& odom_pose, const Chassis::SPtr& chassis) {
  float rotate_degree = 0;
  float bumper_rotate_degree = BumperEventRotateDegree(chassis);
  if (bumper_rotate_degree > 0) {
    bumper_rotate_degree = Maximum(bumper_rotate_degree, 5.0f);
  } else if (bumper_rotate_degree < 0) {
    bumper_rotate_degree = Minimum(bumper_rotate_degree, -5.0f);
  }
  chassis->ClearBumperEvent();
  if (FloatEqual(bumper_rotate_degree, 0)) {
    rotate_degree = obstacle_degree_ + (on_left_ ? -80 : 80);
    rotate_degree = LidarOptimizedRotateDegree(chassis, rotate_degree);
    if (on_left_) {
      rotate_degree = Clip(rotate_degree, -70.f, rotate_degree);
    } else {
      rotate_degree = Clip(rotate_degree, rotate_degree, 70.f);
    }
  } else {
    rotate_degree = LidarOptimizedRotateDegree(chassis, bumper_rotate_degree);
  }

  if (turn_with_less_degree_) {
    if (on_left_) {
      rotate_degree = Clip(rotate_degree, -20.f, rotate_degree);
    } else {
      rotate_degree = Clip(rotate_degree, rotate_degree, 20.f);
    }
  }
  // ZINFO << "Rotate degree:" << rotate_degree;
  return rotate_degree;
}

float EncircleObstacleMovement::BumperEventRotateDegree(
    const Chassis::SPtr& chassis) {
  float rotate_degree = 0;
  std::vector<std::string> triggered_bumpers;

  bool left_bumper_triggered = false;
  bool right_bumper_triggered = false;
  auto left_bumper = chassis->GetBumper(chassis->kLeftBumper_);
  auto right_bumper = chassis->GetBumper(chassis->kRightBumper_);
  if (!chassis_has_center_bumper_) {
    if (chassis->GetBumperEvent(triggered_bumpers)) {
      for (auto&& bumper : triggered_bumpers) {
        if (bumper == chassis->kLeftBumper_) {
          left_bumper_triggered = true;
          // ZINFO << "Left bumper";
        } else if (bumper == chassis->kRightBumper_) {
          right_bumper_triggered = true;
          // ZINFO << "Right bumper";
        }
      }
    }
    if (on_left_) {
      if (right_bumper_triggered && !left_bumper_triggered) {
        rotate_degree = left_bumper->GetCoverRangeMinDegree() - 90;
      } else if (left_bumper_triggered && right_bumper_triggered) {
        rotate_degree = right_bumper->GetCoverRangeMaxDegree() - 90;
      } else if (left_bumper_triggered) {
        rotate_degree = left_bumper->GetCoverRangeMaxDegree() - 90;
        rotate_degree = Minimum(rotate_degree, -1.0f);
      }
    } else {
      if (left_bumper_triggered && !right_bumper_triggered) {
        rotate_degree = right_bumper->GetCoverRangeMaxDegree() - (-90);
      } else if (left_bumper_triggered && right_bumper_triggered) {
        rotate_degree = left_bumper->GetCoverRangeMinDegree() - (-90);
      } else if (right_bumper_triggered) {
        rotate_degree = right_bumper->GetCoverRangeMinDegree() - (-90);
        rotate_degree = Maximum(rotate_degree, 1.0f);
      }
    }
  } else {
    bool center_bumper_triggered = false;
    auto center_bumper = chassis->GetBumper(chassis->kCenterBumper_);
    if (chassis->GetBumperEvent(triggered_bumpers)) {
      for (auto&& bumper : triggered_bumpers) {
        if (bumper == chassis->kLeftBumper_) {
          left_bumper_triggered = true;
          // ZINFO << "Left bumper";
        } else if (bumper == chassis->kRightBumper_) {
          right_bumper_triggered = true;
          // ZINFO << "Right bumper";
        } else if (bumper == chassis->kCenterBumper_) {
          center_bumper_triggered = true;
          // ZINFO << "Center bumper";
        }
      }
    }
    if (on_left_) {
      if (right_bumper_triggered) {
        rotate_degree = right_bumper->GetCoverRangeMaxDegree() - 90;
      } else if (center_bumper_triggered) {
        rotate_degree = center_bumper->GetCoverRangeMaxDegree() - 90;
      } else if (left_bumper_triggered) {
        rotate_degree = left_bumper->GetCoverRangeMaxDegree() - 90;
        rotate_degree = Minimum(rotate_degree, -1.0f);
      }
    } else {
      if (left_bumper_triggered) {
        rotate_degree = left_bumper->GetCoverRangeMinDegree() - (-90);
      } else if (center_bumper_triggered) {
        rotate_degree = center_bumper->GetCoverRangeMinDegree() - (-90);
      } else if (right_bumper_triggered) {
        rotate_degree = right_bumper->GetCoverRangeMinDegree() - (-90);
        rotate_degree = Maximum(rotate_degree, 1.0f);
      }
    }
  }

  // ZINFO << "Bumper event rotate degree:" << rotate_degree;
  return rotate_degree;
}

float EncircleObstacleMovement::LidarOptimizedRotateDegree(
    const Chassis::SPtr& chassis, const float& min_degree) {
  float end_degree = 90;
  if (fabs(min_degree) > fabs(end_degree)) {
    // No need to optmize.
    return min_degree;
  }
  auto current_point_cloud =
      chassis->GetLidar(chassis->kLidar_)->GetPointCloudInChassisFrame();
  ReadLocker lock(current_point_cloud->GetLock());
  if (current_point_cloud->Empty()) {
    ZWARN << "Point cloud empty.";
    return min_degree;
  }

  const float kSimulateMoveForwardDistance =
      config_.encircle_obstacle_motion_config_->stop_for_lidar_obs_distance_ +
      0.03;
  float rotate_degree;
  bool degree_found = false;

  const float kStartDegree = min_degree;
  float degree_step = 3;
  if (on_left_) {
    end_degree *= -1;
    degree_step *= -1;
  }
  rotate_degree = kStartDegree;
  for (; on_left_ ? (rotate_degree > end_degree) : (rotate_degree < end_degree);
       rotate_degree += degree_step) {
    // ZINFO << "Check for " << FloatToString(rotate_degree, 1);
    MapPoint simulate_move_forward_pose(
        kSimulateMoveForwardDistance * cos(DegreesToRadians(rotate_degree)),
        kSimulateMoveForwardDistance * sin(DegreesToRadians(rotate_degree)));
    bool lidar_obs_found = false;
    current_point_cloud->ProcessRangedPoints(
        -180, 180, [&](const PointCloud::Point& point) {
          if (!lidar_obs_found) {
            if (simulate_move_forward_pose.Distance(point.ToMapPoint()) <
                chassis->GetRadius()) {
              lidar_obs_found = true;
            }
          }
        });
    degree_found = !lidar_obs_found;
    if (degree_found) {
      break;
    }
  }
  if (degree_found) {
    rotate_degree +=
        config_.rotate_lidar_cal_degree_compensate_ * (on_left_ ? -1 : 1);

    // ZINFO << "Optimize degree to " << FloatToString(rotate_degree, 1);
    return rotate_degree;
  } else {
    return min_degree;
  }
}

}  // namespace zima
