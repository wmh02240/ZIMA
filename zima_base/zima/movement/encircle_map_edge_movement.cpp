/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/movement/encircle_map_edge_movement.h"

#include <algorithm>

#include "zima/common/util.h"

namespace zima {

EncircleMapEdgeMovement::Config::Config(const Chassis::SPtr& chassis,
                                        const float& map_resolution,
                                        const JsonSPtr& json) {
  // Override if json config is provided.
  if (json != nullptr) {
    // ZINFO << "Load config from json.";
    config_valid_ =
        ParseFromJson(chassis->GetTrackLength(), chassis->GetRadius(),
                      chassis->GetWheel(chassis->kLeftWheel_)->MaxSpeed(),
                      map_resolution, json);
  } else {
    JsonSPtr config;
    if (GlobalJsonConfig::Instance()->GetGlobalConfigObject(kConfigKey_,
                                                            config)) {
      // ZINFO << "Load config from global json config.";
      config_valid_ =
          ParseFromJson(chassis->GetTrackLength(), chassis->GetRadius(),
                        chassis->GetWheel(chassis->kLeftWheel_)->MaxSpeed(),
                        map_resolution, config);
    } else {
      config_valid_ = false;
    }
  }
}

bool EncircleMapEdgeMovement::Config::ParseFromJson(
    const float& track_length, const float& chassis_radius,
    const float& wheel_max_speed, const float& map_resolution,
    const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }

  // Override.
  if (!JsonHelper::GetFloat(*json, kRetreatDistanceKey_, retreat_distance_)) {
    retreat_distance_ = 0.005;
  }
  if (!JsonHelper::GetFloat(*json, kMoveForwardToEscepeCliffDistanceKey_,
                            move_forward_to_escape_cliff_distance_)) {
    move_forward_to_escape_cliff_distance_ = 0.03;
  }
  if (!JsonHelper::GetFloat(*json, kMoveForwardToEscepeCliffTimeKey_,
                            move_forward_to_escape_cliff_time_)) {
    move_forward_to_escape_cliff_time_ = 1;
  }

  JsonSPtr _em_json_config(new Json());
  if (JsonHelper::GetObject(*json, EncircleMapEdgeMotion::Config::kConfigKey_,
                            *_em_json_config)) {
    // ZINFO << "Override config from json.";
    encircle_map_edge_motion_config_.reset(new EncircleMapEdgeMotion::Config(
        wheel_max_speed, map_resolution, _em_json_config));
    if (!encircle_map_edge_motion_config_->config_valid_) {
      ZERROR << "Config " << EncircleMapEdgeMotion::Config::kConfigKey_
             << " in " << kConfigKey_ << " invalid.";
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
  JsonSPtr _move_forward_json_config(new Json());
  if (JsonHelper::GetObject(*json, MoveForwardMotion::Config::kConfigKey_,
                            *_move_forward_json_config)) {
    // ZINFO << "Override config from json: " <<
    // _move_forward_json_config->dump();
    move_forward_motion_config_.reset(new MoveForwardMotion::Config(
        move_forward_to_escape_cliff_distance_,
        move_forward_to_escape_cliff_time_, wheel_max_speed, chassis_radius,
        _move_forward_json_config));
    if (!move_forward_motion_config_->config_valid_) {
      ZERROR << "Config " << MoveForwardMotion::Config::kConfigKey_ << " in "
             << kConfigKey_ << " invalid.";
      return false;
    }
  }

  return true;
}

using State = MotionBase::State;

EncircleMapEdgeMovement::EncircleMapEdgeMovement(
    const Chassis::SPtr& chassis, const float& map_resolution,
    const DynamicMapPointBound& limit_bound,
    const CharGridMap2D::DataType& current_room_index,
    const MapPoint& start_world_pose, const MapPoint& start_odom_pose,
    const bool& on_left, const float& init_rotate_degree)
    : EncircleMovementBase(limit_bound, map_resolution),
      config_(chassis, map_resolution),
      stage_(kEncircleMapEdge),
      on_left_(on_left),
      start_world_pose_(start_world_pose),
      current_room_index_(current_room_index) {
  name_ = typeid(EncircleMapEdgeMovement).name();
  if (!FloatEqual(init_rotate_degree, 0)) {
    SwitchToRotateMotion(chassis, start_odom_pose, init_rotate_degree);
  }

  chassis_has_center_bumper_ =
      chassis->IsDeviceRegistered(chassis->kCenterBumper_);

  ZINFO << "Encircle room edge for bound:" << limit_bound_.DebugString()
        << ", on left(" << on_left_ << ")";
}

void EncircleMapEdgeMovement::Run(const Chassis::SPtr& chassis,
                                  const MapPoint& world_pose,
                                  const MapPoint& odom_pose,
                                  const NavMap::SPtr& map) {
  WriteLocker lock(access_);
  float left_wheel_speed = 0;
  float right_wheel_speed = 0;

  MarkerPoints marker_points;

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
      case kTurnForMapEdge:
      case kTrunForCliff: {
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
          SwitchStageAndClearMotion(kStop);
          break;
        }

        p_rotate_motion_->CheckState(
            world_pose, odom_pose,
            chassis->GetWheel(chassis->kLeftWheel_)->CurrentSpeed(),
            chassis->GetWheel(chassis->kRightWheel_)->CurrentSpeed());
        state = p_rotate_motion_->GetState();

        if (state == State::kReachTarget) {
          if (stage_ == kTurnForMapEdge) {
            SwitchToEncircleMapEdgeMotion(chassis);
          } else {
            SwitchToMoveForwardToEscapeFromCliff(
                chassis, world_pose,
                config_.move_forward_to_escape_cliff_distance_,
                config_.move_forward_to_escape_cliff_time_);
          }
          finish_run = false;
          break;
        }
        // Event checking.
        if (HandleBumper(chassis, world_pose, map)) {
          SwitchToRetreatMotion(chassis, odom_pose);
          finish_run = false;
          break;
        }
        if (HandleCliffSensor(chassis, world_pose, map)) {
          if ((p_rotate_motion_->RotateAngle() > 0 && obstacle_degree_ > 45) ||
              (p_rotate_motion_->RotateAngle() < 0 && obstacle_degree_ < -45)) {
            // It is quite dangerous situation, if it keeps turning, there is a
            // big chance that it will fall down, it is better move forward
            // (away from cliff).
            SwitchToTurnForCliff(chassis, odom_pose,
                                 (obstacle_degree_ > 0 ? -30 : 30));
          } else {
            SwitchToRetreatMotion(chassis, odom_pose);
          }
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
      case kEncircleMapEdge: {
        if (p_encircle_map_edge_motion_ == nullptr) {
          if (!SwitchToEncircleMapEdgeMotion(chassis)) {
            break;
          }
        }
        auto state = p_encircle_map_edge_motion_->GetState();
        // ZINFO << "EO state:" << state;
        if (state == State::kError || state == State::kTimeout ||
            state == State::kStop || state == State::kException) {
          state_ = state;
          SwitchStageAndClearMotion(kStop);
          break;
        }

        p_encircle_map_edge_motion_->CheckState(
            world_pose, odom_pose,
            chassis->GetWheel(chassis->kLeftWheel_)->CurrentSpeed(),
            chassis->GetWheel(chassis->kRightWheel_)->CurrentSpeed());
        state = p_encircle_map_edge_motion_->GetState();

        // Since encircling map edge is so near to cleaned area, it will steps
        // on past path often.
        // if (world_pose.Distance(start_world_pose_) >
        // chassis->GetRadius()) {
        //   step_on_past_path_.store(false);

        //   ReadLocker footstep_lock(footstep_map->GetLock());
        //   MapCell pose_cell;

        //   if (footstep_map->WorldToMap(world_pose, pose_cell)) {
        //     CharGridMap2D::DataType value;
        //     // ZINFO << "Check pose for " << pose_cell;
        //     if (footstep_map->GetValue(pose_cell.X(), pose_cell.Y(), value)
        //     &&
        //         value == NavMap::kFootStep_) {
        //       ZINFO << "Step on past path.";
        //       step_on_past_path_.store(true);
        //     }
        //   }
        // }

        auto path_situation = steps_recorder_.CheckPath();
        // ZERROR << "path_situation: " << path_situation;
        if (path_situation == StepsRecorder::SituationCode::NORMAL_CLOSE_LOOP) {
          SwitchStageAndClearMotion(kFinish);
          state_ = State::kFinish;
          break;
        } else if (world_pose.Distance(start_world_pose_) >
                       map->GetResolution() &&
                   OutOfBound(world_pose, map, marker_points)) {
          ZINFO << "Out of bound: " << world_pose;
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
        if (HandleCliffSensor(chassis, world_pose, map)) {
          SwitchToRetreatMotion(chassis, odom_pose);
          finish_run = false;
          break;
        }

        state_ = state;
        // Update for map edge distance.
        float radian =
            chassis
                ->GetWallSensor(on_left_ ? chassis->kLeftWallSensor_
                                         : chassis->kRightWallSensor_)
                ->GetTf()
                .Radian();
        float check_distance;
        if (!CheckMapEdgeDistance(chassis, radian, world_pose, map,
                                  check_distance)) {
          SwitchStageAndClearMotion(kFinish);
          state_ = State::kFinish;
          ZINFO << "Step outside map.";
          break;
        }

        // Calculate for distance in y.
        auto distance = check_distance;
        auto y_distance = distance * fabs(sin(radian));

        p_encircle_map_edge_motion_->SetCurrentMapEdgeDistance(odom_pose,
                                                               y_distance);
        // ZWARN << "distance: " << FloatToString(distance, 4)
        //       << ", y_distance: " << FloatToString(y_distance, 4);

        if (NearBound(world_pose)) {
          p_encircle_map_edge_motion_->SlowdownTo(
              config_.encircle_map_edge_motion_config_->target_speed_ / 2,
              config_.encircle_map_edge_motion_config_->speed_up_step_);
        }
        p_encircle_map_edge_motion_->SpeedControl(
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
            chassis->ClearCliffSensorEvent();
            break;
          }
        }
        auto state = p_retreat_motion_->GetState();
        if (state == State::kError || state == State::kTimeout ||
            state == State::kStop || state == State::kException) {
          state_ = state;
          SwitchStageAndClearMotion(kStop);
          chassis->ClearBumperEvent();
          chassis->ClearCliffSensorEvent();
          break;
        }

        p_retreat_motion_->CheckState(
            world_pose, odom_pose,
            chassis->GetWheel(chassis->kLeftWheel_)->CurrentSpeed(),
            chassis->GetWheel(chassis->kRightWheel_)->CurrentSpeed());
        state = p_retreat_motion_->GetState();

        if (state == State::kReachTarget) {
          SwitchStageAndClearMotion(kStop);
          state_ = State::kStop;
          chassis->ClearBumperEvent();
          chassis->ClearCliffSensorEvent();
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
      case kForwardToEscapeFromCliff: {
        if (p_move_forward_motion_ == nullptr) {
          if (!SwitchToMoveForwardToEscapeFromCliff(
                  chassis, world_pose,
                  config_.move_forward_to_escape_cliff_distance_,
                  config_.move_forward_to_escape_cliff_time_)) {
            break;
          }
        }

        // Abnormal state checking.
        auto state = p_move_forward_motion_->GetState();
        // ZINFO << "Move forward state:" << state;
        if (state == State::kError || state == State::kTimeout ||
            state == State::kException) {
          SwitchStageAndClearMotion(kStop);
          state_ = State::kStop;
          chassis->ClearBumperEvent();
          chassis->ClearCliffSensorEvent();
          break;
        }

        p_move_forward_motion_->CheckState(
            world_pose, odom_pose,
            chassis->GetWheel(chassis->kLeftWheel_)->CurrentSpeed(),
            chassis->GetWheel(chassis->kRightWheel_)->CurrentSpeed());
        state = p_move_forward_motion_->GetState();
        if (state == State::kReachTarget) {
          std::vector<std::string> triggered_cliff_sensors;
          if (chassis->GetCliffSensorEvent(triggered_cliff_sensors)) {
            // Cliff sensor still trigger.
            if (!SwitchToMoveForwardToEscapeFromCliff(
                    chassis, world_pose,
                    config_.move_forward_to_escape_cliff_distance_,
                    config_.move_forward_to_escape_cliff_time_)) {
              break;
            }
          } else {
            SwitchStageAndClearMotion(kStop);
            state_ = State::kStop;
            chassis->ClearBumperEvent();
            chassis->ClearCliffSensorEvent();
            break;
          }
        }

        // Event checking.
        if (HandleBumper(chassis, world_pose, map)) {
          SwitchToRetreatMotion(chassis, odom_pose);
          finish_run = false;
          break;
        }

        // Exit when cliff sensor is not trigger anymore.
        if (!HandleCliffSensor(chassis, world_pose, map)) {
          SwitchStageAndClearMotion(kStop);
          state_ = State::kStop;
          chassis->ClearBumperEvent();
          chassis->ClearCliffSensorEvent();
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
      default: {
        ZERROR << "Should never run here." << state_;
      }
    }
  }

  if (FloatEqual(left_wheel_speed, 0) && FloatEqual(right_wheel_speed, 0)) {
    ZINFO << "Stopping.";
  }

  chassis->GetWheel(chassis->kLeftWheel_)->SetTargetSpeed(left_wheel_speed);
  chassis->GetWheel(chassis->kRightWheel_)->SetTargetSpeed(right_wheel_speed);
}

float EncircleMapEdgeMovement::GetObstacleDegree() const {
  ReadLocker lock(access_);
  return obstacle_degree_;
}

void EncircleMapEdgeMovement::SwitchStageAndClearMotion(const Stage& stage) {
  ZINFO << "Switch to stage: " << stage;
  stage_ = stage;
  p_encircle_map_edge_motion_.reset();
  p_retreat_motion_.reset();
  p_rotate_motion_.reset();
}

bool EncircleMapEdgeMovement::SwitchToRotateMotion(const Chassis::SPtr& chassis,
                                                   const MapPoint& odom_pose,
                                                   const float& rotate_degree) {
  SwitchStageAndClearMotion(kTurnForMapEdge);
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

bool EncircleMapEdgeMovement::SwitchToRetreatMotion(
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

bool EncircleMapEdgeMovement::SwitchToEncircleMapEdgeMotion(
    const Chassis::SPtr& chassis) {
  SwitchStageAndClearMotion(kEncircleMapEdge);
  p_encircle_map_edge_motion_.reset(new EncircleMapEdgeMotion(
      chassis->GetTrackLength() / 2,
      chassis->GetWheel(chassis->kLeftWheel_)->MaxSpeed(),
      chassis->GetWheel(chassis->kLeftWheel_)->MinSpeed(), on_left_,
      *config_.encircle_map_edge_motion_config_));

  if (p_encircle_map_edge_motion_->GetState() == State::kError) {
    state_ = State::kError;
    SwitchStageAndClearMotion(kStop);
    ZERROR;
    return false;
  }
  return true;
}

bool EncircleMapEdgeMovement::SwitchToTurnForCliff(const Chassis::SPtr& chassis,
                                                   const MapPoint& odom_pose,
                                                   const float& rotate_degree) {
  SwitchStageAndClearMotion(kTrunForCliff);
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

bool EncircleMapEdgeMovement::SwitchToMoveForwardToEscapeFromCliff(
    const Chassis::SPtr& chassis, const MapPoint& world_pose,
    const float& move_distance, const float& move_time) {
  SwitchStageAndClearMotion(kForwardToEscapeFromCliff);
  p_move_forward_motion_.reset(new MoveForwardMotion(
      chassis->GetTrackLength() / 2,
      chassis->GetWheel(chassis->kLeftWheel_)->MaxSpeed(),
      chassis->GetWheel(chassis->kLeftWheel_)->MinSpeed(), move_distance,
      move_time, world_pose, *config_.move_forward_motion_config_, false));

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

bool EncircleMapEdgeMovement::HandleBumper(const Chassis::SPtr& chassis,
                                           const MapPoint& world_pose,
                                           const NavMap::SPtr& map) {
  std::vector<std::string> triggered_bumpers;
  if (chassis->GetBumperEvent(triggered_bumpers)) {
    ZWARN << "Bumper trigger.";
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
          ZINFO << pose_cell.DebugString() << world_pose.DebugString();
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
      std::string trigger_bumper_name;
      if (on_left_) {
        if (right_bumper_triggered) {
          trigger_bumper_name = chassis->kRightBumper_;
        } else if (center_bumper_triggered) {
          trigger_bumper_name = chassis->kCenterBumper_;
        } else if (left_bumper_triggered) {
          trigger_bumper_name = chassis->kLeftBumper_;
        }
      } else {
        if (left_bumper_triggered) {
          trigger_bumper_name = chassis->kLeftBumper_;
        } else if (center_bumper_triggered) {
          trigger_bumper_name = chassis->kCenterBumper_;
        } else if (right_bumper_triggered) {
          trigger_bumper_name = chassis->kRightBumper_;
        }
      }
      marker_points.emplace_back(
          MarkerPoint(chassis->GetBumper(trigger_bumper_name)
                          ->GetEncircleObstacleMovementMarkPoint(on_left_),
                      map->kBumper_));
      obstacle_degree_ =
          chassis->GetBumper(trigger_bumper_name)->GetTf().Degree();

      if (steps_recorder_.AddPathPoint(StepPoint(world_pose, marker_points),
                                       true, true)) {
        MapCell pose_cell;
        if (map->GetFootStepLayer()->WorldToMap(world_pose, pose_cell)) {
          ZINFO << pose_cell.DebugString() << world_pose.DebugString();
        }
      }
    }

    return true;
  }
  return false;
}

bool EncircleMapEdgeMovement::HandleCliffSensor(const Chassis::SPtr& chassis,
                                                const MapPoint& world_pose,
                                                const NavMap::SPtr& map) {
  std::vector<std::string> triggered_cliff_sensors;
  if (chassis->GetCliffSensorEvent(triggered_cliff_sensors)) {
    // ZWARN << "Cliff sensor trigger.";
    MarkerPoints marker_points;
    // Saperate 3 situation.
    bool left_cliff_sensor = false;
    bool left_front_cliff_sensor = false;
    bool right_front_cliff_sensor = false;
    bool right_cliff_sensor = false;
    for (auto&& cliff_sensor : triggered_cliff_sensors) {
      if (cliff_sensor == chassis->kLeftFrontCliffSensor_) {
        left_front_cliff_sensor = true;
      } else if (cliff_sensor == chassis->kRightFrontCliffSensor_) {
        right_front_cliff_sensor = true;
      } else if (cliff_sensor == chassis->kLeftCliffSensor_) {
        left_cliff_sensor = true;
      } else if (cliff_sensor == chassis->kRightCliffSensor_) {
        right_cliff_sensor = true;
      }
    }
    std::string handle_cliff_sensor_name;
    if (left_front_cliff_sensor) {
      handle_cliff_sensor_name = chassis->kLeftFrontCliffSensor_;
    } else if (right_front_cliff_sensor) {
      handle_cliff_sensor_name = chassis->kRightFrontCliffSensor_;
    } else if (left_cliff_sensor) {
      handle_cliff_sensor_name = chassis->kLeftCliffSensor_;
    } else if (right_cliff_sensor) {
      handle_cliff_sensor_name = chassis->kRightCliffSensor_;
    }
    marker_points.emplace_back(
        MarkerPoint(chassis->GetCliffSensor(handle_cliff_sensor_name)
                        ->GetEncircleObstacleMovementMarkPoint(),
                    map->kCliff_));
    obstacle_degree_ =
        chassis->GetCliffSensor(handle_cliff_sensor_name)->GetTf().Degree();

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

bool EncircleMapEdgeMovement::CheckMapEdgeDistance(const Chassis::SPtr& chassis,
                                                   const float& radian,
                                                   const MapPoint& world_pose,
                                                   const NavMap::SPtr& map,
                                                   float& distance) {
  // Update for room edge distance.
  auto room_map = map->GetRoomLayer();
  ReadLocker read_lock(room_map->GetLock());
  // Update for user block distance.
  auto user_block_map = map->GetUserBlockLayer();
  ReadLocker read_lock2(user_block_map->GetLock());
  // Update for user block distance.
  auto user_select_area_map = map->GetUserSelectAreaLayer();
  ReadLocker read_lock3(user_select_area_map->GetLock());
  // Update for cleaning area edge distance.
  auto footstep_map = map->GetFootStepLayer();
  ReadLocker read_lock4(footstep_map->GetLock());
  int map_x, map_y;
  footstep_map->WorldToMap(world_pose.X(), world_pose.Y(), map_x, map_y);
  auto footstep_map_max_clean_bound =
      map->GetMaxCleanBound(std::make_shared<MapCell>(map_x, map_y));
  auto value = room_map->GetDefaultValue();
  float check_distance_side, check_distance_front;
  float front_radian = 0;
  float room_edge_distance = std::numeric_limits<float>::max();
  float user_block_edge_distance = std::numeric_limits<float>::max();
  float user_select_area_edge_distance = std::numeric_limits<float>::max();
  float cleaning_area_edge_distance = std::numeric_limits<float>::max();

  // ZINFO << "World pose: " << world_pose.DebugString();
  bool on_current_cleaning_room = false;
  if (room_map->WorldToMap(world_pose.X(), world_pose.Y(), map_x, map_y)) {
    room_map->GetValue(map_x, map_y, value);
    if (value == current_room_index_ || value == NavMap::kUnknown_) {
      on_current_cleaning_room = true;
    }
    // ZINFO << "map value: " << value;
  } else {
    ZWARN << "Out of room map.";
    return false;
  }

  user_block_map->WorldToMap(world_pose.X(), world_pose.Y(), map_x, map_y);
  user_block_map->GetValue(map_x, map_y, value);
  if (value == NavMap::kVirtualWall_ || value == NavMap::kStrictBlockArea_) {
    ZWARN << "Step onto virtual wall or strict block area.";
    return false;
  }

  bool on_cleaning_area = false;
  if (footstep_map->WorldToMap(world_pose.X(), world_pose.Y(), map_x, map_y)) {
    if (footstep_map_max_clean_bound.Contain(map_x, map_y)) {
      on_cleaning_area = true;
    }
    // ZINFO << "map value: " << value;
  }

  bool on_user_select_area = false;
  if (user_select_area_map->WorldToMap(world_pose.X(), world_pose.Y(), map_x,
                                       map_y)) {
    user_select_area_map->GetValue(map_x, map_y, value);
    if (value == NavMap::kUserSelected_) {
      on_user_select_area = true;
    }
    // ZINFO << "map value: " << value;
  }

  double x, y;
  // Check for room edge distance.
  if (on_current_cleaning_room) {
    // ZINFO << "On current cleaning room.";
    for (check_distance_side = 0; check_distance_side < chassis->GetRadius();
         check_distance_side += 0.002) {
      // Change coordinate to map frame.
      Transform::CoordinateTransformationBA(
          check_distance_side * cos(radian), check_distance_side * sin(radian),
          world_pose.X(), world_pose.Y(), world_pose.Radian(), x, y);
      if (room_map->WorldToMap(x, y, map_x, map_y)) {
        room_map->GetValue(map_x, map_y, value);
        if (value != current_room_index_ && value != NavMap::kUnknown_) {
          break;
        }
      }
    }
    for (check_distance_front = 0; check_distance_front < chassis->GetRadius();
         check_distance_front += 0.002) {
      // Change coordinate to map frame.
      Transform::CoordinateTransformationBA(
          check_distance_front * cos(front_radian),
          check_distance_front * sin(front_radian), world_pose.X(),
          world_pose.Y(), world_pose.Radian(), x, y);
      if (room_map->WorldToMap(x, y, map_x, map_y)) {
        room_map->GetValue(map_x, map_y, value);
        if (value != current_room_index_ && value != NavMap::kUnknown_) {
          break;
        }
      }
    }
    // Shorten front distance for faster turning.
    check_distance_front -=
        config_.encircle_map_edge_motion_config_->turn_circle_radius_ * 2;
    // ZINFO << "check_distance_side: " << FloatToString(check_distance_side, 4)
    //       << ", check_distance_front: "
    //       << FloatToString(check_distance_front, 4);
    room_edge_distance = std::min(check_distance_side, check_distance_front);
  } else {
    // ZINFO << "Not on current cleaning room.";
    for (check_distance_side = 0; check_distance_side > -chassis->GetRadius();
         check_distance_side -= 0.002) {
      // Change coordinate to map frame.
      Transform::CoordinateTransformationBA(
          check_distance_side * cos(radian), check_distance_side * sin(radian),
          world_pose.X(), world_pose.Y(), world_pose.Degree(), x, y);
      if (room_map->WorldToMap(x, y, map_x, map_y)) {
        room_map->GetValue(map_x, map_y, value);
        if (value == current_room_index_ || value == NavMap::kUnknown_) {
          break;
        }
      }
    }

    // Multiple this distance for faster going back to room.
    room_edge_distance = check_distance_side * 3;
  }

  // Check for user block distance.
  for (check_distance_side = 0; check_distance_side < 2 * chassis->GetRadius();
       check_distance_side += 0.002) {
    // Change coordinate to map frame.
    Transform::CoordinateTransformationBA(
        check_distance_side * cos(radian), check_distance_side * sin(radian),
        world_pose.X(), world_pose.Y(), world_pose.Radian(), x, y);
    if (user_block_map->WorldToMap(x, y, map_x, map_y)) {
      user_block_map->GetValue(map_x, map_y, value);
      if (value == NavMap::kVirtualWall_ ||
          value == NavMap::kStrictBlockArea_) {
        break;
      }
    }
  }
  for (check_distance_front = 0;
       check_distance_front < 2 * chassis->GetRadius();
       check_distance_front += 0.002) {
    // Change coordinate to map frame.
    Transform::CoordinateTransformationBA(
        check_distance_front * cos(front_radian),
        check_distance_front * sin(front_radian), world_pose.X(),
        world_pose.Y(), world_pose.Radian(), x, y);
    if (user_block_map->WorldToMap(x, y, map_x, map_y)) {
      user_block_map->GetValue(map_x, map_y, value);
      if (value == NavMap::kVirtualWall_ ||
          value == NavMap::kStrictBlockArea_) {
        break;
      }
    }
  }
  // Shorten front distance for faster turning.
  check_distance_front -=
      config_.encircle_map_edge_motion_config_->turn_circle_radius_ * 2;
  // ZINFO << "check_distance_side: " << FloatToString(check_distance_side, 4)
  //       << ", check_distance_front: "
  //       << FloatToString(check_distance_front, 4);

  // Minus radius because robot should never be that closed.
  user_block_edge_distance =
      std::min(check_distance_side, check_distance_front) -
      chassis->GetRadius();

  // Check for cleaning area edge distance.
  if (on_cleaning_area) {
    // ZINFO << "On cleaning area.";
    for (check_distance_side = 0;
         check_distance_side < chassis->GetRadius() * 2;
         check_distance_side += 0.002) {
      // Change coordinate to map frame.
      Transform::CoordinateTransformationBA(
          check_distance_side * cos(radian), check_distance_side * sin(radian),
          world_pose.X(), world_pose.Y(), world_pose.Radian(), x, y);
      if (footstep_map->WorldToMap(x, y, map_x, map_y)) {
        if (!footstep_map_max_clean_bound.Contain(map_x, map_y)) {
          break;
        }
      }
    }
    for (check_distance_front = 0;
         check_distance_front < chassis->GetRadius() * 2;
         check_distance_front += 0.002) {
      // Change coordinate to map frame.
      Transform::CoordinateTransformationBA(
          check_distance_front * cos(front_radian),
          check_distance_front * sin(front_radian), world_pose.X(),
          world_pose.Y(), world_pose.Radian(), x, y);
      if (footstep_map->WorldToMap(x, y, map_x, map_y)) {
        if (!footstep_map_max_clean_bound.Contain(map_x, map_y)) {
          break;
        }
      }
    }
    // Shorten front distance for faster turning.
    check_distance_front -=
        config_.encircle_map_edge_motion_config_->turn_circle_radius_ * 2;
    // ZINFO << "check_distance_side: " << FloatToString(check_distance_side, 4)
    //       << ", check_distance_front: "
    //       << FloatToString(check_distance_front, 4);
    cleaning_area_edge_distance =
        std::min(check_distance_side, check_distance_front);

    // Minus radius because robot should never be that closed.
    user_block_edge_distance =
        std::min(check_distance_side, check_distance_front) -
        chassis->GetRadius();
  } else {
    // ZINFO << "Not on cleaning area.";
    for (check_distance_side = 0;
         check_distance_side > -chassis->GetRadius() * 2;
         check_distance_side -= 0.002) {
      // Change coordinate to map frame.
      Transform::CoordinateTransformationBA(
          check_distance_side * cos(radian), check_distance_side * sin(radian),
          world_pose.X(), world_pose.Y(), world_pose.Degree(), x, y);
      if (footstep_map->WorldToMap(x, y, map_x, map_y)) {
        if (footstep_map_max_clean_bound.Contain(map_x, map_y)) {
          break;
        }
      }
    }

    // Multiple this distance for faster going back to uesr select area.
    cleaning_area_edge_distance = check_distance_side * 3;
  }

  // Check for room edge distance.
  if (on_user_select_area) {
    // ZINFO << "On user select area.";
    for (check_distance_side = 0; check_distance_side < chassis->GetRadius();
         check_distance_side += 0.002) {
      // Change coordinate to map frame.
      Transform::CoordinateTransformationBA(
          check_distance_side * cos(radian), check_distance_side * sin(radian),
          world_pose.X(), world_pose.Y(), world_pose.Radian(), x, y);
      if (user_select_area_map->WorldToMap(x, y, map_x, map_y)) {
        user_select_area_map->GetValue(map_x, map_y, value);
        if (value != NavMap::kUserSelected_) {
          break;
        }
      }
    }
    for (check_distance_front = 0; check_distance_front < chassis->GetRadius();
         check_distance_front += 0.002) {
      // Change coordinate to map frame.
      Transform::CoordinateTransformationBA(
          check_distance_front * cos(front_radian),
          check_distance_front * sin(front_radian), world_pose.X(),
          world_pose.Y(), world_pose.Radian(), x, y);
      if (user_select_area_map->WorldToMap(x, y, map_x, map_y)) {
        user_select_area_map->GetValue(map_x, map_y, value);
        if (value != NavMap::kUserSelected_) {
          break;
        }
      }
    }
    check_distance_front -=
        config_.encircle_map_edge_motion_config_->turn_circle_radius_ * 2;
    // ZINFO << "check_distance_side: " << FloatToString(check_distance_side, 4)
    //       << ", check_distance_front: "
    //       << FloatToString(check_distance_front, 4);
    user_select_area_edge_distance =
        std::min(check_distance_side, check_distance_front);
  } else {
    // ZINFO << "Not on current cleaning room.";
    for (check_distance_side = 0; check_distance_side > -chassis->GetRadius();
         check_distance_side -= 0.002) {
      // Change coordinate to map frame.
      Transform::CoordinateTransformationBA(
          check_distance_side * cos(radian), check_distance_side * sin(radian),
          world_pose.X(), world_pose.Y(), world_pose.Degree(), x, y);
      user_select_area_map->WorldToMap(x, y, map_x, map_y);
      user_select_area_map->GetValue(map_x, map_y, value);
      if (value == NavMap::kUserSelected_) {
        break;
      }
    }

    // Multiple this distance for faster going back to uesr select area.
    user_select_area_edge_distance = check_distance_side * 3;
  }

  distance = std::min(room_edge_distance, user_block_edge_distance);
  distance = std::min(distance, user_select_area_edge_distance);
  distance = std::min(distance, cleaning_area_edge_distance);

  // ZGWARN << "Map value distance: " << FloatToString(distance, 4)
  //        << " cleaning area edge distance: "
  //        << FloatToString(cleaning_area_edge_distance, 4);
  return true;
}

}  // namespace zima
