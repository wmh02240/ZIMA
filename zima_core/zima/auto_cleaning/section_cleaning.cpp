/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/auto_cleaning/section_cleaning.h"

#include <algorithm>

#include "zima/common/time.h"
#include "zima/common/util.h"
#include "zima/logger/logger.h"

namespace zima {

SectionCleaning::SectionCleaningInfo::SectionCleaningInfo(
    const DynamicMapCellBound& bound)
    : state_(kEdgeCleaning),
      section_bound_(bound),
      section_real_bound_(INT16_MAX, INT16_MIN, INT16_MAX, INT16_MIN),
      current_movement_name_(""),
      current_cell_path_({}),
      current_world_pose_(nullptr) {}

SectionCleaning::SectionCleaningInfo::SectionCleaningInfo(
    const SectionCleaning::SectionCleaningInfo& ref)
    : state_(ref.state_),
      section_bound_(ref.section_bound_),
      section_real_bound_(ref.section_real_bound_),
      current_movement_name_(ref.current_movement_name_),
      current_cell_path_(ref.current_cell_path_),
      current_world_pose_(nullptr) {
  if (ref.current_world_pose_ != nullptr) {
    current_world_pose_ = std::make_shared<MapPoint>(*ref.current_world_pose_);
  }
}

SectionCleaning::SectionCleaning(const DynamicMapCellBound& bound,
                                 const NavMap::SPtr& nav_map)
    : info_(bound),
      section_print_bound_(bound),
      section_point_bound_(0, 0, 0, 0),
      generate_path_request_(true),
      encircle_obs_saved_track_({}),
      next_movement_(nullptr),
      access_(std::make_shared<ReadWriteLock>()),
      section_nav_map_(/*std::make_shared<NavMap>()*/ nullptr) {
  ZGINFO;
  float min_x, min_y, max_x, max_y;
  nav_map->GetFootStepLayer()->MapToWorld(info_.section_bound_.GetMin().X(),
                                          info_.section_bound_.GetMin().Y(),
                                          min_x, min_y);
  nav_map->GetFootStepLayer()->MapToWorld(info_.section_bound_.GetMax().X(),
                                          info_.section_bound_.GetMax().Y(),
                                          max_x, max_y);

  section_point_bound_.Reset(min_x, max_x, min_y, max_y);
  zigzag_planner_.Initialize();
  ZGINFO << "Start auto cleaning for " << info_.section_bound_.DebugString();
  section_print_bound_.Expend(
      section_print_bound_.GetMin() -
      MapCell(nav_map->kRobotCellWidth_, nav_map->kRobotCellWidth_));
  section_print_bound_.Expend(
      section_print_bound_.GetMax() +
      MapCell(nav_map->kRobotCellWidth_, nav_map->kRobotCellWidth_));
}

SectionCleaning::~SectionCleaning() {}

bool SectionCleaning::Pause(const Chassis::SPtr& chassis,
                            const ChassisController::SPtr& chassis_controller,
                            const NavMap::SPtr& nav_map) {
  WriteLocker lock(access_);
  if (chassis_controller == nullptr) {
    ZERROR << "Chassis controller invalid.";
    return false;
  }
  if (!chassis_controller->IsThreadRunning()) {
    ZERROR << "Chassis controller thread not running.";
    return false;
  }

  ZGINFO << "Exec pause.";
  // Stop current movement.
  chassis_controller->ForceStopMovement();

  while (true) {
    ChassisController::InfoWrapper info;
    chassis_controller->GlanceInfo(info);
    if (info.GetControllerState() ==
            ChassisController::InfoWrapper::ControllerState::kStandby ||
        info.GetControllerState() ==
            ChassisController::InfoWrapper::ControllerState::kStop) {
      chassis_controller->ExtractInfo(info);
      auto last_movement = info.GetLastMovement();
      if (last_movement == nullptr) {
        ZGINFO << "Chassis controller standby state without movement.";
        break;
      }
      if (last_movement->Name() == typeid(TracePathMovement).name()) {
        nav_map->MarkForCleaningSteps(
            *TransformManager::Instance(), last_movement->GetSteps(),
            info_.section_bound_,
            dynamic_cast<TracePathMovement*>(last_movement.get())
                ->IsTraceForEdgePath());
        // nav_map->GetPrintLayer()->Print(section_print_bound_, __FILE__,
        //                                 __FUNCTION__, __LINE__);
      } else if (last_movement->Name() ==
                     typeid(EncircleObstacleMovement).name() ||
                 last_movement->Name() ==
                     typeid(EncircleMapEdgeMovement).name()) {
        nav_map->MarkForAlongSideSteps(
            *TransformManager::Instance(), last_movement->GetSteps(),
            chassis->EncircleObstacleOnLeft(), info_.section_bound_);
        // nav_map->GetPrintLayer()->Print(section_print_bound_, __FILE__,
        //                                 __FUNCTION__, __LINE__);
      }
      break;
    }
    Time::SleepMSec(5);
  }

  ZGINFO << "Exec pause finish.";
  return true;
}

bool SectionCleaning::GenerateNextMovement(
    const Chassis::SPtr& chassis, const MapPoint& world_pose,
    const NavMap::SPtr& nav_map, const RoomInfo::SPtr& room_info,
    const MapPointPath& encircle_obs_saved_track,
    MovementBase::UPtr& next_movement) {
  if (!generate_path_request_.load()) {
    return false;
  }

  bool ret = false;

  MapCell pose_cell;
  if (!nav_map->GetFootStepLayer()->WorldToMap(world_pose, pose_cell)) {
    ZWARN << "Robot is not in footprint map.";
    return ret;
  }

  info_.state_ = kEdgeCleaning;

  while (generate_path_request_.load()) {
    auto state = info_.state_;
    switch (state) {
      case kEdgeCleaning: {
        if (!encircle_obs_saved_track.empty()) {
          auto track = encircle_obs_saved_track;
          std::reverse(track.begin(), track.end());
          ZGINFO << "Trace back saved track.";
          next_movement.reset(new TracePathMovement(
              chassis, track, nav_map, room_info->GetRoomIndex(), true));
          CharGridMap2D::PointPathToCellPath(nav_map->GetFootStepLayer(), track,
                                             info_.current_cell_path_);
          generate_path_request_.store(false);
          ret = true;
          break;
        }

        MapPointPath output_path;
        MapCellPath output_cell_path;
        if (edge_planner_.GeneratePath(
                nav_map, section_nav_map_, world_pose, info_.section_bound_,
                room_info->GetRoomIndex(), output_path, output_cell_path,
                chassis->EncircleObstacleOnLeft())) {
          ZGINFO << "Clean for section edge.";
          info_.current_cell_path_ = output_cell_path;
          next_movement.reset(new TracePathMovement(
              chassis, output_path, nav_map, room_info->GetRoomIndex(), true,
              true, true));
          generate_path_request_.store(false);
          ret = true;
          break;
        }

        info_.state_ = kZigZagCleaning;
        break;
      }
      case kZigZagCleaning: {
        ZigZagPlannerManager::ZigZagPath output_path;
        MapCellPath output_cell_path;
        if (zigzag_planner_.GeneratePath(nav_map, section_nav_map_, world_pose,
                                         info_.section_bound_,
                                         room_info->GetRoomIndex(), output_path,
                                         output_cell_path, true)) {
          ZGINFO << "Zig zag clean for section.";
          info_.current_cell_path_ = output_cell_path;
          next_movement.reset(
              new TracePathMovement(chassis, output_path.path, nav_map,
                                    room_info->GetRoomIndex(), true));
          generate_path_request_.store(false);
          ret = true;
          break;
        }

        info_.state_ = kFinished;
        next_movement.reset();
        ZGINFO << "Unable to find uncleaned area in this section.";
        generate_path_request_.store(false);
        break;
      }
      default: {
        return ret;
      }
    }
  }

  return ret;
}

bool SectionCleaning::ChassisSupervise(
    const Chassis::SPtr& chassis,
    const ChassisController::SPtr& chassis_controller,
    const NavMap::SPtr& nav_map, const RoomInfo::SPtr& room_info) {
  WriteLocker lock(access_);
  if (chassis_controller == nullptr) {
    ZERROR << "Chassis controller invalid.";
    return false;
  }
  if (!chassis_controller->IsThreadRunning()) {
    ZERROR << "Chassis controller thread not running.";
    return false;
  }

  ChassisController::InfoWrapper chassis_controller_info;
  chassis_controller->GlanceInfo(chassis_controller_info);
  // ZINFO << chassis_controller_info.DebugString();
  info_.current_world_pose_ =
      std::make_shared<MapPoint>(chassis_controller_info.GetWorldPose());
  if (generate_path_request_.load()) {
    ZGINFO << "World pose: " << chassis_controller_info.GetWorldPose();
    if (!GenerateNextMovement(chassis, chassis_controller_info.GetWorldPose(),
                              nav_map, room_info, encircle_obs_saved_track_,
                              next_movement_)) {
      ZGINFO << "Section cleaning finished.";
      chassis_controller->ForceStopMovement();
      info_.state_ = kFinished;
      return true;
    }

    if (next_movement_ == nullptr) {
      ZGERROR << "No movement generated.";
      chassis_controller->ForceStopMovement();
      info_.state_ = kFinished;
      return false;
    }
  }

  if (next_movement_ != nullptr) {
    chassis_controller->UpdateCachedNavMap(nav_map);
    chassis_controller->ForceStopMovement();
    info_.current_movement_name_ = next_movement_->Name();
    chassis_controller->SetPlan(next_movement_);
    chassis_controller->GlanceInfo(chassis_controller_info);
  }

  switch (info_.state_) {
    case kEdgeCleaning:
    case kZigZagCleaning: {
      if (chassis_controller_info.GetControllerState() ==
          ChassisController::InfoWrapper::ControllerState::kRunning) {
        HandleControllerRunning(info_.state_, chassis, chassis_controller,
                                chassis_controller_info, nav_map, room_info);
      } else if (chassis_controller_info.GetControllerState() ==
                     ChassisController::InfoWrapper::ControllerState::
                         kStandby ||
                 chassis_controller_info.GetControllerState() ==
                     ChassisController::InfoWrapper::ControllerState::kStop) {
        HandleControllerStandbyOrStop(info_.state_, chassis, chassis_controller,
                                      nav_map, room_info);
      }
      break;
    }
    case kFinished: {
      ZGINFO << "Section cleaning finished(Maybe trapped)";
      break;
    }
    default: {
      ZERROR << "Should never run here.";
      return false;
    }
  }

  return true;
}

bool SectionCleaning::HandleControllerRunning(
    const State& cleaning_state, const Chassis::SPtr& chassis,
    const ChassisController::SPtr& chassis_controller,
    const ChassisController::InfoWrapper& info, const NavMap::SPtr& nav_map,
    const RoomInfo::SPtr& room_info) {
  if (info.GetCurrentMovementName() ==
      typeid(EncircleObstacleMovement).name()) {
    if (info.GetIsSteppedOnPastPath()) {
      ZGINFO << "Step on past path.";
      chassis_controller->ForceStopMovement();
    }
  } else if (info.GetCurrentMovementName() ==
             typeid(EncircleObstacleMovement).name()) {
    if (info_.current_cell_path_.size() != info.GetTargetsLeft()) {
      ZGINFO << "Update targets left.";
      Steps steps;
      MapPointPath path;
      if (chassis_controller->GetCurrentPathExecutionInfo(steps, path)) {
        CharGridMap2D::PointPathToCellPath(nav_map->GetFootStepLayer(), path,
                                           info_.current_cell_path_);
      }
    }
  }
  return true;
}

bool SectionCleaning::HandleControllerStandbyOrStop(
    const State& cleaning_state, const Chassis::SPtr& chassis,
    const ChassisController::SPtr& chassis_controller,
    const NavMap::SPtr& nav_map, const RoomInfo::SPtr& room_info) {
  ChassisController::InfoWrapper info;
  chassis_controller->ExtractInfo(info);
  if (info.GetControllerState() !=
          ChassisController::InfoWrapper::ControllerState::kStandby &&
      info.GetControllerState() !=
          ChassisController::InfoWrapper::ControllerState::kStop) {
    ZERROR << "Chassis controller is not in standby or stop state.";
    return false;
  }
  auto last_movement = info.GetLastMovement();
  if (last_movement == nullptr) {
    ZERROR << "Chassis controller standby state without movement.";
    return false;
  }
  if (last_movement->Name() == typeid(TracePathMovement).name()) {
    auto tp = dynamic_cast<TracePathMovement*>(last_movement.get());

    auto mark_for_tp_movement = [&](const Steps& movement_steps) {
      nav_map->MarkForCleaningSteps(*TransformManager::Instance(),
                                    movement_steps, info_.section_bound_,
                                    tp->IsTraceForEdgePath());
      // section_nav_map_->MarkForCleaningSteps(
      //     *TransformManager::Instance(), movement_steps,
      //     info_.section_bound_, tp->IsTraceForEdgePath());

      // if (FLAGS_debug_enable) {
      //   nav_map->GetPrintLayer()->Print(__FILE__, __FUNCTION__, __LINE__);
      // }
    };

    auto movement_state = tp->GetState();
    switch (movement_state) {
      case MotionBase::State::kReachTarget:
      case MotionBase::State::kStop: {
        ZGINFO << "End tp movement, state: " << movement_state;
        mark_for_tp_movement(tp->GetSteps());

        if (movement_state == MotionBase::State::kStop) {
          DynamicMapPointBound new_bound(
              section_point_bound_.GetMin().X() - nav_map->GetResolution() / 2,
              section_point_bound_.GetMax().X() + nav_map->GetResolution() / 2,
              section_point_bound_.GetMin().Y() - nav_map->GetResolution() / 2,
              section_point_bound_.GetMax().Y() + nav_map->GetResolution() / 2);

          if (tp->GetStopReason() ==
                  TracePathMovement::StopReason::kStopForRoomEdge ||
              tp->GetStopReason() ==
                  TracePathMovement::StopReason::kStopForForbidArea ||
              tp->GetStopReason() ==
                  TracePathMovement::StopReason::kStopForUserUnSelectArea ||
              tp->GetStopReason() ==
                  TracePathMovement::StopReason::kStopForCleaningAreaEdge) {
            auto rotate_degree =
                NormalizeDegree(chassis->EncircleObstacleOnLeft()
                                    ? tp->GetMapObstacleDegree() - 90
                                    : tp->GetMapObstacleDegree() + 90);
            next_movement_.reset(new EncircleMapEdgeMovement(
                chassis, nav_map->GetResolution(), new_bound,
                room_info->GetRoomIndex(), info.GetWorldPose(),
                info.GetOdomPose(), chassis->EncircleObstacleOnLeft(),
                rotate_degree));
          } else if (tp->GetStopReason() ==
                     TracePathMovement::StopReason::kStopForObstacle) {
            auto rotate_degree =
                NormalizeDegree(chassis->EncircleObstacleOnLeft()
                                    ? tp->GetObstacleDegree() - 50
                                    : tp->GetObstacleDegree() + 50);
            next_movement_.reset(new EncircleObstacleMovement(
                chassis, nav_map->GetResolution(), new_bound, info.GetWorldPose(),
                info.GetOdomPose(), chassis->EncircleObstacleOnLeft(),
                rotate_degree, room_info->GetRoomIndex()));
          } else {
            generate_path_request_.store(true);
          }
        } else {
          generate_path_request_.store(true);
        }
        break;
      }
      case MotionBase::State::kError:
      case MotionBase::State::kException:
      case MotionBase::State::kTimeout: {
        ZERROR << "End tp movement, state: " << movement_state;
        mark_for_tp_movement(tp->GetSteps());

        generate_path_request_.store(true);
        break;
      }
      default: {
        ZERROR << "It should not end with state: " << movement_state;
        generate_path_request_.store(true);
        break;
      }
    }
  } else if (last_movement->Name() == typeid(EncircleObstacleMovement).name()) {
    auto eo = dynamic_cast<EncircleObstacleMovement*>(last_movement.get());

    auto mark_for_eo_movement = [&](const Steps& movement_steps) {
      nav_map->MarkForAlongSideSteps(
          *TransformManager::Instance(), movement_steps,
          chassis->EncircleObstacleOnLeft(), info_.section_bound_);
      // section_nav_map_->MarkForAlongSideSteps(
      //     *TransformManager::Instance(), movement_steps,
      //     chassis->EncircleObstacleOnLeft(), section_bound_);

      // for (auto&& str :
      //      nav_map->GetPrintLayer()->DebugString(section_print_bound_)) {
      //   ZINFO << str;
      // }
    };

    if (eo->IsSteppedOnPastPath()) {
      ZGINFO << "Step on past path.";
      mark_for_eo_movement(eo->GetSteps());
      generate_path_request_.store(true);
    } else {
      auto movement_state = eo->GetState();
      switch (movement_state) {
        case MotionBase::State::kFinish:
        case MotionBase::State::kStop: {
          mark_for_eo_movement(eo->GetSteps());

          if (movement_state == MotionBase::State::kStop) {
            DynamicMapPointBound new_bound(section_point_bound_.GetMin().X() -
                                               nav_map->GetResolution() / 2,
                                           section_point_bound_.GetMax().X() +
                                               nav_map->GetResolution() / 2,
                                           section_point_bound_.GetMin().Y() -
                                               nav_map->GetResolution() / 2,
                                           section_point_bound_.GetMax().Y() +
                                               nav_map->GetResolution() / 2);

            if (eo->GetStopReason() ==
                    EncircleObstacleMovement::StopReason::kStopForRoomEdge ||
                eo->GetStopReason() ==
                    EncircleObstacleMovement::StopReason::kStopForForbidArea ||
                eo->GetStopReason() == EncircleObstacleMovement::StopReason::
                                           kStopForUserUnSelectArea ||
                eo->GetStopReason() == EncircleObstacleMovement::StopReason::
                                           kStopForCleaningAreaEdge) {
              auto rotate_degree =
                  NormalizeDegree(chassis->EncircleObstacleOnLeft()
                                      ? eo->GetMapObstacleDegree() - 90
                                      : eo->GetMapObstacleDegree() + 90);
              next_movement_.reset(new EncircleMapEdgeMovement(
                  chassis, nav_map->GetResolution(), new_bound,
                  room_info->GetRoomIndex(), info.GetWorldPose(),
                  info.GetOdomPose(), chassis->EncircleObstacleOnLeft(),
                  rotate_degree));
            } else {
              generate_path_request_.store(true);
            }
          } else {
            generate_path_request_.store(true);
          }

          break;
        }
        case MotionBase::State::kError:
        case MotionBase::State::kException:
        case MotionBase::State::kTimeout: {
          ZERROR << "End eo movement, state: " << movement_state;
          mark_for_eo_movement(eo->GetSteps());
          generate_path_request_.store(true);
          break;
        }
        default: {
          ZERROR << "It should not end with state: " << movement_state;
          generate_path_request_.store(true);
          break;
        }
      }
    }
  } else if (last_movement->Name() == typeid(EncircleMapEdgeMovement).name()) {
    auto em = dynamic_cast<EncircleMapEdgeMovement*>(last_movement.get());

    auto mark_for_em_movement = [&](const Steps& movement_steps) {
      nav_map->MarkForAlongSideSteps(
          *TransformManager::Instance(), movement_steps,
          chassis->EncircleObstacleOnLeft(), info_.section_bound_);
      // section_nav_map_->MarkForAlongSideSteps(
      //     *TransformManager::Instance(), movement_steps,
      //     chassis->EncircleObstacleOnLeft(), section_bound_);

      // for (auto&& str :
      //      nav_map->GetPrintLayer()->DebugString(section_print_bound_)) {
      //   ZINFO << str;
      // }
    };

    auto movement_state = em->GetState();
    switch (movement_state) {
      case MotionBase::State::kFinish:
      case MotionBase::State::kStop: {
        auto movement_steps = em->GetSteps();
        mark_for_em_movement(movement_steps);

        if (movement_state == MotionBase::State::kStop) {
          DynamicMapPointBound new_bound(
              section_point_bound_.GetMin().X() - nav_map->GetResolution() / 2,
              section_point_bound_.GetMax().X() + nav_map->GetResolution() / 2,
              section_point_bound_.GetMin().Y() - nav_map->GetResolution() / 2,
              section_point_bound_.GetMax().Y() + nav_map->GetResolution() / 2);

          auto rotate_degree = NormalizeDegree(
              chassis->EncircleObstacleOnLeft() ? em->GetObstacleDegree() - 50
                                                : em->GetObstacleDegree() + 50);
          next_movement_.reset(new EncircleObstacleMovement(
              chassis, nav_map->GetResolution(), new_bound, info.GetWorldPose(),
              info.GetOdomPose(), chassis->EncircleObstacleOnLeft(),
              rotate_degree, room_info->GetRoomIndex()));
        } else {
          generate_path_request_.store(true);
        }

        break;
      }
      case MotionBase::State::kError:
      case MotionBase::State::kException:
      case MotionBase::State::kTimeout: {
        ZERROR << "End em movement, state: " << movement_state;
        mark_for_em_movement(em->GetSteps());

        generate_path_request_.store(true);
        break;
      }
      default: {
        ZERROR << "It should not end with state: " << movement_state;
        generate_path_request_.store(true);
        break;
      }
    }
  }
  return true;
}

SectionCleaning::SectionCleaningInfo SectionCleaning::GetCleaningInfo() const {
  ReadLocker lock(access_);
  return info_;
}

}  // namespace zima
