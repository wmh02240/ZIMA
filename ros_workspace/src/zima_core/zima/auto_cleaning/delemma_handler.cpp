/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/auto_cleaning/delemma_handler.h"

#include "zima/common/time.h"
#include "zima/common/util.h"
#include "zima/logger/logger.h"
#include "zima/movement/encircle_map_edge_movement.h"
#include "zima/movement/encircle_obstacle_movement.h"
#include "zima/movement/trace_path_movement.h"

namespace zima {

static const double kCheckDelemmaIntervalSec = 10;

DelemmaHandler::DelemmaHandlerInfo::DelemmaHandlerInfo()
    : state_(kNotInDelemma),
      current_movement_name_(""),
      current_cell_path_({}),
      current_world_pose_(nullptr),
      run_duration_(0) {}

DelemmaHandler::DelemmaHandlerInfo::DelemmaHandlerInfo(
    const DelemmaHandlerInfo& ref)
    : state_(ref.state_),
      current_movement_name_(ref.current_movement_name_),
      current_cell_path_(ref.current_cell_path_),
      current_world_pose_(nullptr),
      run_duration_(ref.run_duration_) {
  if (ref.current_world_pose_ != nullptr) {
    current_world_pose_ = std::make_shared<MapPoint>(*ref.current_world_pose_);
  }
}

DelemmaHandler::DelemmaHandler()
    : escape_delemma_stop_watch_(
          new StopWatch("escape delemma stop watch", true, true)),
      info_(),
      next_movement_(nullptr),
      last_check_time_(Time::Now()),
      access_(std::make_shared<ReadWriteLock>()),
      current_path_({}) {
  ZGINFO;
}

DelemmaHandler::DelemmaHandler(const DelemmaHandlerInfo::SPtr& cached_info)
    : info_(*cached_info),
      next_movement_(nullptr),
      last_check_time_(Time::Now()),
      access_(std::make_shared<ReadWriteLock>()),
      current_path_({}) {
  if (cached_info != nullptr) {
    escape_delemma_stop_watch_.reset(
        new StopWatch("escape delemma stop watch", info_.run_duration_));
  } else {
    escape_delemma_stop_watch_.reset(
        new StopWatch("escape delemma stop watch", true, true));
  }
}

DelemmaHandler::~DelemmaHandler() {}

bool DelemmaHandler::Pause(const Chassis::SPtr& chassis,
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

  switch (info_.state_) {
    case kInDelemma: {
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
          nav_map->MarkForAlongSideSteps(
              *TransformManager::Instance(), last_movement->GetSteps(),
              chassis->EncircleObstacleOnLeft(),
              nav_map->GetFootStepLayer()->GetAvailableBound());

          // nav_map->GetPrintLayer()->Print(__FILE__, __FUNCTION__, __LINE__);
          break;
        }
        Time::SleepMSec(5);
      }
    }
    default: {
      // Do nothing.
      break;
    }
  }

  ZGINFO << "Exec pause finish.";
  return true;
}

bool DelemmaHandler::ChassisSupervise(
    const Chassis::SPtr& chassis,
    const ChassisController::SPtr& chassis_controller,
    const OperationData::SPtr& operation_data) {
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
  auto world_pose = chassis_controller_info.GetWorldPose();
  info_.current_world_pose_ = std::make_shared<MapPoint>(world_pose);

  auto nav_map = operation_data->GetNavMapRef();
  switch (info_.state_) {
    case kInDelemma: {
      if (escape_delemma_stop_watch_->Duration() > 5 * 60) {
        ZINFO << "Timeout.";
        info_.state_ = kTimeout;
        break;
      }
      if (chassis_controller_info.GetControllerState() ==
              ChassisController::InfoWrapper::ControllerState::kStop ||
          chassis_controller_info.GetControllerState() ==
              ChassisController::InfoWrapper::ControllerState::kStandby) {
        ChassisController::InfoWrapper info;
        chassis_controller->ExtractInfo(info);
        auto last_movement = info.GetLastMovement();
        world_pose = info.GetWorldPose();
        auto footstep_layer = nav_map->GetFootStepLayer();
        auto cell_bound = footstep_layer->GetAvailableBound();
        MapPoint min_point, max_point;
        footstep_layer->MapToWorld(
            MapCell(cell_bound.GetMin().X(), cell_bound.GetMin().Y()),
            min_point);
        footstep_layer->MapToWorld(
            MapCell(cell_bound.GetMax().X(), cell_bound.GetMax().Y()),
            max_point);
        DynamicMapPointBound point_bound(min_point.X(), max_point.X(),
                                         min_point.Y(), max_point.Y());
        next_movement_.reset();
        if (last_movement != nullptr) {
          if (last_movement->Name() ==
              typeid(EncircleObstacleMovement).name()) {
            nav_map->MarkForAlongSideSteps(
                *TransformManager::Instance(), last_movement->GetSteps(),
                chassis->EncircleObstacleOnLeft(),
                nav_map->GetFootStepLayer()->GetAvailableBound());
            auto eo =
                dynamic_cast<EncircleObstacleMovement*>(last_movement.get());
            auto stop_reason = eo->GetStopReason();
            if (stop_reason ==
                    EncircleObstacleMovement::StopReason::kStopForRoomEdge ||
                stop_reason ==
                    EncircleObstacleMovement::StopReason::kStopForForbidArea ||
                stop_reason == EncircleObstacleMovement::StopReason::
                                   kStopForUserUnSelectArea ||
                stop_reason == EncircleObstacleMovement::StopReason::
                                   kStopForCleaningAreaEdge) {
              auto rotate_degree =
                  NormalizeDegree(chassis->EncircleObstacleOnLeft()
                                      ? eo->GetMapObstacleDegree() - 90
                                      : eo->GetMapObstacleDegree() + 90);
              next_movement_.reset(new EncircleMapEdgeMovement(
                  chassis, nav_map->GetResolution(), point_bound,
                  NavMap::kUnknown_, info.GetWorldPose(), info.GetOdomPose(),
                  chassis->EncircleObstacleOnLeft(), rotate_degree));
            } else {
              MapPointPath path;
              auto start_pose = world_pose;
              start_pose.AdjustDegree(chassis->EncircleObstacleOnLeft() ? -90
                                                                        : 90);
              path.emplace_back(start_pose);
              static const float kTryDistance = 20;
              auto destination_pose = MapPoint(kTryDistance, 0);
              double x, y;
              Transform::CoordinateTransformationBA(
                  destination_pose.X(), destination_pose.Y(), start_pose.X(),
                  start_pose.Y(), start_pose.Radian(), x, y);
              destination_pose.SetX(x);
              destination_pose.SetY(y);
              destination_pose.SetDegree(start_pose.Degree());
              path.emplace_back(destination_pose);
              // ZERROR << MapPoint::DebugString(path);

              next_movement_.reset(new TracePathMovement(
                  chassis, path, nav_map, NavMap::kUnknown_, true, true));
              // ZINFO << "Move.";
            }
          } else if (last_movement->Name() ==
                     typeid(TracePathMovement).name()) {
            nav_map->MarkForPassingThroughSteps(
                *TransformManager::Instance(), last_movement->GetSteps(),
                nav_map->GetFootStepLayer()->GetAvailableBound());
            auto tp = dynamic_cast<TracePathMovement*>(last_movement.get());
            auto stop_reason = tp->GetStopReason();
            if (stop_reason ==
                TracePathMovement::StopReason::kStopForObstacle) {
              // Use default logic.
            } else if (stop_reason ==
                           TracePathMovement::StopReason::kStopForRoomEdge ||
                       stop_reason ==
                           TracePathMovement::StopReason::kStopForForbidArea ||
                       stop_reason == TracePathMovement::StopReason::
                                          kStopForUserUnSelectArea ||
                       stop_reason == TracePathMovement::StopReason::
                                          kStopForCleaningAreaEdge) {
              auto rotate_degree =
                  NormalizeDegree(chassis->EncircleObstacleOnLeft()
                                      ? tp->GetMapObstacleDegree() - 90
                                      : tp->GetMapObstacleDegree() + 90);
              next_movement_.reset(new EncircleMapEdgeMovement(
                  chassis, nav_map->GetResolution(), point_bound,
                  NavMap::kUnknown_, info.GetWorldPose(), info.GetOdomPose(),
                  chassis->EncircleObstacleOnLeft(), rotate_degree));
            } else {
              MapPointPath path;
              auto start_pose = world_pose;
              start_pose.AdjustDegree(chassis->EncircleObstacleOnLeft() ? -90
                                                                        : 90);
              path.emplace_back(start_pose);
              static const float kTryDistance = 20;
              auto destination_pose = MapPoint(kTryDistance, 0);
              double x, y;
              Transform::CoordinateTransformationBA(
                  destination_pose.X(), destination_pose.Y(), start_pose.X(),
                  start_pose.Y(), start_pose.Radian(), x, y);
              destination_pose.SetX(x);
              destination_pose.SetY(y);
              destination_pose.SetDegree(start_pose.Degree());
              path.emplace_back(destination_pose);
              // ZERROR << MapPoint::DebugString(path);

              next_movement_.reset(new TracePathMovement(
                  chassis, path, nav_map, NavMap::kUnknown_, true, true));
              // ZINFO << "Move.";
            }
          }

          // nav_map->GetPrintLayer()->Print(__FILE__, __FUNCTION__, __LINE__);
        }
        if (next_movement_ == nullptr) {
          auto odom_pose = chassis_controller_info.GetOdomPose();
          next_movement_.reset(new EncircleObstacleMovement(
              chassis, nav_map->GetResolution(), point_bound, world_pose,
              odom_pose, chassis->EncircleObstacleOnLeft(), 0,
              NavMap::kUnknown_, true));
        }
        chassis_controller->ForceStopMovement();
        chassis_controller->UpdateCachedNavMap(nav_map);
        info_.current_movement_name_ = next_movement_->Name();
        chassis_controller->SetPlan(next_movement_);
      } else if (chassis_controller_info.GetControllerState() ==
                 ChassisController::InfoWrapper::ControllerState::kRunning) {
        if (chassis_controller_info.GetCurrentMovementName() ==
                typeid(EncircleObstacleMovement).name() ||
            chassis_controller_info.GetCurrentMovementName() ==
                typeid(EncircleObstacleMovement).name() ||
            chassis_controller_info.GetCurrentMovementName() ==
                typeid(TracePathMovement).name()) {
          if (Time::Now() - last_check_time_ > kCheckDelemmaIntervalSec) {
            if (!CheckIsInDelemma(chassis, world_pose, operation_data)) {
              ZINFO << "Escape from delemma.";
              chassis_controller->ForceStopMovement();
              info_.state_ = kEscapeFromDelemma;
              break;
            }
            last_check_time_ = Time::Now();
          }
          // if (chassis_controller_info.GetIsSteppedOnPastPath()) {
          //   ZGINFO << "Step on past path.";
          //   chassis_controller->ForceStopMovement();
          //   if (!CheckIsInDelemma(chassis, world_pose, operation_data)) {
          //     info_.state_ = kEscapeFromDelemma;
          //     break;
          //   }
          // }
        }
      }
      break;
    }
    case kEscapeFromDelemma:
    case kNotInDelemma: {
      if (CheckIsInDelemma(chassis, world_pose, operation_data)) {
        info_.state_ = kInDelemma;
      }
      break;
    }
    case kTimeout: {
      break;
    }
    default: {
      ZERROR << "Should never run here.";
      return false;
    }
  }

  return true;
}

MapCellPath DelemmaHandler::GetCurrentPath() const {
  ReadLocker lock(access_);

  MapCellPath path;
  return path;
}

bool DelemmaHandler::CheckIsInDelemma(
    const Chassis::SPtr& chassis, const MapPoint& world_pose,
    const OperationData::SPtr& operation_data) {
  auto nav_map = operation_data->GetNavMapRef();
  MapPointPath output_path;
  MapCellPath output_cell_path;
  bool consider_slam_map = operation_data->GetOperationType() ==
                           OperationData::OperationType::kAllHouseScanning;
  ZGERROR << "Consider slam map " << consider_slam_map;
  auto selected_rooms = !operation_data->GetSelectedRoomsInfo().empty();
  if (operation_data->GetOperationType() ==
      OperationData::OperationType::kAllHouseScanning) {
    if (return_home_planner_.GeneratePathToStartPoint(
            nav_map, world_pose, {operation_data->GetStartPoint()}, output_path,
            output_cell_path, consider_slam_map) ||
        quick_scan_house_planner_.GeneratePath(nav_map, world_pose, output_path,
                                               output_cell_path)) {
      ZGINFO << "Robot has escape from delemma.";
      return false;
    }
  } else {
    if (return_home_planner_.GeneratePathToStartPoint(
            nav_map, world_pose, {operation_data->GetStartPoint()}, output_path,
            output_cell_path, consider_slam_map) ||
        switch_section_planner_.GeneratePath(
            nav_map, world_pose,
            operation_data->GetCurrentRoomInfoRef()->GetRoomIndex(),
            output_path, output_cell_path) ||
        switch_room_planner_.GeneratePathToNearestRoom(
            nav_map, world_pose, output_path, output_cell_path,
            operation_data->GetRoomsInfo(), selected_rooms)) {
      ZGINFO << "Robot has escape from delemma.";
      return false;
    }
  }

  ZGINFO << "Robot still in delemma.";
  return true;
}

DelemmaHandler::DelemmaHandlerInfo DelemmaHandler::GetDelemmaHandlerInfo() {
  ReadLocker lock(access_);
  return info_;
}

}  // namespace zima
