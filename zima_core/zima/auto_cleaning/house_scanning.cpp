/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/auto_cleaning/house_scanning.h"

#include <algorithm>

#include "zima/common/time.h"
#include "zima/common/util.h"
#include "zima/logger/logger.h"

namespace zima {

HouseScanning::HouseScanning()
    : state_(kHouseScanning),
      shutdown_planner_thread_(false),
      generate_path_request_(true),
      movement_(nullptr),
      next_movement_(nullptr),
      access_(std::make_shared<ReadWriteLock>()),
      current_path_({}) {
  ZGINFO << "Start house scanning.";
}

HouseScanning::~HouseScanning() {
  WriteLocker lock(access_);
  movement_.reset();
}

bool HouseScanning::Pause(const Chassis::SPtr& chassis,
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

  switch (state_) {
    case kHouseScanning: {
      while (true) {
        // Stop current movement.
        chassis_controller->ForceStopMovement();
        ChassisController::InfoWrapper info;
        chassis_controller->GlanceInfo(info);
        if (info.GetControllerState() ==
                ChassisController::InfoWrapper::ControllerState::kStandby ||
            info.GetControllerState() ==
                ChassisController::InfoWrapper::ControllerState::kStop) {
          chassis_controller->ExtractInfo(info);
          auto last_movement = info.GetLastMovement();
          if (last_movement == nullptr) {
            ZINFO << "Chassis controller standby state without movement.";
            break;
          }
          if (nav_map->GetSlamLayer()->IsMarked()) {
            nav_map->MarkForPassingThroughSteps(
                *TransformManager::Instance(), last_movement->GetSteps(),
                nav_map->GetFootStepLayer()->GetAvailableBound());
          } else {
            nav_map->MarkForCleaningSteps(
                *TransformManager::Instance(), last_movement->GetSteps(),
                nav_map->GetFootStepLayer()->GetAvailableBound());
          }
          // nav_map->GetPrintLayer()->Print(__FILE__, __FUNCTION__, __LINE__);
          break;
        }
        Time::SleepMSec(5);
      }
      default: {
        // Do nothing.
        break;
      }
    }
  }

  ZGINFO << "Exec pause finish.";
  return true;
}

bool HouseScanning::GenerateNextMovement(const Chassis::SPtr& chassis,
                                         const MapPoint& world_pose,
                                         const NavMap::SPtr& nav_map,
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

  state_ = kHouseScanning;

  while (generate_path_request_.load()) {
    auto state = state_;
    switch (state) {
      case kHouseScanning: {
        MapPointPath output_path;
        MapCellPath output_cell_path;
        if (quick_scan_house_planner_.GeneratePath(
                nav_map, world_pose, output_path, output_cell_path)) {
          ZGINFO << "Continue scan.";
          current_path_ = output_cell_path;
          next_movement.reset(
              new TracePathMovement(chassis, output_path, nav_map));
          generate_path_request_.store(false);
          ret = true;
          break;
        }

        state_ = kFinished;
        break;
      }
      default: {
        return ret;
      }
    }
  }

  return ret;
}

bool HouseScanning::ChassisSupervise(
    const Chassis::SPtr& chassis,
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

  ChassisController::InfoWrapper chassis_controller_info;
  chassis_controller->GlanceInfo(chassis_controller_info);
  // ZINFO << chassis_controller_info.DebugString();
  if (generate_path_request_.load()) {
    ZGINFO << "World pose: " << chassis_controller_info.GetWorldPose();
    if (!GenerateNextMovement(chassis, chassis_controller_info.GetWorldPose(),
                              nav_map, next_movement_)) {
      ZINFO << "House scanning finished.";
      chassis_controller->ForceStopMovement();
      state_ = kFinished;
      return true;
    }

    if (next_movement_ == nullptr) {
      ZERROR << "No movement generated.";
      chassis_controller->ForceStopMovement();
      state_ = kFinished;
      return false;
    }
  }

  if (next_movement_ != nullptr) {
    chassis_controller->UpdateCachedNavMap(nav_map);
    chassis_controller->ForceStopMovement();
    chassis_controller->SetPlan(next_movement_);
    chassis_controller->GlanceInfo(chassis_controller_info);
  }

  switch (state_) {
    case kHouseScanning: {
      if (chassis_controller_info.GetControllerState() ==
          ChassisController::InfoWrapper::ControllerState::kStandby) {
        ChassisController::InfoWrapper info;
        chassis_controller->ExtractInfo(info);
        auto last_movement = info.GetLastMovement();
        if (info.GetControllerState() !=
            ChassisController::InfoWrapper::ControllerState::kStandby) {
          ZERROR << "Chassis controller is not in standby state.";
          return false;
        }
        if (last_movement == nullptr) {
          ZERROR << "Chassis controller standby state without movement.";
          return false;
        }
        auto tp = dynamic_cast<TracePathMovement*>(last_movement.get());

        auto mark_for_step = [&](const Steps& steps) -> void {
          if (nav_map->GetSlamLayer()->IsMarked()) {
            nav_map->MarkForPassingThroughSteps(
                *TransformManager::Instance(), last_movement->GetSteps(),
                nav_map->GetFootStepLayer()->GetAvailableBound());
          } else {
            nav_map->MarkForCleaningSteps(
                *TransformManager::Instance(), last_movement->GetSteps(),
                nav_map->GetFootStepLayer()->GetAvailableBound());
          }
          // nav_map->GetPrintLayer()->Print(__FILE__, __FUNCTION__, __LINE__);
        };

        auto movement_state = tp->GetState();
        switch (movement_state) {
          case MotionBase::State::kReachTarget:
          case MotionBase::State::kStop: {
            // ZINFO << "End tp movement, state: " << movement_state;
            mark_for_step(tp->GetSteps());
            generate_path_request_.store(true);
            break;
          }
          case MotionBase::State::kError:
          case MotionBase::State::kException:
          case MotionBase::State::kTimeout: {
            ZGERROR << "End tp movement, state: " << movement_state;

            mark_for_step(tp->GetSteps());
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

}  // namespace zima
