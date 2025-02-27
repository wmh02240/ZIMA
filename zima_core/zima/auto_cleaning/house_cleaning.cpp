/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/auto_cleaning/house_cleaning.h"

#include <algorithm>

#include "zima/common/time.h"
#include "zima/common/util.h"
#include "zima/logger/logger.h"

namespace zima {

HouseCleaning::HouseCleaningInfo::HouseCleaningInfo(const RoomsInfo& rooms_info)
    : state_(kSwitchRoom),
      rooms_info_(rooms_info),
      current_movement_name_(""),
      current_cell_path_({}),
      current_world_pose_(nullptr),
      room_cleaning_info_(nullptr) {}

HouseCleaning::HouseCleaningInfo::HouseCleaningInfo(
    const HouseCleaningInfo& ref)
    : state_(kSwitchRoom),
      rooms_info_(ref.rooms_info_),
      current_movement_name_(ref.current_movement_name_),
      current_cell_path_(ref.current_cell_path_),
      current_world_pose_(nullptr) {
  if (ref.current_world_pose_ != nullptr) {
    current_world_pose_ = std::make_shared<MapPoint>(*ref.current_world_pose_);
  }
  if (ref.room_cleaning_info_ != nullptr) {
    room_cleaning_info_ = std::make_shared<RoomCleaning::RoomCleaningInfo>(
        *ref.room_cleaning_info_);
  }
}

HouseCleaning::HouseCleaning(
    const RoomsInfo& rooms_info, const bool& consider_slam_map,
    const bool& selected_rooms,
    const RoomCleaning::RoomCleaningInfo::SPtr& cached_room_cleaning_info)
    : info_(rooms_info),
      cached_room_cleaning_info_(cached_room_cleaning_info),
      room_cleaning_(nullptr),
      next_movement_(nullptr),
      access_(std::make_shared<ReadWriteLock>()),
      consider_slam_map_(consider_slam_map),
      selected_rooms_(selected_rooms),
      default_room_info_(std::make_shared<RoomInfo>()) {
  if (info_.rooms_info_.empty()) {
    ZGINFO << "No room list.";
  }
  if (selected_rooms_) {
    ZGINFO << "Clean for selected rooms in house.";
    for (auto&& room_info : info_.rooms_info_) {
      ZGINFO << "Clean for room " << room_info.first;
    }
  }
  ZGINFO << "Consider slam map " << consider_slam_map_;

  if (cached_room_cleaning_info_ != nullptr) {
    ZGINFO << "Has cached room cleaning info.";
  }
}

HouseCleaning::~HouseCleaning() {}

bool HouseCleaning::Pause(const Chassis::SPtr& chassis,
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
    case kRoomCleaning: {
      if (room_cleaning_ == nullptr) {
        ZERROR << "Ptr empty, it should never run here.";
        break;
      }
      room_cleaning_->Pause(chassis, chassis_controller, nav_map);
      break;
    }
    case kSwitchRoom:
    default: {
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

          DynamicMapCellBound infinity_bound(INT_MIN, INT_MAX, INT_MIN,
                                             INT_MAX);
          nav_map->MarkForPassingThroughSteps(*TransformManager::Instance(),
                                              last_movement->GetSteps(),
                                              infinity_bound);
          // nav_map->GetPrintLayer()->Print(__FILE__, __FUNCTION__, __LINE__);
          break;
        }
        Time::SleepMSec(5);
      }

      break;
    }
  }

  ZGINFO << "Exec pause finish.";
  return true;
}

bool HouseCleaning::ChassisSupervise(
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

  auto switch_to_room_cleaning = [&](const MapCell& pose_cell) -> void {
    auto room_map = nav_map->GetRoomLayer();
    ReadLocker lock(room_map->GetLock());
    CharGridMap2D::DataType value;
    room_map->GetValue(pose_cell.X(), pose_cell.Y(), value);
    ZGINFO << "Room value of " << pose_cell.DebugString() << " is \"" << value
           << "\"";
    if (value != NavMap::kUnknown_) {
      auto it = info_.rooms_info_.find(value);
      if (it != info_.rooms_info_.end()) {
        ZGINFO;
        room_cleaning_.reset(new RoomCleaning(it->second, consider_slam_map_));
      } else {
        if (!info_.rooms_info_.empty()) {
          ZGERROR << "Unknown room: " << value << ", not in room list.";
        }
        ZGINFO;
        room_cleaning_.reset(
            new RoomCleaning(default_room_info_, consider_slam_map_));
      }
    } else {
      ZGINFO;
      room_cleaning_.reset(
          new RoomCleaning(default_room_info_, consider_slam_map_));
    }
    ZGINFO << "Switch to room cleaning.";
    info_.state_ = kRoomCleaning;
    info_.current_cell_path_.clear();
    info_.current_movement_name_ = "";
  };

  info_.current_world_pose_ =
      std::make_shared<MapPoint>(chassis_controller_info.GetWorldPose());

  CheckCachedRoomCleaningInfo(nav_map);

  switch (info_.state_) {
    case kSwitchRoom: {
      if (chassis_controller_info.GetControllerState() ==
              ChassisController::InfoWrapper::ControllerState::kStop ||
          chassis_controller_info.GetControllerState() ==
              ChassisController::InfoWrapper::ControllerState::kStandby) {
        chassis_controller->UpdateCachedNavMap(nav_map);
        ChassisController::InfoWrapper info;
        chassis_controller->ExtractInfo(info);
        auto world_pose = info.GetWorldPose();

        auto last_movement = info.GetLastMovement();
        ZGINFO << "switching_room " << switching_room_.load();
        if (last_movement == nullptr || switching_room_.load()) {
          MapPointPath output_path;
          MapCellPath output_cell_path;
          switching_room_.store(false);
          if (!selected_rooms_ &&
              switch_room_planner_.GeneratePathToNearestRoom(
                  nav_map, world_pose, output_path, output_cell_path, {},
                  consider_slam_map_)) {
            if (output_cell_path.size() == 2 &&
                output_cell_path.front() == output_cell_path.back()) {
              switch_to_room_cleaning(output_cell_path.front());
            } else {
              ZGINFO << "Move to next room.";
              info_.current_cell_path_ = output_cell_path;
              next_movement_.reset(
                  new TracePathMovement(chassis, output_path, nav_map));
              info_.current_movement_name_ = next_movement_->Name();
              chassis_controller->SetPlan(next_movement_);
            }
            return true;
          } else if (selected_rooms_ &&
                     switch_room_planner_.GeneratePathToNearestRoom(
                         nav_map, world_pose, output_path, output_cell_path,
                         info_.rooms_info_, consider_slam_map_)) {
            if (output_cell_path.size() == 2 &&
                output_cell_path.front() == output_cell_path.back()) {
              switch_to_room_cleaning(output_cell_path.front());
            } else {
              ZGINFO << "Move to next room.";
              info_.current_cell_path_ = output_cell_path;
              next_movement_.reset(
                  new TracePathMovement(chassis, output_path, nav_map));
              info_.current_movement_name_ = next_movement_->Name();
              chassis_controller->SetPlan(next_movement_);
            }
            return true;
          } else {
            ZGINFO << "house cleaning finished.";
            chassis_controller->ForceStopMovement();
            info_.state_ = kFinished;
            return true;
          }
        } else {
          auto movement_state = last_movement->GetState();
          ZGINFO << "TP movement, state: " << movement_state;
          switch (movement_state) {
            case MotionBase::State::kReachTarget:
            case MotionBase::State::kStop: {
              ZGINFO << "End tp movement, state: " << movement_state;
              DynamicMapCellBound infinity_bound(INT_MIN, INT_MAX, INT_MIN,
                                                 INT_MAX);
              nav_map->MarkForPassingThroughSteps(*TransformManager::Instance(),
                                                  last_movement->GetSteps(),
                                                  infinity_bound);
              // nav_map->GetPrintLayer()->Print(__FILE__, __FUNCTION__, __LINE__);
              // Need to reset for regenerate path.
              if (movement_state == MotionBase::State::kReachTarget) {
                // Generate new bound for current pose.

                MapCell pose_cell;
                if (!nav_map->GetFootStepLayer()->WorldToMap(world_pose,
                                                             pose_cell)) {
                  ZERROR << world_pose
                         << " is not in foot step map available range.";
                  info_.state_ = kFinished;
                  break;
                }
                switch_to_room_cleaning(pose_cell);
              } else {
                switching_room_.store(true);
              }
              break;
            }
            case MotionBase::State::kError:
            case MotionBase::State::kException:
            case MotionBase::State::kTimeout: {
              ZGERROR << "End tp movement, state: " << movement_state;
              switching_room_.store(true);
              break;
            }
            default: {
              ZERROR << "Should never run here.";
              switching_room_.store(true);
              break;
            }
          }
        }
      } else if (chassis_controller_info.GetControllerState() ==
                 ChassisController::InfoWrapper::ControllerState::kRunning) {
        if (info_.current_cell_path_.size() !=
            chassis_controller_info.GetTargetsLeft()) {
          // ZINFO << "Update targets left.";
          Steps steps;
          MapPointPath path;
          if (chassis_controller->GetCurrentPathExecutionInfo(steps, path)) {
            CharGridMap2D::PointPathToCellPath(nav_map->GetFootStepLayer(),
                                               path, info_.current_cell_path_);
          }
        }
      }
      break;
    }
    case kRoomCleaning: {
      auto switch_to_switch_room_state = [&]() -> void {
        info_.state_ = kSwitchRoom;
        info_.room_cleaning_info_.reset();
        switching_room_.store(true);
      };


      if (room_cleaning_ == nullptr) {
        ZERROR << "Ptr empty, it should never run here.";
        switch_to_switch_room_state();
        break;
      }

      room_cleaning_->ChassisSupervise(chassis, chassis_controller, nav_map);
      if (room_cleaning_->GetState() == RoomCleaning::State::kFinished) {
        ZGINFO << "Finish cleaning current room.";
        switch_to_switch_room_state();
      }
      break;
    }
    case kFinished: {
      ZGINFO << "Room cleaning finished(Maybe trapped)";
      break;
    }
    default: {
      ZERROR << "Should never run here.";
      return false;
    }
  }

  return true;
}

MapCellPath HouseCleaning::GetCurrentPath() const {
  ReadLocker lock(access_);
  switch (info_.state_) {
    case kSwitchRoom: {
      return info_.current_cell_path_;
    }
    case kRoomCleaning: {
      if (room_cleaning_ != nullptr) {
        return room_cleaning_->GetCurrentPath();
      }
      break;
    }
    default: {
      break;
    }
  }
  MapCellPath path;
  return path;
}

HouseCleaning::HouseCleaningInfo HouseCleaning::GetCleaningInfo() {
  ReadLocker lock(access_);

  // Get room cleaning info only when house cleaning info is needed.
  switch (info_.state_) {
    case kRoomCleaning: {
      if (room_cleaning_ != nullptr) {
        info_.room_cleaning_info_ =
            std::make_shared<RoomCleaning::RoomCleaningInfo>(
                room_cleaning_->GetCleaningInfo());
      }
      break;
    }
    default: {
      break;
    }
  }

  return info_;
}

void HouseCleaning::CheckCachedRoomCleaningInfo(const NavMap::SPtr& nav_map) {
  if (cached_room_cleaning_info_ != nullptr &&
      cached_room_cleaning_info_->section_cleaning_info_ != nullptr) {
    auto it = info_.rooms_info_.find(
        cached_room_cleaning_info_->room_info_->GetRoomIndex());
    if (it != info_.rooms_info_.end()) {
      ZGINFO << "Switch to room cleaning for cached info.";
      room_cleaning_.reset(
          new RoomCleaning(it->second, consider_slam_map_,
                           cached_room_cleaning_info_->section_cleaning_info_));
      info_.state_ = kRoomCleaning;
      info_.current_cell_path_.clear();
      info_.current_movement_name_ = "";
    } else {
      ZGWARN << "Cached room "
             << cached_room_cleaning_info_->room_info_->GetRoomIndex()
             << " not in room list.";
    }
    cached_room_cleaning_info_.reset();
  }
}

}  // namespace zima
