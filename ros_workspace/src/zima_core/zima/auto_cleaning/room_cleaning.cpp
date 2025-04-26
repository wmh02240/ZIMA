/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/auto_cleaning/room_cleaning.h"

#include <algorithm>

#include "zima/common/time.h"
#include "zima/common/util.h"
#include "zima/logger/logger.h"

namespace zima {

RoomCleaning::RoomCleaningInfo::RoomCleaningInfo(
    const RoomInfo::SPtr& room_info)
    : state_(kSwitchSection),
      room_info_(room_info),
      current_movement_name_(""),
      current_cell_path_({}),
      current_world_pose_(nullptr),
      section_cleaning_info_(nullptr) {}

RoomCleaning::RoomCleaningInfo::RoomCleaningInfo(const RoomCleaningInfo& ref)
    : state_(kSwitchSection),
      room_info_(ref.room_info_),
      current_movement_name_(ref.current_movement_name_),
      current_cell_path_(ref.current_cell_path_),
      current_world_pose_(nullptr),
      section_cleaning_info_(nullptr) {
  if (ref.current_world_pose_ != nullptr) {
    current_world_pose_ = std::make_shared<MapPoint>(*ref.current_world_pose_);
  }
  if (ref.section_cleaning_info_ != nullptr) {
    section_cleaning_info_ =
        std::make_shared<SectionCleaning::SectionCleaningInfo>(
            *ref.section_cleaning_info_);
  }
}

RoomCleaning::RoomCleaning(const RoomInfo::SPtr& room_info,
                           const bool& consider_slam_map,
                           const SectionCleaning::SectionCleaningInfo::SPtr&
                               cached_section_cleaning_info)
    : info_(room_info),
      cached_section_cleaning_info_(cached_section_cleaning_info),
      section_cleaning_(nullptr),
      next_movement_(nullptr),
      go_to_new_section_(true),
      last_check_world_pose_(0, 0),
      access_(std::make_shared<ReadWriteLock>()),
      consider_slam_map_(consider_slam_map) {
  ZGINFO;
  if (room_info != nullptr) {
    ZGINFO << "Clean for room: \"" << room_info->GetRoomIndex() << "\"";
  }
  if (cached_section_cleaning_info_ != nullptr) {
    ZGINFO << "Has cached section cleaning info.";
  }
}

RoomCleaning::~RoomCleaning() {}

bool RoomCleaning::Pause(const Chassis::SPtr& chassis,
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
    case kSectionCleaning: {
      if (section_cleaning_ == nullptr) {
        ZERROR << "Ptr empty, it should never run here.";
        break;
      }
      section_cleaning_->Pause(chassis, chassis_controller, nav_map);
      break;
    }
    case kSwitchSection:
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

bool RoomCleaning::ChassisSupervise(
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

  auto switch_to_section_cleaning = [&](const MapCell& cell) -> void {
    DynamicMapCellBound bound = info_.room_info_->GetSectionBound(cell);
    section_cleaning_.reset(new SectionCleaning(bound, nav_map));
    ZGINFO << "Switch to section cleaning.";
    ZGINFO << bound.DebugString();
    info_.state_ = kSectionCleaning;
    info_.current_cell_path_.clear();
    info_.current_movement_name_ = "";
  };

  info_.current_world_pose_ =
      std::make_shared<MapPoint>(chassis_controller_info.GetWorldPose());

  CheckCachedSectionCleaningInfo(nav_map);

  switch (info_.state_) {
    case kSwitchSection: {
      if (chassis_controller_info.GetControllerState() ==
          ChassisController::InfoWrapper::ControllerState::kRunning) {
        auto world_pose = chassis_controller_info.GetWorldPose();
        if (info_.room_info_->GetRoomIndex() == NavMap::kUnknown_ &&
            world_pose.Distance(last_check_world_pose_) >
                NavMap::GetResolution()) {
          auto room_map = nav_map->GetRoomLayer();
          ReadLocker lock(room_map->GetLock());
          MapCell pose_cell;
          if (room_map->WorldToMap(world_pose, pose_cell)) {
            CharGridMap2D::DataType value;
            room_map->GetValue(pose_cell.X(), pose_cell.Y(), value);
            if (value != NavMap::kUnknown_) {
              info_.room_info_->UpdateRoomIndex(value);
            }
          }
          last_check_world_pose_ = world_pose;
        }
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
      } else if (chassis_controller_info.GetControllerState() ==
                     ChassisController::InfoWrapper::ControllerState::kStop ||
                 chassis_controller_info.GetControllerState() ==
                     ChassisController::InfoWrapper::ControllerState::
                         kStandby) {
        chassis_controller->UpdateCachedNavMap(nav_map);
        ChassisController::InfoWrapper info;
        chassis_controller->ExtractInfo(info);
        auto world_pose = info.GetWorldPose();

        auto last_movement = info.GetLastMovement();
        if (last_movement == nullptr || go_to_new_section_.load()) {
          MapPointPath output_path;
          MapCellPath output_cell_path;
          go_to_new_section_.store(false);
          if (switch_section_planner_.GeneratePath(
                  nav_map, world_pose, info_.room_info_->GetRoomIndex(),
                  output_path, output_cell_path, consider_slam_map_)) {
            if (output_cell_path.size() == 2 &&
                output_cell_path.front() == output_cell_path.back()) {
              switch_to_section_cleaning(output_cell_path.front());
            } else {
              ZGINFO << "Move to next section.";
              info_.current_cell_path_ = output_cell_path;
              chassis_controller->ForceStopMovement();
              next_movement_.reset(
                  new TracePathMovement(chassis, output_path, nav_map,
                                        info_.room_info_->GetRoomIndex()));
              info_.current_movement_name_ = next_movement_->Name();
              chassis_controller->SetPlan(next_movement_);
            }
            return true;
          } else {
            ZGINFO << "Room cleaning finished.";
            chassis_controller->ForceStopMovement();
            info_.state_ = kFinished;
            return true;
          }
        } else {
          auto movement_state = last_movement->GetState();
          // ZINFO << "TP movement, state: " << movement_state;
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

                MapCell cell;
                if (!nav_map->GetFootStepLayer()->WorldToMap(world_pose,
                                                             cell)) {
                  ZERROR << world_pose
                         << " is not in foot step map available range.";
                  break;
                }
                switch_to_section_cleaning(cell);
              } else {
                go_to_new_section_.store(true);
              }
              break;
            }
            case MotionBase::State::kError:
            case MotionBase::State::kException:
            case MotionBase::State::kTimeout: {
              ZGERROR << "End tp movement, state: " << movement_state;
              go_to_new_section_.store(true);
              break;
            }
            default: {
              ZERROR << "Should never run here.";
              go_to_new_section_.store(true);
              break;
            }
          }
        }
      }
      break;
    }
    case kSectionCleaning: {
      auto switch_to_switch_section_state = [&]() -> void {
        info_.state_ = kSwitchSection;
        info_.section_cleaning_info_.reset();
        go_to_new_section_.store(true);
      };

      if (section_cleaning_ == nullptr) {
        ZERROR << "Ptr empty, it should never run here.";
        switch_to_switch_section_state();
        break;
      }

      section_cleaning_->ChassisSupervise(chassis, chassis_controller, nav_map,
                                          info_.room_info_);
      if (section_cleaning_->GetState() == SectionCleaning::State::kFinished) {
        ZGINFO << "Finish cleaning current section.";
        switch_to_switch_section_state();
        break;
      }

      auto world_pose = chassis_controller_info.GetWorldPose();
      if (info_.room_info_->GetRoomIndex() == NavMap::kUnknown_ &&
          world_pose.Distance(last_check_world_pose_) > NavMap::GetResolution()) {
        auto room_map = nav_map->GetRoomLayer();
        ReadLocker lock(room_map->GetLock());
        MapCell pose_cell;
        if (room_map->WorldToMap(world_pose, pose_cell)) {
          CharGridMap2D::DataType value;
          room_map->GetValue(pose_cell.X(), pose_cell.Y(), value);
          if (value != NavMap::kUnknown_) {
            info_.room_info_->UpdateRoomIndex(value);
          }
        }
        last_check_world_pose_ = world_pose;
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

MapCellPath RoomCleaning::GetCurrentPath() const {
  ReadLocker lock(access_);
  switch (info_.state_) {
    case kSwitchSection: {
      return info_.current_cell_path_;
    }
    case kSectionCleaning: {
      if (section_cleaning_ != nullptr) {
        return section_cleaning_->GetCurrentPath();
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

RoomCleaning::RoomCleaningInfo RoomCleaning::GetCleaningInfo() {
  ReadLocker lock(access_);

  // Get section cleaning info only when room cleaning info is needed.
  switch (info_.state_) {
    case kSectionCleaning: {
      if (section_cleaning_ != nullptr) {
        info_.section_cleaning_info_ =
            std::make_shared<SectionCleaning::SectionCleaningInfo>(
                section_cleaning_->GetCleaningInfo());
      }
      break;
    }
    default: {
      break;
    }
  }

  return info_;
}

void RoomCleaning::CheckCachedSectionCleaningInfo(const NavMap::SPtr& nav_map) {
  if (cached_section_cleaning_info_ != nullptr) {
    auto bound = cached_section_cleaning_info_->section_bound_;
    if (bound.GetMin().X() < bound.GetMax().X() &&
        bound.GetMin().Y() < bound.GetMax().Y()) {
      // Valid bound.
      ZGINFO << "Switch to section cleaning for cached info.";
      section_cleaning_.reset(new SectionCleaning(bound, nav_map));
      info_.state_ = kSectionCleaning;
      info_.current_cell_path_.clear();
      info_.current_movement_name_ = "";
    } else {
      ZGWARN << "Invalid bound:" << bound.DebugString();
    }
    cached_section_cleaning_info_.reset();
  }
}

}  // namespace zima
