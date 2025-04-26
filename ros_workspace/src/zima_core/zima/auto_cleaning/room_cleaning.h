/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_ROOM_CLEANING_H
#define ZIMA_ROOM_CLEANING_H

#include <atomic>
#include <memory>

#include "zima/auto_cleaning/section_cleaning.h"
#include "zima/movement/trace_path_movement.h"
#include "zima/path_planner/switch_room_section_planner.h"
#include "zima/robot/chassis_controller.h"
#include "zima/robot_data/nav_data.h"

namespace zima {

class RoomCleaning {
 public:
  RoomCleaning() = delete;
  RoomCleaning(const RoomInfo::SPtr& room_info,
               const bool& consider_slam_map = false,
               const SectionCleaning::SectionCleaningInfo::SPtr&
                   cached_section_cleaning_info = nullptr);
  ~RoomCleaning();

  using UPtr = std::unique_ptr<RoomCleaning>;

  enum State {
    kSectionCleaning,
    kSwitchSection,
    kFinished,
  };

  class RoomCleaningInfo {
   public:
    RoomCleaningInfo() = delete;
    RoomCleaningInfo(const RoomCleaningInfo& ref);
    RoomCleaningInfo(const RoomInfo::SPtr& room_info);
    ~RoomCleaningInfo() = default;

    using SPtr = std::shared_ptr<RoomCleaningInfo>;

    State state_;
    RoomInfo::SPtr room_info_;
    std::string current_movement_name_;
    MapCellPath current_cell_path_;
    MapPoint::SPtr current_world_pose_;

    SectionCleaning::SectionCleaningInfo::SPtr section_cleaning_info_;
  };

  bool Pause(const Chassis::SPtr& chassis,
             const ChassisController::SPtr& chassis_controller,
             const NavMap::SPtr& nav_map);

  bool ChassisSupervise(const Chassis::SPtr& chassis,
                        const ChassisController::SPtr& chassis_controller,
                        const NavMap::SPtr& nav_map);

  State GetState() const {
    ReadLocker lock(access_);
    return info_.state_;
  }

  MapCellPath GetCurrentPath() const;

  RoomCleaningInfo GetCleaningInfo();

 protected:
  void CheckCachedSectionCleaningInfo(const NavMap::SPtr& nav_map);

  RoomCleaningInfo info_;
  SectionCleaning::SectionCleaningInfo::SPtr cached_section_cleaning_info_;

  SectionCleaning::UPtr section_cleaning_;
  MovementBase::UPtr next_movement_;
  atomic_bool go_to_new_section_;
  MapPoint last_check_world_pose_;

  ReadWriteLock::SPtr access_;

  bool consider_slam_map_;

  SwitchSectionPlanner switch_section_planner_;
};

}  // namespace zima

#endif  // ZIMA_ROOM_CLEANING_H
