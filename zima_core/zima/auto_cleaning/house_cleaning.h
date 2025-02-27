/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_HOUSE_CLEANING_H
#define ZIMA_HOUSE_CLEANING_H

#include <atomic>
#include <memory>

#include "zima/auto_cleaning/room_cleaning.h"
#include "zima/movement/trace_path_movement.h"
#include "zima/path_planner/switch_room_section_planner.h"
#include "zima/robot/chassis_controller.h"
#include "zima/robot_data/nav_data.h"

namespace zima {

class HouseCleaning {
 public:
  HouseCleaning() = delete;
  HouseCleaning(const RoomsInfo& rooms_info,
                const bool& consider_slam_map = false,
                const bool& selected_rooms = false,
                const RoomCleaning::RoomCleaningInfo::SPtr&
                    cached_room_cleaning_info = nullptr);
  ~HouseCleaning();

  using SPtr = shared_ptr<HouseCleaning>;

  enum State {
    kRoomCleaning,
    kSwitchRoom,
    kFinished,
  };

  class HouseCleaningInfo {
   public:
    HouseCleaningInfo() = delete;
    HouseCleaningInfo(const HouseCleaningInfo& ref);
    HouseCleaningInfo(const RoomsInfo& rooms_info);
    ~HouseCleaningInfo() = default;

    using SPtr = std::shared_ptr<HouseCleaningInfo>;

    State state_;
    RoomsInfo rooms_info_;
    std::string current_movement_name_;
    MapCellPath current_cell_path_;
    MapPoint::SPtr current_world_pose_;

    RoomCleaning::RoomCleaningInfo::SPtr room_cleaning_info_;
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

  HouseCleaningInfo GetCleaningInfo();

 protected:
  void CheckCachedRoomCleaningInfo(const NavMap::SPtr& nav_map);

  HouseCleaningInfo info_;
  RoomCleaning::RoomCleaningInfo::SPtr cached_room_cleaning_info_;

  RoomCleaning::UPtr room_cleaning_;
  MovementBase::UPtr next_movement_;
  atomic_bool switching_room_;

  ReadWriteLock::SPtr access_;

  bool consider_slam_map_;
  bool selected_rooms_;

  SwitchRoomPlanner switch_room_planner_;

  RoomInfo::SPtr default_room_info_;
};

}  // namespace zima

#endif  // ZIMA_HOUSE_CLEANING_H
