/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_SECTION_CLEANING_H
#define ZIMA_SECTION_CLEANING_H

#include <atomic>
#include <memory>

#include "zima/movement/encircle_map_edge_movement.h"
#include "zima/movement/encircle_obstacle_movement.h"
#include "zima/movement/trace_path_movement.h"
#include "zima/path_planner/edge_path_planner.h"
#include "zima/path_planner/zigzag_path_planner.h"
#include "zima/robot/chassis_controller.h"
#include "zima/robot_data/nav_data.h"

namespace zima {

class SectionCleaning {
 public:
  SectionCleaning() = delete;
  SectionCleaning(const DynamicMapCellBound& bound,
                  const NavMap::SPtr& nav_map);
  ~SectionCleaning();

  using UPtr = std::unique_ptr<SectionCleaning>;

  enum State {
    kEdgeCleaning,
    kZigZagCleaning,
    kFinished,
  };

  class SectionCleaningInfo {
   public:
    SectionCleaningInfo() = delete;
    SectionCleaningInfo(const SectionCleaningInfo& ref);
    SectionCleaningInfo(const DynamicMapCellBound& bound);
    ~SectionCleaningInfo() = default;

    using SPtr = std::shared_ptr<SectionCleaningInfo>;

    State state_;
    DynamicMapCellBound section_bound_;
    DynamicMapCellBound section_real_bound_;
    std::string current_movement_name_;
    MapCellPath current_cell_path_;
    MapPoint::SPtr current_world_pose_;
  };

  bool Pause(const Chassis::SPtr& chassis,
             const ChassisController::SPtr& chassis_controller,
             const NavMap::SPtr& nav_map);

  bool ChassisSupervise(const Chassis::SPtr& chassis,
                        const ChassisController::SPtr& chassis_controller,
                        const NavMap::SPtr& nav_map,
                        const RoomInfo::SPtr& room_info);

  State GetState() const {
    ReadLocker lock(access_);
    return info_.state_;
  }

  MapCellPath GetCurrentPath() const {
    ReadLocker lock(access_);
    return info_.current_cell_path_;
  }

  SectionCleaningInfo GetCleaningInfo() const;

 protected:
  bool GenerateNextMovement(const Chassis::SPtr& chassis,
                            const MapPoint& world_pose,
                            const NavMap::SPtr& nav_map,
                            const RoomInfo::SPtr& room_info,
                            const MapPointPath& encircle_obs_saved_track,
                            MovementBase::UPtr& next_movement);

  bool HandleControllerRunning(
      const State& cleaning_state, const Chassis::SPtr& chassis,
      const ChassisController::SPtr& chassis_controller,
      const ChassisController::InfoWrapper& info,
      const NavMap::SPtr& nav_map, const RoomInfo::SPtr& room_info);
  bool HandleControllerStandbyOrStop(
      const State& cleaning_state, const Chassis::SPtr& chassis,
      const ChassisController::SPtr& chassis_controller,
      const NavMap::SPtr& nav_map, const RoomInfo::SPtr& room_info);

  SectionCleaningInfo info_;
  DynamicMapCellBound section_print_bound_;
  DynamicMapPointBound section_point_bound_;
  atomic_bool generate_path_request_;
  MapPointPath encircle_obs_saved_track_;
  MovementBase::UPtr next_movement_;

  ReadWriteLock::SPtr access_;

  NavMap::SPtr section_nav_map_;
  EdgeCleaningPlanner edge_planner_;
  ZigZagPlannerManager zigzag_planner_;
};

}  // namespace zima

#endif  // ZIMA_SECTION_CLEANING_H
