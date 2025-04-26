/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_SWITCH_ROOM_SECTION_PLANNER_H
#define ZIMA_SWITCH_ROOM_SECTION_PLANNER_H

#include "zima/path_planner/path_planner.h"
#include "zima/robot_data/nav_data.h"

namespace zima {

class SwitchSectionPlanner : public PlannerBase {
 public:
  SwitchSectionPlanner() = default;
  ~SwitchSectionPlanner() = default;

  /**
   * @brief  Interface for generating path to next section.
   *
   * @param  map Multi-layers map shared pointer.
   * @param  start_pose Start(current) pose for robot.
   * @param  current_room_index Index for current cleaning room.
   * @param  output_path Output path, only valid if function returns true.
   * @param  output_cell_path Output path in cells, only valid if function
   * returns true.
   *
   * @return True for generate path successfully.
   */
  bool GeneratePath(NavMap::SPtr map, const MapPoint& start_pose,
                    const CharGridMap2D::DataType& current_room_index,
                    MapPointPath& output_path, MapCellPath& output_cell_path,
                    const bool& consider_slam_map = true);
};

class SwitchRoomPlanner : public PlannerBase {
 public:
  SwitchRoomPlanner() = default;
  ~SwitchRoomPlanner() = default;

  /**
   * @brief  Interface for generating path to specified room.
   *
   * @param  map Multi-layers map shared pointer.
   * @param  start_pose Start(current) pose for robot.
   * @param  next_room_index Index for next cleaning room.
   * @param  output_path Output path, only valid if function returns true.
   * @param  output_cell_path Output path in cells, only valid if function
   * returns true.
   * @param  consider_slam_map True for not going into slam unknown area and
   * slam blocks.
   *
   * @return True for generate path successfully.
   */
  bool GeneratePathToSpecificRoom(
      NavMap::SPtr map, const MapPoint& start_pose,
      const CharGridMap2D::DataType& next_room_index, MapPointPath& output_path,
      MapCellPath& output_cell_path, const bool& consider_slam_map = true);

  /**
   * @brief  Interface for generating path to nearest uncleaned room.
   *
   * @param  map Multi-layers map shared pointer.
   * @param  start_pose Start(current) pose for robot.
   * @param  output_path Output path, only valid if function returns true.
   * @param  output_cell_path Output path in cells, only valid if function
   * returns true.
   * @param  rooms_info All rooms info.
   * @param  consider_slam_map True for not going into slam unknown area and
   * slam blocks.
   *
   * @return True for generate path successfully.
   */
  bool GeneratePathToNearestRoom(NavMap::SPtr map, const MapPoint& start_pose,
                                 MapPointPath& output_path,
                                 MapCellPath& output_cell_path,
                                 const RoomsInfo& rooms_info,
                                 const bool& consider_slam_map = true);
};

}  // namespace zima

#endif  // ZIMA_SWITCH_ROOM_SECTION_PLANNER_H
