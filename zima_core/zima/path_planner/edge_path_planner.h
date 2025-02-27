/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_EDGE_PATH_PLANNER_H
#define ZIMA_EDGE_PATH_PLANNER_H

#include "zima/path_planner/path_planner.h"

namespace zima {

class EdgeCleaningPlanner : public PlannerBase {
 public:
  EdgeCleaningPlanner() = default;
  ~EdgeCleaningPlanner() = default;

  /**
   * @brief  Interface for generating path for edge cleaning.
   *
   * @param  map Multi-layers map shared pointer.
   * @param  section_map Multi-layers section map shared pointer.
   * @param  start_pose Start(current) pose for robot.
   * @param  bound Bound limit for cleaning section.
   * @param  current_room_index Index for current cleaning room.
   * @param  output_path Output path, only valid if function returns true.
   * @param  wall_sensor_on_left Indicator for encircle obs direction.
   *
   * @return True for generate path successfully.
   */
  bool GeneratePath(NavMap::SPtr map, NavMap::SPtr section_map,
                    const MapPoint& start_pose,
                    const DynamicMapCellBound& bound,
                    const CharGridMap2D::DataType& current_room_index,
                    MapPointPath& output_path, MapCellPath& output_cell_path,
                    const bool& wall_sensor_on_left);

 private:
  bool ExpendAlongBound(const bool& wall_sensor_on_left,
                        const MapCell& curr_step, const MapCell& next_step,
                        const DynamicMapCellBound& bound);

  // Method 1.
  // bool HandleOnBound(NavMap::SPtr process_map, const MapCell& start_cell,
  //                    const DynamicMapCellBound& bound,
  //                    MapPointPath& output_path, MapCellPath&
  //                    output_cell_path, const bool& wall_sensor_on_left);
  // bool FindPathToBound(NavMap::SPtr process_map, const MapCell& start_cell,
  //                      const DynamicMapCellBound& bound,
  //                      const CharGridMap2D::DataType& current_room_index,
  //                      MapPointPath& output_path,
  //                      MapCellPath& output_cell_path);

  // Method 2.
  MapCells GetUnsortedBoundTargets(
      NavMap::SPtr process_map, const MapCell& start_cell,
      const DynamicMapCellBound& bound,
      const CharGridMap2D::DataType& current_room_index);

  bool GetAlongBoundPath(NavMap::SPtr process_map, const MapCell& start_cell,
                         const CharGridMap2D::SPtr& target_mark_map,
                         const DynamicMapCellBound& bound,
                         MapPointPath& output_path,
                         MapCellPath& output_cell_path,
                         const bool& wall_sensor_on_left);

  bool GetBoundHeadOfTarget(NavMap::SPtr process_map, const MapCell& start_cell,
                            const CharGridMap2D::SPtr& target_mark_map,
                            const DynamicMapCellBound& bound,
                            MapCell& bound_head,
                            const bool& wall_sensor_on_left);

  bool ShortestPathToBound(NavMap::SPtr process_map, const MapCell& start_cell,
                           const DynamicMapCellBound& bound,
                           const CharGridMap2D::DataType& current_room_index,
                           MapPointPath& output_path,
                           MapCellPath& output_cell_path);
  bool ShortestPathToTarget(NavMap::SPtr process_map, const MapCell& start_cell,
                            const MapCell& target_cell,
                            const DynamicMapCellBound& bound,
                            const CharGridMap2D::DataType& current_room_index,
                            MapPointPath& output_path,
                            MapCellPath& output_cell_path,
                            bool& should_go_shortest_path);
  MapPointPath MovePathToExactlyBoundary(NavMap::SPtr map,
                                         const MapCellPath& output_cell_path,
                                         const DynamicMapCellBound& bound);
};

}  // namespace zima

#endif  // ZIMA_EDGE_PATH_PLANNER_H
