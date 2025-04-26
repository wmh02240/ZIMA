/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_RETURN_HOME_PLANNER_H
#define ZIMA_RETURN_HOME_PLANNER_H

#include "zima/path_planner/path_planner.h"

namespace zima {

class ReturnHomePlanner : public PlannerBase {
 public:
  ReturnHomePlanner() = default;
  ~ReturnHomePlanner() = default;

  /**
   * @brief  Interface for generating path for zigzag path cleaning.
   *
   * @param  map Multi-layers map shared pointer.
   * @param  start_pose Start(current) pose for robot.
   * @param  station_points Points of home station.
   * @param  output_path Output path, only valid if function returns true.
   * @param  output_cell_path Output cell path, only valid if function returns
   * true.
   *
   * @return True for generate path successfully.
   */
  bool GeneratePathToStation(NavMap::SPtr map, const MapPoint& start_pose,
                             const MapPoints& station_points,
                             MapPointPath& output_path,
                             MapCellPath& output_cell_path);

  /**
   * @brief  Interface for generating path for zigzag path cleaning.
   *
   * @param  map Multi-layers map shared pointer.
   * @param  start_pose Start(current) pose for robot.
   * @param  start_points Points of cleaning start point, maybe contains points
   * near it.
   * @param  output_path Output path, only valid if function returns true.
   * @param  output_cell_path Output cell path, only valid if function returns
   * true.
   *
   * @return True for generate path successfully.
   */
  bool GeneratePathToStartPoint(NavMap::SPtr map, const MapPoint& start_pose,
                                const MapPoints& start_points,
                                MapPointPath& output_path,
                                MapCellPath& output_cell_path,
                                const bool& consider_slam_map = false);
};

}  // namespace zima

#endif  // ZIMA_RETURN_HOME_PLANNER_H
