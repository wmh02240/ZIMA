/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_QUICK_SCAN_HOUSE_PATH_PLANNER_H
#define ZIMA_QUICK_SCAN_HOUSE_PATH_PLANNER_H

#include "zima/path_planner/path_planner.h"

namespace zima {

class QuickScanHousePlanner : public PlannerBase {
 public:
  QuickScanHousePlanner() = default;
  ~QuickScanHousePlanner() = default;

  /**
   * @brief  Interface for generating path for zigzag path cleaning.
   *
   * @param  map Multi-layers map shared pointer.
   * @param  start_pose Start(current) pose for robot.
   * @param  output_path Output path, only valid if function returns true.
   *
   * @return True for generate path successfully.
   */
  bool GeneratePath(NavMap::SPtr map, const MapPoint& start_pose,
                    MapPointPath& output_path, MapCellPath& output_cell_path);
};

}  // namespace zima

#endif  // ZIMA_QUICK_SCAN_HOUSE_PATH_PLANNER_H
