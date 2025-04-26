/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_PATH_PLANNER_H
#define ZIMA_PATH_PLANNER_H

#include <deque>
#include <memory>

#include "zima/algorithm/dijkstra.h"
#include "zima/grid_map/nav_map_2d.h"

namespace zima {

class PlannerBase : public DijkstraBase {
 public:
  PlannerBase() = default;
  ~PlannerBase() = default;

  /**
   * @brief  Function for generating truely shortest path base on map.
   *
   * @param  map Layered costmap type, contains multi-layers of costmap_2d.
   * @param  input_cell_path Input CELL path.
   * @param  output_point_path Output POINT path.
   *
   * @return True for generate path successfully.
   */

  bool OptimizePathToShortest(
      NavMap::SPtr map, const MapCells& input_cell_path,
      MapPointPath& output_point_path, MapCellPath& output_cell_path,
      const CharGridMap2D::DataType& current_room_index = NavMap::kUnknown_,
      const bool& consider_slam_map = false);

  /**
   * @brief  Interface for generating path to input target.
   *
   * @param  process_map Multi-layers map shared pointer, normally obs in it
   * should already be inflaceted.
   * @param  start_pose Start(current) pose for robot.
   * @param  output_path Output path, only valid if function returns true.
   *
   * @return True for generate path successfully.
   */
  bool GeneratePathToTarget(
      NavMap::SPtr process_map, const MapPoint& start_pose,
      const MapPoint& target_point, MapPointPath& output_path,
      MapCellPath& output_cell_path,
      const ExpendCondition& additional_expend_cond =
          [](MultiLayersCharGridMap2D::SPtr map, MapCell& curr_step,
             MapCell& next_step) { return true; });

 protected:
  /**
   * @brief  Function for inflating obstacles in sensor / user block layers of
   * map.
   *
   * @param  map Layered costmap type, contains multi-layers of costmap_2d.
   * @param  bound Bound limit for obstacle checking.
   * @param  start_cell .
   * @param  expend_cond The judgement function for deciding whether the search
   * step is qualified.
   *
   * @return True for generate path successfully.
   */
  bool InflactObstacleMap(NavMap::SPtr process_map,
                          const DynamicMapCellBound& bound,
                          const MapCell& start_cell,
                          const uint16_t& inflate_max_iteration_count,
                          const ExpendCondition& extra_expend_cond,
                          const bool& enable_square_inflaction = true,
                          const bool& print_result = false);

  /**
   * @brief  Function for inflating obstacles in sensor / user block layers of
   * map.
   *
   * @param  map Layered costmap type, contains multi-layers of costmap_2d.
   * @param  bound Bound limit for obstacle checking.
   * @param  start_cell .
   * @param  expend_cond The judgement function for deciding whether the search
   * step is qualified.
   *
   * @return True for generate path successfully.
   */
  bool InflactSlamObstacleMap(NavMap::SPtr process_map,
                              const DynamicMapCellBound& bound,
                              const MapCell& start_cell,
                              const uint16_t& inflate_max_iteration_count,
                              const ExpendCondition& extra_expend_cond,
                              const bool& enable_square_inflaction = true,
                              const bool& print_result = false);

  /**
   * @brief  Function for inflating obstacles in ONE layer of map.
   *
   * @param  map Layered costmap type, contains multi-layers of costmap_2d.
   * @param  map_type The layer indicator of map.
   * @param  inflate_max_iteration_count Max iteration count for obstacle
   * inflation.
   * @param  start_cells All original obstacle cells in that layer.
   * @param  expend_cond The judgement function for deciding whether the search
   * step is qualified.
   *
   * @return True for generate path successfully.
   */
  bool InflateObstaclesInMap(NavMap::SPtr map, const std::string& map_type,
                             const uint16_t& inflate_max_iteration_count,
                             const MapCells& start_cells,
                             ExpendCondition& expend_cond,
                             const bool& enable_square_inflaction = true);

  void PrintPathInMap(NavMap::SPtr map, const MapCellPath& path);
  void PrintPathInMap(NavMap::SPtr map, const DynamicMapCellBound& print_bound,
                      const MapCellPath& path);

  void PrintPathInMap(NavMap::SPtr map, const MapPointPath& path);
  void PrintPathInMap(NavMap::SPtr map, const DynamicMapCellBound& print_bound,
                      const MapPointPath& path);
};

}  // namespace zima

#endif  // ZIMA_PATH_PLANNER_H
