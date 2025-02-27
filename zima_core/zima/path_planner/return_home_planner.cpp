/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/path_planner/return_home_planner.h"

#include <algorithm>

#include "zima/common/util.h"
#include "zima/logger/logger.h"

namespace zima {

bool ReturnHomePlanner::GeneratePathToStartPoint(
    NavMap::SPtr map, const MapPoint& start_pose, const MapPoints& start_points,
    MapPointPath& output_path, MapCellPath& output_cell_path,
    const bool& consider_slam_map) {
  bool found = false;
  if (start_points.empty()) {
    ZWARN << "Station points invalid.";
    return false;
  }

  MapCell start_cell;
  if (!map->GetFootStepLayer()->WorldToMap(start_pose, start_cell)) {
    ZWARN << "Start pose is not in footprint map.";
    return false;
  }

  // if (!bound.Contain(start_cell)) {
  //   ZWARN << "Start cell " << start_cell << " is not in bound"
  //         << bound.DebugString();
  //   return false;
  // }

  // Copy nav map.
  ReadLocker source_read_lock(map->GetLock());
  auto process_map = std::make_shared<NavMap>(*map);
  // process_map->GetPrintLayer()->Print(__FILE__, __FUNCTION__, __LINE__);
  source_read_lock.Unlock();

  process_map->ClearRobotCoverCells(start_pose);

  auto start_point = start_points.front();
  process_map->ClearRobotCoverCells(start_point);

  // Get all obstacle cells inside bumper map.
  auto sensor_map = process_map->GetSensorLayer();

  ReadLocker sensor_map_lock(sensor_map->GetLock());
  // sensor_map->Print(__FILE__, __FUNCTION__, __LINE__);
  auto user_block_map = process_map->GetUserBlockLayer();
  auto value = user_block_map->GetDefaultValue();
  ReadLocker user_block_map_read_lock(user_block_map->GetLock());
  auto slam_char_map = process_map->GetSlamLayer();
  ReadLocker slam_char_map_read_lock(slam_char_map->GetLock());
  auto footstep_map_max_clean_bound =
      process_map->GetMaxCleanBound(std::make_shared<MapCell>(start_cell));
  ZGINFO << footstep_map_max_clean_bound.DebugString();

  auto data_bound = sensor_map->GetDataBound();
  data_bound.Expend(user_block_map->GetDataBound().GetMin());
  data_bound.Expend(user_block_map->GetDataBound().GetMax());

  sensor_map_lock.Unlock();
  user_block_map_read_lock.Unlock();

  if (!InflactObstacleMap(
          process_map, data_bound, start_cell, NavMap::kRobotCellWidth_2_,
          [](MultiLayersCharGridMap2D::SPtr map, MapCell& curr_step,
             MapCell& next_step) { return true; })) {
    return found;
  }

  slam_char_map_read_lock.Unlock();
  if (consider_slam_map) {
    if (!InflactSlamObstacleMap(
            process_map, data_bound, start_cell, NavMap::kRobotCellWidth_2_ / 2,
            [](MultiLayersCharGridMap2D::SPtr map, MapCell& curr_step,
               MapCell& next_step) { return true; })) {
      return found;
    }
  }

  slam_char_map_read_lock.Lock();
  user_block_map_read_lock.Lock();
  ExpendCondition expend_cond = [&](MultiLayersCharGridMap2D::SPtr _map,
                                    MapCell& curr_step, MapCell& next_step) {
    // ZINFO << "Expend for " << next_step;
    user_block_map->GetValue(next_step.X(), next_step.Y(), value);
    if (value == NavMap::kVirtualWall_ ||
        value == NavMap::kStrictBlockArea_) {
      return false;
    }

    if (!footstep_map_max_clean_bound.Contain(next_step)) {
      return false;
    }

    if (consider_slam_map) {
      slam_char_map->GetValue(curr_step.X(), curr_step.Y(), value);
      if (value == NavMap::kSlamWall_) {
        return false;
      }
    }

    return true;
  };

  if (GeneratePathToTarget(process_map, start_pose, start_point, output_path,
                           output_cell_path, expend_cond)) {
    return true;
  }

  if (FLAGS_debug_enable) {
    process_map->GetPrintLayer()->Print(__FILE__, __FUNCTION__, __LINE__);
  }
  return false;
}

}  // namespace zima
