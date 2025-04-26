/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/path_planner/edge_path_planner.h"

#include <algorithm>

#include "zima/common/util.h"
#include "zima/logger/logger.h"

namespace zima {

bool EdgeCleaningPlanner::GeneratePath(
    NavMap::SPtr map, NavMap::SPtr section_map, const MapPoint& start_pose,
    const DynamicMapCellBound& bound,
    const CharGridMap2D::DataType& current_room_index,
    MapPointPath& output_path, MapCellPath& output_cell_path,
    const bool& wall_sensor_on_left) {
  bool found = false;
  MapCell start_cell;
  if (!map->GetFootStepLayer()->WorldToMap(start_pose, start_cell)) {
    ZWARN << "Start pose is not in footprint map.";
    return false;
  }
  ZGINFO << "Generate path for " << bound.DebugString() << ", start cell "
         << start_cell.DebugString();

  // if (!bound.Contain(start_cell)) {
  //   ZWARN << "Start cell " << start_cell << " is not in bound"
  //         << bound.DebugString();
  //   return false;
  // }

  // Copy nav map.
  ReadLocker source_read_lock(map->GetLock());
  auto process_map = std::make_shared<NavMap>(*map);
  if (FLAGS_debug_enable) {
    process_map->GetPrintLayer()->Print(__FILE__, __FUNCTION__, __LINE__);
  }
  source_read_lock.Unlock();

  // Replace sensor map with section nav map.
  auto sensor_map = process_map->GetSensorLayer();
  // if (!sensor_map->CopyRange(*section_map->GetSensorLayer(), bound)) {
  //   ZWARN << "Section map is not compatible with bound.";
  //   return false;
  // }

  process_map->ClearRobotCoverCells(start_pose);

  DynamicMapCellBound find_target_tmp_bound(bound);
  find_target_tmp_bound.Expend(find_target_tmp_bound.GetMin() -
                               MapCell(process_map->kRobotMarkWidth_2_,
                                       process_map->kRobotMarkWidth_2_));
  find_target_tmp_bound.Expend(find_target_tmp_bound.GetMax() +
                               MapCell(process_map->kRobotMarkWidth_2_,
                                       process_map->kRobotMarkWidth_2_));

  DynamicMapCellBound obs_tmp_bound(bound);
  obs_tmp_bound.Expend(
      obs_tmp_bound.GetMin() -
      MapCell(process_map->kRobotCellWidth_, process_map->kRobotCellWidth_));
  obs_tmp_bound.Expend(
      obs_tmp_bound.GetMax() +
      MapCell(process_map->kRobotCellWidth_, process_map->kRobotCellWidth_));

  if (!InflactObstacleMap(process_map, obs_tmp_bound, start_cell,
                          NavMap::kRobotCellWidth_2_,
                          [&](MultiLayersCharGridMap2D::SPtr map,
                              MapCell& curr_step, MapCell& next_step) {
                            return find_target_tmp_bound.Contain(next_step);
                          })) {
    return found;
  }

  auto footstep_map = process_map->GetFootStepLayer();
  ReadLocker footstep_map_read_lock(footstep_map->GetLock());
  ReadLocker sensor_map_lock(sensor_map->GetLock());
  // sensor_map->Print(__FILE__, __FUNCTION__, __LINE__);

  // Method 1.
  // if (HandleOnBound(process_map, start_cell, bound, output_path,
  //                   output_cell_path, wall_sensor_on_left)) {
  //   return true;
  // }
  // if (FindPathToBound(process_map, start_cell, bound, output_path,
  //                     output_cell_path)) {
  //   return true;
  // }
  // return false;

  // Method 2.
  bool path_back_to_bound = false;
  MapPointPath cached_path_back_to_bound;
  MapCellPath cached_cell_path_back_to_bound;
  if (!bound.Contain(start_cell)) {
    // Move back into bound.
    if (ShortestPathToBound(process_map, start_cell, bound, current_room_index,
                            output_path, output_cell_path)) {
      if (output_cell_path.size() > 2) {
        DynamicMapCellBound path_bound(INT_MAX, INT_MIN, INT_MAX, INT_MIN);
        for (auto&& cell : output_cell_path) {
          path_bound.Expend(cell);
        }

        if (FLAGS_debug_enable) {
          PrintPathInMap(map, path_bound, output_cell_path);
        }
        return true;
      }
      path_back_to_bound = true;
      cached_path_back_to_bound = output_path;
      cached_cell_path_back_to_bound = output_cell_path;
      // If path is just 2 cells, consider it as reached.
      start_cell = output_cell_path.back();
      ZGINFO << "Change start cell as " << start_cell.DebugString();
    } else {
      return false;
    }
  }

  auto targets = GetUnsortedBoundTargets(process_map, start_cell, bound,
                                         current_room_index);
  if (targets.empty()) {
    return false;
  }

  CharGridMap2D::SPtr target_mark_map(new CharGridMap2D(
      "target_mark_map", bound.GetMax().X() - bound.GetMin().X() + 1,
      bound.GetMax().Y() - bound.GetMin().Y() + 1, process_map->GetResolution()));

  {
    WriteLocker lock(target_mark_map->GetLock());
    target_mark_map->SetValue(bound.GetMin().X(), bound.GetMin().Y(),
                              NavMap::kUnknown_);
    target_mark_map->SetValue(bound.GetMax().X(), bound.GetMax().Y(),
                              NavMap::kUnknown_);
    for (auto&& target : targets) {
      target_mark_map->SetValue(target.X(), target.Y(), NavMap::kPath_);
    }
    ZGINFO << "Target mark map.";
    if (FLAGS_debug_enable) {
      target_mark_map->Print(__FILE__, __FUNCTION__, __LINE__);
    }
  }

  if (bound.OnBound(start_cell)) {
    if (GetAlongBoundPath(process_map, start_cell, target_mark_map, bound,
                          output_path, output_cell_path, wall_sensor_on_left)) {
      output_path =
          MovePathToExactlyBoundary(process_map, output_cell_path, bound);

      if (FLAGS_debug_enable) {
        PrintPathInMap(map, obs_tmp_bound, output_cell_path);
      }
      ZGINFO << "Go along bound.";
      return true;
    }
  }

  auto nearest_target = targets.front();
  ZGINFO << "Nearest target: " << nearest_target.DebugString();
  auto bound_head = nearest_target;
  if (GetBoundHeadOfTarget(process_map, nearest_target, target_mark_map, bound,
                           bound_head, wall_sensor_on_left)) {
    MapPointPath shortest_path;
    MapCellPath shortest_cell_path;
    bool should_go_shortest_path;
    if (ShortestPathToTarget(process_map, start_cell, bound_head,
                             find_target_tmp_bound, current_room_index,
                             shortest_path, shortest_cell_path,
                             should_go_shortest_path)) {
      output_path = shortest_path;
      output_cell_path = shortest_cell_path;
      if (FLAGS_debug_enable) {
        PrintPathInMap(map, obs_tmp_bound, output_cell_path);
      }
      return true;
    }
    ZERROR << "Should never run here.";
  }

  if (path_back_to_bound) {
    // ZINFO << "Use cached path.";
    output_path = cached_path_back_to_bound;
    output_cell_path = cached_cell_path_back_to_bound;
    return true;
  }
  return false;
}

bool EdgeCleaningPlanner::ExpendAlongBound(const bool& wall_sensor_on_left,
                                           const MapCell& curr_step,
                                           const MapCell& next_step,
                                           const DynamicMapCellBound& bound) {
  if (wall_sensor_on_left && curr_step.X() == bound.GetMax().X() &&
      curr_step.Y() <= bound.GetMax().Y() &&
      curr_step.Y() > bound.GetMin().Y()) {
    // Clockwise east
    if (next_step.X() != curr_step.X() || next_step.Y() > curr_step.Y() ||
        next_step.Y() < bound.GetMin().Y()) {
      return false;
    }
  } else if (wall_sensor_on_left && curr_step.Y() == bound.GetMin().Y() &&
             curr_step.X() <= bound.GetMax().X() &&
             curr_step.X() > bound.GetMin().X()) {
    // Clockwise south
    if (next_step.Y() != curr_step.Y() || next_step.X() > curr_step.X() ||
        next_step.X() < bound.GetMin().X()) {
      return false;
    }
  } else if (wall_sensor_on_left && curr_step.X() == bound.GetMin().X() &&
             curr_step.Y() < bound.GetMax().Y() &&
             curr_step.Y() >= bound.GetMin().Y()) {
    // Clockwise west
    if (next_step.X() != curr_step.X() || next_step.Y() < curr_step.Y() ||
        next_step.Y() > bound.GetMax().Y()) {
      return false;
    }
  } else if (wall_sensor_on_left && curr_step.Y() == bound.GetMax().Y() &&
             curr_step.X() < bound.GetMax().X() &&
             curr_step.X() >= bound.GetMin().X()) {
    // Clockwise north
    if (next_step.Y() != curr_step.Y() || next_step.X() < curr_step.X() ||
        next_step.X() > bound.GetMax().X()) {
      return false;
    }
  }

  if (!wall_sensor_on_left && curr_step.X() == bound.GetMax().X() &&
      curr_step.Y() < bound.GetMax().Y() &&
      curr_step.Y() >= bound.GetMin().Y()) {
    // Anti-clockwise east
    if (next_step.X() != curr_step.X() || next_step.Y() < curr_step.Y() ||
        next_step.Y() > bound.GetMax().Y()) {
      return false;
    }
  } else if (!wall_sensor_on_left && curr_step.Y() == bound.GetMin().Y() &&
             curr_step.X() < bound.GetMax().X() &&
             curr_step.X() >= bound.GetMin().X()) {
    // Anti-clockwise south
    if (next_step.Y() != curr_step.Y() || next_step.X() < curr_step.X() ||
        next_step.X() > bound.GetMax().X()) {
      return false;
    }
  } else if (!wall_sensor_on_left && curr_step.X() == bound.GetMin().X() &&
             curr_step.Y() <= bound.GetMax().Y() &&
             curr_step.Y() > bound.GetMin().Y()) {
    // Anti-clockwise west
    if (next_step.X() != curr_step.X() || next_step.Y() > curr_step.Y() ||
        next_step.Y() < bound.GetMin().Y()) {
      return false;
    }
  } else if (!wall_sensor_on_left && curr_step.Y() == bound.GetMax().Y() &&
             curr_step.X() <= bound.GetMax().X() &&
             curr_step.X() > bound.GetMin().X()) {
    // Anti-clockwise north
    if (next_step.Y() != curr_step.Y() || next_step.X() > curr_step.X() ||
        next_step.X() < bound.GetMin().X()) {
      return false;
    }
  }
  return true;
}

MapCells EdgeCleaningPlanner::GetUnsortedBoundTargets(
    NavMap::SPtr process_map, const MapCell& start_cell,
    const DynamicMapCellBound& bound,
    const CharGridMap2D::DataType& current_room_index) {
  CharGridMap2D::DataType value;
  auto sensor_map = process_map->GetSensorLayer();
  ReadLocker sensor_map_lock(sensor_map->GetLock());
  auto footstep_map = process_map->GetFootStepLayer();
  ReadLocker footstep_map_read_lock(footstep_map->GetLock());
  auto footstep_map_max_clean_bound =
      process_map->GetMaxCleanBound(std::make_shared<MapCell>(start_cell));
  footstep_map_max_clean_bound.EqualShrink(1);
  ZGINFO << footstep_map_max_clean_bound.DebugString();

  auto room_map = process_map->GetRoomLayer();
  ReadLocker room_map_lock(room_map->GetLock());
  auto user_block_map = process_map->GetUserBlockLayer();
  ReadLocker user_block_map_read_lock(user_block_map->GetLock());
  auto user_select_area_map = process_map->GetUserSelectAreaLayer();
  ReadLocker user_select_area_map_read_lock(user_select_area_map->GetLock());
  CharGridMap2D::DataType user_block_value;
  // ZINFO << "Find all reachable bound targets.";

  ExpendCondition expend_cond = [&](MultiLayersCharGridMap2D::SPtr _map,
                                    MapCell& curr_step, MapCell& next_step) {
    if (!footstep_map->GetAvailableBound().Contain(next_step)) {
      return false;
    }
    if (!footstep_map_max_clean_bound.Contain(next_step)) {
      return false;
    }
    // Check for block, stop expending if block is ahead.
    if (!process_map->IsClearedInSensorMap(next_step)) {
      return false;
    }

    // Just check for this room, stop expending if this cell belongs to other
    // rooms. (Unknown cells should be considered available for expending)
    room_map->GetValue(next_step.X(), next_step.Y(), value);
    if (value != current_room_index && value != NavMap::kUnknown_) {
      return false;
    }

    user_block_map->GetValue(next_step.X(), next_step.Y(), user_block_value);
    // ZINFO << "Step: " << next_step.DebugString()
    //       << ", user block value: " << user_block_value;
    if (user_block_value == NavMap::kVirtualWall_ ||
        user_block_value == NavMap::kStrictBlockArea_) {
      return false;
    }

    if (!bound.Contain(next_step)) {
      return false;
    }

    return true;
  };

  FinishCondition finish_cond = [&](MultiLayersCharGridMap2D::SPtr map,
                                    MapCell& curr_step) { return false; };

  MapCells unsorted_targets;
  CellStepCallback step_cb = [&](MultiLayersCharGridMap2D::SPtr map,
                                 MapCell& curr_step) {
    // ZINFO << curr_step;
    user_select_area_map->GetValue(curr_step.X(), curr_step.Y(), value);
    if (value != NavMap::kUserSelected_) {
      return;
    }

    if (footstep_map->GetValue(curr_step.X(), curr_step.Y(), value)) {
      // ZINFO << "Value:" << value;
      if (value == NavMap::kUnknown_ && bound.OnBound(curr_step)) {
        unsorted_targets.emplace_back(curr_step);
      }
    } else if (footstep_map->GetAvailableBound().Contain(curr_step)) {
      if (bound.OnBound(curr_step)) {
        unsorted_targets.emplace_back(curr_step);
      }
    }
  };

  // Do nothing.
  IterationStepCallback iter_step_cb = [&](MultiLayersCharGridMap2D::SPtr map,
                                           MapCells& cells) {};

  MapCells start_cells{start_cell};  // Start from current cell.
  MapCell target_cell;
  ZGINFO << typeid(*this).name() << " Start from: " << start_cell.DebugString();

  Search(process_map, start_cells, target_cell, expend_cond, finish_cond,
         step_cb, iter_step_cb);

  if (unsorted_targets.empty()) {
    ZGINFO << "No more targets on bound.";
  }

  ZGINFO << "Found " << unsorted_targets.size() << " targets.";
  return unsorted_targets;
}

bool EdgeCleaningPlanner::GetAlongBoundPath(
    NavMap::SPtr process_map, const MapCell& start_cell,
    const CharGridMap2D::SPtr& target_mark_map,
    const DynamicMapCellBound& bound, MapPointPath& output_path,
    MapCellPath& output_cell_path, const bool& wall_sensor_on_left) {
  if (target_mark_map == nullptr) {
    ZWARN << "target_mark_map invalid.";
    return false;
  }
  ReadLocker lock(target_mark_map->GetLock());
  // target_mark_map->Print(__FILE__, __FUNCTION__, __LINE__);

  auto footstep_map = process_map->GetFootStepLayer();
  ReadLocker footstep_map_read_lock(footstep_map->GetLock());
  auto footstep_map_max_clean_bound =
      process_map->GetMaxCleanBound(std::make_shared<MapCell>(start_cell));
  footstep_map_max_clean_bound.EqualShrink(1);
  ZGINFO << footstep_map_max_clean_bound.DebugString();

  CharGridMap2D::DataType value;
  // ZINFO << "Find along bound path.";

  output_cell_path.clear();
  MapCellPath tmp_path_between_targets;
  tmp_path_between_targets.emplace_back(start_cell);
  const uint8_t kForceMoveMaxStep = 2;
  int8_t force_move_step = -1;
  const uint8_t kOverObsMaxCount = 2;
  int8_t step_over_obs_count = -1;
  if (target_mark_map->GetValue(start_cell.X(), start_cell.Y(), value)) {
    if (value == NavMap::kPath_) {
      // Force move for start_cell it self is a target.
      force_move_step = kForceMoveMaxStep;
    }
  }

  uint32_t expend_step_count = 0;
  const uint32_t kMaxCheckObstacleDistance = 5;
  ExpendCondition expend_cond = [&](MultiLayersCharGridMap2D::SPtr _map,
                                    MapCell& curr_step, MapCell& next_step) {
    // ZERROR << "next: " << next_step;
    if (!ExpendAlongBound(wall_sensor_on_left, curr_step, next_step, bound)) {
      return false;
    }

    if (!footstep_map_max_clean_bound.Contain(next_step)) {
      return false;
    }

    expend_step_count++;
    bool meet_obstacle = !process_map->IsClearedInSensorMap(next_step);
    if (meet_obstacle && step_over_obs_count == -1) {
      // Detect obstacle for the first time.
      auto get_curr_step_value =
          footstep_map->GetValue(curr_step.X(), curr_step.Y(), value);
      if (expend_step_count < kMaxCheckObstacleDistance ||
          ((get_curr_step_value && value == NavMap::kUnknown_) ||
           (!get_curr_step_value &&
            footstep_map->GetAvailableBound().Contain(curr_step)))) {
        // Obstacle near start cell OR current step is uncleaned.
        auto get_next_step_value =
            footstep_map->GetValue(next_step.X(), next_step.Y(), value);
        if ((get_next_step_value && value == NavMap::kUnknown_) ||
            (!get_next_step_value &&
             footstep_map->GetAvailableBound().Contain(next_step))) {
          // Check if robot needs to step into obstacle.
          // ZINFO << "Found uncleaned target under obstacle "
          //       << next_step.DebugString();
          step_over_obs_count = kOverObsMaxCount;
        }
      }
    }

    if (force_move_step > 0) {
      return true;
    }

    if (step_over_obs_count > 0) {
      return true;
    } else if (step_over_obs_count == 0) {
      // Force stop.
      return false;
    }

    return !meet_obstacle;
  };

  // Generate path to farthest target.
  CellStepCallback step_cb = [&](MultiLayersCharGridMap2D::SPtr map,
                                 MapCell& curr_step) {
    if (force_move_step > 0 || step_over_obs_count > 0) {
      if (tmp_path_between_targets.empty() ||
          curr_step != tmp_path_between_targets.back()) {
        tmp_path_between_targets.emplace_back(curr_step);
      }
      output_cell_path.insert(output_cell_path.end(),
                              tmp_path_between_targets.begin(),
                              tmp_path_between_targets.end());
      tmp_path_between_targets.clear();
      // ZINFO << "Force insert at " << curr_step.DebugString();
    } else if (target_mark_map->GetValue(curr_step.X(), curr_step.Y(), value)) {
      if (tmp_path_between_targets.empty() ||
          curr_step != tmp_path_between_targets.back()) {
        tmp_path_between_targets.emplace_back(curr_step);
      }
      if (value == NavMap::kPath_) {
        output_cell_path.insert(output_cell_path.end(),
                                tmp_path_between_targets.begin(),
                                tmp_path_between_targets.end());
        tmp_path_between_targets.clear();
      }
    } else {
      ZWARN << curr_step << " is not in " << target_mark_map->Name();
    }

    if (force_move_step > 0) {
      force_move_step--;
    }
    if (step_over_obs_count > 0) {
      step_over_obs_count--;
    }
  };

  FinishCondition finish_cond = [&](MultiLayersCharGridMap2D::SPtr map,
                                    MapCell& curr_step) { return false; };

  // Do nothing.
  IterationStepCallback iter_step_cb = [&](MultiLayersCharGridMap2D::SPtr map,
                                           MapCells& cells) {};

  MapCells start_cells;  // Start from current cell.
  start_cells.emplace_back(start_cell);
  MapCell unused_target;
  Search(process_map, start_cells, unused_target, expend_cond, finish_cond,
         step_cb, iter_step_cb);
  if (!output_cell_path.empty()) {
    if (output_cell_path.size() == 1) {
      output_cell_path.emplace_back(output_cell_path.front());
    }
    // ZINFO << "Longest path: " << MapCell::DebugString(output_cell_path);
    output_path.clear();

    auto footstep_map = process_map->GetFootStepLayer();
    ReadLocker footstep_map_read_lock(footstep_map->GetLock());
    CharGridMap2D::CellPathToPointPath(footstep_map, output_cell_path,
                                       output_path);
    return true;
  }

  ZGINFO << "Can not find targets on bound.";
  // along_bound = !obs_before_first_target;
  return false;
}

bool EdgeCleaningPlanner::GetBoundHeadOfTarget(
    NavMap::SPtr process_map, const MapCell& start_cell,
    const CharGridMap2D::SPtr& target_mark_map,
    const DynamicMapCellBound& bound, MapCell& bound_head,
    const bool& wall_sensor_on_left) {
  if (target_mark_map == nullptr) {
    ZWARN << "target_mark_map invalid.";
    return false;
  }

  auto user_block_map = process_map->GetUserBlockLayer();
  ReadLocker user_block_map_read_lock(user_block_map->GetLock());
  auto footstep_map = process_map->GetFootStepLayer();
  ReadLocker footstep_map_read_lock(footstep_map->GetLock());
  auto footstep_map_max_clean_bound =
      process_map->GetMaxCleanBound(std::make_shared<MapCell>(start_cell));
  footstep_map_max_clean_bound.EqualShrink(1);
  ZGINFO << footstep_map_max_clean_bound.DebugString();

  CharGridMap2D::DataType user_block_value;
  ReadLocker lock(target_mark_map->GetLock());
  // if (FLAGS_debug_enable) {
  //   target_mark_map->Print(__FILE__, __FUNCTION__, __LINE__);
  // }

  CharGridMap2D::DataType value;
  ExpendCondition expend_cond = [&](MultiLayersCharGridMap2D::SPtr _map,
                                    MapCell& curr_step, MapCell& next_step) {
    // ZERROR << "next: " << next_step;
    // Use !wall_sensor_on_left to search backwards.
    if (!ExpendAlongBound(!wall_sensor_on_left, curr_step, next_step, bound)) {
      return false;
    }
    // Check for block, stop expending if block is ahead.
    if (!process_map->IsClearedInSensorMap(next_step)) {
      return false;
    }

    if (!footstep_map_max_clean_bound.Contain(next_step)) {
      return false;
    }
    user_block_map->GetValue(next_step.X(), next_step.Y(), user_block_value);
    // ZINFO << "Step: " << next_step.DebugString()
    //       << ", user block value: " << user_block_value;
    if (user_block_value == NavMap::kVirtualWall_ ||
        user_block_value == NavMap::kStrictBlockArea_) {
      return false;
    }

    return true;
  };

  bound_head = start_cell;
  CellStepCallback step_cb = [&](MultiLayersCharGridMap2D::SPtr map,
                                 MapCell& curr_step) {
    if (target_mark_map->GetValue(curr_step.X(), curr_step.Y(), value)) {
      // ZINFO << "Value:" << value;
      if (value == NavMap::kPath_) {
        bound_head = curr_step;
      }
    }
  };

  FinishCondition finish_cond = [&](MultiLayersCharGridMap2D::SPtr map,
                                    MapCell& curr_step) { return false; };

  // Do nothing.
  IterationStepCallback iter_step_cb = [&](MultiLayersCharGridMap2D::SPtr map,
                                           MapCells& cells) {};

  MapCells start_cells;  // Start from current cell.
  start_cells.emplace_back(start_cell);
  MapCell unused_target;
  // The output target cell is unused.
  Search(process_map, start_cells, unused_target, expend_cond, finish_cond,
         step_cb, iter_step_cb);

  // Check for head and tail are actually linked, it is a loop.
  if (bound_head.Distance(start_cell) == 1) {
    bound_head = start_cell;
  }
  ZGINFO << "Found bound head " << bound_head.DebugString();

  return true;
}

bool EdgeCleaningPlanner::ShortestPathToBound(
    NavMap::SPtr process_map, const MapCell& start_cell,
    const DynamicMapCellBound& bound,
    const CharGridMap2D::DataType& current_room_index,
    MapPointPath& output_path, MapCellPath& output_cell_path) {
  CharGridMap2D::DataType value;
  auto sensor_map = process_map->GetSensorLayer();
  ReadLocker sensor_map_lock(sensor_map->GetLock());
  auto footstep_map = process_map->GetFootStepLayer();
  ReadLocker footstep_map_read_lock(footstep_map->GetLock());
  auto footstep_map_max_clean_bound =
      process_map->GetMaxCleanBound(std::make_shared<MapCell>(start_cell));
  footstep_map_max_clean_bound.EqualShrink(1);
  ZGINFO << footstep_map_max_clean_bound.DebugString();

  auto room_map = process_map->GetRoomLayer();
  ReadLocker room_map_lock(room_map->GetLock());
  auto user_block_map = process_map->GetUserBlockLayer();
  ReadLocker user_block_map_read_lock(user_block_map->GetLock());
  auto user_select_area_map = process_map->GetUserSelectAreaLayer();
  ReadLocker user_select_area_map_read_lock(user_select_area_map->GetLock());
  CharGridMap2D::DataType user_block_value;

  ZGINFO << "Find shortest path to bound.";
  DynamicMapCellBound find_target_tmp_bound(bound);
  find_target_tmp_bound.Expend(find_target_tmp_bound.GetMin() -
                               MapCell(process_map->kRobotMarkWidth_2_,
                                       process_map->kRobotMarkWidth_2_));
  find_target_tmp_bound.Expend(find_target_tmp_bound.GetMax() +
                               MapCell(process_map->kRobotMarkWidth_2_,
                                       process_map->kRobotMarkWidth_2_));

  if (!find_target_tmp_bound.Contain(start_cell)) {
    ZINFO << "Robot not in bound.";
    return false;
  }

  ExpendCondition expend_cond = [&](MultiLayersCharGridMap2D::SPtr _map,
                                    MapCell& curr_step, MapCell& next_step) {
    if (!footstep_map->GetAvailableBound().Contain(next_step)) {
      return false;
    }
    if (!footstep_map_max_clean_bound.Contain(next_step)) {
      return false;
    }
    // Check for block, stop expending if block is ahead.
    if (!process_map->IsClearedInSensorMap(next_step)) {
      return false;
    }

    // Just check for this room, stop expending if this cell belongs to other
    // rooms. (Unknown cells should be considered available for expending)
    room_map->GetValue(next_step.X(), next_step.Y(), value);
    if (value != current_room_index && value != NavMap::kUnknown_) {
      return false;
    }

    user_block_map->GetValue(next_step.X(), next_step.Y(), user_block_value);
    // ZINFO << "Step: " << next_step.DebugString()
    //       << ", user block value: " << user_block_value;
    if (user_block_value == NavMap::kVirtualWall_ ||
        user_block_value == NavMap::kStrictBlockArea_) {
      return false;
    }

    if (!find_target_tmp_bound.Contain(next_step)) {
      return false;
    }

    return true;
  };

  FinishCondition finish_cond = [&](MultiLayersCharGridMap2D::SPtr map,
                                    MapCell& curr_step) {
    user_select_area_map->GetValue(curr_step.X(), curr_step.Y(), value);
    if (value != NavMap::kUserSelected_) {
      return false;
    }

    return bound.Contain(curr_step);
  };

  CellStepCallback step_cb = [&](MultiLayersCharGridMap2D::SPtr map,
                                 MapCell& curr_step) {};

  // Do nothing.
  IterationStepCallback iter_step_cb = [&](MultiLayersCharGridMap2D::SPtr map,
                                           MapCells& cells) {};

  MapCells start_cells{start_cell};  // Start from current cell.
  MapCell _target_cell;
  ZGINFO << typeid(*this).name() << " Start from: " << start_cell.DebugString();

  if (Search(process_map, start_cells, _target_cell, expend_cond, finish_cond,
             step_cb, iter_step_cb)) {
    // Found uncleaned target.
    output_path.clear();
    output_cell_path.clear();
    TracePath(start_cell, _target_cell, output_cell_path);
    CharGridMap2D::CellPathToPointPath(footstep_map, output_cell_path,
                                       output_path);
    MapPointPath optimized_point_path;
    MapCellPath optimized_cell_path;
    if (OptimizePathToShortest(process_map, output_cell_path,
                               optimized_point_path, optimized_cell_path,
                               current_room_index)) {
      output_cell_path = optimized_cell_path;
      output_path = optimized_point_path;
    }

    // ZINFO << "Optimized path: " << MapCell::DebugString(output_cell_path);

    return true;
  }
  return false;
}

bool EdgeCleaningPlanner::ShortestPathToTarget(
    NavMap::SPtr process_map, const MapCell& start_cell,
    const MapCell& target_cell, const DynamicMapCellBound& bound,
    const CharGridMap2D::DataType& current_room_index,
    MapPointPath& output_path, MapCellPath& output_cell_path,
    bool& should_go_shortest_path) {
  CharGridMap2D::DataType value;
  auto sensor_map = process_map->GetSensorLayer();
  ReadLocker sensor_map_lock(sensor_map->GetLock());
  auto footstep_map = process_map->GetFootStepLayer();
  ReadLocker footstep_map_read_lock(footstep_map->GetLock());
  auto footstep_map_max_clean_bound =
      process_map->GetMaxCleanBound(std::make_shared<MapCell>(start_cell));
  footstep_map_max_clean_bound.EqualShrink(1);
  ZGINFO << footstep_map_max_clean_bound.DebugString();
  auto room_map = process_map->GetRoomLayer();
  ReadLocker room_map_lock(room_map->GetLock());
  auto user_block_map = process_map->GetUserBlockLayer();
  ReadLocker user_block_map_read_lock(user_block_map->GetLock());
  auto user_select_area_map = process_map->GetUserSelectAreaLayer();
  ReadLocker user_select_area_map_read_lock(user_select_area_map->GetLock());
  CharGridMap2D::DataType user_block_value;
  // ZINFO << "Find shortest path to first target.";

  user_select_area_map->GetValue(target_cell.X(), target_cell.Y(), value);
  if (value != NavMap::kUserSelected_) {
    ZWARN << "Target cell " << target_cell.DebugString()
          << " is not in user selected area.";
    return false;
  }

  ExpendCondition expend_cond = [&](MultiLayersCharGridMap2D::SPtr _map,
                                    MapCell& curr_step, MapCell& next_step) {
    if (!footstep_map->GetAvailableBound().Contain(next_step)) {
      return false;
    }
    if (!footstep_map_max_clean_bound.Contain(next_step)) {
      return false;
    }
    // Check for block, stop expending if block is ahead.
    if (!process_map->IsClearedInSensorMap(next_step)) {
      return false;
    }

    // Just check for this room, stop expending if this cell belongs to other
    // rooms. (Unknown cells should be considered available for expending)
    room_map->GetValue(next_step.X(), next_step.Y(), value);
    if (value != current_room_index && value != NavMap::kUnknown_) {
      return false;
    }

    user_block_map->GetValue(next_step.X(), next_step.Y(), user_block_value);
    // ZINFO << "Step: " << next_step.DebugString()
    //       << ", user block value: " << user_block_value;
    if (user_block_value == NavMap::kVirtualWall_ ||
        user_block_value == NavMap::kStrictBlockArea_) {
      return false;
    }

    if (!bound.Contain(next_step)) {
      return false;
    }

    return true;
  };

  FinishCondition finish_cond = [&](MultiLayersCharGridMap2D::SPtr map,
                                    MapCell& curr_step) {
    return curr_step == target_cell;
  };

  CellStepCallback step_cb = [&](MultiLayersCharGridMap2D::SPtr map,
                                 MapCell& curr_step) {};

  // Do nothing.
  IterationStepCallback iter_step_cb = [&](MultiLayersCharGridMap2D::SPtr map,
                                           MapCells& cells) {};

  MapCells start_cells{start_cell};  // Start from current cell.
  MapCell _target_cell;
  ZGINFO << typeid(*this).name() << " Start from: " << start_cell.DebugString();

  if (Search(process_map, start_cells, _target_cell, expend_cond, finish_cond,
             step_cb, iter_step_cb)) {
    // Found uncleaned target.
    output_path.clear();
    output_cell_path.clear();
    TracePath(start_cell, target_cell, output_cell_path);
    CharGridMap2D::CellPathToPointPath(footstep_map, output_cell_path,
                                       output_path);
    MapPointPath optimized_point_path;
    MapCellPath optimized_cell_path;
    if (OptimizePathToShortest(process_map, output_cell_path,
                               optimized_point_path, optimized_cell_path,
                               current_room_index)) {
      output_cell_path = optimized_cell_path;
      output_path = optimized_point_path;
    }

    ZGINFO << "Optimized path: " << MapCell::DebugString(output_cell_path);

    if (static_cast<int>(output_cell_path.size()) <
        NavMap::kRobotCellWidth_ * 3) {
      should_go_shortest_path = false;
      return true;
    }
    should_go_shortest_path = true;
    return true;
  }
  ZERROR << "If target is not available, should not call this function.";
  return false;
}

MapPointPath EdgeCleaningPlanner::MovePathToExactlyBoundary(
    NavMap::SPtr map, const MapCellPath& output_cell_path,
    const DynamicMapCellBound& bound) {
  // ZINFO;
  MapPointPath path;
  auto resolution = map->GetFootStepLayer()->GetResolution();
  for (auto&& cell : output_cell_path) {
    MapPoint point;
    map->GetFootStepLayer()->MapToWorld(cell, point);
    if (bound.OnEastBound(cell)) {
      point.SetX(point.X() + resolution / 2);
      path.emplace_back(point);
    } else if (bound.OnSouthBound(cell)) {
      point.SetY(point.Y() - resolution / 2);
      path.emplace_back(point);
    } else if (bound.OnWestBound(cell)) {
      point.SetX(point.X() - resolution / 2);
      path.emplace_back(point);
    } else if (bound.OnNorthBound(cell)) {
      point.SetY(point.Y() + resolution / 2);
      path.emplace_back(point);
    }
  }

  auto it1 = path.begin();
  auto it2 = it1 + 1;
  float degree = 0;
  while (it1 != path.end()) {
    if (it2 != path.end()) {
      // ZINFO << "1:" << it1->DebugString() << ", 2:" << it2->DebugString();
      degree = NormalizeDegree(MapPoint::GetVector(*it1, *it2).Degree());
    }
    it1->SetDegree(degree);

    it1++;
    if (it1 != path.end()) {
      it2 = it1 + 1;
    }
  }

  return path;
}

}  // namespace zima
