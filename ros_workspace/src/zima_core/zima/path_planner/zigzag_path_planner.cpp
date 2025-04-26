/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/path_planner/zigzag_path_planner.h"

#include <algorithm>

#include "zima/common/util.h"
#include "zima/logger/logger.h"

namespace zima {

bool ZigZagPlannerManager::Initialize() {
  {
    planner_queue_.emplace_back(
        ZigZagPlanner::SPtr(new CurrentLinePriorityPlanner()));
    planner_queue_.emplace_back(
        ZigZagPlanner::SPtr(new OptimizedNextLinePriorityPlanner()));
    planner_queue_.emplace_back(
        ZigZagPlanner::SPtr(new OptimizedNearestTargetPriorityPlanner()));
  }
  // Other planners are to be added.

  generate_path_count_ = 0;
  return true;
}

bool ZigZagPlannerManager::GeneratePath(
    NavMap::SPtr map, NavMap::SPtr section_map, const MapPoint& start_pose,
    const DynamicMapCellBound& bound,
    const CharGridMap2D::DataType& current_room_index, ZigZagPath& output_path,
    MapCellPath& output_cell_path, const bool& use_x_direction) {
  generate_path_count_++;
  bool found = false;

  // Copy nav map.
  ReadLocker source_read_lock(map->GetLock());
  auto process_map = std::make_shared<NavMap>(*map);
  // process_map->GetPrintLayer()->Print(__FILE__, __FUNCTION__, __LINE__)) {
  source_read_lock.Unlock();

  // Replace sensor map with section nav map.
  auto sensor_map = process_map->GetSensorLayer();
  // if (!sensor_map->CopyRange(*section_map->GetSensorLayer(), bound)) {
  //   ZWARN << "Section map is not compatible with bound.";
  //   return false;
  // }

  // ZINFO << process_map->DebugString();
  process_map->ClearRobotCoverCells(start_pose);

  // Inflation for obstacles in maps.
  // For sensor map.
  MapCells obstacle_cells;

  // Get all obstacle cells inside bumper map.
  auto sensor_layer = process_map->GetSensorLayer();

  ReadLocker sensor_map_lock(sensor_layer->GetLock());
  // sensor_map->Print(__FILE__, __FUNCTION__, __LINE__);

  MapCell start_cell;
  sensor_layer->WorldToMap(start_pose, start_cell);

  ZGINFO << "Generate path for " << bound.DebugString() << " starting at "
         << start_cell.DebugString();

  auto tmp_bound = bound;
  sensor_map_lock.Unlock();
  tmp_bound.Expend(tmp_bound.GetMin() - MapCell(process_map->kRobotCellWidth_,
                                                process_map->kRobotCellWidth_));
  tmp_bound.Expend(tmp_bound.GetMax() + MapCell(process_map->kRobotCellWidth_,
                                                process_map->kRobotCellWidth_));
  if (!InflactObstacleMap(
          process_map, tmp_bound, start_cell, NavMap::kRobotCellWidth_2_,
          [&](MultiLayersCharGridMap2D::SPtr map, MapCell& curr_step,
              MapCell& next_step) { return tmp_bound.Contain(next_step); })) {
    return found;
  }

  for (auto&& planner : planner_queue_) {
    // ZINFO << " start pose" << start_pose;
    if (planner->GeneratePath(process_map, start_pose, bound,
                              current_room_index, output_path, output_cell_path,
                              use_x_direction)) {
      found = true;

      if (FLAGS_debug_enable) {
        PrintPathInMap(map, output_cell_path);
      }
      break;
    }
  }

  return found;
}

uint32_t ZigZagPlanner::CountUncleanedCell(
    NavMap::SPtr map, const MapCell& start_cell,
    const DynamicMapCellBound& bound,
    const CharGridMap2D::DataType& current_room_index,
    const bool& use_x_direction, const bool& is_positive) {
  auto footstep_map = map->GetFootStepLayer();
  ReadLocker footstep_map_read_lock(footstep_map->GetLock());
  auto footstep_map_max_clean_bound =
      map->GetMaxCleanBound(std::make_shared<MapCell>(start_cell));
  footstep_map_max_clean_bound.EqualShrink(1);
  ZGINFO << footstep_map_max_clean_bound.DebugString();
  auto sensor_map = map->GetSensorLayer();
  ReadLocker sensor_map_read_lock(sensor_map->GetLock());
  auto room_map = map->GetRoomLayer();
  ReadLocker room_map_read_lock(room_map->GetLock());
  auto user_block_map = map->GetUserBlockLayer();
  ReadLocker user_block_map_read_lock(user_block_map->GetLock());
  auto user_select_area_map = map->GetUserSelectAreaLayer();
  ReadLocker user_select_area_map_read_lock(user_select_area_map->GetLock());

  uint32_t uncleaned_cell_count = 0;
  if (!bound.Contain(start_cell)) {
    ZWARN << "Start cell" << start_cell << " is not in bound.";
    return uncleaned_cell_count;
  }

  CharGridMap2D::DataType value;

  ExpendCondition expend_cond = [&](MultiLayersCharGridMap2D::SPtr _map,
                                    MapCell& curr_step, MapCell& next_step) {
    // ZINFO << "current: " << curr_step << " next: " << next_step;
    if (!footstep_map->GetAvailableBound().Contain(next_step)) {
      // ZINFO;
      return false;
    }
    if (footstep_map->GetValue(curr_step.X(), curr_step.Y(), value)) {
      if (value != NavMap::kUnknown_) {
        return false;
      }
    }
    if (!footstep_map_max_clean_bound.Contain(next_step)) {
      return false;
    }

    // Check for block, stop expending if block is ahead.
    if (!map->IsClearedInSensorMap(next_step)) {
      // ZINFO;
      return false;
    }

    // Just check for this room, stop expending if this cell belongs to other
    // rooms. (Unknown cells should be considered available for expending)
    room_map->GetValue(next_step.X(), next_step.Y(), value);
    // if (!room_map->GetValue(next_step.X(), next_step.Y(), value)) {
    //   return false;
    // }
    if (value != current_room_index && value != NavMap::kUnknown_) {
      return false;
    }

    user_block_map->GetValue(next_step.X(), next_step.Y(), value);
    if (value == NavMap::kVirtualWall_ || value == NavMap::kStrictBlockArea_) {
      return false;
    }

    user_select_area_map->GetValue(curr_step.X(), curr_step.Y(), value);
    if (value != NavMap::kUserSelected_) {
      return false;
    }

    auto next_x = next_step.X();
    auto next_y = next_step.Y();
    // Just check for this cleaning bound.
    if (next_x < bound.GetMin().X() || next_x > bound.GetMax().X()) {
      // ZINFO;
      return false;
    }
    if (next_y < bound.GetMin().Y() || next_y > bound.GetMax().Y()) {
      // ZINFO;
      return false;
    }

    return true;
  };

  FinishCondition finish_cond = [&](MultiLayersCharGridMap2D::SPtr map,
                                    MapCell& curr_step) { return false; };

  CellStepCallback step_cb = [&](MultiLayersCharGridMap2D::SPtr map,
                                 MapCell& curr_step) {
    if (footstep_map->GetValue(curr_step.X(), curr_step.Y(), value)) {
      if (value == NavMap::kUnknown_) {
        return;
      }
    }

    // user_select_area_map->GetValue(curr_step.X(), curr_step.Y(), value);
    // if (value != NavMap::kUserSelected_) {
    //   return;
    // }

    uncleaned_cell_count++;
  };

  // Do nothing.
  IterationStepCallback iter_step_cb = [&](MultiLayersCharGridMap2D::SPtr map,
                                           MapCells& cells) {};

  MapCells start_cells{start_cell};  // Start from current cell.
  MapCell target_cell;
  ZGINFO << typeid(*this).name() << " Start from: " << start_cell.DebugString();

  Search(map, start_cells, target_cell, expend_cond, finish_cond, step_cb,
         iter_step_cb);
  ZGINFO << (is_positive ? "Postive direction " : "Negative direction")
        << " found uncleaned cell: " << uncleaned_cell_count;

  return uncleaned_cell_count;
}

CurrentLinePositivePlanner::CurrentLinePositivePlanner()
    : CurrentLinePlanner(true, half_line_width_) {}

CurrentLineNegativePlanner::CurrentLineNegativePlanner()
    : CurrentLinePlanner(false, half_line_width_) {}

bool CurrentLinePlanner::GeneratePath(
    NavMap::SPtr map, const MapPoint& start_pose,
    const DynamicMapCellBound& bound,
    const CharGridMap2D::DataType& current_room_index, ZigZagPath& output_path,
    MapCellPath& output_cell_path, const bool& use_x_direction) {
  auto footstep_map = map->GetFootStepLayer();
  ReadLocker footstep_map_read_lock(footstep_map->GetLock());
  auto sensor_map = map->GetSensorLayer();
  ReadLocker sensor_map_read_lock(sensor_map->GetLock());
  auto room_map = map->GetRoomLayer();
  ReadLocker room_map_read_lock(room_map->GetLock());
  auto user_block_map = map->GetUserBlockLayer();
  ReadLocker user_block_map_read_lock(user_block_map->GetLock());
  auto user_select_area_map = map->GetUserSelectAreaLayer();
  ReadLocker user_select_area_map_read_lock(user_select_area_map->GetLock());

  MapCell start_cell;
  if (!map->GetFootStepLayer()->WorldToMap(start_pose, start_cell)) {
    ZWARN << "Start pose is not in footprint map.";
    return false;
  }
  if (!bound.Contain(start_cell)) {
    // ZWARN << "Start cell" << start_cell << " is not in bound.";
    return false;
  }
  auto footstep_map_max_clean_bound =
      map->GetMaxCleanBound(std::make_shared<MapCell>(start_cell));
  footstep_map_max_clean_bound.EqualShrink(1);
  ZGINFO << footstep_map_max_clean_bound.DebugString();

  // footstep_map->Print(__FILE__, __FUNCTION__, __LINE__);
  // sensor_map->Print(__FILE__, __FUNCTION__, __LINE__);

  CharGridMap2D::DataType value;

  ExpendCondition expend_cond = [&](MultiLayersCharGridMap2D::SPtr _map,
                                    MapCell& curr_step, MapCell& next_step) {
    if (!footstep_map->GetAvailableBound().Contain(next_step)) {
      return false;
    }
    if (!footstep_map_max_clean_bound.Contain(next_step)) {
      return false;
    }
    // Check for block, stop expending if block is ahead.
    if (!map->IsClearedInSensorMap(next_step)) {
      return false;
    }

    // Just check for this room, stop expending if this cell belongs to other
    // rooms. (Unknown cells should be considered available for expending)
    room_map->GetValue(next_step.X(), next_step.Y(), value);
    // if (!room_map->GetValue(next_step.X(), next_step.Y(), value)) {
    //   return false;
    // }
    if (value != current_room_index && value != NavMap::kUnknown_) {
      return false;
    }

    user_block_map->GetValue(next_step.X(), next_step.Y(), value);
    // ZINFO << next_step.DebugString() << " " << value;
    if (value == NavMap::kVirtualWall_ || value == NavMap::kStrictBlockArea_) {
      return false;
    }

    user_select_area_map->GetValue(curr_step.X(), curr_step.Y(), value);
    if (value != NavMap::kUserSelected_) {
      return false;
    }

    auto next_x = next_step.X();
    auto next_y = next_step.Y();
    // Just check for this cleaning bound.
    if (next_x < bound.GetMin().X() || next_x > bound.GetMax().X()) {
      // ZINFO;
      return false;
    }
    if (next_y < bound.GetMin().Y() || next_y > bound.GetMax().Y()) {
      // ZINFO;
      return false;
    }

    if (use_x_direction) {
      if (positive_direction_) {
        // Just check for positive direction.
        if (next_x < curr_step.X()) {
          // ZINFO;
          return false;
        }
      } else {
        // Just check for negative direction.
        if (next_x > curr_step.X()) {
          // ZINFO;
          return false;
        }
      }
      // Just check for current line.
      if (abs(static_cast<int>(next_step.Y()) -
              static_cast<int>(start_cell.Y())) > 0) {
        // ZINFO;
        return false;
      }
    } else {
      if (positive_direction_) {
        // Just check for positive direction.
        if (next_y < curr_step.Y()) {
          // ZINFO;
          return false;
        }
      } else {
        // Just check for negative direction.
        if (next_y > curr_step.Y()) {
          // ZINFO;
          return false;
        }
      }
      // Just check for current line.
      if (abs(static_cast<int>(next_step.X()) -
              static_cast<int>(start_cell.X())) > 0) {
        // ZINFO;
        return false;
      }
    }
    return true;
  };

  FinishCondition finish_cond = [&](MultiLayersCharGridMap2D::SPtr map,
                                    MapCell& curr_step) { return false; };

  auto target_cell = start_cell;
  CellStepCallback step_cb = [&](MultiLayersCharGridMap2D::SPtr map,
                                 MapCell& curr_step) {
    // Check and mark for the farthest uncleaned target.
    CharGridMap2D::DataType obs_value;
    if (use_x_direction) {
      for (auto y_offset = -NavMap::kRobotMarkWidth_2_;
           y_offset <= NavMap::kRobotMarkWidth_2_; y_offset++) {
        MapCell check_cell(curr_step.X(), curr_step.Y() + y_offset);
        // ZINFO << check_cell;
        if (!bound.Contain(check_cell)) {
          continue;
        }
        if (!footstep_map_max_clean_bound.Contain(check_cell)) {
          continue;
        }
        // user_select_area_map->GetValue(check_cell.X(), check_cell.Y(),
        // value); if (value != NavMap::kUserSelected_) {
        //   continue;
        // }
        bool footstep_check_pass = false;
        if (footstep_map->GetValue(check_cell.X(), check_cell.Y(), value)) {
          if (value == NavMap::kUnknown_) {
            footstep_check_pass = true;
          }
        } else if (footstep_map->GetAvailableBound().Contain(check_cell)) {
          footstep_check_pass = true;
        }
        bool obs_check_pass = false;
        if (sensor_map->GetValue(check_cell.X(), check_cell.Y(), obs_value)) {
          if (obs_value == NavMap::kUnknown_) {
            obs_check_pass = true;
          }
        } else if (sensor_map->GetAvailableBound().Contain(check_cell)) {
          obs_check_pass = true;
        }
        if (footstep_check_pass && obs_check_pass) {
          if ((positive_direction_ && target_cell.X() < curr_step.X()) ||
              (!positive_direction_ && target_cell.X() > curr_step.X())) {
            // ZINFO << "Update target" << curr_step;
            target_cell = curr_step;
          }
        }
      }
    } else {
      for (auto x_offset = -NavMap::kRobotMarkWidth_2_;
           x_offset <= NavMap::kRobotMarkWidth_2_; x_offset++) {
        MapCell check_cell(curr_step.X() + x_offset, curr_step.Y());
        // ZINFO << check_cell;
        if (!bound.Contain(check_cell)) {
          continue;
        }
        if (!footstep_map_max_clean_bound.Contain(check_cell)) {
          continue;
        }
        // user_select_area_map->GetValue(check_cell.X(), check_cell.Y(),
        // value); if (value != NavMap::kUserSelected_) {
        //   continue;
        // }
        bool footstep_check_pass = false;
        if (footstep_map->GetValue(check_cell.X(), check_cell.Y(), value)) {
          if (value == NavMap::kUnknown_) {
            footstep_check_pass = true;
          }
        } else if (footstep_map->GetAvailableBound().Contain(check_cell)) {
          footstep_check_pass = true;
        }
        bool obs_check_pass = false;
        if (sensor_map->GetValue(check_cell.X(), check_cell.Y(), obs_value)) {
          if (obs_value == NavMap::kUnknown_) {
            obs_check_pass = true;
          }
        } else if (sensor_map->GetAvailableBound().Contain(check_cell)) {
          obs_check_pass = true;
        }
        if (footstep_check_pass && obs_check_pass) {
          if ((positive_direction_ && target_cell.Y() < curr_step.Y()) ||
              (!positive_direction_ && target_cell.Y() > curr_step.Y())) {
            // ZINFO << "Update target" << curr_step;
            target_cell = curr_step;
          }
        }
      }
    }
    return;
  };

  // Do nothing.
  IterationStepCallback iter_step_cb = [&](MultiLayersCharGridMap2D::SPtr map,
                                           MapCells& cells) {};

  MapCells start_cells{start_cell};  // Start from current cell.
  MapCell unused_cell;
  ZGINFO << typeid(*this).name() << " Start from: " << start_cell.DebugString();
  Search(map, start_cells, unused_cell, expend_cond, finish_cond, step_cb,
         iter_step_cb);

  if (target_cell != start_cell) {
    // Found uncleaned target.
    output_cell_path =
        footstep_map->GenerateCellsBetweenTwoCells(start_cell, target_cell);
    output_path.path.clear();
    CharGridMap2D::CellPathToPointPath(footstep_map, output_cell_path,
                                       output_path.path);

    output_path.type = PathType::ZIGZAG_PATH;

    ZGINFO << "Found target: " << target_cell.DebugString();
    ZGINFO << "ZigZagPath: " << MapCell::DebugString(output_cell_path);
    return true;
  }
  ZGINFO << typeid(*this).name() << " not found.";

  return false;
}

bool CurrentLinePriorityPlanner::GeneratePath(
    NavMap::SPtr map, const MapPoint& start_pose,
    const DynamicMapCellBound& bound,
    const CharGridMap2D::DataType& current_room_index, ZigZagPath& output_path,
    MapCellPath& output_cell_path, const bool& use_x_direction) {
  ZigZagPath pos_path, neg_path;
  MapCellPath pos_cell_path, neg_cell_path;
  bool pos_get =
      pos_planner_.GeneratePath(map, start_pose, bound, current_room_index,
                                pos_path, pos_cell_path, use_x_direction);
  bool neg_get =
      neg_planner_.GeneratePath(map, start_pose, bound, current_room_index,
                                neg_path, neg_cell_path, use_x_direction);
  if (!pos_get && !neg_get) {
    return false;
  } else if (pos_get && !neg_get) {
    // ZINFO << "Use current line pos";
    output_path = pos_path;
    output_cell_path = pos_cell_path;
  } else if (!pos_get && neg_get) {
    // ZINFO << "Use current line neg";
    output_path = neg_path;
    output_cell_path = neg_cell_path;
  } else {
    if (pos_cell_path.size() < neg_cell_path.size()) {
      // ZINFO << "Use current line pos";
      output_path = pos_path;
      output_cell_path = pos_cell_path;
    } else {
      // ZINFO << "Use current line neg";
      output_path = neg_path;
      output_cell_path = neg_cell_path;
    }
  }
  return true;
}

bool NextLinePlanner::GeneratePath(
    NavMap::SPtr map, const MapPoint& start_pose,
    const DynamicMapCellBound& bound,
    const CharGridMap2D::DataType& current_room_index, ZigZagPath& output_path,
    MapCellPath& output_cell_path, const bool& use_x_direction) {
  auto footstep_map = map->GetFootStepLayer();
  ReadLocker footstep_map_read_lock(footstep_map->GetLock());
  auto sensor_map = map->GetSensorLayer();
  ReadLocker sensor_map_read_lock(sensor_map->GetLock());
  auto room_map = map->GetRoomLayer();
  ReadLocker room_map_read_lock(room_map->GetLock());
  auto user_block_map = map->GetUserBlockLayer();
  ReadLocker user_block_map_read_lock(user_block_map->GetLock());
  auto user_select_area_map = map->GetUserSelectAreaLayer();
  ReadLocker user_select_area_map_read_lock(user_select_area_map->GetLock());

  MapCell start_cell;
  if (!map->GetFootStepLayer()->WorldToMap(start_pose, start_cell)) {
    ZWARN << "Start pose is not in footprint map.";
    return false;
  }
  if (!bound.Contain(start_cell)) {
    // ZWARN << "Start cell" << start_cell << " is not in bound.";
    return false;
  }
  auto footstep_map_max_clean_bound =
      map->GetMaxCleanBound(std::make_shared<MapCell>(start_cell));
  footstep_map_max_clean_bound.EqualShrink(1);
  ZGINFO << footstep_map_max_clean_bound.DebugString();

  CharGridMap2D::DataType value;

  ExpendCondition expend_cond = [&](MultiLayersCharGridMap2D::SPtr _map,
                                    MapCell& curr_step, MapCell& next_step) {
    if (!footstep_map->GetAvailableBound().Contain(next_step)) {
      return false;
    }
    if (!footstep_map_max_clean_bound.Contain(next_step)) {
      return false;
    }
    // Check for block, stop expending if block is ahead.
    if (!map->IsClearedInSensorMap(next_step)) {
      return false;
    }

    // Just check for this room, stop expending if this cell belongs to other
    // rooms. (Unknown cells should be considered available for expending)
    room_map->GetValue(next_step.X(), next_step.Y(), value);
    // if (!room_map->GetValue(next_step.X(), next_step.Y(), value)) {
    //   return false;
    // }
    if (value != current_room_index && value != NavMap::kUnknown_) {
      return false;
    }
    user_block_map->GetValue(next_step.X(), next_step.Y(), value);
    if (value == NavMap::kVirtualWall_ || value == NavMap::kStrictBlockArea_) {
      return false;
    }

    user_select_area_map->GetValue(curr_step.X(), curr_step.Y(), value);
    if (value != NavMap::kUserSelected_) {
      return false;
    }

    auto next_x = next_step.X();
    auto next_y = next_step.Y();
    // Just check for this cleaning bound.
    if (next_x < bound.GetMin().X() || next_x > bound.GetMax().X()) {
      // ZINFO;
      return false;
    }
    if (next_y < bound.GetMin().Y() || next_y > bound.GetMax().Y()) {
      // ZINFO;
      return false;
    }

    // ZINFO << "current: " << curr_step << " next: " << next_step;
    if (use_x_direction) {
      auto next_y = next_step.Y();
      if (positive_direction_) {
        // Just check for positive direction.
        if (next_y < curr_step.Y()) {
          // ZINFO;
          return false;
        }
        // Just check for positive next line.
        if (next_y - start_cell.Y() > NavMap::kRobotCellWidth_2_ + 1) {
          // ZINFO;
          return false;
        }
      } else {
        // Just check for negative direction.
        if (next_y > curr_step.Y()) {
          // ZINFO;
          return false;
        }
        // Just check for positive next line.
        if (start_cell.Y() - next_y > NavMap::kRobotCellWidth_2_ + 1) {
          // ZINFO;
          return false;
        }
      }
      // Just check for next line near start cell.
      if (abs(static_cast<int>(next_step.X()) -
              static_cast<int>(start_cell.X())) > NavMap::kRobotCellWidth_2_) {
        // ZINFO;
        return false;
      }
    } else {
      auto next_x = next_step.X();
      if (positive_direction_) {
        // Just check for positive direction.
        if (next_x < curr_step.X()) {
          // ZINFO;
          return false;
        }
        // Just check for positive next line.
        if (next_x - start_cell.X() > NavMap::kRobotCellWidth_2_ + 1) {
          // ZINFO;
          return false;
        }
      } else {
        // Just check for negative direction.
        if (next_x > curr_step.X()) {
          // ZINFO;
          return false;
        }
        // Just check for positive next line.
        if (start_cell.X() - next_x > NavMap::kRobotCellWidth_2_ + 1) {
          // ZINFO;
          return false;
        }
      }
      // Just check for current line.
      if (abs(static_cast<int>(next_step.Y()) -
              static_cast<int>(start_cell.Y())) > NavMap::kRobotCellWidth_2_) {
        // ZINFO;
        return false;
      }
    }
    return true;
  };

  FinishCondition finish_cond = [&](MultiLayersCharGridMap2D::SPtr map,
                                    MapCell& curr_step) {
    // user_select_area_map->GetValue(curr_step.X(), curr_step.Y(), value);
    // if (value != NavMap::kUserSelected_) {
    //   return false;
    // }

    if (footstep_map->GetValue(curr_step.X(), curr_step.Y(), value)) {
      if (value == NavMap::kUnknown_) {
        return true;
      }
    } else if (footstep_map->GetAvailableBound().Contain(curr_step)) {
      return true;
    }

    return false;
  };

  CellStepCallback step_cb = [&](MultiLayersCharGridMap2D::SPtr map,
                                 MapCell& curr_step) { return; };

  // Do nothing.
  IterationStepCallback iter_step_cb = [&](MultiLayersCharGridMap2D::SPtr map,
                                           MapCells& cells) {};

  MapCells start_cells{start_cell};  // Start from current cell.
  MapCell target_cell;
  ZGINFO << typeid(*this).name() << " Start from: " << start_cell.DebugString();

  if (Search(map, start_cells, target_cell, expend_cond, finish_cond, step_cb,
             iter_step_cb)) {
    // Found uncleaned target.
    output_path.path.clear();
    output_cell_path.clear();
    TracePath(start_cell, target_cell, output_cell_path);
    CharGridMap2D::CellPathToPointPath(footstep_map, output_cell_path,
                                       output_path.path);

    MapPointPath optimized_point_path;
    MapCellPath optimized_cell_path;
    if (OptimizePathToShortest(map, output_cell_path, optimized_point_path,
                               optimized_cell_path, current_room_index)) {
      output_cell_path = optimized_cell_path;
      output_path.path = optimized_point_path;
    }

    output_path.type = PathType::ZIGZAG_PATH;

    ZGINFO << "Found target: " << target_cell.DebugString();
    ZGINFO << "ZigZagPath: " << MapCell::DebugString(output_cell_path);
    return true;
  }
  ZGINFO << typeid(*this).name() << " not found.";

  return false;
}

NextLinePositivePlanner::NextLinePositivePlanner()
    : NextLinePlanner(true, half_line_width_) {}

NextLineNegativePlanner::NextLineNegativePlanner()
    : NextLinePlanner(false, half_line_width_) {}

bool NextLinePriorityPlanner::GeneratePath(
    NavMap::SPtr map, const MapPoint& start_pose,
    const DynamicMapCellBound& bound,
    const CharGridMap2D::DataType& current_room_index, ZigZagPath& output_path,
    MapCellPath& output_cell_path, const bool& use_x_direction) {
  ZigZagPath pos_path, neg_path;
  MapCellPath pos_cell_path, neg_cell_path;
  bool pos_get =
      pos_planner_.GeneratePath(map, start_pose, bound, current_room_index,
                                pos_path, pos_cell_path, use_x_direction);
  bool neg_get =
      neg_planner_.GeneratePath(map, start_pose, bound, current_room_index,
                                neg_path, neg_cell_path, use_x_direction);
  if (!pos_get && !neg_get) {
    return false;
  } else if (pos_get && !neg_get) {
    // ZINFO << "Use next line pos";
    output_path = pos_path;
    output_cell_path = pos_cell_path;
  } else if (!pos_get && neg_get) {
    // ZINFO << "Use next line neg";
    output_path = neg_path;
    output_cell_path = neg_cell_path;
  } else {
    if (CountUncleanedCell(map, pos_cell_path.back(), bound, current_room_index,
                           use_x_direction, true) <
        CountUncleanedCell(map, neg_cell_path.back(), bound, current_room_index,
                           use_x_direction, false)) {
      // ZINFO << "Use next line pos";
      output_path = pos_path;
      output_cell_path = pos_cell_path;
    } else {
      // ZINFO << "Use next line neg";
      output_path = neg_path;
      output_cell_path = neg_cell_path;
    }
  }
  return true;
}

bool OptimizedNextLinePriorityPlanner::GeneratePath(
    NavMap::SPtr map, const MapPoint& start_pose,
    const DynamicMapCellBound& bound,
    const CharGridMap2D::DataType& current_room_index, ZigZagPath& output_path,
    MapCellPath& output_cell_path, const bool& use_x_direction) {
  auto ret = next_line_planner_.GeneratePath(map, start_pose, bound,
                                             current_room_index, output_path,
                                             output_cell_path, use_x_direction);
  if (!ret) {
    return false;
  }

  auto current_line_planner_start_pose = output_path.path.back();
  ZigZagPath current_line_output_path;
  MapCellPath current_line_output_cell_path;

  ZigZagPath pos_path, neg_path;
  MapCellPath pos_cell_path, neg_cell_path;
  bool pos_get = current_line_pos_planner_.GeneratePath(
      map, current_line_planner_start_pose, bound, current_room_index, pos_path,
      pos_cell_path, use_x_direction);
  bool neg_get = current_line_neg_planner_.GeneratePath(
      map, current_line_planner_start_pose, bound, current_room_index, neg_path,
      neg_cell_path, use_x_direction);
  if (pos_get && neg_get) {
    // ZINFO << "Try to optimize.";
    if (pos_cell_path.size() < neg_cell_path.size()) {
      // ZINFO << "Use current line pos";
      current_line_output_path = pos_path;
      current_line_output_cell_path = pos_cell_path;
    } else {
      // ZINFO << "Use current line neg";
      current_line_output_path = neg_path;
      current_line_output_cell_path = neg_cell_path;
    }

    current_line_output_path.path.pop_front();
    current_line_output_cell_path.pop_front();
    output_path.path.insert(output_path.path.end(),
                            current_line_output_path.path.begin(),
                            current_line_output_path.path.end());
    output_cell_path.insert(output_cell_path.end(),
                            current_line_output_cell_path.begin(),
                            current_line_output_cell_path.end());

    MapPointPath optimized_point_path;
    MapCellPath optimized_cell_path;
    if (OptimizePathToShortest(map, output_cell_path, optimized_point_path,
                               optimized_cell_path, current_room_index)) {
      ZGINFO << "Optimize success.";
      output_cell_path = optimized_cell_path;
      output_path.path = optimized_point_path;
    }
  }

  return true;
}

bool NearestTargetPlanner::GeneratePath(
    NavMap::SPtr map, const MapPoint& start_pose,
    const DynamicMapCellBound& bound,
    const CharGridMap2D::DataType& current_room_index, ZigZagPath& output_path,
    MapCellPath& output_cell_path, const bool& use_x_direction) {
  auto footstep_map = map->GetFootStepLayer();
  ReadLocker footstep_map_read_lock(footstep_map->GetLock());
  auto sensor_map = map->GetSensorLayer();
  ReadLocker sensor_map_read_lock(sensor_map->GetLock());
  auto room_map = map->GetRoomLayer();
  ReadLocker room_map_read_lock(room_map->GetLock());
  auto user_block_map = map->GetUserBlockLayer();
  ReadLocker user_block_map_read_lock(user_block_map->GetLock());
  auto user_select_area_map = map->GetUserSelectAreaLayer();
  ReadLocker user_select_area_map_read_lock(user_select_area_map->GetLock());

  MapCell start_cell;
  if (!map->GetFootStepLayer()->WorldToMap(start_pose, start_cell)) {
    ZWARN << "Start pose is not in footprint map.";
    return false;
  }
  auto footstep_map_max_clean_bound =
      map->GetMaxCleanBound(std::make_shared<MapCell>(start_cell));
  footstep_map_max_clean_bound.EqualShrink(1);
  ZGINFO << footstep_map_max_clean_bound.DebugString();

  auto tmp_bound = bound;
  if (!bound.Contain(start_cell)) {
    tmp_bound.Expend(tmp_bound.GetMin() - MapCell(map->kRobotCellWidth_2_ / 2,
                                                  map->kRobotCellWidth_2_ / 2));
    tmp_bound.Expend(tmp_bound.GetMax() + MapCell(map->kRobotCellWidth_2_ / 2,
                                                  map->kRobotCellWidth_2_ / 2));
  }

  if (!tmp_bound.Contain(start_cell)) {
    ZWARN << "Start cell" << start_cell << " is not in bound.";
    return false;
  }

  CharGridMap2D::DataType value;

  ExpendCondition expend_cond = [&](MultiLayersCharGridMap2D::SPtr _map,
                                    MapCell& curr_step, MapCell& next_step) {
    if (!footstep_map->GetAvailableBound().Contain(next_step)) {
      return false;
    }
    footstep_map->GetValue(curr_step.X(), curr_step.Y(), value);
    // Forbid robot cross unstepped area in bound but allow robot outside bound
    // to come back.
    if (bound.Contain(curr_step) && value == footstep_map->GetDefaultValue()) {
      return false;
    }

    if (!footstep_map_max_clean_bound.Contain(next_step)) {
      return false;
    }
    // Check for block, stop expending if block is ahead.
    if (!map->IsClearedInSensorMap(next_step)) {
      return false;
    }

    // Just check for this room, stop expending if this cell belongs to other
    // rooms. (Unknown cells should be considered available for expending)
    room_map->GetValue(next_step.X(), next_step.Y(), value);
    // if (!room_map->GetValue(next_step.X(), next_step.Y(), value)) {
    //   return false;
    // }
    if (value != current_room_index && value != NavMap::kUnknown_) {
      return false;
    }
    user_block_map->GetValue(next_step.X(), next_step.Y(), value);
    if (value == NavMap::kVirtualWall_ || value == NavMap::kStrictBlockArea_) {
      return false;
    }

    user_select_area_map->GetValue(curr_step.X(), curr_step.Y(), value);
    if (value != NavMap::kUserSelected_) {
      return false;
    }

    if (!tmp_bound.Contain(next_step)) {
      return false;
    }
    return true;
  };

  FinishCondition finish_cond = [&](MultiLayersCharGridMap2D::SPtr map,
                                    MapCell& curr_step) {
    if (!bound.Contain(curr_step)) {
      return false;
    }
    // user_select_area_map->GetValue(curr_step.X(), curr_step.Y(), value);
    // if (value != NavMap::kUserSelected_) {
    //   return false;
    // }
    if (footstep_map->GetValue(curr_step.X(), curr_step.Y(), value)) {
      if (value == NavMap::kUnknown_) {
        if (use_x_direction) {
          if (positive_direction_ && curr_step.Y() >= start_cell.Y()) {
            return true;
          }
          if (!positive_direction_ && curr_step.Y() <= start_cell.Y()) {
            return true;
          }
        } else {
          if (positive_direction_ && curr_step.X() >= start_cell.X()) {
            return true;
          }
          if (!positive_direction_ && curr_step.X() <= start_cell.X()) {
            return true;
          }
        }
      }
    } else if (footstep_map->GetAvailableBound().Contain(curr_step)) {
      return true;
    }

    return false;
  };

  CellStepCallback step_cb = [&](MultiLayersCharGridMap2D::SPtr map,
                                 MapCell& curr_step) { return; };

  // Do nothing.
  IterationStepCallback iter_step_cb = [&](MultiLayersCharGridMap2D::SPtr map,
                                           MapCells& cells) {};

  MapCells start_cells{start_cell};  // Start from current cell.
  MapCell target_cell;
  ZGINFO << typeid(*this).name() << " Start from: " << start_cell.DebugString();

  if (Search(map, start_cells, target_cell, expend_cond, finish_cond, step_cb,
             iter_step_cb)) {
    // Found uncleaned target.
    output_path.path.clear();
    output_cell_path.clear();
    TracePath(start_cell, target_cell, output_cell_path);
    CharGridMap2D::CellPathToPointPath(footstep_map, output_cell_path,
                                       output_path.path);
    MapPointPath optimized_point_path;
    MapCellPath optimized_cell_path;
    if (OptimizePathToShortest(map, output_cell_path, optimized_point_path,
                               optimized_cell_path, current_room_index)) {
      output_cell_path = optimized_cell_path;
      output_path.path = optimized_point_path;
    }

    output_path.type = PathType::NEAREST_UNCLEAN_TARGET_PATH;
    ZGINFO << "Found target: " << target_cell.DebugString();
    ZGINFO << "ZigZagPath: " << MapCell::DebugString(output_cell_path);
    return true;
  }
  ZGINFO << typeid(*this).name() << " not found.";

  return false;
}

NearestTargetPositivePlanner::NearestTargetPositivePlanner()
    : NearestTargetPlanner(true) {}

NearestTargetNegativePlanner::NearestTargetNegativePlanner()
    : NearestTargetPlanner(false) {}

bool NearestTargetPriorityPlanner::GeneratePath(
    NavMap::SPtr map, const MapPoint& start_pose,
    const DynamicMapCellBound& bound,
    const CharGridMap2D::DataType& current_room_index, ZigZagPath& output_path,
    MapCellPath& output_cell_path, const bool& use_x_direction) {
  auto generate_path = [&](const NavMap::SPtr& _map) {
    ZigZagPath pos_path, neg_path;
    MapCellPath pos_cell_path, neg_cell_path;
    bool pos_get =
        pos_planner_.GeneratePath(_map, start_pose, bound, current_room_index,
                                  pos_path, pos_cell_path, use_x_direction);
    bool neg_get =
        neg_planner_.GeneratePath(_map, start_pose, bound, current_room_index,
                                  neg_path, neg_cell_path, use_x_direction);
    if (!pos_get && !neg_get) {
      return false;
    } else if (pos_get && !neg_get) {
      // ZINFO << "Use nearest target pos";
      output_path = pos_path;
      output_cell_path = pos_cell_path;
    } else if (!pos_get && neg_get) {
      // ZINFO << "Use nearest target neg";
      output_path = neg_path;
      output_cell_path = neg_cell_path;
    } else {
      auto pos_count =
          CountUncleanedCell(map, pos_cell_path.back(), bound,
                             current_room_index, use_x_direction, true);
      auto neg_count =
          CountUncleanedCell(map, neg_cell_path.back(), bound,
                             current_room_index, use_x_direction, false);
      if (pos_count < neg_count) {
        // ZINFO << "Use nearest target pos";
        output_path = pos_path;
        output_cell_path = pos_cell_path;
      } else if (pos_count > neg_count) {
        // ZINFO << "Use nearest target neg";
        output_path = neg_path;
        output_cell_path = neg_cell_path;
      } else {
        if (pos_cell_path.size() < neg_cell_path.size()) {
          // ZINFO << "Use nearest target pos";
          output_path = pos_path;
          output_cell_path = pos_cell_path;
        } else {
          // ZINFO << "Use nearest target neg";
          output_path = neg_path;
          output_cell_path = neg_cell_path;
        }
      }
    }
    return true;
  };

  {
    // Try generate with larger inflacted obstacles to avoid bumping into same
    // obstacle.
    ReadLocker source_read_lock(map->GetLock());
    auto map2 = std::make_shared<NavMap>(*map);
    // map2->GetPrintLayer()->Print(__FILE__, __FUNCTION__, __LINE__);
    source_read_lock.Unlock();

    MapCell start_cell;
    map2->GetSensorLayer()->WorldToMap(start_pose, start_cell);

    auto tmp_bound = bound;
    tmp_bound.Expend(tmp_bound.GetMin() -
                     MapCell(map2->kRobotCellWidth_, map2->kRobotCellWidth_));
    tmp_bound.Expend(tmp_bound.GetMax() +
                     MapCell(map2->kRobotCellWidth_, map2->kRobotCellWidth_));
    if (InflactObstacleMap(
            map2, tmp_bound, start_cell, NavMap::kRobotCellWidth_2_ / 2,
            [&](MultiLayersCharGridMap2D::SPtr map, MapCell& curr_step,
                MapCell& next_step) { return tmp_bound.Contain(next_step); },
            false)) {
      // Prevent start cell and target cell covered by inflacted obstacle.
      auto sensor_layer = map2->GetSensorLayer();
      WriteLocker sensor_lock(sensor_layer->GetLock());
      sensor_layer->SetValue(start_cell.X(), start_cell.Y(), NavMap::kUnknown_);
      sensor_lock.Unlock();

      // map2->GetPrintLayer()->Print(__FILE__, __FUNCTION__, __LINE__);
      if (generate_path(map2)) {
        // ZINFO << "For larger obstacle distance.";
        return true;
      }
    }
  }

  ZGINFO;
  return generate_path(map);
}

bool OptimizedNearestTargetPriorityPlanner::GeneratePath(
    NavMap::SPtr map, const MapPoint& start_pose,
    const DynamicMapCellBound& bound,
    const CharGridMap2D::DataType& current_room_index, ZigZagPath& output_path,
    MapCellPath& output_cell_path, const bool& use_x_direction) {
  auto ret = nearest_target_planner_.GeneratePath(
      map, start_pose, bound, current_room_index, output_path, output_cell_path,
      use_x_direction);
  if (!ret) {
    return false;
  }

  auto current_line_planner_start_pose = output_path.path.back();
  ZigZagPath current_line_output_path;
  MapCellPath current_line_output_cell_path;

  ZigZagPath pos_path, neg_path;
  MapCellPath pos_cell_path, neg_cell_path;
  bool pos_get = current_line_pos_planner_.GeneratePath(
      map, current_line_planner_start_pose, bound, current_room_index, pos_path,
      pos_cell_path, use_x_direction);
  bool neg_get = current_line_neg_planner_.GeneratePath(
      map, current_line_planner_start_pose, bound, current_room_index, neg_path,
      neg_cell_path, use_x_direction);
  if (pos_get && neg_get) {
    // ZINFO << "Try to optimize.";
    if (pos_cell_path.size() < neg_cell_path.size()) {
      // ZINFO << "Use current line pos";
      current_line_output_path = pos_path;
      current_line_output_cell_path = pos_cell_path;
    } else {
      // ZINFO << "Use current line neg";
      current_line_output_path = neg_path;
      current_line_output_cell_path = neg_cell_path;
    }

    current_line_output_path.path.pop_front();
    current_line_output_cell_path.pop_front();
    output_path.path.insert(output_path.path.end(),
                            current_line_output_path.path.begin(),
                            current_line_output_path.path.end());
    output_cell_path.insert(output_cell_path.end(),
                            current_line_output_cell_path.begin(),
                            current_line_output_cell_path.end());

    MapPointPath optimized_point_path;
    MapCellPath optimized_cell_path;
    if (OptimizePathToShortest(map, output_cell_path, optimized_point_path,
                               optimized_cell_path, current_room_index)) {
      ZGINFO << "Optimize success.";
      output_cell_path = optimized_cell_path;
      output_path.path = optimized_point_path;
    }
  }

  return true;
}

}  // namespace zima
