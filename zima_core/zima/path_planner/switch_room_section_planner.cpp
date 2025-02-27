/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/path_planner/switch_room_section_planner.h"

#include <algorithm>

#include "zima/common/util.h"
#include "zima/logger/logger.h"

namespace zima {

bool SwitchSectionPlanner::GeneratePath(
    NavMap::SPtr map, const MapPoint& start_pose,
    const CharGridMap2D::DataType& current_room_index,
    MapPointPath& output_path, MapCellPath& output_cell_path,
    const bool& consider_slam_map) {
  bool found = false;
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

  // Get all obstacle cells inside bumper map.
  auto sensor_map = process_map->GetSensorLayer();

  CharGridMap2D::DataType value;
  ReadLocker sensor_map_lock(sensor_map->GetLock());
  // sensor_map->Print(__FILE__, __FUNCTION__, __LINE__);
  auto user_block_map = process_map->GetUserBlockLayer();
  ReadLocker user_block_map_read_lock(user_block_map->GetLock());
  CharGridMap2D::DataType user_block_value;

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

  auto footstep_map = process_map->GetFootStepLayer();
  ReadLocker footstep_map_read_lock(footstep_map->GetLock());
  auto footstep_map_max_clean_bound =
      process_map->GetMaxCleanBound(std::make_shared<MapCell>(start_cell));
  footstep_map_max_clean_bound.EqualShrink(1);
  ZGINFO << footstep_map_max_clean_bound.DebugString();
  sensor_map_lock.Lock();
  user_block_map_read_lock.Lock();

  auto room_map = process_map->GetRoomLayer();
  ReadLocker room_map_read_lock(room_map->GetLock());
  auto user_select_area_map = map->GetUserSelectAreaLayer();
  ReadLocker user_select_area_map_read_lock(user_select_area_map->GetLock());
  auto slam_char_map = map->GetSlamLayer();
  ReadLocker slam_char_map_read_lock(slam_char_map->GetLock());
  CharGridMap2D::DataType room_value;

  const uint8_t kMoveInsideRoomMaxStep = 2;
  uint8_t move_step_count = 0;
  bool inside_room = false;

  ExpendCondition expend_cond = [&](MultiLayersCharGridMap2D::SPtr _map,
                                    MapCell& curr_step, MapCell& next_step) {
    // ZINFO << "Expend for " << next_step;
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
    // rooms and far from target room. (Unknown cells should be considered
    // available for expending)
    room_map->GetValue(next_step.X(), next_step.Y(), room_value);
    // ZINFO << "Step: " << next_step.DebugString()
    //       << ", Room value: " << room_value
    //       << ", current room index: " << current_room_index;
    if (room_value != current_room_index && room_value != NavMap::kUnknown_) {
      if (inside_room) {
        return false;
      }

      if (!inside_room && move_step_count >= kMoveInsideRoomMaxStep) {
        return false;
      }
    } else {
      inside_room = true;
    }

    user_block_map->GetValue(next_step.X(), next_step.Y(), user_block_value);
    // ZINFO << "Step: " << next_step.DebugString()
    //       << ", user block value: " << user_block_value;
    if (user_block_value == NavMap::kVirtualWall_ ||
        user_block_value == NavMap::kStrictBlockArea_) {
      return false;
    }

    if (consider_slam_map) {
      slam_char_map->GetValue(curr_step.X(), curr_step.Y(), value);
      if (value != NavMap::kSlamFloor_) {
        return false;
      }
    }

    return true;
  };

  FinishCondition finish_cond = [&](MultiLayersCharGridMap2D::SPtr map,
                                    MapCell& curr_step) {
    user_select_area_map->GetValue(curr_step.X(), curr_step.Y(), value);
    if (value != NavMap::kUserSelected_) {
      return false;
    }

    room_map->GetValue(curr_step.X(), curr_step.Y(), room_value);
    // ZINFO << "Step: " << curr_step.DebugString()
    //       << ", Room value: " << room_value
    //       << ", current room index: " << current_room_index;
    if (room_value != current_room_index && room_value != NavMap::kUnknown_) {
      return false;
    }

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
                                           MapCells& cells) {
    if (move_step_count < kMoveInsideRoomMaxStep) {
      move_step_count++;
    }
  };

  MapCells start_cells{start_cell};  // Start from current cell.
  MapCell target_cell;
  ZGINFO << typeid(*this).name() << " Start from: " << start_cell.DebugString()
         << ", consider slam map " << std::to_string(consider_slam_map);

  // process_map->GetPrintLayer()->Print(__FILE__, __FUNCTION__, __LINE__);

  if (Search(process_map, start_cells, target_cell, expend_cond, finish_cond,
             step_cb, iter_step_cb)) {
    // Found uncleaned target.
    ZGINFO << typeid(*this).name() << " found: " << target_cell.DebugString();
    output_cell_path.clear();
    TracePath(start_cell, target_cell, output_cell_path);
    output_path.clear();
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

    ZGINFO << "Found target: " << target_cell.DebugString();
    ZGINFO << "Path: " << MapCell::DebugString(output_cell_path);

    if (FLAGS_debug_enable) {
      // PrintPathInMap(process_map, output_cell_path);
      // DynamicMapCellBound path_bound(INT_MAX, INT_MIN, INT_MAX, INT_MIN);
      // for (auto&& cell : output_cell_path) {
      //   path_bound.Expend(cell);
      // }
      // PrintPathInMap(map, path_bound, output_cell_path);
      PrintPathInMap(map, output_cell_path);
    }
    return true;
  }
  ZGINFO << typeid(*this).name() << " not found.";

  return false;
}

bool SwitchRoomPlanner::GeneratePathToSpecificRoom(
    NavMap::SPtr map, const MapPoint& start_pose,
    const CharGridMap2D::DataType& next_room_index, MapPointPath& output_path,
    MapCellPath& output_cell_path, const bool& consider_slam_map) {
  bool found = false;
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

  // Get all obstacle cells inside bumper map.
  auto sensor_map = process_map->GetSensorLayer();

  CharGridMap2D::DataType value;
  ReadLocker sensor_map_lock(sensor_map->GetLock());
  // sensor_map->Print(__FILE__, __FUNCTION__, __LINE__);

  auto user_block_map = process_map->GetUserBlockLayer();
  ReadLocker user_block_map_read_lock(user_block_map->GetLock());
  auto user_select_area_map = map->GetUserSelectAreaLayer();
  ReadLocker user_select_area_map_read_lock(user_select_area_map->GetLock());
  auto slam_char_map = map->GetSlamLayer();
  ReadLocker slam_char_map_read_lock(slam_char_map->GetLock());
  CharGridMap2D::DataType user_block_value;

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

  auto footstep_map = process_map->GetFootStepLayer();
  ReadLocker footstep_map_read_lock(footstep_map->GetLock());
  auto footstep_map_max_clean_bound =
      process_map->GetMaxCleanBound(std::make_shared<MapCell>(start_cell));
  footstep_map_max_clean_bound.EqualShrink(1);
  ZGINFO << footstep_map_max_clean_bound.DebugString();
  sensor_map_lock.Lock();
  user_block_map_read_lock.Lock();
  // CharGridMap2D::SPtr room_map;
  // map->GetLayer(NavMap::kRoomMapName_, room_map);

  ExpendCondition expend_cond = [&](MultiLayersCharGridMap2D::SPtr _map,
                                    MapCell& curr_step, MapCell& next_step) {
    // ZINFO << "Expend for " << next_step;
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
    // if (!room_map->GetValue(next_step.X(), next_step.Y(), value)) {
    //   return false;
    // }
    // if (value != current_room_index && value != NavMap::kUnknown_) {
    //   return false;
    // }

    user_block_map->GetValue(next_step.X(), next_step.Y(), user_block_value);
    // ZINFO << "Step: " << next_step.DebugString()
    //       << ", user block value: " << user_block_value;
    if (user_block_value == NavMap::kVirtualWall_ ||
        user_block_value == NavMap::kStrictBlockArea_) {
      return false;
    }

    if (consider_slam_map) {
      slam_char_map->GetValue(curr_step.X(), curr_step.Y(), value);
      if (value != NavMap::kSlamFloor_) {
        return false;
      }
    }

    return true;
  };

  auto room_map = process_map->GetRoomLayer();
  ReadLocker room_map_read_lock(room_map->GetLock());
  CharGridMap2D::DataType room_value;

  FinishCondition finish_cond = [&](MultiLayersCharGridMap2D::SPtr map,
                                    MapCell& curr_step) {
    user_select_area_map->GetValue(curr_step.X(), curr_step.Y(), value);
    if (value != NavMap::kUserSelected_) {
      return false;
    }

    room_map->GetValue(curr_step.X(), curr_step.Y(), room_value);
    if (room_value != next_room_index) {
      return false;
    }

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
  ZGINFO << typeid(*this).name() << " Start from: " << start_cell.DebugString()
         << ", consider slam map " << std::to_string(consider_slam_map);

  // sensor_map->Print(__FILE__, __FUNCTION__, __LINE__);

  if (Search(process_map, start_cells, target_cell, expend_cond, finish_cond,
             step_cb, iter_step_cb)) {
    // Found uncleaned target.
    output_cell_path.clear();
    TracePath(start_cell, target_cell, output_cell_path);
    output_path.clear();
    CharGridMap2D::CellPathToPointPath(footstep_map, output_cell_path,
                                       output_path);

    MapPointPath optimized_point_path;
    MapCellPath optimized_cell_path;
    if (OptimizePathToShortest(process_map, output_cell_path,
                               optimized_point_path, optimized_cell_path)) {
      output_cell_path = optimized_cell_path;
      output_path = optimized_point_path;
    }

    ZGINFO << "Found target: " << target_cell.DebugString();
    ZGINFO << "Path: " << MapCell::DebugString(output_cell_path);

    if (FLAGS_debug_enable) {
      // PrintPathInMap(process_map, output_cell_path);
      // DynamicMapCellBound path_bound(INT_MAX, INT_MIN, INT_MAX, INT_MIN);
      // for (auto&& cell : output_cell_path) {
      //   path_bound.Expend(cell);
      // }
      // PrintPathInMap(map, path_bound, output_cell_path);
      PrintPathInMap(map, output_cell_path);
    }

    return true;
  }
  ZGINFO << typeid(*this).name() << " not found.";

  return false;
}

bool SwitchRoomPlanner::GeneratePathToNearestRoom(
    NavMap::SPtr map, const MapPoint& start_pose, MapPointPath& output_path,
    MapCellPath& output_cell_path, const RoomsInfo& rooms_info,
    const bool& consider_slam_map) {
  bool found = false;
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

  // Get all obstacle cells inside bumper map.
  auto sensor_map = process_map->GetSensorLayer();

  CharGridMap2D::DataType value;
  ReadLocker sensor_map_lock(sensor_map->GetLock());
  // sensor_map->Print(__FILE__, __FUNCTION__, __LINE__);

  auto user_block_map = process_map->GetUserBlockLayer();
  ReadLocker user_block_map_read_lock(user_block_map->GetLock());
  auto user_select_area_map = map->GetUserSelectAreaLayer();
  ReadLocker user_select_area_map_read_lock(user_select_area_map->GetLock());
  auto slam_char_map = map->GetSlamLayer();
  ReadLocker slam_char_map_read_lock(slam_char_map->GetLock());
  CharGridMap2D::DataType user_block_value;

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

  auto footstep_map = process_map->GetFootStepLayer();
  ReadLocker footstep_map_read_lock(footstep_map->GetLock());
  auto footstep_map_max_clean_bound =
      process_map->GetMaxCleanBound(std::make_shared<MapCell>(start_cell));
  footstep_map_max_clean_bound.EqualShrink(1);
  ZGINFO << footstep_map_max_clean_bound.DebugString();
  sensor_map_lock.Lock();
  user_block_map_read_lock.Lock();
  auto room_map = process_map->GetRoomLayer();
  ReadLocker room_map_read_lock(room_map->GetLock());

  ExpendCondition expend_cond = [&](MultiLayersCharGridMap2D::SPtr _map,
                                    MapCell& curr_step, MapCell& next_step) {
    // ZINFO << "Expend for " << next_step;
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
    // if (!room_map->GetValue(next_step.X(), next_step.Y(), value)) {
    //   return false;
    // }
    // if (value != current_room_index && value != NavMap::kUnknown_) {
    //   return false;
    // }

    user_block_map->GetValue(next_step.X(), next_step.Y(), user_block_value);
    // ZINFO << "Step: " << next_step.DebugString()
    //       << ", user block value: " << user_block_value;
    if (user_block_value == NavMap::kVirtualWall_ ||
        user_block_value == NavMap::kStrictBlockArea_) {
      return false;
    }

    if (consider_slam_map) {
      slam_char_map->GetValue(curr_step.X(), curr_step.Y(), value);
      if (value != NavMap::kSlamFloor_) {
        return false;
      }
    }

    return true;
  };

  bool selected_rooms = !rooms_info.empty();
  FinishCondition finish_cond = [&](MultiLayersCharGridMap2D::SPtr map,
                                    MapCell& curr_step) {
    if (selected_rooms) {
      if (!room_map->GetValue(curr_step.X(), curr_step.Y(), value)) {
        return false;
      }
      if (rooms_info.count(value) == 0) {
        return false;
      }
    }

    user_select_area_map->GetValue(curr_step.X(), curr_step.Y(), value);
    if (value != NavMap::kUserSelected_) {
      return false;
    }

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
  ZGINFO << typeid(*this).name() << " Start from: " << start_cell.DebugString()
         << ", consider slam map " << std::to_string(consider_slam_map);
  if (selected_rooms) {
    ZGINFO << "Search for selected rooms.";
  }

  // sensor_map->Print(__FILE__, __FUNCTION__, __LINE__);

  if (Search(process_map, start_cells, target_cell, expend_cond, finish_cond,
             step_cb, iter_step_cb)) {
    // Found uncleaned target.
    output_cell_path.clear();
    TracePath(start_cell, target_cell, output_cell_path);
    output_path.clear();
    CharGridMap2D::CellPathToPointPath(footstep_map, output_cell_path,
                                       output_path);

    MapPointPath optimized_point_path;
    MapCellPath optimized_cell_path;
    if (OptimizePathToShortest(process_map, output_cell_path,
                               optimized_point_path, optimized_cell_path)) {
      output_cell_path = optimized_cell_path;
      output_path = optimized_point_path;
    }

    ZGINFO << "Found target: " << target_cell.DebugString();
    ZGINFO << "Path: " << MapCell::DebugString(output_cell_path);
    if (selected_rooms) {
      room_map->GetValue(target_cell.X(), target_cell.Y(), value);
      ZGINFO << "Move to room " << value;
    }

    if (FLAGS_debug_enable) {
      // PrintPathInMap(process_map, output_cell_path);
      // DynamicMapCellBound path_bound(INT_MAX, INT_MIN, INT_MAX, INT_MIN);
      // for (auto&& cell : output_cell_path) {
      //   path_bound.Expend(cell);
      // }
      // PrintPathInMap(map, path_bound, output_cell_path);
      PrintPathInMap(map, output_cell_path);
    }
    return true;
  }
  ZGINFO << typeid(*this).name() << " not found.";

  return false;
}

}  // namespace zima
