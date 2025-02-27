/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/path_planner/path_planner.h"

#include <algorithm>

#include "zima/common/util.h"
#include "zima/grid_map/nav_map_2d.h"
#include "zima/logger/logger.h"

namespace zima {

bool PlannerBase::InflactObstacleMap(
    NavMap::SPtr process_map, const DynamicMapCellBound& bound,
    const MapCell& start_cell, const uint16_t& inflate_max_iteration_count,
    const ExpendCondition& extra_expend_cond,
    const bool& enable_square_inflaction, const bool& print_result) {
  // Inflation for obstacles or user blocks in maps.
  MapCells obstacle_cells;
  MapCells user_block_cells;

  // Get all obstacle cells inside sensor map and user block map.
  auto sensor_map = process_map->GetSensorLayer();
  auto user_block_map = process_map->GetUserBlockLayer();

  CharGridMap2D::DataType value;
  ReadLocker sensor_map_lock(sensor_map->GetLock());
  // sensor_map->Print(__FILE__, __FUNCTION__, __LINE__);
  if (sensor_map->IsMarked()) {
    auto data_bound = sensor_map->GetDataBound();
    // ZINFO << "sensor map data bound: " << data_bound.DebugString();
    for (auto x = data_bound.GetMin().X(); x <= data_bound.GetMax().X(); x++) {
      for (auto y = data_bound.GetMin().Y(); y <= data_bound.GetMax().Y();
           y++) {
        if (!bound.Contain(x, y)) {
          continue;
        }
        sensor_map->GetValue(x, y, value);
        // ZINFO << "Check for(" << x << ", " << y << ")" << value << ".";
        if (value != NavMap::kUnknown_) {
          obstacle_cells.emplace_back(MapCell(x, y));
          // ZINFO << "Found obs(" << x << ", " << y << ").";
        }
      }
    }
  }
  sensor_map_lock.Unlock();
  ReadLocker user_block_map_lock(user_block_map->GetLock());
  if (user_block_map->IsMarked()) {
    auto data_bound = user_block_map->GetDataBound();
    // ZINFO << "user block map data bound: " << data_bound.DebugString();
    for (auto x = data_bound.GetMin().X(); x <= data_bound.GetMax().X(); x++) {
      for (auto y = data_bound.GetMin().Y(); y <= data_bound.GetMax().Y();
           y++) {
        if (!bound.Contain(x, y)) {
          continue;
        }
        user_block_map->GetValue(x, y, value);
        // ZINFO << "Check for(" << x << ", " << y << ")" << value << ".";
        if (value == NavMap::kVirtualWall_ ||
            value == NavMap::kStrictBlockArea_) {
          user_block_cells.emplace_back(MapCell(x, y));
          // ZINFO << "Found user block(" << x << ", " << y << ").";
        }
      }
    }
  }
  user_block_map_lock.Unlock();

  ExpendCondition sensor_expend_cond = [&](MultiLayersCharGridMap2D::SPtr _map,
                                           MapCell& curr_step,
                                           MapCell& next_step) {
    // Dont expend to robot.
    if (next_step == start_cell) {
      return false;
    }
    if (!extra_expend_cond(_map, curr_step, next_step)) {
      return false;
    }
    // Step forward while next step is not marked as sensor point.
    return process_map->IsClearedInSensorMap(next_step);
  };

  ExpendCondition user_block_expend_cond =
      [&](MultiLayersCharGridMap2D::SPtr _map, MapCell& curr_step,
          MapCell& next_step) {
        // Dont expend to robot.
        if (next_step == start_cell) {
          return false;
        }
        if (!extra_expend_cond(_map, curr_step, next_step)) {
          return false;
        }
        return true;
      };

  // Start inlfation with all sensor obstacles.

  // ZINFO << inflate_max_iteration_count;
  if (!obstacle_cells.empty()) {
    if (!InflateObstaclesInMap(process_map, NavMap::kSensorMapName_,
                               inflate_max_iteration_count, obstacle_cells,
                               sensor_expend_cond, enable_square_inflaction)) {
      ZWARN;
      return false;
    }
  }
  if (!user_block_cells.empty()) {
    if (!InflateObstaclesInMap(process_map, NavMap::kUserBlockMapName_,
                               inflate_max_iteration_count, user_block_cells,
                               user_block_expend_cond,
                               enable_square_inflaction)) {
      ZWARN;
      return false;
    }
  }
  if (print_result) {
    for (auto&& str : process_map->GetPrintLayer()->DebugString(bound)) {
      ZINFO << str;
    }
    // for (auto&& str : sensor_map->DebugString(bound)) {
    //   ZINFO << str;
    // }
    // for (auto&& str : user_block_map->DebugString(bound)) {
    //   ZINFO << str;
    // }
  }

  return true;
}

bool PlannerBase::InflactSlamObstacleMap(
    NavMap::SPtr process_map, const DynamicMapCellBound& bound,
    const MapCell& start_cell, const uint16_t& inflate_max_iteration_count,
    const ExpendCondition& extra_expend_cond,
    const bool& enable_square_inflaction, const bool& print_result) {
  // Inflation for walls in maps.
  MapCells slam_wall_cells;

  // Get all wall cells inside slam map.
  auto slam_map = process_map->GetSlamLayer();

  CharGridMap2D::DataType value;
  ReadLocker slam_map_lock(slam_map->GetLock());
  // slam_map->Print(__FILE__, __FUNCTION__, __LINE__);
  if (slam_map->IsMarked()) {
    auto data_bound = slam_map->GetDataBound();
    // ZINFO << "sensor map data bound: " << data_bound.DebugString();
    for (auto x = data_bound.GetMin().X(); x <= data_bound.GetMax().X(); x++) {
      for (auto y = data_bound.GetMin().Y(); y <= data_bound.GetMax().Y();
           y++) {
        if (!bound.Contain(x, y)) {
          continue;
        }
        slam_map->GetValue(x, y, value);
        // ZINFO << "Check for(" << x << ", " << y << ")" << value << ".";
        if (value == NavMap::kSlamWall_) {
          slam_wall_cells.emplace_back(MapCell(x, y));
          // ZINFO << "Found obs(" << x << ", " << y << ").";
        }
      }
    }
  }
  slam_map_lock.Unlock();
  ExpendCondition slam_expend_cond = [&](MultiLayersCharGridMap2D::SPtr _map,
                                         MapCell& curr_step,
                                         MapCell& next_step) {
    // Dont expend to robot.
    if (next_step == start_cell) {
      return false;
    }
    if (!extra_expend_cond(_map, curr_step, next_step)) {
      return false;
    }

    // Step forward while next step is not marked as slam wall.
    if (slam_map->GetValue(next_step.X(), next_step.Y(), value)) {
      // ZISODBG << "value: " << value;
      if (value == NavMap::kSlamWall_) {
        return false;
      }
    } else if (!slam_map->GetAvailableBound().Contain(next_step)) {
      return false;
    }

    return true;
  };

  // Start inlfation with all bumper obstacles.

  // ZINFO << inflate_max_iteration_count;
  if (!slam_wall_cells.empty()) {
    if (!InflateObstaclesInMap(process_map, NavMap::kSlamMapName_,
                               inflate_max_iteration_count, slam_wall_cells,
                               slam_expend_cond, enable_square_inflaction)) {
      ZWARN;
      return false;
    }
  }
  if (print_result) {
    for (auto&& str : slam_map->DebugString(bound)) {
      ZINFO << str;
    }
  }

  return true;
}

bool PlannerBase::InflateObstaclesInMap(
    NavMap::SPtr map, const std::string& map_type,
    const uint16_t& inflate_max_iteration_count, const MapCells& start_cells,
    ExpendCondition& expend_cond, const bool& enable_square_inflaction) {
  if (start_cells.empty()) {
    ZINFO << "Start cells is empty.";
    return false;
  }

  CharGridMap2D::SPtr target_map;
  if (!map->GetLayer(map_type, target_map)) {
    ZWARN;
    return false;
  }
  WriteLocker layer_lock(target_map->GetLock());

  CharGridMap2D::SPtr print_map;
  if (!map->GetLayer(map->kPrintMapName_, print_map)) {
    ZWARN;
    return false;
  }
  WriteLocker print_lock(print_map->GetLock());

  CharGridMap2D::DataType obs_value;
  if (!target_map->GetValue(start_cells.front().X(), start_cells.front().Y(),
                            obs_value)) {
    ZWARN << start_cells.front().DebugString() << " is not in " << map_type
          << " layer.";
    return false;
  }

  FinishCondition finish_cond = [&](MultiLayersCharGridMap2D::SPtr map,
                                    MapCell& curr_step) { return false; };

  CellStepCallback step_cb = [&](MultiLayersCharGridMap2D::SPtr map,
                                 MapCell& curr_step) {
    // ZINFO << "Set (" << curr_step.X() << ", " << curr_step.Y()
    //       << ") = " << obs_value;
    target_map->SetValue(curr_step.X(), curr_step.Y(), obs_value);
    print_map->SetValue(curr_step.X(), curr_step.Y(), obs_value);
  };

  unsigned int inflate_iteration_count = 0;
  IterationStepCallback iter_step_cb = [&](MultiLayersCharGridMap2D::SPtr map,
                                           MapCells& cells) {
    // target_map->Print(__FILE__, __FUNCTION__, __LINE__);
    if (++inflate_iteration_count > inflate_max_iteration_count) {
      cells.clear();
    }
  };

  MapCell unused_cell;
  Search(map, start_cells, unused_cell, expend_cond, finish_cond, step_cb,
         iter_step_cb);

  if (enable_square_inflaction) {
    // Specially expend original cells as square.
    auto square_inflate = [&](const MapCell& cell) {
      for (auto i = -NavMap::kRobotCellWidth_2_;
           i <= NavMap::kRobotCellWidth_2_; i++) {
        for (auto j = -NavMap::kRobotCellWidth_2_;
             j <= NavMap::kRobotCellWidth_2_; j++) {
          auto inflate_cell = cell + MapCell(i, j);
          if (expend_cond(map, inflate_cell, inflate_cell)) {
            // ZERROR << "Mark for " << inflate_cell;
            target_map->SetValue(inflate_cell.X(), inflate_cell.Y(), obs_value);
            print_map->SetValue(inflate_cell.X(), inflate_cell.Y(), obs_value);
          }
        }
      }
    };

    for (auto&& cell : start_cells) {
      square_inflate(cell);
    }
  }

  return true;
}

void PlannerBase::PrintPathInMap(NavMap::SPtr map, const MapCellPath& path) {
  if (FLAGS_debug_enable) {
    PrintPathInMap(map, map->GetPrintLayer()->GetDataBound(), path);
  } else {
    PrintPathInMap(map, map->GetSlamLayer()->GetDataBound(), path);
  }
}

void PlannerBase::PrintPathInMap(NavMap::SPtr map,
                                 const DynamicMapCellBound& print_bound,
                                 const MapCellPath& path) {
  // Print for path.
  std::shared_ptr<CharGridMap2D> print_map;
  if (FLAGS_debug_enable) {
    print_map = std::make_shared<CharGridMap2D>(*map->GetPrintLayer());
  } else {
    print_map = std::make_shared<CharGridMap2D>(*map->GetSlamLayer());
  }
  auto bound = print_bound;
  WriteLocker print_map_lock(print_map->GetLock());
  bool first_cell = true;
  for (auto&& cell : path) {
    if (first_cell) {
      print_map->SetValue(cell.X(), cell.Y(), NavMap::kPathStart_);
      bound.Expend(cell);
      first_cell = false;
      continue;
    }
    bound.Expend(cell);
    print_map->SetValue(cell.X(), cell.Y(), NavMap::kPath_);
  }

  CharGridMap2D::OverridePrintCellFunc print_func =
      [](const CharGridMap2D::DataType& value) {
        std::string str;
        if (value == 0) {
          str = ".";
        } else if (value == NavMap::kPathStart_) {
          str += ZCOLOR_YELLOW;
          str += NavMap::kPathStart_;
          str += ZCOLOR_NONE;
        } else if (value == NavMap::kPath_) {
          str += ZCOLOR_GREEN;
          str += NavMap::kPath_;
          str += ZCOLOR_NONE;
        } else {
          str += value;
        }
        return str;
      };
  for (auto&& str : print_map->DebugString(bound, print_func)) {
    ZINFO << str;
  }
}

void PlannerBase::PrintPathInMap(NavMap::SPtr map, const MapPointPath& path) {
  PrintPathInMap(map, map->GetPrintLayer()->GetDataBound(), path);
}

void PlannerBase::PrintPathInMap(NavMap::SPtr map,
                                 const DynamicMapCellBound& print_bound,
                                 const MapPointPath& path) {
  MapCellPath cell_path;
  CharGridMap2D::PointPathToCellPath(map->GetFootStepLayer(), path, cell_path);
  PrintPathInMap(map, print_bound, cell_path);
}

bool PlannerBase::OptimizePathToShortest(
    NavMap::SPtr map, const MapCells& input_cell_path,
    MapPointPath& output_point_path, MapCellPath& output_cell_path,
    const CharGridMap2D::DataType& current_room_index,
    const bool& consider_slam_map) {
  if (input_cell_path.size() < 3) {
    ZGWARN << "No need to optimize.";
    return false;
  }

  // Copy nav map.
  ReadLocker source_read_lock(map->GetLock());
  auto process_map = std::make_shared<NavMap>(*map);
  // process_map->GetPrintLayer()->Print(__FILE__, __FUNCTION__, __LINE__);
  source_read_lock.Unlock();

  // ZINFO << process_map->DebugString();

  // Inflation for obstacles in maps.
  // For sensor map.
  MapCells obstacle_cells;

  // Get all obstacle cells inside sensor map.
  auto sensor_layer = process_map->GetSensorLayer();

  ReadLocker sensor_layer_lock(sensor_layer->GetLock());
  auto data_bound = sensor_layer->GetDataBound();
  auto user_block_layer = process_map->GetUserBlockLayer();
  ReadLocker user_block_layer_read_lock(user_block_layer->GetLock());
  data_bound.Expend(user_block_layer->GetDataBound().GetMin());
  data_bound.Expend(user_block_layer->GetDataBound().GetMax());
  auto slam_layer = process_map->GetSlamLayer();
  ReadLocker slam_map_read_lock(slam_layer->GetLock());
  if (consider_slam_map) {
    data_bound.Expend(slam_layer->GetDataBound().GetMin());
    data_bound.Expend(slam_layer->GetDataBound().GetMax());
  }

  sensor_layer_lock.Unlock();
  user_block_layer_read_lock.Unlock();
  slam_map_read_lock.Unlock();
  // process_map->GetSensorLayer()->Print(__FILE__, __FUNCTION__, __LINE__);
  InflactObstacleMap(
      process_map, data_bound, input_cell_path.front(),
      NavMap::kRobotCellWidth_2_ / 2,
      [](MultiLayersCharGridMap2D::SPtr map, MapCell& curr_step,
         MapCell& next_step) { return true; },
      false);
  if (consider_slam_map) {
    InflactSlamObstacleMap(
        process_map, data_bound, input_cell_path.front(),
        NavMap::kRobotCellWidth_2_ / 2,
        [](MultiLayersCharGridMap2D::SPtr map, MapCell& curr_step,
           MapCell& next_step) { return true; },
        false);
  }
  {
    WriteLocker sensor_map_write_lock(sensor_layer->GetLock());
    // For case that path is along obstacles.
    for (auto&& path_cell : input_cell_path) {
      sensor_layer->SetValue(path_cell.X(), path_cell.Y(), NavMap::kUnknown_);
    }
  }

  MapCells new_cell_path;
  auto start_it = input_cell_path.begin();
  auto straight_end_it = start_it;
  auto footstep_layer = process_map->GetFootStepLayer();
  auto room_layer = process_map->GetRoomLayer();
  ReadLocker room_layer_lock(room_layer->GetLock());
  user_block_layer_read_lock.Lock();
  sensor_layer_lock.Lock();
  slam_map_read_lock.Lock();
  // process_map->GetSensorLayer()->Print(__FILE__, __FUNCTION__, __LINE__);

  // ZINFO << "Current room index: " << current_room_index;
  auto check_near_room_edge = [&](const MapCell& cell) {
    // ZINFO << "Check for " << cell.DebugString();
    if (current_room_index != NavMap::kUnknown_) {
      auto room_value = room_layer->GetDefaultValue();
      for (auto x = -1; x <= 1; x++) {
        for (auto y = -1; y <= 1; y++) {
          room_layer->GetValue(cell.X() + x, cell.Y() + y, room_value);
          if (room_value != current_room_index &&
              room_value != NavMap::kUnknown_) {
            // ZERROR;
            return true;
          }
        }
      }
    }
    return false;
  };

  auto check_near_user_block_area = [&](const MapCell& cell) {
    // ZINFO << "Check for " << cell.DebugString();
    auto user_block_value = user_block_layer->GetDefaultValue();
    for (auto x = -1; x <= 1; x++) {
      for (auto y = -1; y <= 1; y++) {
        user_block_layer->GetValue(cell.X() + x, cell.Y() + y,
                                   user_block_value);
        if (user_block_value == NavMap::kVirtualWall_ ||
            user_block_value == NavMap::kStrictBlockArea_) {
          // ZERROR;
          return true;
        }
      }
    }
    return false;
  };

  auto check_near_slam_wall = [&](const MapCell& cell) {
    if (consider_slam_map) {
      // ZINFO << "Check for " << cell.DebugString();
      CharGridMap2D::DataType slam_value = NavMap::kUnknown_;
      for (auto x = -1; x <= 1; x++) {
        for (auto y = -1; y <= 1; y++) {
          slam_layer->GetValue(cell.X() + x, cell.Y() + y, slam_value);
          if (slam_value == NavMap::kSlamWall_) {
            // ZERROR;
            return true;
          }
        }
      }
    }
    return false;
  };

  while (true) {
    // Step 1: Initiazlize and find for straight_end_it.
    new_cell_path.emplace_back(*start_it);
    if (start_it == input_cell_path.end() - 1) {
      // ZINFO << "Finish optimize";
      break;
    }
    // ZINFO << "Start it: " << *start_it;
    // Find straight end it. straight_end_it = xxx.
    do {
      auto it1 = start_it;
      auto it2 = it1 + 1;
      if (it2 == input_cell_path.end()) {
        break;
      }
      MapCell offset_cell(it2->X() - it1->X(), it2->Y() - it1->Y());
      while (true) {
        if (it2 == input_cell_path.end()) {
          break;
        }

        MapCell _offset_cell(it2->X() - it1->X(), it2->Y() - it1->Y());
        if (_offset_cell != offset_cell) {
          break;
        }
        straight_end_it = it2;

        it1 = it2;
        it2 = it1 + 1;
      }
      // ZINFO << "Straight end it: " << *straight_end_it;
    } while (false);

    // Step 2: Find farthest direct cell.
    auto target_it = input_cell_path.end() - 1;
    bool start_from_end = false;
    bool path_available = true;
    while (*target_it != *straight_end_it) {
      // Step 3: Check for path.
      auto tmp_cells =
          footstep_layer->GenerateCellsBetweenTwoCells(*start_it, *target_it);
      // ZINFO << "Check start and " << *target_it;
      path_available = true;
      if (start_from_end) {
        for (auto it = tmp_cells.end() - 1;;) {
          if (it == tmp_cells.begin()) {
            break;
          }
          if (!process_map->IsClearedInSensorMap(*it) ||
              check_near_room_edge(*it) || check_near_user_block_area(*it) ||
              check_near_slam_wall(*it)) {
            path_available = false;
            if (std::distance(tmp_cells.begin(), it) <
                static_cast<int>(tmp_cells.size() / 2)) {
              start_from_end = false;
            }
            // ZINFO << "Break for " << it->DebugString();
            break;
          }
          it--;
        }
      } else {
        for (auto it = tmp_cells.begin();;) {
          if (it == tmp_cells.end() - 1) {
            break;
          }
          if (!process_map->IsClearedInSensorMap(*it) ||
              check_near_room_edge(*it) || check_near_user_block_area(*it) ||
              check_near_slam_wall(*it)) {
            path_available = false;
            if (std::distance(tmp_cells.begin(), it) >
                static_cast<int>(tmp_cells.size() / 2)) {
              start_from_end = true;
            }
            // ZINFO << "Break for " << it->DebugString();
            break;
          }
          it++;
        }
      }
      if (path_available) {
        break;
      } else {
        target_it--;
      }
    }
    if (path_available) {
      start_it = target_it;
      // ZINFO << "Get next cell " << *start_it;
    } else {
      start_it++;
      // ZINFO << "Move to next cell " << *start_it;
    }
    straight_end_it = start_it;
  }
  // ZINFO << "New cell path: " << MapCell::DebugString(new_cell_path);

  // Now new_cell_path only contains turning cells.
  output_point_path.clear();
  output_cell_path.clear();
  {
    auto it1 = new_cell_path.begin();
    auto it2 = it1 + 1;
    MapPointPath tmp_path;
    while (true) {
      if (it2 == new_cell_path.end()) {
        break;
      }

      auto _cells = footstep_layer->GenerateCellsBetweenTwoCells(*it1, *it2);
      output_cell_path.insert(output_cell_path.end(), _cells.begin(),
                              _cells.end() - 1);

      MapPoint p1, p2;
      footstep_layer->MapToWorld(*it1, p1);
      footstep_layer->MapToWorld(*it2, p2);
      // ZINFO;
      tmp_path = GenerateInterpolationPoints(
          p1, p2, process_map->GetResolution() * 0.99);
      output_point_path.insert(output_point_path.end(), tmp_path.begin(),
                               tmp_path.end() - 1);

      it1 = it2;
      it2 = it1 + 1;
    }
    output_point_path.emplace_back(tmp_path.back());
    output_cell_path.emplace_back(new_cell_path.back());
  }
  // ZINFO << "New cell path: " << MapCell::DebugString(output_cell_path);
  // ZINFO << "New point path: " << MapPoint::DebugString(output_point_path);

  return true;
}

bool PlannerBase::GeneratePathToTarget(
    NavMap::SPtr process_map, const MapPoint& start_pose,
    const MapPoint& target_point, MapPointPath& output_path,
    MapCellPath& output_cell_path,
    const ExpendCondition& additional_expend_cond) {
  auto sensor_map = process_map->GetSensorLayer();
  ReadLocker sensor_map_lock(sensor_map->GetLock());
  auto footstep_map = process_map->GetFootStepLayer();
  ReadLocker footstep_map_read_lock(footstep_map->GetLock());

  MapCell start_cell, target_cell;
  if (!process_map->GetFootStepLayer()->WorldToMap(start_pose, start_cell)) {
    ZWARN << "Start pose is not in footprint map.";
    return false;
  }
  if (!process_map->GetFootStepLayer()->WorldToMap(target_point, target_cell)) {
    ZWARN << "Target pose is not in footprint map.";
    return false;
  }

  ExpendCondition expend_cond = [&](MultiLayersCharGridMap2D::SPtr _map,
                                    MapCell& curr_step, MapCell& next_step) {
    // ZINFO << "Expend for " << next_step;
    if (!footstep_map->GetAvailableBound().Contain(next_step)) {
      return false;
    }
    // Check for block, stop expending if block is ahead.
    if (!process_map->IsClearedInSensorMap(next_step)) {
      return false;
    }

    if (!additional_expend_cond(_map, curr_step, next_step)) {
      return false;
    }

    return true;
  };

  FinishCondition finish_cond = [&](MultiLayersCharGridMap2D::SPtr map,
                                    MapCell& curr_step) {
    return curr_step == target_cell;
  };

  CellStepCallback step_cb = [&](MultiLayersCharGridMap2D::SPtr map,
                                 MapCell& curr_step) { return; };

  // Do nothing.
  IterationStepCallback iter_step_cb = [&](MultiLayersCharGridMap2D::SPtr map,
                                           MapCells& cells) {};

  MapCells start_cells{start_cell};  // Start from current cell.
  MapCell null_cell;
  ZGINFO << typeid(*this).name() << " Start from: " << start_cell.DebugString()
         << ", target: " << target_cell.DebugString();

  // sensor_map->Print(__FILE__, __FUNCTION__, __LINE__);

  if (Search(process_map, start_cells, null_cell, expend_cond, finish_cond,
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
      PrintPathInMap(process_map, output_cell_path);
    }
    return true;
  }
  ZGINFO << typeid(*this).name() << " not found.";

  return false;
}

}  // namespace zima
