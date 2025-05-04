/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/path_planner/quick_scan_house_path_planner.h"

#include <algorithm>

#include "zima/common/util.h"
#include "zima/logger/logger.h"

namespace zima {

bool QuickScanHousePlanner::GeneratePath(NavMap::SPtr map, const MapPoint &start_pose, MapPointPath &output_path, MapCellPath &output_cell_path) {
    bool found = false;
    MapCell start_cell;
    if (!map->GetFootStepLayer()->WorldToMap(start_pose, start_cell)) {
        ZWARN << "Start pose is not in footprint map.";
        return false;
    }

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
    auto slam_char_map = process_map->GetSlamLayer();
    ReadLocker slam_char_map_read_lock(slam_char_map->GetLock());
    bool is_slam_char_map_marked = slam_char_map->IsMarked();
    // ZINFO << "Slam char map marked " << is_slam_char_map_marked;
    if (is_slam_char_map_marked && FLAGS_debug_enable) {
        slam_char_map->Print(__FILE__, __FUNCTION__, __LINE__);
    }
    CharGridMap2D::DataType user_block_value;

    auto data_bound = slam_char_map->GetDataBound();
    data_bound.Expend(user_block_map->GetDataBound().GetMin());
    data_bound.Expend(user_block_map->GetDataBound().GetMax());

    sensor_map_lock.Unlock();
    user_block_map_read_lock.Unlock();
    slam_char_map_read_lock.Unlock();

    if (!InflactObstacleMap(process_map, data_bound, start_cell, NavMap::kRobotCellWidth_2_,
                            [](MultiLayersCharGridMap2D::SPtr map, MapCell &curr_step, MapCell &next_step) { return true; })) {
        return found;
    }

    if (!InflactSlamObstacleMap(process_map, data_bound, start_cell, NavMap::kRobotCellWidth_2_ / 2,
                                [](MultiLayersCharGridMap2D::SPtr map, MapCell &curr_step, MapCell &next_step) { return true; })) {
        return found;
    }

    auto footstep_map = process_map->GetFootStepLayer();
    ReadLocker footstep_map_read_lock(footstep_map->GetLock());
    auto footstep_map_max_clean_bound = process_map->GetMaxCleanBound(std::make_shared<MapCell>(start_cell));
    footstep_map_max_clean_bound.EqualShrink(1);
    ZGINFO << footstep_map_max_clean_bound.DebugString();

    sensor_map_lock.Lock();
    user_block_map_read_lock.Lock();
    slam_char_map_read_lock.Lock();

    ExpendCondition expend_cond = [&](MultiLayersCharGridMap2D::SPtr _map, MapCell &curr_step, MapCell &next_step) {
        // ZINFO << "Expend for " << next_step.DebugString();
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
        if (user_block_value == NavMap::kVirtualWall_ || user_block_value == NavMap::kStrictBlockArea_) {
            return false;
        }

        slam_char_map->GetValue(curr_step.X(), curr_step.Y(), value);
        if (value == NavMap::kSlamWall_) {
            return false;
        }

        // ZINFO << "Expend for " << next_step.DebugString() << " success.";
        return true;
    };

    FinishCondition finish_cond = [&](MultiLayersCharGridMap2D::SPtr map, MapCell &curr_step) {
        if (is_slam_char_map_marked) {
            if (curr_step != start_cell) {
                slam_char_map->GetValue(curr_step.X(), curr_step.Y(), value);
                if (value == NavMap::kUnknown_) {
                    return true;
                }
            }
        } else {
            if (footstep_map->GetValue(curr_step.X(), curr_step.Y(), value)) {
                if (value == NavMap::kUnknown_) {
                    return true;
                }
            } else if (footstep_map->GetAvailableBound().Contain(curr_step)) {
                return true;
            }
        }
        return false;
    };

    CellStepCallback step_cb = [&](MultiLayersCharGridMap2D::SPtr map, MapCell &curr_step) { return; };

    // Do nothing.
    IterationStepCallback iter_step_cb = [&](MultiLayersCharGridMap2D::SPtr map, MapCells &cells) {};

    MapCells start_cells{start_cell}; // Start from current cell.
    MapCell target_cell;
    ZGINFO << typeid(*this).name() << " Start from: " << start_cell.DebugString();

    if (Search(process_map, start_cells, target_cell, expend_cond, finish_cond, step_cb, iter_step_cb)) {
        // Found uncleaned target.
        output_cell_path.clear();
        TracePath(start_cell, target_cell, output_cell_path);
        output_path.clear();
        CharGridMap2D::CellPathToPointPath(footstep_map, output_cell_path, output_path);

        MapPointPath optimized_point_path;
        MapCellPath optimized_cell_path;
        if (OptimizePathToShortest(process_map, output_cell_path, optimized_point_path, optimized_cell_path, NavMap::kUnknown_, true)) {
            output_cell_path = optimized_cell_path;
            output_path = optimized_point_path;
        }

        ZGINFO << "Found target: " << target_cell.DebugString();
        ZGINFO << "Path: " << MapCell::DebugString(output_cell_path);
        if (FLAGS_debug_enable) {
            PrintPathInMap(map, output_cell_path);
        }
        return true;
    }
    ZGINFO << typeid(*this).name() << " not found.";

    // PrintPathInMap(process_map, slam_char_map->GetDataBound(), output_cell_path);
    // PrintPathInMap(map, slam_char_map->GetDataBound(), output_cell_path);
    return false;
}

} // namespace zima
