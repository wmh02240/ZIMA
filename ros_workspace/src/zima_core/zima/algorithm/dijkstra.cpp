/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/algorithm/dijkstra.h"

#include <algorithm>

#include "zima/logger/logger.h"

namespace zima {

bool DijkstraBase::Search(MultiLayersCharGridMap2D::SPtr map,
                          const MapCells& start_cells, MapCell& target,
                          const ExpendCondition& expend_cond,
                          const FinishCondition& finish_cond,
                          const CellStepCallback& step_cb,
                          const IterationStepCallback& iter_step_cb) {
  bool found = false;

  // ZINFO << map->GetResolution();
  // ZINFO << map.get();
  // ZINFO << map->DebugString();
  // Initialize for costmap.
  const char kCostMapDefaultValue = '.';
  cost_map_.ResetMap(map->GetRangeX(), map->GetRangeY(), map->GetResolution(),
                     kCostMapDefaultValue);
  // cost_map_.Print(__FILE__, __FUNCTION__, __LINE__);

  MapCells step_cells;
  // Expend for 4 directions.
  MapCells relative_expend_step_cells;
  relative_expend_step_cells.emplace_back(MapCell(1, 0));
  relative_expend_step_cells.emplace_back(MapCell(-1, 0));
  relative_expend_step_cells.emplace_back(MapCell(0, 1));
  relative_expend_step_cells.emplace_back(MapCell(0, -1));

  // Init for first steps.
  step_cells = start_cells;

  CharGridMap2D::DataType current_cost = kCostValueMin_;

  bool stop = false;

  WriteLocker lock(cost_map_.GetLock());
  // Start dijkstra search loop.
  while (!stop) {
    MapCellsSet next_step_cells_set;

    // cost_map_.Print(__FILE__, __FUNCTION__, __LINE__);

    for (auto&& current_step : step_cells) {
      // Executing for this step.
      step_cb(map, current_step);

      if (finish_cond(map, current_step)) {
        cost_map_.SetValue(current_step.X(), current_step.Y(), current_cost);
        target = current_step;
        found = true;
        stop = true;
        break;
      }
      // ZINFO << current_step.DebugString() << " with "
      //       << current_cost;
      cost_map_.SetValue(current_step.X(), current_step.Y(), current_cost);

      // Try to find next steps.
      for (auto&& expend_step_cell : relative_expend_step_cells) {
        auto next_step = current_step + expend_step_cell;
        // Check if next step in valid range.
        if (!cost_map_.GetAvailableBound().Contain(next_step)) {
          continue;
        }

        // check if next step has been stepped on.
        CharGridMap2D::DataType next_value;
        if (cost_map_.GetValue(next_step.X(), next_step.Y(), next_value)) {
          if (next_value != kCostMapDefaultValue) {
            continue;
          }

        } else if (!cost_map_.GetAvailableBound().Contain(next_step)) {
          continue;
        }
        if (expend_cond(map, current_step, next_step)) {
          next_step_cells_set.emplace(next_step);
        }
      }
    }

    MapCells next_step_cells;
    for (auto&& cell : next_step_cells_set) {
      next_step_cells.emplace_back(cell);
    }
    iter_step_cb(map, next_step_cells);

    if (next_step_cells.empty()) {
      break;
    }

    // Search for next level.
    step_cells.swap(next_step_cells);

    current_cost += 1;
    // current_cost =
    //     static_cast<unsigned char>(static_cast<unsigned int>(current_cost) +
    //     1);
    // ZINFO << "current cost: " << current_cost;

    if (current_cost > kCostValueMax_) {
      current_cost = kCostValueMin_;
    }
  }

  // for (auto&& str : cost_map_.DebugString()) {
  //   ZISODBG << str;
  // }
  return found;
}

bool DijkstraBase::TracePath(const MapCell& start, const MapCell& goal,
                             MapCellPath& path) {
  auto step = goal;
  path.emplace_back(step);
  MapCells relative_expend_step_cells;
  relative_expend_step_cells.emplace_back(MapCell(1, 0));
  relative_expend_step_cells.emplace_back(MapCell(-1, 0));
  relative_expend_step_cells.emplace_back(MapCell(0, 1));
  relative_expend_step_cells.emplace_back(MapCell(0, -1));

  ReadLocker lock(cost_map_.GetLock());

  // cost_map_.Print(__FILE__, __FUNCTION__, __LINE__);

  auto current_step_cost = cost_map_.GetDefaultValue();
  cost_map_.GetValue(step.X(), step.Y(), current_step_cost);
  auto _current_step_cost = current_step_cost;
  auto _next_step_cost = _current_step_cost;
  while (step != start) {
    if (_current_step_cost <= kCostValueMin_) {
      _next_step_cost = kCostValueMax_;
    } else {
      _next_step_cost = _current_step_cost - 1;
    }

    // ZINFO << "Current " << _current_step_cost;
    // ZINFO << "Next " << _next_step_cost;

    // Find next step
    for (auto it = relative_expend_step_cells.begin();
         it != relative_expend_step_cells.end(); it++) {
      auto next_step = step + *it;
      // ZINFO << "Next step " << next_step;
      // Check if next step in valid range.
      if (!cost_map_.GetDataBound().Contain(next_step)) {
        continue;
      }

      auto cost = cost_map_.GetDefaultValue();
      cost_map_.GetValue(next_step.X(), next_step.Y(), cost);
      // ZINFO << "Next step " << next_step << " cost " << cost;
      if (cost == _next_step_cost) {
        path.emplace_back(next_step);
        step = next_step;

        // Re-order relative_expend_step_cells for piority search direction
        auto expend_step = *it;
        relative_expend_step_cells.erase(it);
        relative_expend_step_cells.emplace_front(expend_step);
        // ZINFO << next_step;
        break;
      }
    }
    _current_step_cost = _next_step_cost;
  }

  if (path.size() == 1) {
    path.emplace_back(start);
  }

  std::reverse(path.begin(), path.end());
  return true;
}

}  // namespace zima
