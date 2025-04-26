/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_DIJKSTRA_H
#define ZIMA_DIJKSTRA_H

#include <functional>

#include "zima/common/debug.h"
#include "zima/grid_map/multi_layers_map_2d.h"

namespace zima {

class DijkstraBase : public DebugBase {
 public:
  DijkstraBase() : cost_map_("Dijkstra cost map", 0, 0, 0, '.'){};
  ~DijkstraBase() = default;

  /**
   * @brief  Function type of judgement for whether next step is qualified based
   * on current step info and next step info.
   * @param  map Multi-layers map shared pointer.
   * @param  curr_step The cell for current step.
   * @param  next_step The cell for next step.
   * @return true for next step is qualified.
   */
  using ExpendCondition =
      std::function<bool(MultiLayersCharGridMap2D::SPtr map, MapCell& curr_step,
                         MapCell& next_step)>;

  /**
   * @brief  Function type of judgement for whether current step is the target.
   * @param  map Multi-layers map shared pointer.
   * @param  curr_step The cell for current step.
   * @return true for current step is target, and search will be stoped.
   */
  using FinishCondition = std::function<bool(MultiLayersCharGridMap2D::SPtr map,
                                             MapCell& curr_step)>;

  /**
   * @brief  Function type of callback for every step it takes.
   * @param  map Multi-layers map shared pointer.
   * @param  curr_step The cell for current step.
   */
  using CellStepCallback = std::function<void(
      MultiLayersCharGridMap2D::SPtr map, MapCell& curr_step)>;

  /**
   * @brief  Function type of callback before iteration update.
   * @param  map Multi-layers map shared pointer.
   * @param  new_iter_cells The cells for next iteration.
   */
  using IterationStepCallback = std::function<void(
      MultiLayersCharGridMap2D::SPtr map, MapCells& new_iter_cells)>;

  /**
   * @brief  Framework of map searching, inspired by dijkstra algorithm.
   * @param  map Multi-layers map shared pointer, the framework itself will
   * not access to any layers, it will just pass it to the condition/callback
   * functions.
   * @param  start_cells The cells for start searching iteration.
   * @param  target The target cell output interface for finding target cell
   * using finish_cond.
   * @param  expend_cond The judgement function for deciding whether the search
   * step is qualified.
   * @param  finish_cond The judgement function for deciding whether this step
   * is the target cell we want, if this fucntion returns true, it will stop
   * searching.
   * @param  cell_step_cb The callback function which will be executed for every
   * steps.
   * @param  iter_step_cb The callback function which will be executed for every
   * level iterations.
   * @return True for finish_cond is met.
   */
  bool Search(MultiLayersCharGridMap2D::SPtr map, const MapCells& start_cells,
              MapCell& target, const ExpendCondition& expend_cond,
              const FinishCondition& finish_cond,
              const CellStepCallback& cell_step_cb,
              const IterationStepCallback& iter_step_cb);
  /**
   * @brief  Trace path from goal back to start point according to cost map.
   * @param  start The start cell of this path.
   * @param  goal The goal cell of this path.
   * @param  path The output path from start to goal.
   * @return True for successfully traced.
   */
  bool TracePath(const MapCell& start, const MapCell& goal, MapCellPath& path);

  //   const unsigned int kCostValueMin_ = 1;
  //   const unsigned int kCostValueMax_ = 5;
  const CharGridMap2D::DataType kCostValueMin_ = '1';
  const CharGridMap2D::DataType kCostValueMax_ = '5';

 protected:
  // For marking searched cells.
  CharGridMap2D cost_map_;
};

}  // namespace zima

#endif  // ZIMA_DIJKSTRA_H
