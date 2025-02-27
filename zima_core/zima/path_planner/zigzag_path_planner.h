/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_ZIGZAG_PATH_PLANNER_H
#define ZIMA_ZIGZAG_PATH_PLANNER_H

#include "zima/path_planner/path_planner.h"

namespace zima {

class ZigZagPlanner : public PlannerBase {
 public:
  ZigZagPlanner() = default;
  ~ZigZagPlanner() = default;

  using SPtr = std::shared_ptr<ZigZagPlanner>;

  enum PathType {
    ZIGZAG_PATH,
    NEAREST_UNCLEAN_TARGET_PATH,
  };

  struct ZigZagPath {
    PathType type;
    MapPointPath path;
  };

  /**
   * @brief  Interface for generating path for zigzag path cleaning.
   *
   * @param  map Multi-layers map shared pointer.
   * @param  start_pose Start(current) pose for robot.
   * @param  bound Bound limit for cleaning section.
   * @param  current_room_index Index for current cleaning room.
   * @param  output_path Output path, only valid if function returns true.
   * @param  use_x_direction Indicator for cleaning along x axix or y axix.
   *
   * @return True for generate path successfully.
   */
  virtual bool GeneratePath(NavMap::SPtr map, const MapPoint& start_pose,
                            const DynamicMapCellBound& bound,
                            const CharGridMap2D::DataType& current_room_index,
                            ZigZagPath& output_path,
                            MapCellPath& output_cell_path,
                            const bool& use_x_direction) = 0;

 protected:
  uint32_t CountUncleanedCell(NavMap::SPtr map, const MapCell& start_pose,
                              const DynamicMapCellBound& bound,
                              const CharGridMap2D::DataType& current_room_index,
                              const bool& use_x_direction,
                              const bool& is_positive);

  const unsigned int kInflationSize = 3;
};

class CurrentLinePlanner : public ZigZagPlanner {
 public:
  CurrentLinePlanner() = delete;
  CurrentLinePlanner(const bool& positive_direction,
                     const unsigned int& half_line_width) {
    positive_direction_ = positive_direction;
    half_line_width_ = half_line_width;
    ZGINFO << (positive_direction_ ? "Positive" : "Negative");
  };
  ~CurrentLinePlanner() = default;

 protected:
  friend class ZigZagPlannerManager;
  friend class CurrentLinePriorityPlanner;
  friend class OptimizedNextLinePriorityPlanner;
  friend class OptimizedNearestTargetPriorityPlanner;
  /**
   * @brief  Function for generating path for uncleaned cell in current
   * line(This line means cell lines covered by robot).
   * @param  * See interface.
   * @return * See interface.
   */
  bool GeneratePath(NavMap::SPtr map, const MapPoint& start_pose,
                    const DynamicMapCellBound& bound,
                    const CharGridMap2D::DataType& current_room_index,
                    ZigZagPath& output_path, MapCellPath& output_cell_path,
                    const bool& use_x_direction) override;

  bool positive_direction_;
  unsigned int half_line_width_;
};

class CurrentLinePositivePlanner : public CurrentLinePlanner {
 public:
  CurrentLinePositivePlanner();
  ~CurrentLinePositivePlanner() = default;
};

class CurrentLineNegativePlanner : public CurrentLinePlanner {
 public:
  CurrentLineNegativePlanner();
  ~CurrentLineNegativePlanner() = default;
};

class CurrentLinePriorityPlanner : public ZigZagPlanner {
 public:
  CurrentLinePriorityPlanner() = default;
  ~CurrentLinePriorityPlanner() = default;

 protected:
  friend class ZigZagPlannerManager;
  /**
   * @brief  Function for generating path for uncleaned cell in current
   * line(This line means cell lines covered by robot).
   * @param  * See interface.
   * @return * See interface.
   */
  bool GeneratePath(NavMap::SPtr map, const MapPoint& start_pose,
                    const DynamicMapCellBound& bound,
                    const CharGridMap2D::DataType& current_room_index,
                    ZigZagPath& output_path, MapCellPath& output_cell_path,
                    const bool& use_x_direction) override;

 private:
  CurrentLinePositivePlanner pos_planner_;
  CurrentLineNegativePlanner neg_planner_;
};

class NextLinePlanner : public ZigZagPlanner {
 public:
  NextLinePlanner() = delete;
  NextLinePlanner(const bool& positive_direction,
                  const unsigned int& half_line_width) {
    positive_direction_ = positive_direction;
    half_line_width_ = half_line_width;
    ZGINFO << (positive_direction_ ? "Positive" : "Negative");
  };
  ~NextLinePlanner() = default;

 protected:
  friend class ZigZagPlannerManager;
  friend class NextLinePriorityPlanner;
  /**
   * @brief  Function for generating path for uncleaned cell in next
   * line(This line means cell lines covered by robot).
   * @param  * See interface.
   * @return * See interface.
   */
  bool GeneratePath(NavMap::SPtr map, const MapPoint& start_pose,
                    const DynamicMapCellBound& bound,
                    const CharGridMap2D::DataType& current_room_index,
                    ZigZagPath& output_path, MapCellPath& output_cell_path,
                    const bool& use_x_direction) override;

  bool positive_direction_;
  unsigned int half_line_width_;
};

class NextLinePositivePlanner : public NextLinePlanner {
 public:
  NextLinePositivePlanner();
  ~NextLinePositivePlanner() = default;
};

class NextLineNegativePlanner : public NextLinePlanner {
 public:
  NextLineNegativePlanner();
  ~NextLineNegativePlanner() = default;
};

class NextLinePriorityPlanner : public ZigZagPlanner {
 public:
  NextLinePriorityPlanner() = default;
  ~NextLinePriorityPlanner() = default;

 protected:
  friend class ZigZagPlannerManager;
  friend class OptimizedNextLinePriorityPlanner;
  /**
   * @brief  Function for generating path for uncleaned cell in next line.
   * @param  * See interface.
   * @return * See interface.
   */
  bool GeneratePath(NavMap::SPtr map, const MapPoint& start_pose,
                    const DynamicMapCellBound& bound,
                    const CharGridMap2D::DataType& current_room_index,
                    ZigZagPath& output_path, MapCellPath& output_cell_path,
                    const bool& use_x_direction) override;

 private:
  NextLinePositivePlanner pos_planner_;
  NextLineNegativePlanner neg_planner_;
};

class OptimizedNextLinePriorityPlanner : public ZigZagPlanner {
 public:
  OptimizedNextLinePriorityPlanner() = default;
  ~OptimizedNextLinePriorityPlanner() = default;

 protected:
  friend class ZigZagPlannerManager;
  /**
   * @brief  Function for generating path for uncleaned cell in next line, and
   * optimize it.
   * @param  * See interface.
   * @return * See interface.
   */
  bool GeneratePath(NavMap::SPtr map, const MapPoint& start_pose,
                    const DynamicMapCellBound& bound,
                    const CharGridMap2D::DataType& current_room_index,
                    ZigZagPath& output_path, MapCellPath& output_cell_path,
                    const bool& use_x_direction) override;

 private:
  NextLinePriorityPlanner next_line_planner_;
  CurrentLinePositivePlanner current_line_pos_planner_;
  CurrentLineNegativePlanner current_line_neg_planner_;
};

class NearestTargetPlanner : public ZigZagPlanner {
 public:
  NearestTargetPlanner() = delete;
  explicit NearestTargetPlanner(const bool& positive_direction) {
    positive_direction_ = positive_direction;
    ZGINFO << (positive_direction_ ? "Positive" : "Negative");
  };
  ~NearestTargetPlanner() = default;

 protected:
  friend class ZigZagPlannerManager;
  friend class NearestTargetPriorityPlanner;
  /**
   * @brief  Function for generating path for uncleaned cell in current
   * bound.
   * @param  * See interface.
   * @return * See interface.
   */
  bool GeneratePath(NavMap::SPtr map, const MapPoint& start_pose,
                    const DynamicMapCellBound& bound,
                    const CharGridMap2D::DataType& current_room_index,
                    ZigZagPath& output_path, MapCellPath& output_cell_path,
                    const bool& use_x_direction) override;

 private:
  bool positive_direction_;
};

class NearestTargetPositivePlanner : public NearestTargetPlanner {
 public:
  NearestTargetPositivePlanner();
  ~NearestTargetPositivePlanner() = default;
};

class NearestTargetNegativePlanner : public NearestTargetPlanner {
 public:
  NearestTargetNegativePlanner();
  ~NearestTargetNegativePlanner() = default;
};

class NearestTargetPriorityPlanner : public ZigZagPlanner {
 public:
  NearestTargetPriorityPlanner() = default;
  ~NearestTargetPriorityPlanner() = default;

 protected:
  friend class ZigZagPlannerManager;
  friend class OptimizedNearestTargetPriorityPlanner;
  /**
   * @brief  Function for generating path for nearest uncleaned cell.
   * @param  * See interface.
   * @return * See interface.
   */
  bool GeneratePath(NavMap::SPtr map, const MapPoint& start_pose,
                    const DynamicMapCellBound& bound,
                    const CharGridMap2D::DataType& current_room_index,
                    ZigZagPath& output_path, MapCellPath& output_cell_path,
                    const bool& use_x_direction) override;

 private:
  NearestTargetPositivePlanner pos_planner_;
  NearestTargetNegativePlanner neg_planner_;
};

class OptimizedNearestTargetPriorityPlanner : public ZigZagPlanner {
 public:
  OptimizedNearestTargetPriorityPlanner() = default;
  ~OptimizedNearestTargetPriorityPlanner() = default;

 protected:
  friend class ZigZagPlannerManager;
  /**
   * @brief  Function for generating path for uncleaned cell in next line, and
   * optimize it.
   * @param  * See interface.
   * @return * See interface.
   */
  bool GeneratePath(NavMap::SPtr map, const MapPoint& start_pose,
                    const DynamicMapCellBound& bound,
                    const CharGridMap2D::DataType& current_room_index,
                    ZigZagPath& output_path, MapCellPath& output_cell_path,
                    const bool& use_x_direction) override;

 private:
  NearestTargetPriorityPlanner nearest_target_planner_;
  CurrentLinePositivePlanner current_line_pos_planner_;
  CurrentLineNegativePlanner current_line_neg_planner_;
};


class ZigZagPlannerManager : public ZigZagPlanner {
 public:
  ZigZagPlannerManager() = default;
  ~ZigZagPlannerManager() = default;

  /**
   * @brief  Function for initializing planners queue, the sequence matters
   * because once path is generated, the planners left will not be tried.
   */
  bool Initialize();

  /**
   * @brief  Function for generating path for zigzag path cleaning.
   * It will inflate different obstacle map and then try different types of
   * planners for generating.
   * @param  * See interface.
   * @return * See interface.
   */
  bool GeneratePath(NavMap::SPtr map, NavMap::SPtr section_map,
                    const MapPoint& start_pose,
                    const DynamicMapCellBound& bound,
                    const CharGridMap2D::DataType& current_room_index,
                    ZigZagPath& output_path, MapCellPath& output_cell_path,
                    const bool& use_x_direction);

 protected:
  /**
   * @brief  Not using.
   */
  bool GeneratePath(NavMap::SPtr map, const MapPoint& start_pose,
                    const DynamicMapCellBound& bound,
                    const CharGridMap2D::DataType& current_room_index,
                    ZigZagPath& output_path, MapCellPath& output_cell_path,
                    const bool& use_x_direction) override {
    ZERROR << "DO NOT USE THIS INTERFACE.";
    return false;
  };

 private:
  std::deque<ZigZagPlanner::SPtr> planner_queue_;
  uint8_t generate_path_count_;
};

}  // namespace zima

#endif  // ZIMA_ZIGZAG_PATH_PLANNER_H
