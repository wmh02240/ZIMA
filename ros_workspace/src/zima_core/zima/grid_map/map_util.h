/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_MAP_UTIL_H
#define ZIMA_MAP_UTIL_H

#include "zima/algorithm/slam/probability_map.h"
#include "zima/common/debug.h"
#include "zima/grid_map/nav_map_2d.h"
#include "zima/robot_data/nav_data.h"

namespace zima {

class MapConverter : public DebugBase {
 public:
  /**
   * @brief  Function for converting slam map to a single room map, it should be
   * run after optimization of small visiable area.
   * @param  slam_map input slam value grid map pointer.
   * @param  room_map output room map(char grid map) pointer.
   * @return true for convertion finished.
   */
  bool ConvertSlamValueGridMap2DToRoomMap(
      const SlamValueGridMap2D::SPtr& slam_map, CharGridMap2D::SPtr& room_map);

  /**
   * @brief  Function for converting slam map to a char 2D map, it should be
   * run after optimization of small visiable area.
   * @param  slam_map input slam value grid map pointer.
   * @param  char_slam_map output room map(char grid map) pointer.
   * @return true for convertion finished.
   */
  bool ConvertSlamValueGridMap2DToCharSlamMap(
      const SlamValueGridMap2D::SPtr& slam_map,
      CharGridMap2D::SPtr& char_slam_map);

  /**
   * @brief  Function for converting slam probability map to a slam value 2D
   * map.
   * @param  probability_map input probability grid map pointer.
   * @param  slam_map output slam value grid map pointer.
   * @return true for convertion finished.
   */
  bool ConvertProbabilityGridMap2DToSlamValueGridMap(
      const ProbabilityIndexGridMap2D::SPtr& probability_map,
      SlamValueGridMap2D::SPtr& slam_map);

};

class SlamMapUtil : public DebugBase {
 public:
  /**
   * @brief  Function for removing small area of visiable area in slam map.
   * @param  slam_map input slam value grid map pointer.
   * @param  room_map output room map(char grid map) pointer.
   * @return true for convertion finished.
   */
  bool OptimizeSlamValueGridMap2DForUser(SlamValueGridMap2D::SPtr& slam_map);

  static bool IsVisiable(const SlamValueGridMap2D::DataType& value,
                         const SlamValueGridMap2D::DataType& min_value =
                             SlamValueGridMap2D::GetPredefineDefaultValue() + 1,
                         const SlamValueGridMap2D::DataType& max_value =
                             SlamValueGridMap2D::GetPredefineMediumValue()) {
    return value >= min_value && value <= max_value;
  }
  static bool IsObstacle(
      const SlamValueGridMap2D::DataType& value,
      const SlamValueGridMap2D::DataType& min_value =
          SlamValueGridMap2D::GetPredefineMediumValue() + 1,
      const SlamValueGridMap2D::DataType& max_value =
          SlamValueGridMap2D::GetPredefineMaxObstacleValue()) {
    return value >= min_value && value <= max_value;
  }
  static bool IsUnknown(const SlamValueGridMap2D::DataType& value,
                        const SlamValueGridMap2D::DataType& min_value =
                            SlamValueGridMap2D::GetPredefineMediumValue() + 1,
                        const SlamValueGridMap2D::DataType& max_value =
                            SlamValueGridMap2D::GetPredefineMediumValue() - 1) {
    return (value >= min_value && value <= max_value) ||
           value == SlamValueGridMap2D::GetPredefineDefaultValue();
  }
};

class RoomMapUtil : public DebugBase {
 public:
  /**
   * @brief  Function for checking if rooms' area are changed with slam map.
   * @param  room_map output room map(char grid map) pointer.
   * @return true for pass checking or successfully fixed.
   */
  bool CheckAndUpdateRoomMap(const SlamValueGridMap2D::SPtr& slam_map,
                             CharGridMap2D::SPtr& room_map);

  /**
   * @brief  Function for generate corresponding rooms info of room map.
   * Normally used after auto split room map.
   * @param  room_map input room map(char grid map) pointer.
   * @param  rooms_info output rooms info.
   * @return true for generation succeeded.
   */
  bool UpdateRoomsInfo(const CharGridMap2D::SPtr& room_map,
                       RoomsInfo& rooms_info);

  /**
   * @brief  Function for spliting room into several rooms, and update rooms
   * info.
   * @param  room_map output room map(char grid map) pointer.
   * @param  target_room_value input selected room.
   * @param  split_line input TWO edge cells of the split line.
   * @param  rooms_info output new rooms info.
   * @return true for spliting succeeded.
   */
  bool SplitRoom(CharGridMap2D::SPtr& room_map,
                 const CharGridMap2D::DataType& target_room_value,
                 const MapCells& split_line, RoomsInfo& rooms_info);

  /**
   * @brief  Function for merging two connected room into one room, and update
   * rooms info.
   * @param  room_map output room map(char grid map) pointer.
   * @param  merge_room_1 input selected room 1.
   * @param  merge_room_2 input selected room 2.
   * @param  rooms_info output new rooms info.
   * @return true for merge succeeded.
   */
  bool MergeRoom(CharGridMap2D::SPtr& room_map,
                 const CharGridMap2D::DataType& merge_room_1,
                 const CharGridMap2D::DataType& merge_room_2,
                 RoomsInfo& rooms_info);

  /**
   * @brief  Function for generating correct cell range for room.
   * @param  room_map input room map(char grid map) pointer.
   * @param  room_bound input cell bound of this room.
   * @param  range_x output x_range.
   * @param  range_y output y_range.
   * @return true for generation succeeded.
   */
  bool GenerateXYRangeForRoom(const CharGridMap2D::SPtr& room_map,
                              const DynamicMapCellBound& room_bound,
                              uint16_t& range_x, uint16_t& range_y);
};

/*
 * Practical pipeline for processing slam map:
 *
 *                           Raw slam map
 *                                 |
 *                                 v
 *                 OptimizeSlamValueGridMap2DForUser
 *                                 |
 *                    ------------------------
 *                    |                      |
 *                    v                      v
 *  ConvertSlamValueGridMap2DToRoomMap    CheckAndUpdateRoomMap
 *                    |                      |
 *                    v                      v
 *        Get initial room map.          Update existing room map.
 *                    |                      |
 *                    v                      v
 *                    ------------------------
 *                                 |
 *                                 v
 *                            UpdateRoomsInfo
 */

class BicubicInterpolator : public DebugBase {
 public:
  BicubicInterpolator() = default;
  ~BicubicInterpolator() = default;

  static float BiCubicBaseFunctionCalculateWeight(const float& a,
                                                  const float& input);
  static std::shared_ptr<float> Interpolate(
      const ProbabilityIndexGridMap2D::SPtr probability_index_grid_map,
      const MapPoint& interpolate_point,
      const float& a = -0.5);
  static FloatValueGridMap2D::SPtr Interpolate(
      const ProbabilityIndexGridMap2D::SPtr probability_index_grid_map,
      const float& scale, const float& a = -0.5);
};

}  // namespace zima

#endif  // ZIMA_MAP_UTIL_H
