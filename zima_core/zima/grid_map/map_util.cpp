/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/grid_map/map_util.h"

#include <algorithm>

#include "zima/algorithm/dijkstra.h"
#include "zima/common/util.h"

namespace zima {

const uint32_t kMinRoomCellsCount = 250;

bool MapConverter::ConvertSlamValueGridMap2DToRoomMap(
    const SlamValueGridMap2D::SPtr& slam_map, CharGridMap2D::SPtr& room_map) {
  if (slam_map == nullptr) {
    ZERROR << "Slam map ptr invalid.";
    return false;
  }
  if (room_map == nullptr) {
    ZERROR << "Room map ptr invalid.";
    return false;
  }
  if (!FloatEqual(slam_map->GetResolution(), room_map->GetResolution())) {
    ZERROR << "Slam map resolution: "
           << FloatToString(slam_map->GetResolution(), 3)
           << ", room map resolution: "
           << FloatToString(room_map->GetResolution(), 3);
    return false;
  }
  // for (auto&& str : slam_map->DebugString()) {
  //   ZISODBG << str;
  // }
  ReadLocker slam_map_lock(slam_map->GetLock());
  room_map->ResetMap(slam_map->GetRangeX(), slam_map->GetRangeY(),
                     slam_map->GetResolution(), room_map->GetDefaultValue());

  WriteLocker room_map_lock(room_map->GetLock());
  auto data_bound = slam_map->GetDataBound();
  auto check_map_value = [&](const SlamValueGridMap2D::DataType& slam_map_value,
                             const CharGridMap2D::DataType& room_map_value) {
    return SlamMapUtil::IsVisiable(slam_map_value) &&
           room_map_value == room_map->GetDefaultValue();
  };
  SlamValueGridMap2D::DataType slam_map_value;
  CharGridMap2D::DataType room_map_value;
  for (auto x = data_bound.GetMin().X(); x <= data_bound.GetMax().X(); x++) {
    for (auto y = data_bound.GetMin().Y(); y <= data_bound.GetMax().Y(); y++) {
      slam_map->GetValue(x, y, slam_map_value);
      room_map->GetValue(x, y, room_map_value);
      if (check_map_value(slam_map_value, room_map_value)) {
        room_map->SetValue(x, y, NavMap::kRoomA_);
      }
    }
  }

  // for (auto&& str : room_map->DebugString()) {
  //   ZISODBG << str;
  // }
  return true;
}

bool MapConverter::ConvertSlamValueGridMap2DToCharSlamMap(
    const SlamValueGridMap2D::SPtr& slam_map,
    CharGridMap2D::SPtr& char_slam_map) {
  if (slam_map == nullptr) {
    ZERROR << "Slam map ptr invalid.";
    return false;
  }
  if (char_slam_map == nullptr) {
    ZERROR << "Char slam map ptr invalid.";
    return false;
  }
  if (!FloatEqual(slam_map->GetResolution(), char_slam_map->GetResolution())) {
    ZERROR << "Slam map resolution: "
           << FloatToString(slam_map->GetResolution(), 3)
           << ", char slam map resolution: "
           << FloatToString(char_slam_map->GetResolution(), 3);
    return false;
  }
  // for (auto&& str : slam_map->DebugString()) {
  //   ZISODBG << str;
  // }
  ReadLocker slam_map_lock(slam_map->GetLock());
  char_slam_map->ResetMap(slam_map->GetRangeX(), slam_map->GetRangeY(),
                          slam_map->GetResolution(),
                          char_slam_map->GetDefaultValue());

  WriteLocker char_slam_map_lock(char_slam_map->GetLock());
  auto data_bound = slam_map->GetDataBound();
  SlamValueGridMap2D::DataType slam_map_value;
  for (auto x = data_bound.GetMin().X(); x <= data_bound.GetMax().X(); x++) {
    for (auto y = data_bound.GetMin().Y(); y <= data_bound.GetMax().Y(); y++) {
      slam_map->GetValue(x, y, slam_map_value);
      if (SlamMapUtil::IsVisiable(slam_map_value)) {
        char_slam_map->SetValue(x, y, NavMap::kSlamFloor_);
      } else if (SlamMapUtil::IsObstacle(slam_map_value)) {
        char_slam_map->SetValue(x, y, NavMap::kSlamWall_);
      }
    }
  }

  // EnableDebugLog();
  // for (auto&& str : char_slam_map->DebugString()) {
  //   ZISODBG << str;
  // }
  return true;
}

bool MapConverter::ConvertProbabilityGridMap2DToSlamValueGridMap(
    const ProbabilityIndexGridMap2D::SPtr& probability_map,
    SlamValueGridMap2D::SPtr& slam_map) {
  if (probability_map == nullptr) {
    ZERROR << "Probability map ptr invalid.";
    return false;
  }
  if (slam_map == nullptr) {
    ZERROR << "Slam map ptr invalid.";
    return false;
  }
  if (!FloatEqual(probability_map->GetResolution(),
                  slam_map->GetResolution())) {
    ZERROR << "Probability map resolution: "
           << FloatToString(probability_map->GetResolution(), 3)
           << ", slam map resolution: "
           << FloatToString(slam_map->GetResolution(), 3);
    return false;
  }
  // for (auto&& str : probability_map->DebugString()) {
  //   ZINFO << str;
  // }
  ReadLocker probability_map_lock(probability_map->GetLock());
  slam_map->ResetMap(probability_map->GetRangeX(), probability_map->GetRangeY(),
                     probability_map->GetResolution(),
                     slam_map->GetDefaultValue());

  WriteLocker slam_map_lock(slam_map->GetLock());
  auto data_bound = probability_map->GetDataBound();
  float probability = probability_map->GetMinProbabilityForObstacleInCell();
  for (auto x = data_bound.GetMin().X(); x <= data_bound.GetMax().X(); x++) {
    for (auto y = data_bound.GetMin().Y(); y <= data_bound.GetMax().Y(); y++) {
      if (!probability_map->IsUnknown(x, y)) {
        probability_map->GetProbability(x, y, probability);
        slam_map->SetValue(
            x, y,
            Clip(static_cast<SlamValueGridMap2D::DataType>(
                     round(probability * 100)),
                 static_cast<SlamValueGridMap2D::DataType>(0),
                 static_cast<SlamValueGridMap2D::DataType>(100)));
      }
    }
  }

  // EnableDebugLog();
  // for (auto&& str : slam_map->DebugString()) {
  //   ZINFO << str;
  // }
  return true;
}

bool SlamMapUtil::OptimizeSlamValueGridMap2DForUser(
    SlamValueGridMap2D::SPtr& slam_map) {
  if (slam_map == nullptr) {
    ZERROR << "Slam map ptr invalid.";
    return false;
  }

  // ZISODBG << "Before optimization.";
  // for (auto&& str : slam_map->DebugString()) {
  //   ZISODBG << str;
  // }

  // Consider the connectivity of room value.
  WriteLocker slam_map_lock(slam_map->GetLock());
  auto checked_map = std::make_shared<CharGridMap2D>(
      "Check map", slam_map->GetRangeX(), slam_map->GetRangeY(),
      slam_map->GetResolution(), slam_map->GetDefaultValue());
  WriteLocker checked_map_lock(checked_map->GetLock());
  auto data_bound = slam_map->GetDataBound();
  auto check_visiable = [&](const SlamValueGridMap2D::DataType& slam_map_value,
                            const CharGridMap2D::DataType& room_map_value) {
    return SlamMapUtil::IsVisiable(slam_map_value) &&
           room_map_value == checked_map->GetDefaultValue();
  };

  MapCells check_cells;
  check_cells.emplace_back(MapCell(1, 0));
  check_cells.emplace_back(MapCell(-1, 0));
  check_cells.emplace_back(MapCell(0, 1));
  check_cells.emplace_back(MapCell(0, -1));

  SlamValueGridMap2D::DataType check_slam_map_value;
  auto check_unneeded_obstacle =
      [&](const SlamValueGridMap2D::DataType& slam_map_value,
          const MapCell& cell) {
        if (SlamMapUtil::IsObstacle(slam_map_value)) {
          for (auto&& check_cell : check_cells) {
            auto _check_cell = cell + check_cell;
            slam_map->GetValue(_check_cell.X(), _check_cell.Y(),
                               check_slam_map_value);
            if (SlamMapUtil::IsVisiable(check_slam_map_value)) {
              return false;
            }
          }
          return true;
        }
        return false;
      };

  DijkstraBase dijkstra;
  SlamValueGridMap2D::DataType slam_map_value;
  CharGridMap2D::DataType checked_map_value;
  // 1. Fill all unknown cell to default value.
  for (auto x = data_bound.GetMin().X(); x <= data_bound.GetMax().X(); x++) {
    for (auto y = data_bound.GetMin().Y(); y <= data_bound.GetMax().Y(); y++) {
      slam_map->GetValue(x, y, slam_map_value);
      if (SlamMapUtil::IsUnknown(slam_map_value) &&
          slam_map_value != slam_map->GetDefaultValue()) {
        slam_map->SetValue(x, y, slam_map->GetDefaultValue());
      }
    }
  }

  // 2. Remove independent small visiable area.
  for (auto x = data_bound.GetMin().X(); x <= data_bound.GetMax().X(); x++) {
    for (auto y = data_bound.GetMin().Y(); y <= data_bound.GetMax().Y(); y++) {
      checked_map->GetValue(x, y, checked_map_value);
      slam_map->GetValue(x, y, slam_map_value);
      if (check_visiable(slam_map_value, checked_map_value)) {
        // Count for connected room cells;
        auto start_cell = MapCell(x, y);
        // ZISODBG << "Start count from: " << start_cell.DebugString();
        MapCells tmp_room_cells;
        MultiLayersCharGridMap2D::SPtr tmp_multi_layered_map =
            std::make_shared<MultiLayersCharGridMap2D>(
                "tmp_multi_layered_map", slam_map->GetRangeX(),
                slam_map->GetRangeY(), slam_map->GetResolution());
        DijkstraBase::ExpendCondition expend_cond =
            [&](MultiLayersCharGridMap2D::SPtr _map, MapCell& curr_step,
                MapCell& next_step) {
              slam_map->GetValue(next_step.X(), next_step.Y(), slam_map_value);
              checked_map->GetValue(next_step.X(), next_step.Y(),
                                    checked_map_value);
              // ZISODBG << "Check: " << next_step.DebugString()
              //        << " value: " << checked_map_value;
              return check_visiable(slam_map_value, checked_map_value);
            };
        // Check nothing.
        DijkstraBase::FinishCondition finish_cond =
            [&](MultiLayersCharGridMap2D::SPtr _map, MapCell& curr_step) {
              return false;
            };
        DijkstraBase::CellStepCallback step_cb =
            [&](MultiLayersCharGridMap2D::SPtr map, MapCell& curr_step) {
              tmp_room_cells.emplace_back(curr_step);
              checked_map->SetValue(curr_step.X(), curr_step.Y(),
                                    NavMap::kRoomA_);
              // for (auto&& str : checked_map->DebugString()) {
              //   ZISODBG << str;
              // }
              return;
            };

        // Do nothing.
        DijkstraBase::IterationStepCallback iter_step_cb =
            [&](MultiLayersCharGridMap2D::SPtr map, MapCells& cells) {};
        MapCell target_container;
        dijkstra.Search(tmp_multi_layered_map, {start_cell}, target_container,
                        expend_cond, finish_cond, step_cb, iter_step_cb);
        // ZISODBG << "tmp_room_cells.size(): " << tmp_room_cells.size();
        if (tmp_room_cells.size() < kMinRoomCellsCount) {
          for (auto&& cell : tmp_room_cells) {
            // ZISODBG << cell.DebugString();
            slam_map->SetValue(cell.X(), cell.Y(), slam_map->GetDefaultValue());
          }
          // for (auto&& str : slam_map->DebugString()) {
          //   ZISODBG << str;
          // }
        }
      }
    }
  }

  // 3. Remove unnecessary obstacles which is not connected to visiable cells.
  for (auto x = data_bound.GetMin().X(); x <= data_bound.GetMax().X(); x++) {
    for (auto y = data_bound.GetMin().Y(); y <= data_bound.GetMax().Y(); y++) {
      slam_map->GetValue(x, y, slam_map_value);
      if (check_unneeded_obstacle(slam_map_value, MapCell(x, y))) {
        slam_map->SetValue(x, y, slam_map->GetDefaultValue());
      }
    }
  }

  // ZISODBG << "After optimization.";
  // for (auto&& str : slam_map->DebugString()) {
  //   ZISODBG << str;
  // }

  return true;
}

bool RoomMapUtil::CheckAndUpdateRoomMap(
    const SlamValueGridMap2D::SPtr& slam_map, CharGridMap2D::SPtr& room_map) {
  if (room_map == nullptr) {
    ZERROR << "Room map pointer invalid.";
    return false;
  }

  MapCells check_cells;
  check_cells.emplace_back(MapCell(1, 0));
  check_cells.emplace_back(MapCell(-1, 0));
  check_cells.emplace_back(MapCell(0, 1));
  check_cells.emplace_back(MapCell(0, -1));

  ReadLocker slam_map_lock(slam_map->GetLock());
  WriteLocker room_map_lock(room_map->GetLock());
  auto data_bound = slam_map->GetDataBound();
  if (!data_bound.Contain(MapCell(room_map->GetDataBound().GetMin().X(),
                                  room_map->GetDataBound().GetMin().Y())) ||
      !data_bound.Contain(MapCell(room_map->GetDataBound().GetMax().X(),
                                  room_map->GetDataBound().GetMax().Y()))) {
    ZWARN << "Slam map data bound:" << data_bound.DebugString()
          << " can not cover room map data bound: "
          << room_map->GetDataBound().DebugString();
    slam_map->Print(__FILE__, __FUNCTION__, __LINE__);
    room_map->Print(__FILE__, __FUNCTION__, __LINE__);
  }
  auto need_to_fullfill =
      [&](const SlamValueGridMap2D::DataType& slam_map_value,
          const CharGridMap2D::DataType& room_map_value,
          const MapCell& check_cell) {
        if (SlamMapUtil::IsVisiable(slam_map_value) &&
            room_map_value != room_map->GetDefaultValue()) {
          CharGridMap2D::DataType check_map_value;
          for (auto&& cell : check_cells) {
            MapCell _check_cell = check_cell + cell;
            room_map->GetValue(_check_cell.X(), _check_cell.Y(),
                               check_map_value);
            if (check_map_value == room_map->GetDefaultValue()) {
              return true;
            }
          }
        }
        return false;
      };
  auto need_to_restore = [&](const SlamValueGridMap2D::DataType& slam_map_value,
                             const CharGridMap2D::DataType& room_map_value) {
    return !SlamMapUtil::IsVisiable(slam_map_value) &&
           room_map_value != room_map->GetDefaultValue();
  };

  std::set<CharGridMap2D::DataType> room_value_set;
  DijkstraBase dijkstra;
  SlamValueGridMap2D::DataType slam_map_value;
  CharGridMap2D::DataType room_map_value;
  for (auto x = data_bound.GetMin().X(); x <= data_bound.GetMax().X(); x++) {
    for (auto y = data_bound.GetMin().Y(); y <= data_bound.GetMax().Y(); y++) {
      slam_map->GetValue(x, y, slam_map_value);
      room_map->GetValue(x, y, room_map_value);
      if (room_map_value != room_map->GetDefaultValue()) {
        room_value_set.emplace(room_map_value);
      }
      if (need_to_fullfill(slam_map_value, room_map_value, MapCell(x, y))) {
        // Count for connected room cells;
        MapCell start_cell(x, y);
        CharGridMap2D::DataType room_value;
        room_map->GetValue(x, y, room_value);
        // ZISODBG << "Start count from: " << start_cell.DebugString();
        MultiLayersCharGridMap2D::SPtr tmp_multi_layered_map =
            std::make_shared<MultiLayersCharGridMap2D>(
                "tmp_multi_layered_map", slam_map->GetRangeX(),
                slam_map->GetRangeY(), slam_map->GetResolution());
        DijkstraBase::ExpendCondition expend_cond =
            [&](MultiLayersCharGridMap2D::SPtr _map, MapCell& curr_step,
                MapCell& next_step) {
              slam_map->GetValue(next_step.X(), next_step.Y(), slam_map_value);
              room_map->GetValue(next_step.X(), next_step.Y(), room_map_value);
              // ZISODBG << "Check: " << next_step.DebugString()
              //        << " value: " << checked_map_value;
              return SlamMapUtil::IsVisiable(slam_map_value) &&
                     room_map_value == room_map->GetDefaultValue();
            };
        // Check nothing.
        DijkstraBase::FinishCondition finish_cond =
            [&](MultiLayersCharGridMap2D::SPtr _map, MapCell& curr_step) {
              return false;
            };
        DijkstraBase::CellStepCallback step_cb =
            [&](MultiLayersCharGridMap2D::SPtr map, MapCell& curr_step) {
              room_map->SetValue(curr_step.X(), curr_step.Y(), room_value);
              // for (auto&& str : room_map->DebugString()) {
              //   ZISODBG << str;
              // }
              return;
            };

        // Do nothing.
        DijkstraBase::IterationStepCallback iter_step_cb =
            [&](MultiLayersCharGridMap2D::SPtr map, MapCells& cells) {};
        MapCell target_container;
        dijkstra.Search(tmp_multi_layered_map, {start_cell}, target_container,
                        expend_cond, finish_cond, step_cb, iter_step_cb);
        // ZISODBG << "tmp_room_cells.size(): " << tmp_room_cells.size();

        // for (auto&& str : room_map->DebugString()) {
        //   ZISODBG << str;
        // }
      } else if (need_to_restore(slam_map_value, room_map_value)) {
        room_map->SetValue(x, y, room_map->GetDefaultValue());
      }
    }
  }

  // Check for new visiable area without connecting to any existing room.
  auto is_new_area = [&](const SlamValueGridMap2D::DataType& slam_map_value,
                         const CharGridMap2D::DataType& room_map_value) {
    return SlamMapUtil::IsVisiable(slam_map_value) &&
           room_map_value == room_map->GetDefaultValue();
  };
  for (auto x = data_bound.GetMin().X(); x <= data_bound.GetMax().X(); x++) {
    for (auto y = data_bound.GetMin().Y(); y <= data_bound.GetMax().Y(); y++) {
      slam_map->GetValue(x, y, slam_map_value);
      room_map->GetValue(x, y, room_map_value);
      if (is_new_area(slam_map_value, room_map_value)) {
        // Count for connected room cells;
        auto start_cell = MapCell(x, y);
        // ZISODBG << "Start count from: " << start_cell.DebugString();
        MapCells tmp_room_cells;
        MultiLayersCharGridMap2D::SPtr tmp_multi_layered_map =
            std::make_shared<MultiLayersCharGridMap2D>(
                "tmp_multi_layered_map", slam_map->GetRangeX(),
                slam_map->GetRangeY(), slam_map->GetResolution());
        DijkstraBase::ExpendCondition expend_cond =
            [&](MultiLayersCharGridMap2D::SPtr _map, MapCell& curr_step,
                MapCell& next_step) {
              slam_map->GetValue(next_step.X(), next_step.Y(), slam_map_value);
              room_map->GetValue(next_step.X(), next_step.Y(), room_map_value);
              // ZISODBG << "Check: " << next_step.DebugString()
              //        << " value: " << room_map_value;
              return is_new_area(slam_map_value, room_map_value);
            };
        // Check nothing.
        DijkstraBase::FinishCondition finish_cond =
            [&](MultiLayersCharGridMap2D::SPtr _map, MapCell& curr_step) {
              return false;
            };
        DijkstraBase::CellStepCallback step_cb =
            [&](MultiLayersCharGridMap2D::SPtr map, MapCell& curr_step) {
              tmp_room_cells.emplace_back(curr_step);
              return;
            };

        // Do nothing.
        DijkstraBase::IterationStepCallback iter_step_cb =
            [&](MultiLayersCharGridMap2D::SPtr map, MapCells& cells) {};
        MapCell target_container;
        dijkstra.Search(tmp_multi_layered_map, {start_cell}, target_container,
                        expend_cond, finish_cond, step_cb, iter_step_cb);
        // ZISODBG << "tmp_room_cells.size(): " << tmp_room_cells.size();
        if (tmp_room_cells.size() >= kMinRoomCellsCount) {
          ZINFO << "Found new area with size: " << tmp_room_cells.size();
          auto new_room_value = NavMap::kRoomA_;
          for (; new_room_value <=
                 static_cast<CharGridMap2D::DataType>(
                     NavMap::kRoomA_ + NavMap::kMaxRoomCount_ - 1);
               new_room_value++) {
            if (room_value_set.count(new_room_value) == 0) {
              break;
            }
          }
          room_value_set.emplace(new_room_value);
          for (auto&& cell : tmp_room_cells) {
            // ZISODBG << cell.DebugString();
            room_map->SetValue(cell.X(), cell.Y(), new_room_value);
          }
          // for (auto&& str : room_map->DebugString()) {
          //   ZISODBG << str;
          // }
        }
      }
    }
  }

  return true;
}

bool RoomMapUtil::UpdateRoomsInfo(const CharGridMap2D::SPtr& room_map,
                                  RoomsInfo& rooms_info) {
  if (room_map == nullptr) {
    ZERROR << "Room map pointer invalid.";
    return false;
  }
  // We suppose the rooms count is less equal NavMap::kMaxRoomCount_.
  rooms_info.clear();
  CharGridMap2D::SPtr tmp_room_map =
      std::make_shared<CharGridMap2D>(*room_map, "tmp room map", true);
  WriteLocker tmp_room_map_lock(tmp_room_map->GetLock());

  // Update for room info even if it is not connected as one slice.
  ReadLocker room_map_lock(room_map->GetLock());
  auto data_bound = room_map->GetDataBound();

  DynamicMapCellBound default_room_bound(INT_MAX, INT_MIN, INT_MAX, INT_MIN);
  std::map<CharGridMap2D::DataType, DynamicMapCellBound> rooms_bound_info;
  for (auto x = data_bound.GetMin().X(); x <= data_bound.GetMax().X(); x++) {
    for (auto y = data_bound.GetMin().Y(); y <= data_bound.GetMax().Y(); y++) {
      CharGridMap2D::DataType room_map_value;
      room_map->GetValue(x, y, room_map_value);
      if (room_map_value != room_map->GetDefaultValue()) {
        if (rooms_bound_info.count(room_map_value) == 0) {
          rooms_bound_info.emplace(room_map_value, default_room_bound);
        }
        rooms_bound_info.at(room_map_value).Expend(MapCell(x, y));
      }
    }
  }

  for (auto&& room_bound_pair : rooms_bound_info) {
    if (rooms_info.count(room_bound_pair.first) == 0) {
      ZGINFO << "Emplace new room: " << room_bound_pair.first;
      rooms_info.emplace(room_bound_pair.first, std::make_shared<RoomInfo>());
      rooms_info.at(room_bound_pair.first)
          ->UpdateRoomIndex(room_bound_pair.first);
    } else {
      ZGINFO << "Update room: " << room_bound_pair.first;
    }

    rooms_info.at(room_bound_pair.first)->SetRoomBound(room_bound_pair.second);
    rooms_info.at(room_bound_pair.first)
        ->SetSectionBaseCell(MapCell(room_bound_pair.second.GetMin().X(),
                                     room_bound_pair.second.GetMin().Y()));
    uint16_t range_x, range_y;
    if (!GenerateXYRangeForRoom(room_map, room_bound_pair.second, range_x,
                                range_y)) {
      return false;
    }
    rooms_info.at(room_bound_pair.first)->SetSectionXRange(range_x);
    rooms_info.at(room_bound_pair.first)->SetSectionYRange(range_y);
  }

  return true;
}

bool RoomMapUtil::SplitRoom(CharGridMap2D::SPtr& room_map,
                            const CharGridMap2D::DataType& target_room_value,
                            const MapCells& split_line, RoomsInfo& rooms_info) {
  if (room_map == nullptr) {
    ZERROR << "Room map pointer invalid.";
    return false;
  }
  if (target_room_value == NavMap::kUnknown_) {
    ZERROR << "Room value " << target_room_value << " invalid.";
    return false;
  }
  if (split_line.size() != 2) {
    ZERROR << "Split line length invalid: " << split_line.size()
           << ", only two cells are required";
    return false;
  }
  if (rooms_info.count(target_room_value) == 0) {
    ZERROR << "Selected room " << target_room_value
           << " is not in rooms info map.";
    return false;
  }

  WriteLocker room_map_lock(room_map->GetLock());

  auto split_line_cells = room_map->GenerateCellsBetweenTwoCells(
      split_line.front(), split_line.back());

  CharGridMap2D::DataType room_value;
  while (!split_line_cells.empty()) {
    room_map->GetValue(split_line_cells.front().X(),
                       split_line_cells.front().Y(), room_value);
    if (room_value != target_room_value) {
      split_line_cells.pop_front();
    } else {
      break;
    }
  }
  while (!split_line_cells.empty()) {
    room_map->GetValue(split_line_cells.back().X(), split_line_cells.back().Y(),
                       room_value);
    if (room_value != target_room_value) {
      split_line_cells.pop_back();
    } else {
      break;
    }
  }

  if (split_line_cells.empty()) {
    ZERROR << "Line is not on room.";
    return false;
  }

  CharGridMap2D::SPtr tmp_room_map =
      std::make_shared<CharGridMap2D>(*room_map, "tmp room map", true);

  // for (auto&& str : tmp_room_map->DebugString()) {
  //   ZISODBG << str;
  // }

  WriteLocker tmp_room_map_lock(tmp_room_map->GetLock());
  for (auto&& split_line_cell : split_line_cells) {
    tmp_room_map->SetValue(split_line_cell.X(), split_line_cell.Y(),
                           NavMap::kRoomP_);
  }

  // for (auto&& str : tmp_room_map->DebugString()) {
  //   ZISODBG << str;
  // }

  CharGridMap2D::DataType tmp_room_value;
  auto get_start_checking_cell = [&](const MapCell& curr_cell,
                                     MapCell& start_checking_cell) {
    room_map->GetValue(curr_cell.X(), curr_cell.Y(), room_value);
    tmp_room_map->GetValue(curr_cell.X(), curr_cell.Y(), tmp_room_value);
    if (room_value == target_room_value &&
        tmp_room_value == tmp_room_map->GetDefaultValue()) {
      start_checking_cell = curr_cell;
      return true;
    }
    return false;
  };

  using RoomSlice = std::pair<MapCells, RoomInfo::SPtr>;
  std::deque<RoomSlice> room_slices_deque;
  DijkstraBase dijkstra;

  MapCells check_cells;
  check_cells.emplace_back(MapCell(1, 0));
  check_cells.emplace_back(MapCell(-1, 0));
  check_cells.emplace_back(MapCell(0, 1));
  check_cells.emplace_back(MapCell(0, -1));

  for (auto&& split_line_cell : split_line_cells) {
    MapCell start_checking_cell;
    for (auto&& check_cell : check_cells) {
      if (get_start_checking_cell(split_line_cell + check_cell,
                                  start_checking_cell)) {
        DynamicMapCellBound room_slice_bound(INT_MAX, INT_MIN, INT_MAX,
                                             INT_MIN);
        MapCells room_slice_cells;
        ZISODBG << "Start genenrate room from: "
                << start_checking_cell.DebugString();
        MultiLayersCharGridMap2D::SPtr tmp_multi_layered_map =
            std::make_shared<MultiLayersCharGridMap2D>(
                "tmp_multi_layered_map", room_map->GetRangeX(),
                room_map->GetRangeY(), room_map->GetResolution());
        auto tmp_map_value = tmp_room_map->GetDefaultValue();
        DijkstraBase::ExpendCondition expend_cond =
            [&](MultiLayersCharGridMap2D::SPtr _map, MapCell& curr_step,
                MapCell& next_step) {
              room_map->GetValue(next_step.X(), next_step.Y(), room_value);
              tmp_room_map->GetValue(next_step.X(), next_step.Y(),
                                     tmp_map_value);
              // ZISODBG << "Check: " << next_step.DebugString()
              //        << " value: " << checked_map_value;
              return room_value == target_room_value &&
                     tmp_map_value == tmp_room_map->GetDefaultValue();
            };
        // Check nothing.
        DijkstraBase::FinishCondition finish_cond =
            [&](MultiLayersCharGridMap2D::SPtr _map, MapCell& curr_step) {
              return false;
            };
        DijkstraBase::CellStepCallback step_cb =
            [&](MultiLayersCharGridMap2D::SPtr map, MapCell& curr_step) {
              room_slice_cells.emplace_back(curr_step);
              room_slice_bound.Expend(curr_step);
              tmp_room_map->SetValue(curr_step.X(), curr_step.Y(),
                                     target_room_value);
              return;
            };

        // Do nothing.
        DijkstraBase::IterationStepCallback iter_step_cb =
            [&](MultiLayersCharGridMap2D::SPtr map, MapCells& cells) {};
        MapCell target_container;
        dijkstra.Search(tmp_multi_layered_map, {start_checking_cell},
                        target_container, expend_cond, finish_cond, step_cb,
                        iter_step_cb);

        // for (auto&& str : tmp_room_map->DebugString()) {
        //   ZISODBG << str;
        // }

        uint16_t range_x, range_y;
        if (!GenerateXYRangeForRoom(room_map, room_slice_bound, range_x,
                                    range_y)) {
          return false;
        }
        room_slices_deque.emplace_back(RoomSlice(
            room_slice_cells,
            std::make_shared<RoomInfo>(target_room_value, room_slice_bound,
                                       MapCell(room_slice_bound.GetMin().X(),
                                               room_slice_bound.GetMin().Y()),
                                       range_x, range_y)));
      }
    }
  }

  if (room_slices_deque.size() < 2) {
    ZERROR << "Separate rooms size invalid: " << room_slices_deque.size();
    return false;
  }

  std::sort(room_slices_deque.begin(), room_slices_deque.end(),
            [](const RoomSlice& room_1, const RoomSlice& room_2) {
              return room_1.first.size() > room_2.first.size();
            });

  for (auto&& room_slice : room_slices_deque) {
    ZINFO << room_slice.first.size();
  }
  // Mark second biggest room slice as new room.
  room_slices_deque.pop_front();
  auto new_room_value = NavMap::kRoomA_;
  for (; new_room_value <= static_cast<CharGridMap2D::DataType>(
                               NavMap::kRoomA_ + NavMap::kMaxRoomCount_ - 1);
       new_room_value++) {
    if (rooms_info.count(new_room_value) == 0) {
      break;
    }
  }
  ZINFO << "New room: " << new_room_value;
  rooms_info.emplace(
      new_room_value,
      std::make_shared<RoomInfo>(
          new_room_value, room_slices_deque.front().second->GetRoomBound(),
          room_slices_deque.front().second->GetSectionBaseCell(),
          room_slices_deque.front().second->GetSectionXRange(),
          room_slices_deque.front().second->GetSectionYRange()));
  for (auto&& cell : room_slices_deque.front().first) {
    room_map->SetValue(cell.X(), cell.Y(), new_room_value);
  }

  for (auto&& room_pair : rooms_info) {
    ZINFO << room_pair.second->DebugString();
  }

  DynamicMapCellBound old_room_bound(INT_MAX, INT_MIN, INT_MAX, INT_MIN);
  auto data_bound = room_map->GetDataBound();
  for (auto x = data_bound.GetMin().X(); x <= data_bound.GetMax().X(); x++) {
    for (auto y = data_bound.GetMin().Y(); y <= data_bound.GetMax().Y(); y++) {
      room_map->GetValue(x, y, room_value);
      if (room_value == target_room_value) {
        old_room_bound.Expend(MapCell(x, y));
      }
    }
  }

  rooms_info.at(target_room_value)->SetRoomBound(old_room_bound);
  rooms_info.at(target_room_value)
      ->SetSectionBaseCell(
          MapCell(old_room_bound.GetMin().X(), old_room_bound.GetMin().Y()));

  uint16_t range_x, range_y;
  if (!GenerateXYRangeForRoom(room_map, old_room_bound, range_x, range_y)) {
    return false;
  }

  rooms_info.at(target_room_value)->SetSectionXRange(range_x);
  rooms_info.at(target_room_value)->SetSectionYRange(range_y);

  // for (auto && str : room_map->DebugString()) {
  //   ZISODBG << str;
  // }

  return true;
}

bool RoomMapUtil::MergeRoom(CharGridMap2D::SPtr& room_map,
                            const CharGridMap2D::DataType& merge_room_1,
                            const CharGridMap2D::DataType& merge_room_2,
                            RoomsInfo& rooms_info) {
  if (room_map == nullptr) {
    ZERROR << "Room map pointer invalid.";
    return false;
  }
  if (merge_room_1 == NavMap::kUnknown_) {
    ZERROR << "Room value " << merge_room_1 << " invalid.";
    return false;
  }
  if (merge_room_2 == NavMap::kUnknown_) {
    ZERROR << "Room value " << merge_room_2 << " invalid.";
    return false;
  }
  if (rooms_info.count(merge_room_1) == 0) {
    ZERROR << "Room " << merge_room_1 << " not exist.";
    return false;
  }
  if (rooms_info.count(merge_room_2) == 0) {
    ZERROR << "Room " << merge_room_2 << " not exist.";
    return false;
  }

  ZISODBG << "Try to merge room " << std::string(1, merge_room_1) << " and room "
         << std::string(1, merge_room_2);
  // We suppose the input room map is valid. It will not have two separated room
  // with same room index.
  WriteLocker room_map_lock(room_map->GetLock());
  auto data_bound = room_map->GetDataBound();
  DijkstraBase dijkstra;
  for (auto x = data_bound.GetMin().X(); x <= data_bound.GetMax().X(); x++) {
    for (auto y = data_bound.GetMin().Y(); y <= data_bound.GetMax().Y(); y++) {
      CharGridMap2D::DataType room_value;
      room_map->GetValue(x, y, room_value);
      if (room_value == merge_room_1 || room_value == merge_room_2) {
        // Expend for connected room cells;
        ZISODBG << "Found room " << room_value;

        CharGridMap2D::DataType this_room_value = room_value;
        CharGridMap2D::DataType another_room_value =
            (room_value == merge_room_1) ? merge_room_2 : merge_room_1;
        MapCells this_room_cells;
        MapCells another_room_cells;

        auto start_cell = MapCell(x, y);
        ZISODBG << "Start check room from: " << start_cell.DebugString();
        MultiLayersCharGridMap2D::SPtr tmp_multi_layered_map =
            std::make_shared<MultiLayersCharGridMap2D>(
                "tmp_multi_layered_map", room_map->GetRangeX(),
                room_map->GetRangeY(), room_map->GetResolution());
        DijkstraBase::ExpendCondition expend_cond =
            [&](MultiLayersCharGridMap2D::SPtr _map, MapCell& curr_step,
                MapCell& next_step) {
              room_map->GetValue(next_step.X(), next_step.Y(), room_value);
              // ZISODBG << "Check: " << next_step.DebugString()
              //        << " value: " << checked_map_value;
              return room_value == this_room_value;
            };
        // Check nothing.
        DijkstraBase::FinishCondition finish_cond =
            [&](MultiLayersCharGridMap2D::SPtr _map, MapCell& curr_step) {
              return false;
            };
        DynamicMapCellBound new_room_bound(INT_MAX, INT_MIN, INT_MAX, INT_MIN);
        MapCell another_room_start_cell;
        bool found_another_room = false;

        MapCells check_cells;
        check_cells.emplace_back(MapCell(1, 0));
        check_cells.emplace_back(MapCell(-1, 0));
        check_cells.emplace_back(MapCell(0, 1));
        check_cells.emplace_back(MapCell(0, -1));
        DijkstraBase::CellStepCallback step_cb =
            [&](MultiLayersCharGridMap2D::SPtr map, MapCell& curr_step) {
              new_room_bound.Expend(curr_step);
              this_room_cells.emplace_back(curr_step);
              if (!found_another_room) {
                CharGridMap2D::DataType check_room_value =
                    room_map->GetDefaultValue();
                for (auto&& check_cell : check_cells) {
                  MapCell _check_cell = check_cell + curr_step;
                  room_map->GetValue(_check_cell.X(), _check_cell.Y(),
                                     check_room_value);
                  if (check_room_value == another_room_value) {
                    ZISODBG << "Found another room "
                           << std::string(1, another_room_value) << " at "
                           << curr_step.DebugString();
                    found_another_room = true;
                    another_room_start_cell = curr_step;
                    break;
                  }
                }
              }
              return;
            };

        // Do nothing.
        DijkstraBase::IterationStepCallback iter_step_cb =
            [&](MultiLayersCharGridMap2D::SPtr map, MapCells& cells) {};
        MapCell target_container;
        dijkstra.Search(tmp_multi_layered_map, {start_cell}, target_container,
                        expend_cond, finish_cond, step_cb, iter_step_cb);
        // ZISODBG << "tmp_room_cells.size(): " << tmp_room_cells.size();

        if (!found_another_room) {
          ZWARN << "Room " << merge_room_1 << " is not connectted to room "
                << merge_room_2;
          return false;
        }

        expend_cond = [&](MultiLayersCharGridMap2D::SPtr _map,
                          MapCell& curr_step, MapCell& next_step) {
          room_map->GetValue(next_step.X(), next_step.Y(), room_value);
          // ZISODBG << "Check: " << next_step.DebugString()
          //        << " value: " << room_value;
          return room_value == another_room_value;
        };

        step_cb = [&](MultiLayersCharGridMap2D::SPtr map, MapCell& curr_step) {
          another_room_cells.emplace_back(curr_step);
          new_room_bound.Expend(curr_step);
          return;
        };

        dijkstra.Search(tmp_multi_layered_map, {another_room_start_cell},
                        target_container, expend_cond, finish_cond, step_cb,
                        iter_step_cb);

        uint16_t range_x, range_y;
        if (!GenerateXYRangeForRoom(room_map, new_room_bound, range_x,
                                    range_y)) {
          return false;
        }

        ZISODBG << "this room: " << std::string(1, this_room_value)
               << " this_room_cells.size()" << this_room_cells.size();
        ZISODBG << "another room: " << std::string(1, another_room_value)
               << " another_room_cells.size()" << another_room_cells.size();
        if (this_room_cells.size() > another_room_cells.size()) {
          ZINFO << "Merge room " << another_room_value << " into room "
                << this_room_value;
          auto another_room = rooms_info.find(another_room_value);
          rooms_info.erase(another_room);
          rooms_info.at(this_room_value) = std::make_shared<RoomInfo>(
              this_room_value, new_room_bound,
              MapCell(new_room_bound.GetMin().X(), new_room_bound.GetMin().Y()),
              range_x, range_y);
          for (auto&& cell : another_room_cells) {
            room_map->SetValue(cell.X(), cell.Y(), this_room_value);
          }
        } else {
          ZINFO << "Merge room " << this_room_value << " into room "
                << another_room_value;
          auto this_room = rooms_info.find(this_room_value);
          rooms_info.erase(this_room);
          rooms_info.at(another_room_value) = std::make_shared<RoomInfo>(
              another_room_value, new_room_bound,
              MapCell(new_room_bound.GetMin().X(), new_room_bound.GetMin().Y()),
              range_x, range_y);
          for (auto&& cell : this_room_cells) {
            room_map->SetValue(cell.X(), cell.Y(), another_room_value);
          }
        }
        return true;
      }
    }
  }

  ZERROR << "No room found.";
  return false;
}

bool RoomMapUtil::GenerateXYRangeForRoom(const CharGridMap2D::SPtr& room_map,
                                         const DynamicMapCellBound& room_bound,
                                         uint16_t& range_x, uint16_t& range_y) {
  if (room_map == nullptr) {
    ZERROR << "Room map pointer invalid.";
    return false;
  }

  auto bigger_point_range =
      std::max(room_bound.GetMax().X() - room_bound.GetMin().X(),
               room_bound.GetMax().Y() - room_bound.GetMin().Y()) *
      room_map->GetResolution();
  if (bigger_point_range < 0) {
    ZERROR << "Room bound invalid: " << room_bound.DebugString();
    return false;
  }
  auto multiply_factor = bigger_point_range / RoomInfo::kMaxSectionWidth_;
  if (multiply_factor < 1) {
    range_x = RoomInfo::kMaxSectionWidth_ / NavMap::GetResolution();
  } else {
    range_x = (bigger_point_range / floor(multiply_factor + 1.0)) /
              NavMap::GetResolution();
  }
  range_y = range_x;
  return true;
}

float BicubicInterpolator::BiCubicBaseFunctionCalculateWeight(
    const float& a, const float& input) {
  auto abs_input = fabs(input);
  if (abs_input <= 1) {
    return (a + 2) * pow(abs_input, 3) - (a + 3) * pow(abs_input, 2) + 1;
  } else if (abs_input < 2) {
    return a * pow(abs_input, 3) - 5 * a * pow(abs_input, 2) +
           8 * a * abs_input - 4 * a;
  }
  return 0;
}

std::shared_ptr<float> BicubicInterpolator::Interpolate(
    const ProbabilityIndexGridMap2D::SPtr probability_index_grid_map,
    const MapPoint& interpolate_point, const float& a) {
  if (probability_index_grid_map == nullptr) {
    ZERROR << "Map pointer invalid, interpolation failed.";
    return nullptr;
  }

  MapCell base_cell;
  probability_index_grid_map->WorldToMap(interpolate_point, base_cell);
  MapPoint base_point;
  probability_index_grid_map->MapToWorld(base_cell, base_point);
  auto x_offset = (interpolate_point.X() - base_point.X()) /
                  probability_index_grid_map->GetResolution();
  auto y_offset = (interpolate_point.Y() - base_point.Y()) /
                  probability_index_grid_map->GetResolution();
  base_cell.AdjustX(-1);
  base_cell.AdjustY(-1);
  // ZGINFO << "x_offset " << FloatToString(x_offset, 3) << ", y_offset "
  //        << FloatToString(y_offset, 3) << ", base_cell "
  //        << base_cell.DebugString();

  std::deque<float> x_check_unified_distances;
  x_check_unified_distances.emplace_back(x_offset + 1);
  x_check_unified_distances.emplace_back(x_offset);
  x_check_unified_distances.emplace_back(1 - x_offset);
  x_check_unified_distances.emplace_back(2 - x_offset);

  std::deque<float> y_check_unified_distances;
  y_check_unified_distances.emplace_back(y_offset + 1);
  y_check_unified_distances.emplace_back(y_offset);
  y_check_unified_distances.emplace_back(1 - y_offset);
  y_check_unified_distances.emplace_back(2 - y_offset);

  float result = 0;
  auto medium_probability =
      probability_index_grid_map->GetMediumProbabilityForObstacleInCell();
  // probability_index_grid_map->EnableDebugLog();
  for (auto i = 0; i <= 3; i++) {
    for (auto j = 0; j <= 3; j++) {
      float probability = 0;
      auto cell = base_cell + MapCell(i, j);
      if (probability_index_grid_map->IsUnknown(cell.X(), cell.Y())) {
        probability = medium_probability;
      } else {
        probability_index_grid_map->GetProbability(cell.X(), cell.Y(),
                                                   probability);
      }
      if (probability < 0) {
        probability = medium_probability;
      }

      auto x_weight =
          BiCubicBaseFunctionCalculateWeight(a, x_check_unified_distances[i]);
      auto y_weight =
          BiCubicBaseFunctionCalculateWeight(a, y_check_unified_distances[j]);
      probability *= x_weight * y_weight;
      result += probability;

      // ZGINFO << cell.DebugString() << "(" << i << ", " << j << "), xw "
      //        << FloatToString(x_weight, 3) << ", yw "
      //        << FloatToString(y_weight, 3) << ", probability "
      //        << FloatToString(probability, 3) << ", result "
      //        << FloatToString(result, 3);
    }
  }
  // probability_index_grid_map->DisableDebugLog();

  return std::make_shared<float>(result);
}

FloatValueGridMap2D::SPtr BicubicInterpolator::Interpolate(
    const ProbabilityIndexGridMap2D::SPtr probability_index_grid_map,
    const float& scale, const float& a) {
  if (probability_index_grid_map == nullptr) {
    ZERROR << "Map pointer invalid, interpolation failed.";
    return nullptr;
  }
  if (scale < 1) {
    ZERROR << "Scale should be larger than 1, interpolation failed.";
    return nullptr;
  }

  if (FLAGS_debug_enable) {
    probability_index_grid_map->Print(__FILE__, __FUNCTION__, __LINE__);
  }

  auto origin_resolution = probability_index_grid_map->GetResolution();
  auto new_resolution = origin_resolution / scale;
  auto origin_point_bound = probability_index_grid_map->GetDataPointBound();
  origin_point_bound.Expend(origin_point_bound.GetMin() -
                            MapPoint(origin_resolution, origin_resolution));
  origin_point_bound.Expend(origin_point_bound.GetMax() +
                            MapPoint(origin_resolution, origin_resolution));

  ZGINFO << "Scale " << FloatToString(scale, 4);
  ZGINFO << "New resolution " << FloatToString(new_resolution, 4);

  FloatValueGridMap2D::SPtr new_map(new FloatValueGridMap2D(
      probability_index_grid_map->Name() + "(interpolated)",
      ceil(probability_index_grid_map->GetRangeX() * scale),
      ceil(probability_index_grid_map->GetRangeY() * scale), new_resolution,
      -1));
  WriteLocker lock(new_map->GetLock());
  CharGridMap2D debug_map(
      probability_index_grid_map->Name() + "(interpolated)",
      ceil(probability_index_grid_map->GetRangeX() * scale),
      ceil(probability_index_grid_map->GetRangeY() * scale), new_resolution);
  WriteLocker lock2(debug_map.GetLock());
  for (auto x = origin_point_bound.GetMin().X();
       x <= origin_point_bound.GetMax().X(); x += new_resolution) {
    for (auto y = origin_point_bound.GetMin().Y();
         y <= origin_point_bound.GetMax().Y(); y += new_resolution) {
      auto new_probability =
          Interpolate(probability_index_grid_map, MapPoint(x, y));
      if (new_probability == nullptr) {
        continue;
      }
      MapCell cell;
      if (new_map->WorldToMap(MapPoint(x, y), cell)) {
        ZGINFO << cell.DebugString() << " set to " << FloatToString(*new_probability, 3);
        new_map->SetValue(cell.X(), cell.Y(), *new_probability);
        debug_map.SetValue(cell.X(), cell.Y(),
                           std::to_string(floor(*new_probability * 10))[0]);
      }
    }
  }
  ZGINFO << "Origin point bound: " << origin_point_bound.DebugString()
         << ", new point bound: " << new_map->GetDataPointBound().DebugString();
  if (FLAGS_debug_enable) {
    // new_map->PrintWithPrecision(__FILE__, __FUNCTION__, __LINE__, 2);
    probability_index_grid_map->Print(__FILE__, __FUNCTION__, __LINE__);
    debug_map.Print(__FILE__, __FUNCTION__, __LINE__);
  }

  return new_map;
}

}  // namespace zima
