/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/robot_data/nav_data.h"

#include "zima/algorithm/dijkstra.h"
#include "zima/common/config.h"
#include "zima/grid_map/map_util.h"
#include "zima/logger/logger.h"

namespace zima {

float RoomInfo::kMaxSectionWidth_ = std::numeric_limits<float>::max();
const std::string NavData::kRawSlamMapName_ = "Raw slam map";
const std::string NavData::kOptimizedSlamMapName_ = "Optimized slam map";
const uint8_t NavData::kMaxVirtualWallSize_ = 10;
const uint8_t NavData::kMaxBlockAreaSize_ = 10;

RoomInfo::Config::Config() : Config(nullptr){};

RoomInfo::Config::Config(const JsonSPtr& json) : config_valid_(false) {
  // Load default setting.
  max_section_width_ = 4;

  if (json != nullptr) {
    // ZINFO << "Load config from json.";
    config_valid_ = ParseFromJson(json);
  } else {
    JsonSPtr config;
    if (GlobalJsonConfig::Instance()->GetGlobalConfigObject(kRoomInfoConfigKey_,
                                                            config)) {
      // ZINFO << "Load config from global json config.";
      config_valid_ = ParseFromJson(config);
    } else {
      ZERROR;
    }
  }
};

bool RoomInfo::Config::ParseFromJson(const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }

  // Override.
  if (!JsonHelper::GetFloat(*json, kMaxSectionWidthKey_, max_section_width_)) {
    ZERROR << "Config " << kMaxSectionWidthKey_ << " not found.";
    return false;
  }

  return true;
}

RoomInfo::RoomInfo()
    : access_(std::make_shared<ReadWriteLock>()),
      room_index_(NavMap::kUnknown_),
      room_bound_(INT_MAX, INT_MIN, INT_MAX, INT_MIN),
      section_base_cell_(
          MapCell(kMaxSectionWidth_ / NavMap::GetResolution() / 2 - 1,
                  kMaxSectionWidth_ / NavMap::GetResolution() / 2)),
      section_x_range_(kMaxSectionWidth_ / NavMap::GetResolution()),
      section_y_range_(kMaxSectionWidth_ / NavMap::GetResolution()),
      cleaned_(false) {
  if (DoubleEqual(kMaxSectionWidth_, std::numeric_limits<float>::max())) {
    Initialize();
    section_x_range_ = kMaxSectionWidth_ / NavMap::GetResolution();
    section_y_range_ = kMaxSectionWidth_ / NavMap::GetResolution();
  }
}

RoomInfo::RoomInfo(const RoomInfo& ref)
    : access_(std::make_shared<ReadWriteLock>()),
      room_index_(ref.GetRoomIndex()),
      room_bound_(ref.GetRoomBound()),
      section_base_cell_(ref.GetSectionBaseCell()),
      section_x_range_(ref.GetSectionXRange()),
      section_y_range_(ref.GetSectionYRange()),
      cleaned_(ref.IsCleaned()) {}

RoomInfo::RoomInfo(const CharGridMap2D::DataType& room_index,
                   const DynamicMapCellBound room_bound,
                   const MapCell& section_base_cell,
                   const unsigned int& section_x_range,
                   const unsigned int& section_y_range)
    : access_(std::make_shared<ReadWriteLock>()),
      room_index_(room_index),
      room_bound_(room_bound),
      section_base_cell_(section_base_cell),
      section_x_range_(section_x_range),
      section_y_range_(section_y_range),
      cleaned_(false) {
  if (section_x_range_ == 0) {
    ZERROR << "Section x range invalid: " << section_x_range_;
    section_x_range_ = kMaxSectionWidth_ / NavMap::GetResolution();
  }
  if (section_y_range_ == 0) {
    ZERROR << "Section y range invalid: " << section_y_range_;
    section_y_range_ = kMaxSectionWidth_ / NavMap::GetResolution();
  }
}

void RoomInfo::Initialize() {
  RoomInfo::Config config;
  if (config.config_valid_) {
    kMaxSectionWidth_ = config.max_section_width_;
    ZINFO << "Initialize room info max section width from config as: "
          << FloatToString(kMaxSectionWidth_, 3);
  } else {
    kMaxSectionWidth_ = 4;
    ZWARN << "Initialize room info max section width as: "
          << FloatToString(kMaxSectionWidth_, 3);
  }
}

bool RoomInfo::UpdateRoomIndex(const CharGridMap2D::DataType& index) {
  WriteLocker lock(access_);
  if (room_index_ != NavMap::kUnknown_ || index == NavMap::kUnknown_) {
    ZWARN << "You can't change a known index from " << room_index_ << " to "
          << index;
    return false;
  }

  room_index_ = index;
  ZGINFO << "Update room index as: " << room_index_;
  return true;
}

void RoomInfo::SetRoomBound(const DynamicMapCellBound& room_bound) {
  WriteLocker lock(access_);
  room_bound_ = room_bound;
};

DynamicMapCellBound RoomInfo::GetRoomBound() const {
  ReadLocker lock(access_);
  return room_bound_;
};

void RoomInfo::SetSectionBaseCell(const MapCell& section_base_cell) {
  WriteLocker lock(access_);
  section_base_cell_ = section_base_cell;
};

MapCell RoomInfo::GetSectionBaseCell() const {
  ReadLocker lock(access_);
  return section_base_cell_;
}

void RoomInfo::SetSectionXRange(const uint16_t& section_x_range) {
  WriteLocker lock(access_);
  section_x_range_ = section_x_range;
};

uint16_t RoomInfo::GetSectionXRange() const {
  ReadLocker lock(access_);
  return section_x_range_;
}

void RoomInfo::SetSectionYRange(const uint16_t& section_y_range) {
  WriteLocker lock(access_);
  section_y_range_ = section_y_range;
};

uint16_t RoomInfo::GetSectionYRange() const {
  ReadLocker lock(access_);
  return section_y_range_;
}

std::string RoomInfo::DebugString() const {
  ReadLocker lock(access_);
  std::string str;
  str += "Room " + std::string(1, room_index_);
  str += "\nBase cell" + section_base_cell_.DebugString();
  str += "\nX range: " + std::to_string(section_x_range_) +
         ", y range: " + std::to_string(section_y_range_);
  return str;
}

DynamicMapCellBound RoomInfo::GetSectionBound(const MapCell& cell) {
  int x_min(section_base_cell_.X());
  int x_max(section_base_cell_.X() + section_x_range_ - 1);
  int y_min(section_base_cell_.Y());
  int y_max(section_base_cell_.Y() + section_y_range_ - 1);
  ZGINFO << "x: " << x_min << ", " << x_max << ", y: " << y_min << ", " << y_max
         << ", cell " << cell;
  while (cell.X() < x_min || cell.X() > x_max) {
    ZGINFO << "x: " << x_min << ", " << x_max << ", y: " << y_min << ", "
           << y_max << ", cell " << cell;
    if (cell.X() < x_min) {
      x_min -= section_x_range_;
    } else {
      x_min += section_x_range_;
    }
    x_max = x_min + section_x_range_ - 1;
  }
  while (cell.Y() < y_min || cell.Y() > y_max) {
    ZGINFO << "x: " << x_min << ", " << x_max << ", y: " << y_min << ", "
           << y_max << ", cell " << cell;
    if (cell.Y() < y_min) {
      y_min -= section_y_range_;
    } else {
      y_min += section_y_range_;
    }
    y_max = y_min + section_y_range_ - 1;
  }

  DynamicMapCellBound bound(x_min, x_max, y_min, y_max);
  return std::move(bound);
}

NavData::NavData()
    : index_(0),
      nav_map_(std::make_shared<NavMap>()),
      raw_slam_map_(std::make_shared<SlamValueGridMap2D>("no data", 1, 1, 1)),
      optimized_slam_map_(
          std::make_shared<SlamValueGridMap2D>("no data", 1, 1, 1)),
      probability_grid_map_(nullptr),
      slam_map_filename_(""),
      current_room_info_(std::make_shared<RoomInfo>()),
      virtual_walls_({}),
      block_areas_({}),
      access_(std::make_shared<ReadWriteLock>()) {
  rooms_info_.emplace(current_room_info_->GetRoomIndex(), current_room_info_);
  ZGINFO << "Initialize for a new nav data.";
}

NavData::NavData(const bool& quiet)
    : index_(0),
      nav_map_(std::make_shared<NavMap>()),
      raw_slam_map_(std::make_shared<SlamValueGridMap2D>("no data", 1, 1, 1)),
      optimized_slam_map_(
          std::make_shared<SlamValueGridMap2D>("no data", 1, 1, 1)),
      probability_grid_map_(nullptr),
      slam_map_filename_(""),
      current_room_info_(std::make_shared<RoomInfo>()),
      virtual_walls_({}),
      block_areas_({}),
      access_(std::make_shared<ReadWriteLock>()) {
  rooms_info_.emplace(current_room_info_->GetRoomIndex(), current_room_info_);
  if (!quiet) {
    ZGINFO << "Initialize for a new nav data.";
  }
}

NavData::NavData(const NavData& ref)
    : index_(ref.GetIndex()),
      nav_map_(std::make_shared<NavMap>(*ref.GetNavMapRef())),
      raw_slam_map_(std::make_shared<SlamValueGridMap2D>(
          *ref.GetRawSlamValueGridMap2DRef())),
      optimized_slam_map_(std::make_shared<SlamValueGridMap2D>(
          *ref.GetOptimizedSlamValueGridMap2DRef())),
      probability_grid_map_(ref.GetProbabilityIndexGridMap2DRef()),
      slam_map_filename_(ref.GetSlamMapFileName()),
      current_room_info_(
          std::make_shared<RoomInfo>(*ref.GetCurrentRoomInfoRef())),
      virtual_walls_(ref.virtual_walls_),
      block_areas_(ref.block_areas_),
      access_(std::make_shared<ReadWriteLock>()) {
  for (auto&& room_info : ref.GetRoomsInfo()) {
    rooms_info_.emplace(room_info.first,
                        std::make_shared<RoomInfo>(*room_info.second));
  }
  ZGINFO << "Copy from " << std::to_string(index_) << " for a new nav data.";
}

NavData::~NavData() { ZGINFO << "Release nav data " << std::to_string(index_); }

NavData::VirtualWall::VirtualWall(const MapPoint& point1,
                                  const MapPoint& point2, const bool& enable,
                                  const UserBlockType& type)
    : enable_(enable), type_(type), map_value_(NavMap::kVirtualWall_) {
  points_.emplace_back(point1);
  points_.emplace_back(point2);
  valid_ = point1.Distance(point2) >
           NavMap::kRobotCellWidth_ * NavMap::GetResolution();
  ZGINFO << "Add " << (valid_ ? "" : "invalid ") << "virtual wall(type "
         << type_ << ") from " << point1.DebugString() << " to "
         << point2.DebugString();
}

MapCells NavData::VirtualWall::ToMapCells(const CharGridMap2D::SPtr& map) {
  if (map == nullptr) {
    ZERROR << "Map invalid.";
    return {};
  }
  if (!valid_) {
    ZERROR << "Virtual wall invalid.";
    return {};
  }
  MapCell cell1, cell2;
  map->WorldToMap(points_.front(), cell1);
  map->WorldToMap(points_.back(), cell2);

  return map->GenerateCellsBetweenTwoCells(cell1, cell2);
}

std::string NavData::VirtualWall::DebugString(
    const CharGridMap2D::SPtr& map) const {
  std::string str = "Valid: " + std::to_string(valid_) + ", type: ";
  switch (type_) {
    case UserBlockType::kStrictBlock: {
      str += "\"Strict block\", ";
      break;
    }
    case UserBlockType::kAvoidWaterBlock: {
      str += "\"Avoid water\", ";
      break;
    }
    case UserBlockType::kCarpetBlock: {
      str += "\"Carpet\", ";
      break;
    }
  }
  str += "with value " + std::to_string(map_value_) + ".\n";
  str += points_.front().DebugString() + "->" + points_.back().DebugString();

  str += "\n";
  MapCell cell1, cell2;
  map->WorldToMap(points_.front(), cell1);
  map->WorldToMap(points_.back(), cell2);
  str += cell1.DebugString() + "->" + cell2.DebugString();
  return str;
}

NavData::BlockArea::BlockArea(const MapPoint& point1, const MapPoint& point2,
                              const MapPoint& point3, const MapPoint& point4,
                              const bool& enable, const UserBlockType& type)
    : enable_(enable), type_(type) {
  switch (type_) {
    case NavData::UserBlockType::kStrictBlock: {
      map_value_ = NavMap::kStrictBlockArea_;
      break;
    }
    case NavData::UserBlockType::kAvoidWaterBlock: {
      map_value_ = NavMap::kAvoidWaterBlockArea_;
      break;
    }
    case NavData::UserBlockType::kCarpetBlock: {
      map_value_ = NavMap::kCarpetBlockArea_;
      break;
    }
    default: {
      ZERROR << "Invalid type: " << type_;
      valid_ = false;
      break;
    }
  }

  points_.emplace_back(point1);
  points_.emplace_back(point2);
  points_.emplace_back(point3);
  points_.emplace_back(point4);
  if (point1.Distance(point2) <=
          NavMap::kRobotCellWidth_ * NavMap::GetResolution() ||
      point2.Distance(point3) <=
          NavMap::kRobotCellWidth_ * NavMap::GetResolution() ||
      point3.Distance(point4) <=
          NavMap::kRobotCellWidth_ * NavMap::GetResolution() ||
      point4.Distance(point1) <=
          NavMap::kRobotCellWidth_ * NavMap::GetResolution()) {
    ZERROR << "Points too close.";
    valid_ = false;
  } else if (!(NormalizeDegree(MapPoint::GetVector(point1, point2).Degree() -
                               MapPoint::GetVector(point2, point3).Degree()) >
                   0 &&
               NormalizeDegree(MapPoint::GetVector(point2, point3).Degree() -
                               MapPoint::GetVector(point3, point4).Degree()) >
                   0 &&
               NormalizeDegree(MapPoint::GetVector(point3, point4).Degree() -
                               MapPoint::GetVector(point4, point1).Degree()) >
                   0 &&
               NormalizeDegree(MapPoint::GetVector(point4, point1).Degree() -
                               MapPoint::GetVector(point1, point2).Degree()) >
                   0) &&
             !(NormalizeDegree(MapPoint::GetVector(point1, point2).Degree() -
                               MapPoint::GetVector(point2, point3).Degree()) <
                   0 &&
               NormalizeDegree(MapPoint::GetVector(point2, point3).Degree() -
                               MapPoint::GetVector(point3, point4).Degree()) <
                   0 &&
               NormalizeDegree(MapPoint::GetVector(point3, point4).Degree() -
                               MapPoint::GetVector(point4, point1).Degree()) <
                   0 &&
               NormalizeDegree(MapPoint::GetVector(point4, point1).Degree() -
                               MapPoint::GetVector(point1, point2).Degree()) <
                   0)) {
    ZERROR << "Points are not always clockwise or always anti-clockwise.";
    valid_ = false;
  } else {
    valid_ = true;
  }

  ZGINFO << "Add " << (valid_ ? "" : "invalid ") << "block area (type " << type_
         << ") from " << point1.DebugString() << " to " << point2.DebugString()
         << " to " << point3.DebugString() << " to " << point4.DebugString();
}

MapCells NavData::BlockArea::ToMapCells(const CharGridMap2D::SPtr& map) {
  if (map == nullptr) {
    ZERROR << "Map invalid.";
    return {};
  }
  if (!valid_) {
    ZERROR << "Block area invalid.";
    return {};
  }
  MapCell cell1, cell2, cell3, cell4;
  auto it = points_.begin();
  map->WorldToMap(*(it++), cell1);
  map->WorldToMap(*(it++), cell2);
  map->WorldToMap(*(it++), cell3);
  map->WorldToMap(*(it++), cell4);

  DynamicMapCellBound bound(INT_MAX, INT_MIN, INT_MAX, INT_MIN);
  bound.Expend(cell1);
  bound.Expend(cell2);
  bound.Expend(cell3);
  bound.Expend(cell4);
  auto tmp_map = std::make_shared<CharGridMap2D>(
      "tmp_map", static_cast<uint16_t>(bound.GetMax().X() - bound.GetMin().X()),
      static_cast<uint16_t>(bound.GetMax().Y() - bound.GetMin().Y()),
      map->GetResolution());
  WriteLocker lock(tmp_map->GetLock());

  MapCells ret_cells;
  auto cells1 = map->GenerateCellsBetweenTwoCells(cell1, cell2);
  ret_cells.insert(ret_cells.end(), cells1.begin(), cells1.end());
  auto cells2 = map->GenerateCellsBetweenTwoCells(cell2, cell3);
  ret_cells.insert(ret_cells.end(), cells2.begin(), cells2.end());
  auto cells3 = map->GenerateCellsBetweenTwoCells(cell3, cell4);
  ret_cells.insert(ret_cells.end(), cells3.begin(), cells3.end());
  auto cells4 = map->GenerateCellsBetweenTwoCells(cell4, cell1);
  ret_cells.insert(ret_cells.end(), cells4.begin(), cells4.end());
  for (auto&& cell : ret_cells) {
    tmp_map->SetValue(cell.X(), cell.Y(), NavMap::kStrictBlockArea_);
  }
  tmp_map->Print(__FILE__, __FUNCTION__, __LINE__);

  MapCell start_cell;
  start_cell.SetX((cell1.X() + cell2.X() + cell3.X() + cell4.X()) / 4);
  start_cell.SetY((cell1.Y() + cell2.Y() + cell3.Y() + cell4.Y()) / 4);

  DijkstraBase dijkstra;
  MultiLayersCharGridMap2D::SPtr tmp_multi_layered_map =
      std::make_shared<MultiLayersCharGridMap2D>(
          "tmp_multi_layered_map", tmp_map->GetRangeX(), tmp_map->GetRangeY(),
          tmp_map->GetResolution());
  CharGridMap2D::DataType map_value;
  DijkstraBase::ExpendCondition expend_cond =
      [&](MultiLayersCharGridMap2D::SPtr _map, MapCell& curr_step,
          MapCell& next_step) {
        tmp_map->GetValue(next_step.X(), next_step.Y(), map_value);
        // ZISODBG << "Check: " << next_step.DebugString()
        //        << " value: " << map_value;
        return map_value == tmp_map->GetDefaultValue();
      };
  // Check nothing.
  DijkstraBase::FinishCondition finish_cond =
      [&](MultiLayersCharGridMap2D::SPtr _map, MapCell& curr_step) {
        return false;
      };
  DijkstraBase::CellStepCallback step_cb =
      [&](MultiLayersCharGridMap2D::SPtr map, MapCell& curr_step) {
        ret_cells.emplace_back(curr_step);
        tmp_map->SetValue(curr_step.X(), curr_step.Y(),
                          NavMap::kStrictBlockArea_);
        return;
      };
  // Do nothing.
  DijkstraBase::IterationStepCallback iter_step_cb =
      [&](MultiLayersCharGridMap2D::SPtr map, MapCells& cells) {};
  MapCell target_container;
  dijkstra.Search(tmp_multi_layered_map, {start_cell}, target_container,
                  expend_cond, finish_cond, step_cb, iter_step_cb);

  tmp_map->Print(__FILE__, __FUNCTION__, __LINE__);

  return ret_cells;
}

std::string NavData::BlockArea::DebugString(
    const CharGridMap2D::SPtr& map) const {
  std::string str = "Valid: " + std::to_string(valid_) + ", type: ";
  switch (type_) {
    case UserBlockType::kStrictBlock: {
      str += "\"Strict block\", ";
      break;
    }
    case UserBlockType::kAvoidWaterBlock: {
      str += "\"Avoid water\", ";
      break;
    }
    case UserBlockType::kCarpetBlock: {
      str += "\"Carpet\", ";
      break;
    }
  }
  str += "with value " + std::to_string(map_value_) + ".\n";
  str += points_.at(0).DebugString() + "->" + points_.at(1).DebugString() +
         "->" + points_.at(2).DebugString() + "->" +
         points_.at(3).DebugString();

  str += "\n";
  MapCell cell1, cell2, cell3, cell4;
  auto it = points_.begin();
  map->WorldToMap(*(it++), cell1);
  map->WorldToMap(*(it++), cell2);
  map->WorldToMap(*(it++), cell3);
  map->WorldToMap(*(it++), cell4);
  str += cell1.DebugString() + "->" + cell2.DebugString() + "->" +
         cell3.DebugString() + "->" + cell4.DebugString();

  return str;
}

uint32_t NavData::GetIndex() const {
  ReadLocker lock(access_);
  return index_;
}

void NavData::SetIndex(const uint32_t& value) {
  WriteLocker lock(access_);
  index_ = value;
}

NavMap::SPtr NavData::GetNavMapRef() const {
  ReadLocker lock(access_);
  return nav_map_;
}

NavMap::SCPtr NavData::GetNavMapConstRef() const {
  ReadLocker lock(access_);
  return nav_map_;
}

SlamValueGridMap2D::SPtr NavData::GetRawSlamValueGridMap2DRef() const {
  ReadLocker lock(access_);
  return raw_slam_map_;
}

void NavData::UpdateRawSlamValueGridMap2D(
    const SlamValueGridMap2D::SPtr& value) {
  WriteLocker lock(access_);
  raw_slam_map_ = value;
}

SlamValueGridMap2D::SPtr NavData::GetOptimizedSlamValueGridMap2DRef() const {
  ReadLocker lock(access_);
  return optimized_slam_map_;
}

void NavData::UpdateOptimizedSlamValueGridMap2D(
    const SlamValueGridMap2D::SPtr& value) {
  WriteLocker lock(access_);
  optimized_slam_map_ = value;
  optimized_slam_map_->ChangeName(kOptimizedSlamMapName_);
}

ProbabilityIndexGridMap2D::SPtr NavData::GetProbabilityIndexGridMap2DRef()
    const {
  ReadLocker lock(access_);
  return probability_grid_map_;
}

void NavData::UpdateProbabilityIndexGridMap2D(
    const ProbabilityIndexGridMap2D::SPtr& value) {
  WriteLocker lock(access_);
  probability_grid_map_ = value;
}

std::string NavData::GetSlamMapFileName() const {
  ReadLocker lock(access_);
  return slam_map_filename_;
}

void NavData::SetSlamMapFileName(const std::string& value) {
  WriteLocker lock(access_);
  slam_map_filename_ = value;
}

RoomInfo::SPtr NavData::GetCurrentRoomInfoRef() const {
  ReadLocker lock(access_);
  return current_room_info_;
}

RoomsInfo NavData::GetRoomsInfo() const {
  ReadLocker lock(access_);
  return rooms_info_;
}

void NavData::SetRoomsInfo(const RoomsInfo& value) {
  WriteLocker lock(access_);
  rooms_info_ = value;
  selected_rooms_info_ = rooms_info_;
}

RoomsInfo NavData::GetSelectedRoomsInfo() const {
  ReadLocker lock(access_);
  return selected_rooms_info_;
}

bool NavData::UpdateSelectedRoomsInfo(const RoomsInfo& selected_rooms) {
  WriteLocker lock(access_);
  for (auto&& select_room_pair : selected_rooms) {
    if (rooms_info_.count(select_room_pair.first) == 0) {
      ZERROR << "Room " << select_room_pair.first << " does not exist.";
      return false;
    }
    ZINFO << "Select room " << select_room_pair.first;
  }
  selected_rooms_info_ = selected_rooms;
  return true;
}

void NavData::ResumeSelectedRoomsInfo() {
  ZINFO;
  WriteLocker lock(access_);
  selected_rooms_info_ = rooms_info_;
}

std::string NavData::DebugAvailableRoomsInfo() {
  ReadLocker lock(access_);
  std::string str = "\n";
  str += "There are " + std::to_string(rooms_info_.size()) +
         " available rooms, they are: ";
  for (auto&& room : rooms_info_) {
    ZINFO << room.first;
    str += std::string(1, room.first) + " ";
  }
  return str;
}

std::string NavData::DebugUserSelectRoomsInfo() {
  ReadLocker lock(access_);
  std::string str = "\n";
  str += "User select " + std::to_string(selected_rooms_info_.size()) +
         " rooms, they are: ";
  for (auto&& room : selected_rooms_info_) {
    str += std::string(1, room.first) + " ";
  }
  return str;
}

bool NavData::AddVirtualWall(const NavData::VirtualWall::SPtr& virtual_wall) {
  if (virtual_wall == nullptr) {
    ZERROR << "Ptr empty.";
    return false;
  }

  WriteLocker lock(access_);
  if (virtual_walls_.size() >= kMaxVirtualWallSize_) {
    ZWARN << "Virtual wall list is full.";
    return false;
  }

  uint8_t empty_index = kMaxVirtualWallSize_;
  for (auto index = 0; index < kMaxVirtualWallSize_; index++) {
    if (virtual_walls_.count(index) == 0) {
      empty_index = index;
      break;
    }
  }

  if (empty_index == kMaxVirtualWallSize_) {
    ZWARN << "Virtual wall list is full.";
    return false;
  }

  virtual_walls_.emplace(empty_index, virtual_wall);
  lock.Unlock();
  ZINFO << "Add for virtual wall " << static_cast<int>(empty_index) << ": "
        << virtual_wall->DebugString(GetNavMapConstRef()->GetUserBlockLayer());

  return true;
}

bool NavData::UpdateVirtualWall(
    const uint8_t& index, const NavData::VirtualWall::SPtr& virtual_wall) {
  if (virtual_wall == nullptr) {
    ZERROR << "Ptr empty.";
    return false;
  }

  WriteLocker lock(access_);
  if (virtual_walls_.count(index) == 0) {
    ZERROR << "Virtual wall " << static_cast<int>(index) << " not exist.";
    return false;
  }

  virtual_walls_.at(index) = virtual_wall;
  lock.Unlock();
  ZINFO << "Update for virtual wall " << static_cast<int>(index) << ": "
        << virtual_wall->DebugString(GetNavMapConstRef()->GetUserBlockLayer());

  return true;
}

bool NavData::RemoveVirtualWall(const uint8_t& index) {
  if (virtual_walls_.count(index) == 0) {
    ZWARN << "Virtual wall " << static_cast<int>(index) << " not exist.";
    return true;
  }
  {
    ReadLocker lock(access_);
    ZINFO << "Remove for virtual wall " << static_cast<int>(index) << ": "
          << virtual_walls_.at(index)->DebugString(
                 GetNavMapConstRef()->GetUserBlockLayer());
  }
  WriteLocker lock(access_);
  virtual_walls_.erase(index);

  return true;
}

bool NavData::GetVirtualWall(const uint8_t& index,
                             NavData::VirtualWall::SPtr& virtual_wall) const {
  if (virtual_wall == nullptr) {
    ZERROR << "Ptr empty.";
    return false;
  }

  ReadLocker lock(access_);
  if (virtual_walls_.count(index) == 0) {
    ZWARN << "Virtual wall " << static_cast<int>(index) << " not exist.";
    return false;
  }

  virtual_wall = virtual_walls_.at(index);

  return true;
}

NavData::VirtualWalls NavData::GetAllVirtualWall() const {
  ReadLocker lock(access_);
  return virtual_walls_;
}

bool NavData::AddBlockArea(const NavData::BlockArea::SPtr& block_area) {
  if (block_area == nullptr) {
    ZERROR << "Ptr empty.";
    return false;
  }

  WriteLocker lock(access_);
  if (block_areas_.size() >= kMaxBlockAreaSize_) {
    ZWARN << "Block area list is full.";
    return false;
  }

  uint8_t empty_index = kMaxBlockAreaSize_;
  for (auto index = 0; index < kMaxBlockAreaSize_; index++) {
    if (block_areas_.count(index) == 0) {
      empty_index = index;
      break;
    }
  }

  if (empty_index == kMaxBlockAreaSize_) {
    ZWARN << "Block area list is full.";
    return false;
  }

  block_areas_.emplace(empty_index, block_area);
  lock.Unlock();
  ZINFO << "Add for block area " << static_cast<int>(empty_index) << ": "
        << block_area->DebugString(GetNavMapConstRef()->GetUserBlockLayer());

  return true;
}

bool NavData::UpdateBlockArea(const uint8_t& index,
                              const NavData::BlockArea::SPtr& block_area) {
  if (block_area == nullptr) {
    ZERROR << "Ptr empty.";
    return false;
  }

  WriteLocker lock(access_);
  if (block_areas_.count(index) == 0) {
    ZERROR << "Block area " << static_cast<int>(index) << " not exist.";
    return false;
  }

  block_areas_.at(index) = block_area;
  lock.Unlock();
  ZINFO << "Update for block area " << static_cast<int>(index) << ": "
        << block_area->DebugString(GetNavMapConstRef()->GetUserBlockLayer());

  return true;
}

bool NavData::RemoveBlockArea(const uint8_t& index) {
  if (block_areas_.count(index) == 0) {
    ZWARN << "Block area " << static_cast<int>(index) << " not exist.";
    return true;
  }
  {
    ReadLocker lock(access_);
    ZINFO << "Remove for block area " << static_cast<int>(index) << ": "
          << block_areas_.at(index)->DebugString(
                 GetNavMapConstRef()->GetUserBlockLayer());
  }
  WriteLocker lock(access_);
  block_areas_.erase(index);

  return true;
}

bool NavData::GetBlockArea(const uint8_t& index,
                           NavData::BlockArea::SPtr& block_area) const {
  if (block_area == nullptr) {
    ZERROR << "Ptr empty.";
    return false;
  }

  ReadLocker lock(access_);
  if (block_areas_.count(index) == 0) {
    ZWARN << "Block area " << static_cast<int>(index) << " not exist.";
    return false;
  }

  block_area = block_areas_.at(index);

  return true;
}

NavData::BlockAreas NavData::GetAllBlockArea() const {
  ReadLocker lock(access_);
  return block_areas_;
}

void NavData::UpdateUserBlockMapInNavMap() {
  WriteLocker lock(access_);
  DynamicMapCellBound bound(INT_MAX, INT_MIN, INT_MAX, INT_MIN);
  std::map<MapCell, CharGridMap2D::DataType> cell_value_map;
  auto user_block_layer = nav_map_->GetUserBlockLayer();
  WriteLocker user_block_layer_lock(user_block_layer->GetLock());
  for (auto&& virtual_wall_pair : virtual_walls_) {
    for (auto&& cell : virtual_wall_pair.second->ToMapCells(user_block_layer)) {
      // ZINFO << "Insert " << cell.DebugString() << " as "
      //       << virtual_wall_pair.second.GetMapValue();
      cell_value_map.emplace(cell, virtual_wall_pair.second->GetMapValue());
      bound.Expend(cell);
    }
  }
  for (auto&& block_area_pair : block_areas_) {
    for (auto&& cell : block_area_pair.second->ToMapCells(user_block_layer)) {
      // ZINFO << "Insert " << cell.DebugString() << " as "
      //       << block_area_pair.second.GetMapValue();
      cell_value_map.emplace(cell, block_area_pair.second->GetMapValue());
      bound.Expend(cell);
    }
  }
  if (cell_value_map.empty()) {
    ZINFO << "No available data.";
    return;
  }
  user_block_layer_lock.Unlock();
  // For inflaction, we should set larger.
  user_block_layer->ResetMap(bound.GetMax().X() - bound.GetMin().X() + 10,
                             bound.GetMax().Y() - bound.GetMin().Y() + 10,
                             user_block_layer->GetResolution(),
                             user_block_layer->GetDefaultValue());
  // user_block_layer->Print(__FILE__, __FUNCTION__, __LINE__);
  user_block_layer_lock.Lock();
  user_block_layer->SetValue(bound.GetMin().X(), bound.GetMin().Y(),
                             user_block_layer->GetDefaultValue());
  user_block_layer->SetValue(bound.GetMax().X(), bound.GetMax().Y(),
                             user_block_layer->GetDefaultValue());
  // user_block_layer->Print(__FILE__, __FUNCTION__, __LINE__);
  for (auto&& cell_value_elem : cell_value_map) {
    // ZINFO << "Set " << cell_value_elem.first.DebugString() << " as "
    //       << cell_value_elem.second;
    user_block_layer->SetValue(cell_value_elem.first.X(),
                               cell_value_elem.first.Y(),
                               cell_value_elem.second);
  }
  ZINFO << "Update user block map:";
  user_block_layer->Print(__FILE__, __FUNCTION__, __LINE__);
}

std::string NavData::DebugUserBlocksInfo() {
  ReadLocker lock(access_);
  std::string str;
  for (auto&& virtual_wall_pair : virtual_walls_) {
    str += "\nVirtual wall:\n" +
           virtual_wall_pair.second->DebugString(nav_map_->GetUserBlockLayer());
  }
  for (auto&& block_area_pair : block_areas_) {
    str += "\nBlock area:\n" +
           block_area_pair.second->DebugString(nav_map_->GetUserBlockLayer());
  }
  return str;
}

void NavData::UpdateUserSelectAreaInNavMap(
    const CharGridMap2D::SPtr& new_user_select_area_map) {
  ZGINFO << "For auto nav data.";
  WriteLocker lock(access_);
  nav_map_->ChangeUserSelectAreaLayer(new_user_select_area_map);
}

void NavData::ResumeUserSelectAreaInNavMap() {
  ZGINFO;
  WriteLocker lock(access_);
  auto map = nav_map_->GetUserSelectAreaLayer();
  map->ResetMap();
}

ZimaProto::NavData::PRoomInfo NavDataSerializer::ToProto(
    const RoomInfo::SPtr& room_info) {
  ZimaProto::NavData::PRoomInfo proto;
  if (room_info == nullptr) {
    ZERROR << "Room info empty.";
    return proto;
  }
  proto.set_room_index(std::string(1, room_info->GetRoomIndex()));
  auto bound = room_info->GetRoomBound();
  proto.mutable_room_bound()->set_x_min(bound.GetMin().X());
  proto.mutable_room_bound()->set_x_max(bound.GetMax().X());
  proto.mutable_room_bound()->set_y_min(bound.GetMin().Y());
  proto.mutable_room_bound()->set_y_max(bound.GetMax().Y());
  proto.mutable_section_base_cell()->set_x(room_info->GetSectionBaseCell().X());
  proto.mutable_section_base_cell()->set_y(room_info->GetSectionBaseCell().Y());
  proto.set_section_x_range(room_info->GetSectionXRange());
  proto.set_section_y_range(room_info->GetSectionYRange());
  return proto;
}

bool NavDataSerializer::FromProto(RoomInfo::SPtr& room_info,
                                  const ZimaProto::NavData::PRoomInfo& proto) {
  if (room_info == nullptr) {
    ZERROR << "Room info empty.";
    return false;
  }
  CharGridMap2D::DataType room_index;
  if (proto.room_index().empty() || proto.room_index().size() > 1) {
    ZERROR << "Proto room index is damaged: " << proto.room_index() << "("
           << proto.room_index().size() << ")";
    return false;
  } else {
    room_index = proto.room_index().front();
  }
  DynamicMapCellBound bound(INT_MAX, INT_MIN, INT_MAX, INT_MIN);
  if (!proto.has_room_bound()) {
    ZERROR << "Proto room bound is damaged.";
    return false;
  } else {
    bound.Expend(
        MapCell(proto.room_bound().x_min(), proto.room_bound().y_min()));
    bound.Expend(
        MapCell(proto.room_bound().x_max(), proto.room_bound().y_max()));
  }
  MapCell section_base_cell;
  if (!proto.has_section_base_cell()) {
    ZERROR << "Proto section base cell is damaged.";
    return false;
  } else {
    section_base_cell.SetX(proto.section_base_cell().x());
    section_base_cell.SetY(proto.section_base_cell().y());
  }
  auto section_x_range = proto.section_x_range();
  auto section_y_range = proto.section_y_range();
  room_info = std::make_shared<RoomInfo>(room_index, bound, section_base_cell,
                                         section_x_range, section_y_range);
  return true;
}

ZimaProto::NavData::PVirtualWall NavDataSerializer::ToProto(
    const NavData::VirtualWall::SPtr& virtual_wall) {
  ZimaProto::NavData::PVirtualWall proto;
  if (virtual_wall == nullptr) {
    ZERROR << "Ptr empty.";
    return proto;
  }

  proto.set_valid(virtual_wall->IsValid());
  proto.set_enable(virtual_wall->enable_);
  switch (virtual_wall->type_) {
    case NavData::UserBlockType::kStrictBlock: {
      proto.set_type(ZimaProto::NavData::PUserBlockType::kStrictBlock);
      break;
    }
    case NavData::UserBlockType::kAvoidWaterBlock: {
      proto.set_type(ZimaProto::NavData::PUserBlockType::kAvoidWaterBlock);
      break;
    }
    case NavData::UserBlockType::kCarpetBlock: {
      proto.set_type(ZimaProto::NavData::PUserBlockType::kCarpetBlock);
      break;
    }
    default: {
      ZERROR << "Invalid type " << std::to_string(virtual_wall->type_)
             << " change to "
             << std::to_string(
                    ZimaProto::NavData::PUserBlockType::kStrictBlock);
      proto.set_type(ZimaProto::NavData::PUserBlockType::kStrictBlock);
    }
  }
  proto.set_map_value(std::string(1, virtual_wall->map_value_));
  for (auto&& point : virtual_wall->points_) {
    ZimaProto::Map::PMapPoint point_proto;
    point_proto.set_x(point.X());
    point_proto.set_y(point.Y());
    point_proto.set_degree(point.Degree());
    *proto.add_points() = point_proto;
  }
  return proto;
}

bool NavDataSerializer::FromProto(
    NavData::VirtualWall::SPtr& virtual_wall,
    const ZimaProto::NavData::PVirtualWall& proto) {
  if (virtual_wall == nullptr) {
    virtual_wall.reset(new NavData::VirtualWall(MapPoint(), MapPoint()));
  }
  virtual_wall->valid_ = proto.valid();
  virtual_wall->enable_ = proto.enable();
  switch (proto.type()) {
    case ZimaProto::NavData::PUserBlockType::kStrictBlock: {
      virtual_wall->type_ = NavData::UserBlockType::kStrictBlock;
      break;
    }
    case ZimaProto::NavData::PUserBlockType::kAvoidWaterBlock: {
      virtual_wall->type_ = NavData::UserBlockType::kAvoidWaterBlock;
      break;
    }
    case ZimaProto::NavData::PUserBlockType::kCarpetBlock: {
      virtual_wall->type_ = NavData::UserBlockType::kCarpetBlock;
      break;
    }
    default: {
      ZERROR << "Invalid type " << std::to_string(proto.type());
      virtual_wall->type_ = NavData::UserBlockType::kStrictBlock;
      return false;
    }
  }
  virtual_wall->map_value_ = proto.map_value().empty()
                                 ? NavMap::kVirtualWall_
                                 : proto.map_value().front();
  virtual_wall->points_.clear();
  for (auto&& proto_point : proto.points()) {
    virtual_wall->points_.emplace_back(proto_point.x(), proto_point.y());
  }
  if (virtual_wall->points_.empty()) {
    ZERROR << "Insufficient points.";
    return false;
  }
  return true;
}

ZimaProto::NavData::PBlockArea NavDataSerializer::ToProto(
    const NavData::BlockArea::SPtr& block_area) {
  ZimaProto::NavData::PBlockArea proto;
  if (block_area == nullptr) {
    ZERROR << "Ptr empty.";
    return proto;
  }

  proto.set_valid(block_area->IsValid());
  proto.set_enable(block_area->enable_);
  switch (block_area->type_) {
    case NavData::UserBlockType::kStrictBlock: {
      proto.set_type(ZimaProto::NavData::PUserBlockType::kStrictBlock);
      break;
    }
    case NavData::UserBlockType::kAvoidWaterBlock: {
      proto.set_type(ZimaProto::NavData::PUserBlockType::kAvoidWaterBlock);
      break;
    }
    case NavData::UserBlockType::kCarpetBlock: {
      proto.set_type(ZimaProto::NavData::PUserBlockType::kCarpetBlock);
      break;
    }
    default: {
      ZERROR << "Invalid type " << std::to_string(block_area->type_)
             << " change to "
             << std::to_string(
                    ZimaProto::NavData::PUserBlockType::kStrictBlock);
      proto.set_type(ZimaProto::NavData::PUserBlockType::kStrictBlock);
    }
  }
  proto.set_map_value(std::string(1, block_area->map_value_));
  for (auto&& point : block_area->points_) {
    ZimaProto::Map::PMapPoint point_proto;
    point_proto.set_x(point.X());
    point_proto.set_y(point.Y());
    point_proto.set_degree(point.Degree());
    *proto.add_points() = point_proto;
  }
  return proto;
}

bool NavDataSerializer::FromProto(NavData::BlockArea::SPtr& block_area,
                                  const ZimaProto::NavData::PBlockArea& proto) {
  if (block_area == nullptr) {
    block_area.reset(
        new NavData::BlockArea(MapPoint(), MapPoint(), MapPoint(), MapPoint()));
  }
  block_area->valid_ = proto.valid();
  block_area->enable_ = proto.enable();
  switch (proto.type()) {
    case ZimaProto::NavData::PUserBlockType::kStrictBlock: {
      block_area->type_ = NavData::UserBlockType::kStrictBlock;
      block_area->map_value_ = proto.map_value().empty()
                                   ? NavMap::kStrictBlockArea_
                                   : proto.map_value().front();
      break;
    }
    case ZimaProto::NavData::PUserBlockType::kAvoidWaterBlock: {
      block_area->type_ = NavData::UserBlockType::kAvoidWaterBlock;
      block_area->map_value_ = proto.map_value().empty()
                                   ? NavMap::kAvoidWaterBlockArea_
                                   : proto.map_value().front();
      break;
    }
    case ZimaProto::NavData::PUserBlockType::kCarpetBlock: {
      block_area->type_ = NavData::UserBlockType::kCarpetBlock;
      block_area->map_value_ = proto.map_value().empty()
                                   ? NavMap::kCarpetBlockArea_
                                   : proto.map_value().front();
      break;
    }
    default: {
      ZERROR << "Invalid type " << std::to_string(proto.type());
      block_area->type_ = NavData::UserBlockType::kStrictBlock;
      block_area->map_value_ = proto.map_value().empty()
                                   ? NavMap::kStrictBlockArea_
                                   : proto.map_value().front();
      return false;
    }
  }
  block_area->points_.clear();
  for (auto&& proto_point : proto.points()) {
    block_area->points_.emplace_back(proto_point.x(), proto_point.y());
  }
  if (block_area->points_.empty()) {
    ZERROR << "Insufficient points.";
    return false;
  }
  return true;
}

ZimaProto::NavData::PNavData NavDataSerializer::ToProto(
    const NavData::SPtr& data) {
  ZimaProto::NavData::PNavData proto;
  if (data == nullptr) {
    ZERROR << "Data empty.";
    return proto;
  }
  ReadLocker lock(data->access_);
  proto.set_index(data->index_);
  *proto.mutable_nav_map() = MultiLayerMap2DSerializer::ToProto(data->nav_map_);
  *proto.mutable_slam_map() =
      DynamicMap2DSerializer::ToProto(data->raw_slam_map_);
  if (data->probability_grid_map_ != nullptr) {
    *proto.mutable_probability_map() =
        ProbabilityIndexGridMap2DSerializer::ToProto(
            data->probability_grid_map_);
  }
  proto.set_slam_map_filename(data->slam_map_filename_);
  for (auto&& room_info_pair : data->rooms_info_) {
    (*proto.mutable_rooms_info())[std::to_string(room_info_pair.first)] =
        ToProto(room_info_pair.second);
  }
  for (auto&& virtual_wall_pair : data->virtual_walls_) {
    (*proto.mutable_virtual_walls())[virtual_wall_pair.first] =
        ToProto(virtual_wall_pair.second);
  }
  for (auto&& block_area_pair : data->block_areas_) {
    (*proto.mutable_block_areas())[block_area_pair.first] =
        ToProto(block_area_pair.second);
  }
  return proto;
}

bool NavDataSerializer::FromProto(NavData::SPtr& data,
                                  const ZimaProto::NavData::PNavData& proto) {
  if (data == nullptr) {
    ZERROR << "Data empty.";
    return false;
  }
  WriteLocker lock(data->access_);
  data->index_ = proto.index();
  if (proto.has_nav_map()) {
    auto ptr =
        std::dynamic_pointer_cast<MultiLayersCharGridMap2D>(data->nav_map_);
    MultiLayerMap2DSerializer::FromProto(
        ptr, proto.nav_map(), NavMap::max_width_ / NavMap::GetResolution(),
        NavMap::max_height_ / NavMap::GetResolution());
    lock.Unlock();
    data->nav_map_->RefreshLayerPtrs();
    lock.Lock();
  } else {
    ZERROR << "Proto nav map is damaged.";
    data->nav_map_ = std::make_shared<NavMap>();
    return false;
  }
  if (proto.has_slam_map()) {
    DynamicMap2DSerializer::FromProto(data->raw_slam_map_, proto.slam_map());
    auto optimized_slam_grid_map =
        std::make_shared<SlamValueGridMap2D>(*(data->raw_slam_map_));
    optimized_slam_grid_map->ChangeName(NavData::kOptimizedSlamMapName_);
    SlamMapUtil slam_map_util;
    slam_map_util.OptimizeSlamValueGridMap2DForUser(optimized_slam_grid_map);
    lock.Unlock();
    data->UpdateOptimizedSlamValueGridMap2D(optimized_slam_grid_map);
    lock.Lock();
  } else {
    ZERROR << "Proto slam map is damaged.";
    data->raw_slam_map_ =
        std::make_shared<SlamValueGridMap2D>("no data", 1, 1, 1);
    data->optimized_slam_map_ =
        std::make_shared<SlamValueGridMap2D>("no data", 1, 1, 1);
    return false;
  }
  if (proto.has_probability_map()) {
    if (data->probability_grid_map_ == nullptr) {
      data->probability_grid_map_.reset(new ProbabilityIndexGridMap2D(
          "tmp name", 1, 1, 1, ProbabilityIndexGridMap2D::Config()));
    }
    ProbabilityIndexGridMap2DSerializer::FromProto(data->probability_grid_map_,
                                                   proto.probability_map());
  } else {
    ZGWARN << "Proto does not have probability map.";
    data->probability_grid_map_.reset();
  }

  data->slam_map_filename_ = proto.slam_map_filename();
  data->rooms_info_.clear();
  for (auto&& room_info_pair : proto.rooms_info()) {
    RoomInfo::SPtr room_info = std::make_shared<RoomInfo>();
    if (FromProto(room_info, room_info_pair.second)) {
      data->rooms_info_.emplace(room_info->GetRoomIndex(), room_info);
    } else {
      ZERROR << "Proto room info is damaged";
      return false;
    }
  }
  data->current_room_info_ = std::make_shared<RoomInfo>();

  for (auto&& virtual_wall_pair : proto.virtual_walls()) {
    NavData::VirtualWall::SPtr virtual_wall;
    if (FromProto(virtual_wall, virtual_wall_pair.second)) {
      data->virtual_walls_.emplace(virtual_wall_pair.first, virtual_wall);
    } else {
      ZERROR << "Proto virtual wall is damaged";
      return false;
    }
  }

  for (auto&& block_area_pair : proto.block_areas()) {
    NavData::BlockArea::SPtr block_area;
    if (FromProto(block_area, block_area_pair.second)) {
      data->block_areas_.emplace(block_area_pair.first, block_area);
    } else {
      ZERROR << "Proto block area is damaged";
      return false;
    }
  }

  return true;
}

bool NavDataLoader::LoadData(NavData::SPtr& data, const bool& quiet) {
  ZimaProto::NavData::PNavData proto;
  if (GetProtoFromBinaryFile(&proto) || GetProtoFromASCIIFile(&proto)) {
    if (NavDataSerializer::FromProto(data, proto)) {
      if (!quiet) {
        ZGINFO << "Load " << file_name_ << " success.";
      }
      return true;
    }
  }
  ZERROR << "Load " << file_name_ << " failed.";
  return false;
}

bool NavDataLoader::LoadData(NavData::VirtualWall::SPtr& data) {
  ZimaProto::NavData::PVirtualWall proto;
  if (GetProtoFromBinaryFile(&proto) || GetProtoFromASCIIFile(&proto)) {
    if (NavDataSerializer::FromProto(data, proto)) {
      ZGINFO << "Load " << file_name_ << " success.";
      return true;
    }
  }
  ZERROR << "Load " << file_name_ << " failed.";
  return false;
}

bool NavDataLoader::LoadData(NavData::BlockArea::SPtr& data) {
  ZimaProto::NavData::PBlockArea proto;
  if (GetProtoFromBinaryFile(&proto) || GetProtoFromASCIIFile(&proto)) {
    if (NavDataSerializer::FromProto(data, proto)) {
      ZGINFO << "Load " << file_name_ << " success.";
      return true;
    }
  }
  ZERROR << "Load " << file_name_ << " failed.";
  return false;
}

bool NavDataWriter::WriteData(const NavData::SPtr& data,
                              const bool& is_binary) {
  auto proto = NavDataSerializer::ToProto(data);
  if (is_binary ? SetProtoToBinaryFile(proto) : SetProtoToASCIIFile(proto)) {
    ZGINFO << "Write " << file_name_ << " success.";
    return true;
  } else {
    ZERROR << "Write " << file_name_ << " failed.";
    return false;
  }
}

bool NavDataWriter::WriteData(const NavData::VirtualWall::SPtr& data,
                              const bool& is_binary) {
  auto proto = NavDataSerializer::ToProto(data);
  if (is_binary ? SetProtoToBinaryFile(proto) : SetProtoToASCIIFile(proto)) {
    ZGINFO << "Write " << file_name_ << " success.";
    return true;
  } else {
    ZERROR << "Write " << file_name_ << " failed.";
    return false;
  }
}

bool NavDataWriter::WriteData(const NavData::BlockArea::SPtr& data,
                              const bool& is_binary) {
  auto proto = NavDataSerializer::ToProto(data);
  if (is_binary ? SetProtoToBinaryFile(proto) : SetProtoToASCIIFile(proto)) {
    ZGINFO << "Write " << file_name_ << " success.";
    return true;
  } else {
    ZERROR << "Write " << file_name_ << " failed.";
    return false;
  }
}

}  // namespace zima
