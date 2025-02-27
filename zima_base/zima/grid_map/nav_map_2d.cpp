/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/grid_map/nav_map_2d.h"

#include "zima/common/config.h"
#include "zima/logger/logger.h"

namespace zima {

bool NavMap::is_config_loaded_ = false;
double NavMap::resolution_ = 0.07;
const int NavMap::kRobotCellWidth_ = 5;
const int NavMap::kRobotCellWidth_2_ = 2;
const int NavMap::kRobotMarkWidth_2_ = 1;

const std::string NavMap::kPrintMapName_ = "Print map";
const std::string NavMap::kFootStepMapName_ = "Foot step map";
// const std::string NavMap::kBumperMapName_ = "Bumper map";
// const std::string NavMap::kCliffMapName_ = "Cliff map";
// const std::string NavMap::kWallMapName_ = "Wall map";
const std::string NavMap::kSensorMapName_ = "Sensor map";
const std::string NavMap::kUserBlockMapName_ = "User block map";
const std::string NavMap::kSlamMapName_ = "Slam map";
const std::string NavMap::kRoomMapName_ = "Room map";
const std::string NavMap::kUserSelectAreaMapName_ = "User select area map";

const CharGridMap2D::DataType NavMap::kUnknown_ = '.';
const CharGridMap2D::DataType NavMap::kFootStep_ = '+';

// Sensor accurate obstacles
const CharGridMap2D::DataType NavMap::kBumper_ = 'B';
const CharGridMap2D::DataType NavMap::kCliff_ = 'C';
const CharGridMap2D::DataType NavMap::kWall_ = 'W';

// Slam semantic value
const CharGridMap2D::DataType NavMap::kSlamWall_ = 'W';
const CharGridMap2D::DataType NavMap::kSlamFloor_ = 'i';

// Markers
const CharGridMap2D::DataType NavMap::kVirtualWall_ = 'K';
const CharGridMap2D::DataType NavMap::kStrictBlockArea_ = 'X';
const CharGridMap2D::DataType NavMap::kAvoidWaterBlockArea_ = 'v';
const CharGridMap2D::DataType NavMap::kCarpetBlockArea_ = 'x';
const CharGridMap2D::DataType NavMap::kCharger_ = 'C';
const CharGridMap2D::DataType NavMap::kUserSelected_ = NavMap::kUnknown_;
const CharGridMap2D::DataType NavMap::kUserUnSelected_ =
    NavMap::kStrictBlockArea_;

// Room id
#define DefineRoomValue(value) \
  const CharGridMap2D::DataType NavMap::kRoom##value##_ = *#value;

DefineRoomValue(A)
DefineRoomValue(B)
DefineRoomValue(C)
DefineRoomValue(D)
DefineRoomValue(E)
DefineRoomValue(F)
DefineRoomValue(G)
DefineRoomValue(H)
DefineRoomValue(I)
DefineRoomValue(J)
DefineRoomValue(K)
DefineRoomValue(L)
DefineRoomValue(M)
DefineRoomValue(N)
DefineRoomValue(O)
DefineRoomValue(P)

const uint8_t NavMap::kMaxRoomCount_ = 16;

// Debug marker
const CharGridMap2D::DataType NavMap::kPathStart_ = 'S';
const CharGridMap2D::DataType NavMap::kPath_ = 's';

float NavMap::max_width_ = 30;
float NavMap::max_height_ = 30;
int16_t NavMap::max_clean_cell_width_ =
    NavMap::max_width_ / NavMap::resolution_;
int16_t NavMap::max_clean_cell_height_ =
    NavMap::max_height_ / NavMap::resolution_;

// ReadWriteLock::SPtr nav_map_count_lock = make_shared<ReadWriteLock>();
// std::map<uint, std::string> nav_map_map;
// atomic_uint nav_map_count;

NavMap::Config::Config() : Config(nullptr){};

NavMap::Config::Config(const JsonSPtr& json) : config_valid_(false) {
  resolution_ = 0.07;
  max_width_ = 30;
  max_height_ = 30;
  max_clean_width_ = 10;
  max_clean_width_ = 10;

  if (json != nullptr) {
    // ZINFO << "Load config from json.";
    config_valid_ = ParseFromJson(json);
  } else {
    JsonSPtr config;
    if (GlobalJsonConfig::Instance()->GetGlobalConfigObject(kConfigKey_,
                                                            config)) {
      // ZINFO << "Load config from global json config.";
      config_valid_ = ParseFromJson(config);
    } else {
      ZERROR;
    }
  }
};

bool NavMap::Config::ParseFromJson(const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }

  // Override.
  if (!JsonHelper::GetFloat(*json, kResolutionKey_, resolution_)) {
    ZERROR << "Config " << kResolutionKey_ << " not found.";
    return false;
  }
  if (resolution_ < 0) {
    ZERROR << "Config " << kResolutionKey_
           << " invalid: " << FloatToString(resolution_, 3) << ".";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kMaxWidthKey_, max_width_)) {
    ZGERROR << "Config " << kMaxWidthKey_ << " not found.";
    // return false;
  }
  if (max_width_ < 0) {
    ZERROR << "Config " << kMaxWidthKey_
           << " invalid: " << FloatToString(max_width_, 3) << ".";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kMaxHeightKey_, max_height_)) {
    ZGERROR << "Config " << kMaxHeightKey_ << " not found.";
    // return false;
  }
  if (max_height_ < 0) {
    ZERROR << "Config " << kMaxHeightKey_
           << " invalid: " << FloatToString(max_height_, 3) << ".";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kMaxCleanWidthKey_, max_clean_width_)) {
    ZGERROR << "Config " << kMaxCleanWidthKey_ << " not found.";
    // return false;
  }
  if (max_clean_width_ < 0 || max_clean_width_ > max_width_) {
    ZERROR << "Config " << kMaxCleanWidthKey_
           << " invalid: " << FloatToString(max_clean_width_, 3) << ".";
    return false;
  }
  // For demo release.
  max_clean_width_ = 15;

  if (!JsonHelper::GetFloat(*json, kMaxCleanHeightKey_, max_clean_height_)) {
    ZGERROR << "Config " << kMaxCleanHeightKey_ << " not found.";
    // return false;
  }
  if (max_clean_height_ < 0 || max_clean_height_ > max_height_) {
    ZERROR << "Config " << kMaxCleanHeightKey_
           << " invalid: " << FloatToString(max_clean_height_, 3) << ".";
    return false;
  }
  // For demo release.
  max_clean_height_ = 15;

  return true;
}

NavMap::NavMap()
    : MultiLayersCharGridMap2D(
          "Clean map", static_cast<unsigned int>(max_width_ / resolution_),
          static_cast<unsigned int>(max_height_ / resolution_), resolution_) {
  if (!is_config_loaded_) {
    LoadConfig();
    range_x_ = static_cast<unsigned int>((max_width_ + 5) / resolution_);
    range_y_ = static_cast<unsigned int>((max_height_ + 5) / resolution_);
  }

  AddLayer(kPrintMapName_,
           std::make_shared<CharGridMap2D>(kPrintMapName_, range_x_, range_y_,
                                           resolution_, kUnknown_));
  AddLayer(kFootStepMapName_,
           std::make_shared<CharGridMap2D>(kFootStepMapName_, range_x_,
                                           range_y_, resolution_, kUnknown_));
  AddLayer(kSensorMapName_,
           std::make_shared<CharGridMap2D>(kSensorMapName_, range_x_, range_y_,
                                           resolution_, kUnknown_));
  AddLayer(kSlamMapName_,
           std::make_shared<CharGridMap2D>(kSlamMapName_, range_x_, range_y_,
                                           resolution_, kUnknown_));
  AddLayer(kRoomMapName_,
           std::make_shared<CharGridMap2D>(kRoomMapName_, range_x_, range_y_,
                                           resolution_, kUnknown_));
  AddLayer(kUserBlockMapName_,
           std::make_shared<CharGridMap2D>(kUserBlockMapName_, 1, 1,
                                           resolution_, kUnknown_));
  AddLayer(kUserSelectAreaMapName_, std::make_shared<CharGridMap2D>(
                                        kUserSelectAreaMapName_, range_x_,
                                        range_y_, resolution_, kUserSelected_));

  RefreshLayerPtrs();

  for (auto x = -kRobotCellWidth_2_; x <= kRobotCellWidth_2_; x++) {
    for (auto y = -kRobotCellWidth_2_; y <= kRobotCellWidth_2_; y++) {
      // For round robot.
      if (x == -kRobotCellWidth_2_) {
        if (y == -kRobotCellWidth_2_ || y == kRobotCellWidth_2_) {
          continue;
        }
      } else if (x == kRobotCellWidth_2_) {
        if (y == -kRobotCellWidth_2_ || y == kRobotCellWidth_2_) {
          continue;
        }
      }

      MapPoint robot_point;
      footstep_layer_->MapToWorld(MapCell(x, y), robot_point);
      robot_cover_points_.emplace_back(robot_point);
    }
  }

  for (auto x = -kRobotMarkWidth_2_; x <= kRobotMarkWidth_2_; x++) {
    for (auto y = -kRobotMarkWidth_2_; y <= kRobotMarkWidth_2_; y++) {
      MapPoint foot_print_point;
      footstep_layer_->MapToWorld(MapCell(x, y), foot_print_point);
      foot_print_points_.emplace_back(foot_print_point);
    }
  }

  map_near_4_cells_.emplace_back(MapCell(-kRobotMarkWidth_2_, 0));
  map_near_4_cells_.emplace_back(MapCell(kRobotMarkWidth_2_, 0));
  map_near_4_cells_.emplace_back(MapCell(0, -kRobotMarkWidth_2_));
  map_near_4_cells_.emplace_back(MapCell(0, kRobotMarkWidth_2_));

  // nav_map_count.fetch_add(1);
  // debug_id_ = nav_map_count.load();
  // ZGINFO << "Count " << debug_id_ << " ptr " << this << " name "
  //        << name_;
  // WriteLocker lock(nav_map_count_lock);
  // nav_map_map[debug_id_] = name_;
}

NavMap::NavMap(const NavMap& map) : MultiLayersCharGridMap2D(map) {
  ReadLocker lock(map.GetLock());
  RefreshLayerPtrs();
  robot_cover_points_ = map.robot_cover_points_;
  foot_print_points_ = map.foot_print_points_;
  map_near_4_cells_ = map.map_near_4_cells_;
  max_clean_cell_width_ = map.max_clean_cell_width_;
  max_clean_cell_height_ = map.max_clean_cell_height_;
  // footstep_layer_->Print(__FILE__, __FUNCTION__, __LINE__);
  // ReadLocker lock2(map.GetFootStepLayer()->GetLock());
  // map.GetFootStepLayer()->Print(__FILE__, __FUNCTION__, __LINE__);

  // nav_map_count.fetch_add(1);
  // debug_id_ = nav_map_count.load();
  // ZGINFO << "Count " << debug_id_ << " ptr " << this << " name "
  //        << name_;
  // WriteLocker lock_(nav_map_count_lock);
  // nav_map_map[debug_id_] = name_;
}

NavMap::~NavMap() {
  // ZGINFO << "Release nav map.";
  // WriteLocker lock(nav_map_count_lock);
  // nav_map_map.erase(debug_id_);
  // std::string debug_str;
  // for (auto&& map_id_pair : nav_map_map) {
  //   debug_str += std::to_string(map_id_pair.first) + ",";
  // }
  // ZGINFO << "Release for " << std::to_string(debug_id_) << " ptr " << this
  //        << " remaining: " << debug_str;
}

NavMap& NavMap::operator=(const NavMap& map) {
  if (&map == this) {
    return *this;
  }

  MultiLayersCharGridMap2D::operator=(map);

  this->RefreshLayerPtrs();
  this->robot_cover_points_ = map.robot_cover_points_;
  this->foot_print_points_ = map.foot_print_points_;
  this->map_near_4_cells_ = map.map_near_4_cells_;

  return *this;
}

void NavMap::LoadConfig() {
  NavMap::Config config;
  if (config.config_valid_) {
    resolution_ = config.resolution_;
    max_width_ = config.max_width_;
    max_height_ = config.max_height_;
    max_clean_cell_width_ = config.max_clean_width_ / resolution_;
    max_clean_cell_height_ = config.max_clean_height_ / resolution_;
  }
  ZINFO << "Initialize navigation map resolution as: "
        << FloatToString(resolution_, 3);
  is_config_loaded_ = true;
}

double NavMap::GetResolution() {
  if (!is_config_loaded_) {
    LoadConfig();
  }
  return resolution_;
}

void NavMap::RefreshLayerPtrs() {
  WriteLocker lock(access_);
  GetLayer(kPrintMapName_, print_layer_);
  GetLayer(kFootStepMapName_, footstep_layer_);
  GetLayer(kSensorMapName_, sensor_layer_);
  GetLayer(kSlamMapName_, slam_layer_);
  GetLayer(kRoomMapName_, room_layer_);
  GetLayer(kUserBlockMapName_, user_block_layer_);
  GetLayer(kUserSelectAreaMapName_, user_select_area_layer_);
}

CharGridMap2D::SPtr NavMap::GetPrintLayer() const {
  ReadLocker lock(access_);
  CharGridMap2D::SPtr map = nullptr;
  GetLayer(kPrintMapName_, map);
  if (map == nullptr) {
    ZERROR << "Layer " << kPrintMapName_ << " not exist.";
  }
  return map;
}

CharGridMap2D::SPtr NavMap::GetFootStepLayer() const {
  ReadLocker lock(access_);
  CharGridMap2D::SPtr map = nullptr;
  GetLayer(kFootStepMapName_, map);
  if (map == nullptr) {
    ZERROR << "Layer " << kFootStepMapName_ << " not exist.";
  }
  return map;
}

CharGridMap2D::SPtr NavMap::GetSensorLayer() const {
  ReadLocker lock(access_);
  CharGridMap2D::SPtr map = nullptr;
  GetLayer(kSensorMapName_, map);
  if (map == nullptr) {
    ZERROR << "Layer " << kSensorMapName_ << " not exist.";
  }
  return map;
}

CharGridMap2D::SPtr NavMap::GetSlamLayer() const {
  ReadLocker lock(access_);
  CharGridMap2D::SPtr map = nullptr;
  GetLayer(kSlamMapName_, map);
  if (map == nullptr) {
    ZERROR << "Layer " << kSlamMapName_ << " not exist.";
  }
  return map;
}

CharGridMap2D::SPtr NavMap::GetRoomLayer() const {
  ReadLocker lock(access_);
  CharGridMap2D::SPtr map = nullptr;
  GetLayer(kRoomMapName_, map);
  if (map == nullptr) {
    ZERROR << "Layer " << kRoomMapName_ << " not exist.";
  }
  return map;
}

CharGridMap2D::SPtr NavMap::GetUserBlockLayer() const {
  ReadLocker lock(access_);
  CharGridMap2D::SPtr map = nullptr;
  GetLayer(kUserBlockMapName_, map);
  if (map == nullptr) {
    ZERROR << "Layer " << kUserBlockMapName_ << " not exist.";
  }
  return map;
}

CharGridMap2D::SPtr NavMap::GetUserSelectAreaLayer() const {
  ReadLocker lock(access_);
  CharGridMap2D::SPtr map = nullptr;
  GetLayer(kUserSelectAreaMapName_, map);
  if (map == nullptr) {
    ZERROR << "Layer " << kUserSelectAreaMapName_ << " not exist.";
  }
  return map;
}

bool NavMap::ChangeSlamLayer(const CharGridMap2D::SPtr& new_char_slam_map) {
  WriteLocker lock(access_);
  if (CheckLayerExist(kSlamMapName_)) {
    char_grid_map_layers_.at(kSlamMapName_) = new_char_slam_map;
    lock.Unlock();
    slam_layer_ = GetSlamLayer();
  } else {
    ZERROR << "Missing layer " << kSlamMapName_;
    lock.Unlock();
    AddLayer(kSlamMapName_, new_char_slam_map);
    lock.Lock();
    slam_layer_ = GetSlamLayer();
  }
  return true;
}

bool NavMap::ChangeRoomLayer(const CharGridMap2D::SPtr& new_room_map) {
  WriteLocker lock(access_);
  if (CheckLayerExist(kRoomMapName_)) {
    char_grid_map_layers_.at(kRoomMapName_) = new_room_map;
    lock.Unlock();
    room_layer_ = GetRoomLayer();
  } else {
    ZERROR << "Missing layer " << kRoomMapName_;
    lock.Unlock();
    AddLayer(kRoomMapName_, new_room_map);
    lock.Lock();
    room_layer_ = GetRoomLayer();
  }
  return true;
}

bool NavMap::ChangeUserSelectAreaLayer(
    const CharGridMap2D::SPtr& new_user_select_area_map) {
  ZINFO;
  WriteLocker lock(access_);
  if (CheckLayerExist(kUserSelectAreaMapName_)) {
    char_grid_map_layers_.at(kUserSelectAreaMapName_) =
        new_user_select_area_map;
    lock.Unlock();
    user_select_area_layer_ = GetUserSelectAreaLayer();
  } else {
    ZERROR << "Missing layer " << kUserSelectAreaMapName_;
    lock.Unlock();
    AddLayer(kUserSelectAreaMapName_, new_user_select_area_map);
    lock.Lock();
    user_select_area_layer_ = GetUserSelectAreaLayer();
  }
  return true;
}

MapCells NavMap::GetRobotCoverCells(const MapPoint& pose) const {
  MapCells robot_cells;
  for (auto&& robot_point : robot_cover_points_) {
    double x, y;
    Transform::CoordinateTransformationBA(robot_point.X(), robot_point.Y(),
                                          pose.X(), pose.Y(), 0, x, y);
    MapPoint robot_point_in_world(x, y);
    MapCell robot_cell;
    if (!footstep_layer_->WorldToMap(robot_point_in_world, robot_cell)) {
      ZERROR << robot_cell.DebugString() << " outside of "
             << footstep_layer_->Name();
    } else {
      robot_cells.emplace_back(robot_cell);
    }
  }

  return robot_cells;
}

MapCells NavMap::GetFootstepCoverCells(const MapPoint& pose,
                                       const MapPoints& addition_point) const {
  auto foot_print_points = foot_print_points_;
  foot_print_points.insert(foot_print_points.end(), addition_point.begin(),
                           addition_point.end());
  MapCells footstep_cells;
  for (auto&& foot_print_point : foot_print_points) {
    // ZINFO << step.Pose();
    double x, y;
    Transform::CoordinateTransformationBA(foot_print_point.X(),
                                          foot_print_point.Y(), pose.X(),
                                          pose.Y(), pose.Radian(), x, y);
    MapPoint foot_print_point_in_world(x, y);
    MapCell footstep_cell;
    if (!footstep_layer_->WorldToMap(foot_print_point_in_world,
                                     footstep_cell)) {
      ZERROR << "Mark " << footstep_cell.DebugString() << " outside of "
             << footstep_layer_->Name();
    }
    footstep_cells.emplace_back(footstep_cell);
  }

  return footstep_cells;
}

void NavMap::MarkForCleaningSteps(const TransformManager& tf_manager,
                                  const Steps& path,
                                  const DynamicMapCellBound& section_bound_,
                                  const bool& mark_for_edge) {
  if (path.empty()) {
    return;
  }
  // ZINFO;

  WriteLocker print_lock(print_layer_->GetLock());
  WriteLocker footstep_lock(footstep_layer_->GetLock());
  WriteLocker sensor_lock(sensor_layer_->GetLock());
  for (auto&& step : path) {
    // ZGINFO << step.Pose();

    auto foot_print_cells = GetFootstepCoverCells(step.Pose());
    for (auto&& cell : foot_print_cells) {
      if (section_bound_.Contain(cell)) {
        if (mark_for_edge && !section_bound_.OnBound(cell)) {
          MarkForEmptyFootStep(cell);
          continue;
        }
        auto max_clean_bound =
            GetMaxCleanBound(std::make_shared<MapCell>(cell));
        if (max_clean_bound.IsValid() && !max_clean_bound.Contain(cell)) {
          continue;
        }
        MarkForFootStep(cell, kFootStep_);
      }
    }

    auto robot_cover_cells = GetRobotCoverCells(step.Pose());
    for (auto&& cell : robot_cover_cells) {
      ClearSensor(cell);
    }
  }

  // footstep_layer_->Print(__FILE__, __FUNCTION__, __LINE__);
  // GetPrintLayer()->Print(__FILE__, __FUNCTION__, __LINE__);
  footstep_lock.Unlock();

  for (auto&& step : path) {
    if (step.Markers() != nullptr) {
      for (auto&& obs_marker_point : *step.Markers()) {
        double x, y;
        Transform::CoordinateTransformationBA(
            obs_marker_point.Point().X(), obs_marker_point.Point().Y(),
            step.Pose().X(), step.Pose().Y(),
            DegreesToRadians(step.Pose().Degree()), x, y);
        MapPoint obs_point_in_world(x, y);
        MapCell obs_cell;
        if (!sensor_layer_->WorldToMap(obs_point_in_world, obs_cell)) {
          ZERROR << "Mark " << obs_cell.DebugString() << " outside of "
                 << sensor_layer_->Name();
        }
        MarkForSensor(obs_cell, obs_marker_point.Value());
        // ZINFO << "Mark " << obs_cell << " for " << obs_marker_point.Value();
      }
    }
  }

  print_lock.Unlock();
  footstep_lock.Unlock();
  sensor_lock.Unlock();
  ClearRobotCoverCells(path.back().Pose());
}

void NavMap::MarkForPassingThroughSteps(
    const TransformManager& tf_manager, const Steps& path,
    const DynamicMapCellBound& section_bound_) {
  if (path.empty()) {
    return;
  }

  WriteLocker print_lock(print_layer_->GetLock());
  WriteLocker footstep_lock(footstep_layer_->GetLock());
  WriteLocker sensor_lock(sensor_layer_->GetLock());
  for (auto&& step : path) {
    // ZINFO << step.Pose();
    auto foot_print_cells = GetFootstepCoverCells(step.Pose());
    for (auto&& cell : foot_print_cells) {
      MarkForEmptyFootStep(cell);
    }
    auto robot_cover_cells = GetRobotCoverCells(step.Pose());
    for (auto&& cell : robot_cover_cells) {
      ClearSensor(cell);
    }
  }
  footstep_lock.Unlock();

  // GetPrintLayer()->Print(__FILE__, __FUNCTION__, __LINE__);

  for (auto&& step : path) {
    if (step.Markers() != nullptr) {
      for (auto&& obs_marker_point : *step.Markers()) {
        double x, y;
        Transform::CoordinateTransformationBA(
            obs_marker_point.Point().X(), obs_marker_point.Point().Y(),
            step.Pose().X(), step.Pose().Y(),
            DegreesToRadians(step.Pose().Degree()), x, y);
        MapPoint obs_point_in_world(x, y);
        MapCell obs_cell;
        if (!sensor_layer_->WorldToMap(obs_point_in_world, obs_cell)) {
          ZERROR << "Mark " << obs_cell.DebugString() << " outside of "
                 << sensor_layer_->Name();
        }
        MarkForSensor(obs_cell, obs_marker_point.Value());
        // ZINFO << "Mark " << obs_cell << " for " << obs_marker_point.Value();
      }
    }
  }

  print_lock.Unlock();
  footstep_lock.Unlock();
  sensor_lock.Unlock();
  ClearRobotCoverCells(path.back().Pose());
}

void NavMap::MarkForAlongSideSteps(const TransformManager& tf_manager,
                                   const Steps& path, const bool& on_left,
                                   const DynamicMapCellBound& section_bound_) {
  if (path.empty()) {
    return;
  }
  // ZINFO;

  WriteLocker print_lock(print_layer_->GetLock());

  // ZINFO << "foot print points size " << foot_print_points.size();
  // ZINFO << footstep_layer_->Name();

  WriteLocker footstep_lock(footstep_layer_->GetLock());
  WriteLocker sensor_lock(sensor_layer_->GetLock());
  MapPoint addition_foot_print_point;
  if (on_left) {
    footstep_layer_->MapToWorld(MapCell(0, kRobotMarkWidth_2_ + 1),
                                addition_foot_print_point);
  } else {
    footstep_layer_->MapToWorld(MapCell(0, -kRobotMarkWidth_2_ - 1),
                                addition_foot_print_point);
  }

  MapCell last_obs_cell(0, 0);
  CharGridMap2D::DataType last_obs_value = NavMap::kUnknown_;

  for (auto&& step : path) {
    // ZINFO << step.Pose();
    // Mark foot prints.
    auto foot_print_cells =
        GetFootstepCoverCells(step.Pose(), {addition_foot_print_point});
    for (auto&& cell : foot_print_cells) {
      // Bound may be updated every time we change foot step map.
      auto max_clean_bound = GetMaxCleanBound(std::make_shared<MapCell>(cell));
      if (section_bound_.Contain(cell) && max_clean_bound.Contain(cell)) {
        MarkForFootStep(cell, kFootStep_);
      }
    }

    auto robot_cover_cells = GetRobotCoverCells(step.Pose());
    for (auto&& cell : robot_cover_cells) {
      CharGridMap2D::DataType value;
      sensor_layer_->GetValue(cell.X(), cell.Y(), value);
      if (ClearSensor(cell)) {
        // ZINFO << step.Pose();
        // Push obs to outside.
        MapPoint cover_point;
        sensor_layer_->MapToWorld(cell, cover_point);
        double x, y;
        Transform::CoordinateTransformationAB(cover_point.X(), cover_point.Y(),
                                              step.Pose().X(), step.Pose().Y(),
                                              step.Pose().Radian(), x, y);
        // MapPoint cover_point_in_robot(x, y);
        // ZINFO << "y: " << FloatToString(y, 2);
        if (on_left && y >= 0) {
          y = (kRobotCellWidth_2_ + 1) * resolution_;
          MapPoint new_obs_point(x, y);
          Transform::CoordinateTransformationBA(
              new_obs_point.X(), new_obs_point.Y(), step.Pose().X(),
              step.Pose().Y(), step.Pose().Radian(), x, y);
          MapPoint new_obs_point_in_world(x, y);
          MapCell new_obs_cell;
          if (sensor_layer_->WorldToMap(new_obs_point_in_world, new_obs_cell)) {
            // ZINFO << "Push obs to  " << new_obs_cell;
            MarkForSensor(new_obs_cell, value);
          } else {
            ZERROR << "Mark " << new_obs_cell << " outside of "
                   << sensor_layer_->Name();
          }
        } else if (!on_left && y <= 0) {
          y = -1 * (kRobotCellWidth_2_ + 1) * resolution_;
          MapPoint new_obs_point(x, y);
          Transform::CoordinateTransformationBA(
              new_obs_point.X(), new_obs_point.Y(), step.Pose().X(),
              step.Pose().Y(), step.Pose().Radian(), x, y);
          MapPoint new_obs_point_in_world(x, y);
          MapCell new_obs_cell;
          if (sensor_layer_->WorldToMap(new_obs_point_in_world, new_obs_cell)) {
            // ZINFO << "Push obs to  " << new_obs_cell;
            MarkForSensor(new_obs_cell, value);
          } else {
            ZERROR << "Mark " << new_obs_cell << " outside of "
                   << sensor_layer_->Name();
          }
        }
      }
    }

    if (step.Markers() != nullptr) {
      for (auto&& obs_point : *step.Markers()) {
        // ZINFO << step.Pose();
        double x, y;
        Transform::CoordinateTransformationBA(
            obs_point.Point().X(), obs_point.Point().Y(), step.Pose().X(),
            step.Pose().Y(), DegreesToRadians(step.Pose().Degree()), x, y);
        MapPoint obs_point_in_world(x, y);
        MapCell obs_cell;
        if (!sensor_layer_->WorldToMap(obs_point_in_world, obs_cell)) {
          ZERROR << "Mark " << obs_cell.DebugString() << " outside of "
                 << sensor_layer_->Name();
        }
        MarkForSensor(obs_cell, obs_point.Value());
        if (last_obs_cell != obs_cell || last_obs_value != obs_point.Value()) {
          last_obs_cell = obs_cell;
          last_obs_value = obs_point.Value();
          // ZINFO << "Mark " << obs_cell << " for " << obs_point.Value();
        }
      }
    }
  }

  footstep_lock.Unlock();
  sensor_lock.Unlock();

  print_lock.Unlock();
  ClearRobotCoverCells(path.back().Pose());
}

void NavMap::MarkForFootStep(const MapCell& cell,
                             const CharGridMap2D::DataType& value) {
  // Please lock:
  // 1. foot step layer (Write)
  // 2. print layer (Write)
  // outside this function.
  footstep_layer_->SetValue(cell.X(), cell.Y(), value);
  print_layer_->SetValue(cell.X(), cell.Y(), value);
  // ZGINFO << "Mark for " << cell.DebugString();
  ClearSensor(cell);
}

void NavMap::MarkForEmptyFootStep(const MapCell& cell) {
  // Please lock:
  // 1. foot step layer (Write)
  // 2. print layer (Write)
  // outside this function.
  CharGridMap2D::DataType value = kUnknown_;
  footstep_layer_->GetValue(cell.X(), cell.Y(), value);
  if (value == kUnknown_) {
    // For expending data bound but not overriding existing foot steps.
    footstep_layer_->SetValue(cell.X(), cell.Y(), kUnknown_);
    print_layer_->SetValue(cell.X(), cell.Y(), kUnknown_);
  }
  // ZGINFO << "Mark for " << cell.DebugString();
  ClearSensor(cell);
}

bool NavMap::ClearSensor(const MapCell& cell) {
  // Please lock:
  // 1. foot step layer (Write)
  // 2. sensor layer (Write)
  // 3. print layer (Write)
  // outside this function.
  CharGridMap2D::DataType sensor_value = kUnknown_;
  sensor_layer_->GetValue(cell.X(), cell.Y(), sensor_value);
  if (sensor_value != kUnknown_) {
    // ZINFO << "Clear obs on " << cell;
    sensor_layer_->SetValue(cell.X(), cell.Y(), kUnknown_);
    CharGridMap2D::DataType footstep_value = kUnknown_;
    footstep_layer_->GetValue(cell.X(), cell.Y(), footstep_value);
    if (footstep_value == kUnknown_) {
      print_layer_->SetValue(cell.X(), cell.Y(), kUnknown_);
    } else {
      print_layer_->SetValue(cell.X(), cell.Y(), footstep_value);
    }
    return true;
  }
  return false;
}

void NavMap::MarkForSensor(const MapCell& cell,
                           const CharGridMap2D::DataType& value) {
  // Please lock:
  // 1. sensor layer (Write)
  // 2. print layer (Write)
  // outside this function.
  sensor_layer_->SetValue(cell.X(), cell.Y(), value);
  print_layer_->SetValue(cell.X(), cell.Y(), value);
}

void NavMap::ClearRobotCoverCells(const MapPoint& pose) {
  WriteLocker print_lock(print_layer_->GetLock());
  WriteLocker sensor_lock(sensor_layer_->GetLock());
  ReadLocker footstep_lock(footstep_layer_->GetLock());
  auto robot_cover_cells = GetRobotCoverCells(pose);
  MapCell robot_cell;
  sensor_layer_->WorldToMap(pose, robot_cell);
  // ZINFO << "Clear robot cover cells at " << robot_cell.DebugString();
  for (auto&& cell : robot_cover_cells) {
    CharGridMap2D::DataType value;
    sensor_layer_->GetValue(cell.X(), cell.Y(), value);
    if (ClearSensor(cell) && value == NavMap::kWall_ &&
        (abs(cell.X() - robot_cell.X()) >= kRobotCellWidth_2_ ||
         abs(cell.Y() - robot_cell.Y()) >= kRobotCellWidth_2_)) {
      // Push wall to outside.
      for (auto&& map_near_4_cell : map_near_4_cells_) {
        auto check_cell = map_near_4_cell + cell;
        // ZINFO << "Check for " << check_cell.DebugString();
        if (!(abs(check_cell.X() - robot_cell.X()) > kRobotCellWidth_2_ ||
              abs(check_cell.Y() - robot_cell.Y()) > kRobotCellWidth_2_)) {
          continue;
        }
        sensor_layer_->SetValue(check_cell.X(), check_cell.Y(), NavMap::kWall_);
        print_layer_->SetValue(check_cell.X(), check_cell.Y(), NavMap::kWall_);
        // ZINFO << "Push wall from " << cell.DebugString() << " to "
        //       << check_cell.DebugString();
        break;
      }
    }
  }
}

bool NavMap::IsClearedInSensorMap(const MapCell& cell) const {
  CharGridMap2D::DataType value;
  if (sensor_layer_->GetValue(cell.X(), cell.Y(), value)) {
    // ZISODBG << "value: " << value;
    if (value == sensor_layer_->GetDefaultValue()) {
      return true;
    }
  } else if (sensor_layer_->GetAvailableBound().Contain(cell)) {
    return true;
  }
  return false;
}

float NavMap::GetStepAreaSize() const {
  ReadLocker lock(footstep_layer_->GetLock());

  auto data_bound = footstep_layer_->GetDataBound();
  uint32_t step_cell_count = 0;
  CharGridMap2D::DataType value;
  for (auto x = data_bound.GetMin().X(); x <= data_bound.GetMax().X(); x++) {
    for (auto y = data_bound.GetMin().Y(); y <= data_bound.GetMax().Y(); y++) {
      if (footstep_layer_->GetValue(x, y, value) && value == kFootStep_) {
        step_cell_count++;
      }
    }
  }

  return step_cell_count * Square(resolution_);
}

void NavMap::ClearSteps() {
  WriteLocker access_lock(access_);
  ZGINFO << "Clear steps for " << name_;
  print_layer_->ResetMap();
  footstep_layer_->ResetMap();
  sensor_layer_->ResetMap();
}

DynamicMapCellBound NavMap::GetMaxCleanBound(
    const MapCell::SPtr& curr_pose) const {
  auto footstep_map = GetFootStepLayer();
  ReadLocker footstep_map_read_lock(footstep_map->GetLock(), false);
  if (!footstep_map->GetLock()->IsLockedByThisThread()) {
    footstep_map_read_lock.Lock();
  }
  auto footstep_map_data_bound = footstep_map->GetDataBound();
  auto footstep_map_max_clean_bound = footstep_map_data_bound;
  // ZINFO << "MaxCleanCellWidth: " << GetMaxCleanCellWidth();
  // ZINFO << "MaxCleanCellHeight: " << GetMaxCleanCellHeight();
  if (!footstep_map_data_bound.IsValid() && curr_pose != nullptr) {
    footstep_map_max_clean_bound.Reset(
        curr_pose->X() - GetMaxCleanCellWidth(),
        curr_pose->X() + GetMaxCleanCellWidth(),
        curr_pose->Y() - GetMaxCleanCellHeight(),
        curr_pose->Y() + GetMaxCleanCellHeight());
  } else {
    footstep_map_max_clean_bound.Reset(
        footstep_map_data_bound.GetMax().X() - GetMaxCleanCellWidth(),
        footstep_map_data_bound.GetMin().X() + GetMaxCleanCellWidth(),
        footstep_map_data_bound.GetMax().Y() - GetMaxCleanCellHeight(),
        footstep_map_data_bound.GetMin().Y() + GetMaxCleanCellHeight());
  }
  // ZINFO << footstep_map_max_clean_bound.DebugString();
  return footstep_map_max_clean_bound;
}

bool NavMapLoader::LoadMap(NavMap::SPtr& map) {
  MultiLayersCharGridMap2D::SPtr _map = map;
  auto ret = MultiLayersCharGridMap2DLoader::LoadMap(_map);
  if (!ret) {
    return ret;
  }
  if (!map->CheckLayerExist(NavMap::kPrintMapName_)) {
    ZERROR << "Missing map: " << NavMap::kPrintMapName_;
    return false;
  }
  if (!map->CheckLayerExist(NavMap::kSensorMapName_)) {
    ZERROR << "Missing map: " << NavMap::kSensorMapName_;
    return false;
  }
  if (!map->CheckLayerExist(NavMap::kFootStepMapName_)) {
    ZERROR << "Missing map: " << NavMap::kFootStepMapName_;
    return false;
  }
  if (!map->CheckLayerExist(NavMap::kRoomMapName_)) {
    ZERROR << "Missing map: " << NavMap::kRoomMapName_;
    return false;
  }

  return ret;
}

bool NavMapWriter::WriteMap(const NavMap::SPtr& map, const bool& is_binary) {
  MultiLayersCharGridMap2D::SPtr _map = map;
  return MultiLayersCharGridMap2DWriter::WriteMap(_map, is_binary);
}

}  // namespace zima
