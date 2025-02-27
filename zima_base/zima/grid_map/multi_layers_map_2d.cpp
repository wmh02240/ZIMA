/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/grid_map/multi_layers_map_2d.h"

#include <algorithm>

#include "zima/common/gflags.h"
#include "zima/logger/logger.h"

namespace zima {

MultiLayersCharGridMap2D::MultiLayersCharGridMap2D(const std::string name,
                                                   const unsigned int& range_x,
                                                   const unsigned int& range_y,
                                                   const double& resolution)
    : access_(make_shared<ReadWriteLock>()),
      name_(name),
      resolution_(resolution),
      range_x_(range_x),
      range_y_(range_y) {
  char_grid_map_layers_.clear();
}

MultiLayersCharGridMap2D::MultiLayersCharGridMap2D(
    const MultiLayersCharGridMap2D& map)
    : access_(make_shared<ReadWriteLock>()),
      name_(map.name_ + " copy"),
      resolution_(map.resolution_),
      range_x_(map.range_x_),
      range_y_(map.range_y_) {
  char_grid_map_layers_.clear();
  for (auto&& pair : map.char_grid_map_layers_) {
    char_grid_map_layers_.emplace(
        pair.first, CharGridMap2D::SPtr(new CharGridMap2D(*pair.second)));
  }
}

MultiLayersCharGridMap2D& MultiLayersCharGridMap2D::operator=(
    const MultiLayersCharGridMap2D& map) {
  if (&map == this) {
    return *this;
  }

  ReadLocker lock(map.GetLock());
  this->access_ = make_shared<ReadWriteLock>();
  this->name_ = map.name_ + CharGridMap2D::kCopySuffix_;
  this->char_grid_map_layers_.clear();
  for (auto&& pair : map.char_grid_map_layers_) {
    this->char_grid_map_layers_.emplace(
        pair.first, CharGridMap2D::SPtr(new CharGridMap2D(*pair.second)));
  }
  this->resolution_ = map.resolution_;
  this->range_x_ = map.range_x_;
  this->range_y_ = map.range_y_;

  return *this;
}

bool MultiLayersCharGridMap2D::CheckLayerExist(
    const std::string& layer_name) const {
  auto it_layer = char_grid_map_layers_.find(layer_name);
  return it_layer != char_grid_map_layers_.end();
}

bool MultiLayersCharGridMap2D::GetLayer(const std::string& layer_name,
                                        CharGridMap2D::SPtr& p_map) const {
  if (CheckLayerExist(layer_name)) {
    p_map = char_grid_map_layers_.at(layer_name);
    return true;
  }
  return false;
}
void MultiLayersCharGridMap2D::RemoveCopySuffix() {
  WriteLocker lock(access_);
  // ZINFO << "Name before: " << name_;
  while (StringEndsWith(name_, CharGridMap2D::kCopySuffix_)) {
    for (auto len = 0u; len < CharGridMap2D::kCopySuffix_.size(); len++) {
      name_.pop_back();
    }
  }
  // ZINFO << "Name after: " << name_;

  for (auto&& layer_pair : char_grid_map_layers_) {
    layer_pair.second->RemoveCopySuffix();
  }
}

std::string MultiLayersCharGridMap2D::DebugString() const {
  std::string msg;
  msg += "\nMulti layer map(" + name_ + "):\n";
  msg += "Resolution:(" + std::to_string(resolution_) + ")\n";
  msg += "Range:(" + std::to_string(range_x_) + ", " +
         std::to_string(range_y_) + ")\n";
  msg += "Contains:\n";
  msg += "========";
  for (auto&& pair : char_grid_map_layers_) {
    msg += pair.second->BasicInfoString();
  }
  msg += "========";
  return msg;
}

bool MultiLayersCharGridMap2D::AddLayer(const std::string& layer_name,
                                        CharGridMap2D::SPtr p_map) {
  WriteLocker lock(access_);
  if (CheckLayerExist(layer_name)) {
    ZWARN << "Layer: " << layer_name << " is already added.";
    return false;
  }
  char_grid_map_layers_.emplace(layer_name, p_map);
  return true;
}

class SlidingWindowMaximum {
 public:
  SlidingWindowMaximum() = delete;
  explicit SlidingWindowMaximum(const uint16_t& width) : width_(width) {}

  void MoveToCell(const MapCell& cell, const int8_t& value) {
    // ZINFO << cell.DebugString() << " value: " << std::to_string(value);
    while (!non_ascending_maxima_.empty() &&
           value > non_ascending_maxima_.back()) {
      non_ascending_maxima_.pop_back();
    }
    non_ascending_maxima_.push_back(value);
    contain_cells_.push_back(std::pair<MapCell, int8_t>(cell, value));
    // ZWARN << DebugMaxima();
    // ZWARN << DebugCells();

    while (contain_cells_.size() > width_) {
      if (contain_cells_.front().second == non_ascending_maxima_.front()) {
        // ZINFO << "Pop: " << std::to_string(non_ascending_maxima_.front());
        non_ascending_maxima_.pop_front();
      }
      // ZINFO << "Pop: " << contain_cells_.front().first.DebugString();
      contain_cells_.pop_front();
    }

    // ZWARN << DebugMaxima();
    // ZWARN << DebugCells();
    // ZWARN << "============";
  }

  int8_t GetMaximum() const {
    if (!non_ascending_maxima_.empty()) {
      return non_ascending_maxima_.front();
    } else {
      return INT8_MIN;
    }
  }

  std::string DebugMaxima() {
    std::string str = "Maxima: ";
    for (auto&& value : non_ascending_maxima_) {
      str += std::to_string(value) + ",";
    }
    return str;
  }

  std::string DebugCells() {
    std::string str = "Cells: ";
    for (auto&& pair : contain_cells_) {
      str += pair.first.DebugString() +
             " value: " + std::to_string(pair.second) + ", ";
    }
    return str;
  }

 private:
  uint16_t width_;
  std::deque<int8_t> non_ascending_maxima_;
  std::deque<std::pair<MapCell, int8_t>> contain_cells_;
};

MultiResolutionSlamValueGridMap2D::MultiResolutionSlamValueGridMap2D(
    const std::string name,
    SlamValueGridMap2D::SPtr highest_resolution_slam_value_grid_map,
    const uint8_t& depth)
    : access_(make_shared<ReadWriteLock>()),
      name_(name),
      highest_resolution_slam_value_grid_map_(
          highest_resolution_slam_value_grid_map),
      depth_(depth) {
  ZGINFO << "Layers depth (" << std::to_string(depth_) << ").";
  ZGINFO << "Source: " << highest_resolution_slam_value_grid_map_->Name();
  if (depth_ == 0) {
    ZWARN << "Layers depth invalid(" << depth_ << ").";
  }

  tmp_grid_map_.reset(new SlamValueGridMap2D(
      highest_resolution_slam_value_grid_map_->Name() + " tmp copy",
      highest_resolution_slam_value_grid_map_->GetRangeX() +
          (1 << (depth_ - 1)),
      highest_resolution_slam_value_grid_map_->GetRangeY() +
          (1 << (depth_ - 1)),
      highest_resolution_slam_value_grid_map_->GetResolution()));

  GenerateLayers();
}

bool MultiResolutionSlamValueGridMap2D::GetLayer(
    const uint8_t& level, SlamValueGridMap2D::SPtr& p_map) {
  if (level < depth_) {
    p_map = slam_value_grid_map_layers_[level];
    return true;
  }
  return false;
}

SlamValueGridMap2D::SPtr MultiResolutionSlamValueGridMap2D::GenerateLayer(
    const uint8_t& level) {
  if (level == 0) {
    return highest_resolution_slam_value_grid_map_;
  }

  auto window_width = 1 << level;

  SlamValueGridMap2D::SPtr new_map(new SlamValueGridMap2D(
      highest_resolution_slam_value_grid_map_->Name() + " " +
          std::to_string(level),
      highest_resolution_slam_value_grid_map_->GetRangeX() + window_width - 1,
      highest_resolution_slam_value_grid_map_->GetRangeY() + window_width - 1,
      highest_resolution_slam_value_grid_map_->GetResolution()));
  // new_map->Print(__FILE__, __FUNCTION__, __LINE__);

  // Generate lower resolution map.
  auto source_data_bound =
      highest_resolution_slam_value_grid_map_->GetDataBound();
  auto new_data_bound = source_data_bound;
  new_data_bound.Expend(new_data_bound.GetMax() +
                        MapCell(window_width - 1, window_width - 1));
  // ZINFO << source_data_bound.DebugString();
  // ZINFO << new_data_bound.DebugString();

  if (new_data_bound.GetMin().X() + window_width - 1 > INT_MAX ||
      new_data_bound.GetMin().Y() + window_width - 1 > INT_MAX) {
    ZERROR << "Map too large: " << new_data_bound.DebugString()
           << ", window width: " << window_width
           << ", x max: " << new_data_bound.GetMin().X()
           << ", y max: " << new_data_bound.GetMin().Y()
           << ", x max: " << new_data_bound.GetMin().X() + window_width - 1
           << ", y max: " << new_data_bound.GetMin().Y() + window_width - 1;
  }

  auto tmp_x_max =
      std::max(static_cast<int>(new_data_bound.GetMin().X() + window_width - 1),
               new_data_bound.GetMax().X());
  auto tmp_y_max =
      std::max(static_cast<int>(new_data_bound.GetMin().Y() + window_width - 1),
               new_data_bound.GetMax().Y());

  ReadLocker highest_resolution_map_lock(
      highest_resolution_slam_value_grid_map_->GetLock());
  WriteLocker tmp_map_lock(tmp_grid_map_->GetLock());
  WriteLocker new_map_lock(new_map->GetLock());
  for (auto y = source_data_bound.GetMin().Y();
       y <= source_data_bound.GetMax().Y(); y++) {
    // ZINFO << "Y: " << y;
    SlidingWindowMaximum window(window_width);
    for (int x = new_data_bound.GetMin().X(); x <= tmp_x_max; x++) {
      SlamValueGridMap2D::DataType value;
      if (!highest_resolution_slam_value_grid_map_->GetValue(x, y, value)) {
        value = highest_resolution_slam_value_grid_map_->GetDefaultValue();
      }
      window.MoveToCell(MapCell(x, y), value);
      tmp_grid_map_->SetValue(x, y, window.GetMaximum());
    }
  }

  // tmp_grid_map_->Print(__FILE__, __FUNCTION__, __LINE__);

  for (auto x = new_data_bound.GetMin().X(); x <= tmp_x_max; x++) {
    // ZINFO << "X: " << x;
    SlidingWindowMaximum window(window_width);
    for (int y = new_data_bound.GetMin().Y(); y <= tmp_y_max; y++) {
      SlamValueGridMap2D::DataType value;
      if (!tmp_grid_map_->GetValue(x, y, value)) {
        value = tmp_grid_map_->GetDefaultValue();
      }
      window.MoveToCell(MapCell(x, y), value);
      new_map->SetValue(x, y, window.GetMaximum());
    }
  }

  // new_map->Print(__FILE__, __FUNCTION__, __LINE__);
  return new_map;
}

void MultiResolutionSlamValueGridMap2D::GenerateLayers() {
  for (auto level = 0u; level < depth_; level++) {
    slam_value_grid_map_layers_.emplace(level, GenerateLayer(level));
    ZGINFO << "emplace level " << level << " map.";
  }
}

ZimaProto::Map::PMultiLayerCharGridMap2D MultiLayerMap2DSerializer::ToProto(
    const MultiLayersCharGridMap2D::SPtr& map) {
  ZimaProto::Map::PMultiLayerCharGridMap2D proto;
  if (map == nullptr) {
    ZERROR << "Map pointer empty.";
    return proto;
  }
  ReadLocker lock(map->GetLock());
  for (auto&& layers_pair : map->char_grid_map_layers_) {
    ReadLocker lock2(layers_pair.second->GetLock());
    proto.mutable_map_dict()->insert(
        {layers_pair.first,
         DynamicMap2DSerializer::ToProto(layers_pair.second)});
  }
  return proto;
}

bool MultiLayerMap2DSerializer::FromProto(
    MultiLayersCharGridMap2D::SPtr& map,
    const ZimaProto::Map::PMultiLayerCharGridMap2D& proto,
    const unsigned int& max_width, const unsigned int& max_height) {
  if (map == nullptr) {
    ZERROR << "Map pointer empty.";
    return false;
  }
  WriteLocker lock(map->GetLock());
  for (auto&& proto_layers_pair : proto.map_dict()) {
    // ZINFO << "Load for layer: " << proto_layers_pair.first;
    CharGridMap2D::SPtr layer(
        new CharGridMap2D("temp map", max_width, max_height, 1));
    if (DynamicMap2DSerializer::FromProto(layer, proto_layers_pair.second)) {
      if (map->CheckLayerExist(proto_layers_pair.first)) {
        map->char_grid_map_layers_[proto_layers_pair.first].swap(layer);
      } else {
        lock.Unlock();
        map->AddLayer(proto_layers_pair.first, layer);
        lock.Lock();
      }
    }
  }
  return true;
}

bool MultiLayersCharGridMap2DLoader::LoadMap(
    MultiLayersCharGridMap2D::SPtr& map) {
  if (map == nullptr) {
    ZERROR << "Map pointer empty.";
    return false;
  }
  ZimaProto::Map::PMultiLayerCharGridMap2D proto;
  if (GetProtoFromBinaryFile(&proto) || GetProtoFromASCIIFile(&proto)) {
    if (MultiLayerMap2DSerializer::FromProto(map, proto, map->GetRangeX(),
                                             map->GetRangeY())) {
      ZINFO << "Load " << file_name_ << " success.";
      return true;
    }
  }
  ZERROR << "Load " << file_name_ << " failed.";
  return false;
}

bool MultiLayersCharGridMap2DWriter::WriteMap(
    const MultiLayersCharGridMap2D::SPtr& map, const bool& is_binary) {
  if (map == nullptr) {
    ZERROR << "Map pointer empty.";
    return false;
  }
  auto proto = MultiLayerMap2DSerializer::ToProto(map);
  if (is_binary ? SetProtoToBinaryFile(proto) : SetProtoToASCIIFile(proto)) {
    ZINFO << "Write " << file_name_ << " success.";
    return true;
  } else {
    ZERROR << "Write " << file_name_ << " failed.";
    return false;
  }
}

}  // namespace zima
