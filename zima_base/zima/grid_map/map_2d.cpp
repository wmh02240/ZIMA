/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/grid_map/map_2d.h"

#include "zima/common/config.h"
#include "zima/common/gflags.h"
#include "zima/common/maths.h"
#include "zima/logger/logger.h"

namespace zima {

const std::string StaticMap2DBase::kCopySuffix_ = " copy";

bool SlamValueGridMap2D::is_config_loaded_ = false;
SlamValueGridMap2D::DataType SlamValueGridMap2D::predefine_default_value_ = -1;
SlamValueGridMap2D::DataType SlamValueGridMap2D::predefine_min_space_value_ = 0;
SlamValueGridMap2D::DataType SlamValueGridMap2D::predefine_medium_value_ = 50;
SlamValueGridMap2D::DataType SlamValueGridMap2D::predefine_max_obstacle_value_ =
    100;

const CharGridMap2D::DataType CharGridMap2D::kCharGridMap2DDefaultValue_ = '.';

// ReadWriteLock::SPtr dynamic_map_count_lock = make_shared<ReadWriteLock>();
// std::map<uint, std::string> dynamic_map_map;
// atomic_uint dynamic_map_count;

StaticMap2DBase::StaticMap2DBase(const std::string& name,
                                   const float& resolution)
    : access_(make_shared<ReadWriteLock>()),
      name_(name),
      resolution_(resolution),
      data_bound_(INT32_MAX, INT32_MIN, INT32_MAX, INT32_MIN),
      marked_(false) {
  // ZINFO << name_ << " optimize range " << optimize_range_x_ << ", "
  //       << optimize_range_y_;
  // static_map_count.fetch_add(1);
  // ZGINFO << "Count " << static_map_count.load() << " ptr " << this << " name "
  //        << name_;
  // WriteLocker lock(static_map_count_lock);
  // debug_id_ = static_map_count.load();
  // static_map_map[debug_id_] = name_;
}

StaticMap2DBase::StaticMap2DBase(const StaticMap2DBase& map)
    : StaticMap2DBase(map, map.name_ + kCopySuffix_) {
  // ZERROR << name_;
}

StaticMap2DBase::StaticMap2DBase(const StaticMap2DBase& map,
                                 const std::string& name)
    : access_(make_shared<ReadWriteLock>()),
      name_(name),
      resolution_(map.resolution_),
      data_bound_(map.data_bound_),
      marked_(map.marked_) {
  // ZERROR << name_;
  // static_map_count.fetch_add(1);
  // ZGINFO << "Count " << static_map_count.load() << " ptr " << this << " name
  // "
  //        << name_;
  // WriteLocker lock(static_map_count_lock);
  // debug_id_ = static_map_count.load();
  // static_map_map[debug_id_] = name_;
}

StaticMap2DBase::~StaticMap2DBase() {
  // WriteLocker lock(static_map_count_lock);
  // static_map_map.erase(debug_id_);
  // std::string debug_str;
  // for (auto && map_id_pair : static_map_map) {
  //   debug_str += std::to_string(map_id_pair.first) + ",";
  // }
  // ZGINFO << "Release for " << std::to_string(debug_id_) << " ptr " << this
  //        << " remaining: " << debug_str;
}

StaticMap2DBase& StaticMap2DBase::operator=(const StaticMap2DBase& map) {
  if (&map == this) {
    return *this;
  }

  this->access_ = make_shared<ReadWriteLock>();
  this->name_ = map.name_ + kCopySuffix_;
  // ZERROR << this->name_;
  this->resolution_ = map.resolution_;
  this->data_bound_ = map.data_bound_;
  this->marked_ = map.marked_;

  return *this;
}

void StaticMap2DBase::MapToWorld(const int& x, const int& y, float& wx,
                                 float& wy) const {
  wx = (x)*resolution_;
  wy = (y)*resolution_;
}

void StaticMap2DBase::MapToWorld(const MapCell& map_cell,
                                 MapPoint& map_point) const {
  map_point.X((map_cell.X()) * resolution_);
  map_point.Y((map_cell.Y()) * resolution_);
}

bool StaticMap2DBase::WorldToMap(const float& wx, const float& wy, int& x,
                                 int& y) const {
  x = static_cast<int>(round(wx / resolution_));
  y = static_cast<int>(round(wy / resolution_));

  return true;
}

bool StaticMap2DBase::WorldToMap(const MapPoint& map_point,
                                 MapCell& map_cell) const {
  map_cell.X(static_cast<int>(round(map_point.X() / resolution_)));
  map_cell.Y(static_cast<int>(round(map_point.Y() / resolution_)));

  return true;
}

MapCells StaticMap2DBase::GenerateCellsBetweenTwoCells(
    const MapCell& a, const MapCell& b) const {
  MapCells ret;
  auto x_step = abs(b.X() - a.X());
  auto y_step = abs(b.Y() - a.Y());
  auto total_step = Maximum(x_step, y_step);
  if (total_step == 0) {
    ret.emplace_back(a);
    ret.emplace_back(b);
    return ret;
  }

  MapPoint a_point, b_point;
  MapToWorld(a, a_point);
  MapToWorld(b, b_point);
  double step_point_x =
      x_step == 0 ? 0 : (b_point.X() - a_point.X()) / total_step;
  double step_point_y =
      y_step == 0 ? 0 : (b_point.Y() - a_point.Y()) / total_step;
  MapPoint step_point(step_point_x, step_point_y);

  int step_count = 0;
  MapPoint tmp_point = a_point;
  ret.emplace_back(a);
  while (step_count < total_step) {
    tmp_point += step_point;
    MapCell tmp_cell;
    WorldToMap(tmp_point, tmp_cell);
    ret.emplace_back(tmp_cell);
    step_count++;
  }

  CHECK(ret.back() == b);

  return ret;
}

std::string StaticMap2DBase::BasicInfoString() const {
  ReadLocker lock(access_, false);
  if (!access_->IsLockedByThisThread()) {
    lock.Lock();
  }

  std::string msg;
  msg += "\n";
  msg += "Name:(" + name_ + ")(" + std::to_string(marked_) + ")\n";
  msg += "Resolution:(" + std::to_string(resolution_) + ")\n";
  msg += "Data bound: " + data_bound_.DebugString() + "\n";
  return msg;
}

std::vector<std::string> StaticMap2DBase::DebugStringWrapper(
    const DynamicMapCellBound& bound, PrintCellFunc func) const {
  ReadLocker lock(access_, false);
  if (!access_->IsLockedByThisThread()) {
    lock.Lock();
  }

  std::vector<std::string> msgs;
  std::string msg = BasicInfoString();
  // ZINFO << msg;
  if (!marked_) {
    // ZINFO;
    msgs.emplace_back(msg);
    return msgs;
  }

  msg += "\n";
  for (int y = bound.GetMax().Y(); y >= bound.GetMin().Y(); y--) {
    if (msg.length() > 25000) {
      msgs.emplace_back(msg);
      msg.clear();
      msg += "\n";
    }
    if (y >= 0) {
      msg += " " + std::to_string(y) + "\t";
    } else {
      msg += std::to_string(y) + "\t";
    }

    for (auto x = bound.GetMin().X(); x <= bound.GetMax().X(); x++) {
      if (!access_->IsLockedByThisThread()) {
        ZGINFO << x << ", " << y;
      }
      if (func) {
        msg += func(x, y);
      }
    }
    msg += "\n";
  }

  if (msg.length() > 25000) {
    msgs.emplace_back(msg);
    msg.clear();
    msg += "\n";
  }

  msg += "\n";
  {
    std::vector<std::string> str_vector;

    uint8_t max_num_size = 1;
    {
      auto _x = bound.GetMin().X();
      uint8_t num_size = _x < 0 ? 2 : 1;
      while (_x / 10 != 0) {
        num_size++;
        _x /= 10;
      }
      max_num_size = Maximum(max_num_size, num_size);
    }
    {
      auto _x = bound.GetMax().X();
      uint8_t num_size = _x < 0 ? 2 : 1;
      while (_x / 10 != 0) {
        num_size++;
        _x /= 10;
      }
      max_num_size = Maximum(max_num_size, num_size);
    }
    // Initialize for line count.
    while (str_vector.size() < max_num_size) {
      str_vector.emplace_back("\t");
    }

    for (auto x = bound.GetMin().X(); x <= bound.GetMax().X(); x++) {
      auto _x = x;
      uint8_t str_index = 0;
      // ZINFO << "Process for " << _x;
      do {
        str_vector.at(str_index) += std::to_string(abs(_x) % 10);
        str_index++;
        _x /= 10;
      } while (_x != 0);

      if (x < 0) {
        str_vector.at(str_index) += "-";
        str_index++;
      }

      while (str_index < str_vector.size()) {
        str_vector.at(str_index) += " ";
        str_index++;
      }
    }

    std::reverse(str_vector.begin(), str_vector.end());
    for (auto&& str : str_vector) {
      msg += str;
      msg += "\n";
    }
  }
  msg += "\n";

  msgs.emplace_back(msg);
  return msgs;
}

std::vector<std::string> StaticMap2DBase::DebugStringWrapper(
    PrintCellFunc func) const {
  return DebugStringWrapper(data_bound_, func);
}

void StaticMap2DBase::RemoveCopySuffix() {
  WriteLocker lock(access_);
  // ZINFO << "Name before: " << name_;
  while (StringEndsWith(name_, kCopySuffix_)) {
    for (auto len = 0u; len < kCopySuffix_.size(); len++) {
      name_.pop_back();
    }
  }
  // ZINFO << "Name after: " << name_;
}

DynamicMap2DBase::DynamicMap2DBase(const std::string& name,
                                   const uint16_t& cells_range_x,
                                   const uint16_t& cells_range_y,
                                   const float& resolution)
    : StaticMap2DBase(name, resolution),
      memory_optimize_factor_(5),
      range_x_(cells_range_x),
      range_y_(cells_range_y),
      data_size_(cells_range_x * cells_range_y),
      available_bound_(INT32_MIN, INT32_MAX, INT32_MIN, INT32_MAX) {
  if (range_x_ < memory_optimize_factor_) {
    optimize_range_x_ = range_x_;
  } else {
    optimize_range_x_ = range_x_ / memory_optimize_factor_;
  }
  if (range_y_ < memory_optimize_factor_) {
    optimize_range_y_ = range_y_;
  } else {
    optimize_range_y_ = range_y_ / memory_optimize_factor_;
  }
  optimize_data_size_ = Minimum(static_cast<uint32_t>(optimize_range_x_) *
                                    static_cast<uint32_t>(optimize_range_y_),
                                data_size_);
  // ZINFO << name_ << " range " << range_x_ << ", " << range_y_
  //       << ", optimize range " << optimize_range_x_ << ", "
  //       << optimize_range_y_;
  // dynamic_map_count.fetch_add(1);
  // ZGINFO << "Count " << dynamic_map_count.load() << " ptr " << this << " name
  // "
  //        << name_;
  // WriteLocker lock(dynamic_map_count_lock);
  // debug_id_ = dynamic_map_count.load();
  // dynamic_map_map[debug_id_] = name_;
}

DynamicMap2DBase::DynamicMap2DBase(const DynamicMap2DBase& map)
    : DynamicMap2DBase(map, map.name_ + kCopySuffix_) {
  // ZERROR << name_;
}

DynamicMap2DBase::DynamicMap2DBase(const DynamicMap2DBase& map,
                                   const std::string& name)
    : StaticMap2DBase(map, name),
      memory_optimize_factor_(map.memory_optimize_factor_),
      range_x_(map.range_x_),
      optimize_range_x_(map.optimize_range_x_),
      range_y_(map.range_y_),
      optimize_range_y_(map.optimize_range_y_),
      data_size_(map.data_size_),
      optimize_data_size_(map.optimize_data_size_),
      available_bound_(map.available_bound_) {
  // ZERROR << name_;
  // dynamic_map_count.fetch_add(1);
  // ZGINFO << "Count " << dynamic_map_count.load() << " ptr " << this << " name "
  //        << name_;
  // WriteLocker lock(dynamic_map_count_lock);
  // debug_id_ = dynamic_map_count.load();
  // dynamic_map_map[debug_id_] = name_;
}

DynamicMap2DBase::~DynamicMap2DBase() {
  // WriteLocker lock(dynamic_map_count_lock);
  // dynamic_map_map.erase(debug_id_);
  // std::string debug_str;
  // for (auto && map_id_pair : dynamic_map_map) {
  //   debug_str += std::to_string(map_id_pair.first) + ",";
  // }
  // ZGINFO << "Release for " << std::to_string(debug_id_) << " ptr " << this
  //        << " remaining: " << debug_str;
}

DynamicMap2DBase& DynamicMap2DBase::operator=(const DynamicMap2DBase& map) {
  if (&map == this) {
    return *this;
  }

  StaticMap2DBase::operator=(map);
  this->memory_optimize_factor_ = map.memory_optimize_factor_;
  this->range_x_ = map.range_x_;
  this->optimize_range_x_ = map.optimize_range_x_;
  this->range_y_ = map.range_y_;
  this->optimize_range_y_ = map.optimize_range_y_;
  this->data_size_ = map.data_size_;
  this->optimize_data_size_ = map.optimize_data_size_;
  this->available_bound_ = map.available_bound_;

  return *this;
}

bool DynamicMap2DBase::WorldToMap(const float& wx, const float& wy, int& x,
                                  int& y) const {
  if (!StaticMap2DBase::WorldToMap(wx, wy, x, y)) {
    return false;
  }

  if (!available_bound_.Contain(x, y)) {
    return false;
  }
  return true;
}

bool DynamicMap2DBase::WorldToMap(const MapPoint& map_point,
                                  MapCell& map_cell) const {
  if (!StaticMap2DBase::WorldToMap(map_point, map_cell)) {
    return false;
  }

  if (!available_bound_.Contain(map_cell)) {
    return false;
  }
  return true;
}

uint32_t DynamicMap2DBase::GetIndex(const int& x, const int& y) const {
  return GetIndex(x, y, optimize_range_x_, optimize_range_y_);
}

uint32_t DynamicMap2DBase::GetIndex(const int& x, const int& y,
                                    const int& x_range,
                                    const int& y_range) const {
  auto _x = x % static_cast<int>(x_range);
  if (_x < 0) {
    _x += x_range;
  }
  auto _y = y % static_cast<int>(y_range);
  if (_y < 0) {
    _y += y_range;
  }
  // ZINFO << _x << ", " << _y << ", " << x_range << ", " << y_range;
  if (IsDebugFuncEnabled()) {
    CHECK(_x >= 0 && _x <= static_cast<int>(x_range));
    CHECK(_y >= 0 && _y <= static_cast<int>(y_range));
  }

  return _x + _y * x_range;
}

std::string DynamicMap2DBase::BasicInfoString() const {
  ReadLocker lock(access_, false);
  if (!access_->IsLockedByThisThread()) {
    lock.Lock();
  }

  std::string msg = StaticMap2DBase::BasicInfoString();
  msg += "Data size:(" + std::to_string(range_x_) + "*" +
         std::to_string(range_y_) + ")\n";
  msg += "Available bound: " + available_bound_.DebugString() + "\n";
  return msg;
}

std::vector<std::string> DynamicMap2DBase::DebugStringWrapper(
    const DynamicMapCellBound& bound, PrintCellFunc func) const {
  ReadLocker lock(access_, false);
  if (!access_->IsLockedByThisThread()) {
    lock.Lock();
  }

  if (range_x_ == 0 || range_y_ == 0) {
    std::vector<std::string> msgs;
    std::string msg = BasicInfoString();
    // ZINFO << msg;
    // ZINFO;
    msgs.emplace_back(msg);
    return msgs;
  }

  return StaticMap2DBase::DebugStringWrapper(bound, func);
}

std::vector<std::string> DynamicMap2DBase::DebugStringWrapper(
    PrintCellFunc func) const {
  return DebugStringWrapper(data_bound_, func);
}

void CharGridMap2D::CellPathToPointPath(CharGridMap2D::SPtr map,
                                        const MapCellPath& cell_path,
                                        MapPointPath& point_path) {
  point_path.clear();
  auto it1 = cell_path.begin();
  auto it2 = it1 + 1;
  float degree = 0;
  while (it1 != cell_path.end()) {
    MapPoint point1, point2;
    map->MapToWorld(*it1, point1);
    if (it2 != cell_path.end()) {
      map->MapToWorld(*it2, point2);
      degree = NormalizeDegree(MapPoint::GetVector(point1, point2).Degree());
    }
    point1.SetDegree(degree);

    point_path.emplace_back(point1);
    it1++;
    if (it1 != cell_path.end()) {
      it2 = it1 + 1;
    }
  }
}

void CharGridMap2D::PointPathToCellPath(CharGridMap2D::SPtr map,
                                        const MapPointPath& point_path,
                                        MapCellPath& cell_path) {
  cell_path.clear();
  auto it = point_path.begin();
  while (it != point_path.end()) {
    MapCell cell;
    if (map->WorldToMap(*it, cell)) {
      cell_path.emplace_back(cell);
    } else {
      ZWARN << "Point " << *it << " is not in map.";
      break;
    }
    it++;
  }
}

SlamValueGridMap2D::Config::Config() : Config(nullptr){};

SlamValueGridMap2D::Config::Config(const JsonSPtr& json)
    : config_valid_(false) {
  // Load default setting.
  default_value_ = -1;
  min_space_value_ = 0;
  medium_value_ = 50;
  max_obstacle_value_ = 100;

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
      ZGERROR;
    }
  }
};

bool SlamValueGridMap2D::Config::ParseFromJson(const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }

  // Override.
  if (!JsonHelper::GetInt(*json, kDefaultValueKey_, default_value_)) {
    ZGERROR << "Config " << kDefaultValueKey_ << " not found.";
    return false;
  }
  if (!JsonHelper::GetInt(*json, kMinSpaceValueKey_, min_space_value_)) {
    ZGERROR << "Config " << kMinSpaceValueKey_ << " not found.";
    return false;
  }
  if (!JsonHelper::GetInt(*json, kMediumValueKey_, medium_value_)) {
    ZGERROR << "Config " << kMediumValueKey_ << " not found.";
    return false;
  }
  if (!JsonHelper::GetInt(*json, kMaxObstacleValueKey_, max_obstacle_value_)) {
    ZGERROR << "Config " << kMaxObstacleValueKey_ << " not found.";
    return false;
  }

  return true;
}

SlamValueGridMap2D::SlamValueGridMap2D(const std::string& name,
                                       const uint16_t& cells_range_x,
                                       const uint16_t& cells_range_y,
                                       const float& resolution)
    : Int8GridMap2D(name, cells_range_x, cells_range_y, resolution,
                    predefine_default_value_) {
  if (!is_config_loaded_) {
    LoadConfig();
    default_value_ = predefine_default_value_;
    ResetMap();
  }
}

void SlamValueGridMap2D::LoadConfig() {
  SlamValueGridMap2D::Config config;
  if (config.config_valid_) {
    predefine_default_value_ = config.default_value_;
    predefine_min_space_value_ = config.min_space_value_;
    predefine_medium_value_ = config.medium_value_;
    predefine_max_obstacle_value_ = config.max_obstacle_value_;
  }

  ZGINFO << "Initialize default value: "
         << static_cast<int>(predefine_default_value_) << ", min space value: "
         << static_cast<int>(predefine_min_space_value_)
         << ", medium value: " << static_cast<int>(predefine_medium_value_)
         << ", max obstacle value: "
         << static_cast<int>(predefine_max_obstacle_value_);

  is_config_loaded_ = true;
}

SlamValueGridMap2D::DataType SlamValueGridMap2D::GetPredefineDefaultValue() {
  if (!is_config_loaded_) {
    LoadConfig();
  }
  return predefine_default_value_;
}

SlamValueGridMap2D::DataType SlamValueGridMap2D::GetPredefineMinSpaceValue() {
  if (!is_config_loaded_) {
    LoadConfig();
  }
  return predefine_min_space_value_;
}

SlamValueGridMap2D::DataType SlamValueGridMap2D::GetPredefineMediumValue() {
  if (!is_config_loaded_) {
    LoadConfig();
  }
  return predefine_medium_value_;
}

SlamValueGridMap2D::DataType
SlamValueGridMap2D::GetPredefineMaxObstacleValue() {
  if (!is_config_loaded_) {
    LoadConfig();
  }
  return predefine_max_obstacle_value_;
}

std::vector<std::string> SlamValueGridMap2D::DebugString(
    const DynamicMapCellBound& bound, OverridePrintCellFunc func) const {
  PrintCellFunc _func = [&](const int& x, const int& y) {
    std::string msg;
    DataType value = default_value_;
    GetValue(x, y, value);
    if (func == nullptr) {
      if (value < predefine_default_value_) {
        msg += ZCOLOR_RED;
        msg += "@";
        msg += ZCOLOR_NONE;
      } else if (value == predefine_default_value_) {
        msg += '.';
      } else if (value >= predefine_min_space_value_ &&
                 value <= predefine_medium_value_) {
        msg += '+';
      } else if (value > predefine_medium_value_ &&
                 value <= predefine_max_obstacle_value_) {
        msg += '@';
      } else {
        msg += ZCOLOR_GREEN;
        msg += "@";
        msg += ZCOLOR_NONE;
      }
    } else {
      msg += func(value);
    }
    return msg;
  };
  return DebugStringWrapper(bound, _func);
}

std::vector<std::string> SlamValueGridMap2D::DebugString(
    OverridePrintCellFunc func) const {
  return DebugString(data_bound_, func);
}

FloatValueGridMap2D::FloatValueGridMap2D(const std::string& name,
                                         const uint16_t& cells_range_x,
                                         const uint16_t& cells_range_y,
                                         const float& resolution,
                                         const float& default_value)
    : FloatGridMap2D(name, cells_range_x, cells_range_y, resolution,
                     default_value),
      min_value_in_data_(std::numeric_limits<float>::max()),
      max_value_in_data_(std::numeric_limits<float>::min()) {}

FloatValueGridMap2D::FloatValueGridMap2D(const FloatValueGridMap2D& map)
    : FloatGridMap2D(map),
      min_value_in_data_(std::numeric_limits<float>::max()),
      max_value_in_data_(std::numeric_limits<float>::min()) {}

FloatValueGridMap2D::FloatValueGridMap2D(const FloatValueGridMap2D& map,
                                         const std::string& name,
                                         const bool& reserve)
    : FloatGridMap2D(map, name),
      min_value_in_data_(std::numeric_limits<float>::max()),
      max_value_in_data_(std::numeric_limits<float>::min()) {}

FloatValueGridMap2D& FloatValueGridMap2D::operator=(
    const FloatValueGridMap2D& map) {
  FloatGridMap2D::operator=(map);
  this->min_value_in_data_ = map.min_value_in_data_;
  this->max_value_in_data_ = map.max_value_in_data_;
  return *this;
}

bool FloatValueGridMap2D::SetValue(const int& x, const int& y,
                                   const DataType& value) {
  if (access_->InWriteByThisThread()) {
    min_value_in_data_ = Minimum(value, min_value_in_data_);
    max_value_in_data_ = Maximum(value, max_value_in_data_);
  }
  return FloatGridMap2D::SetValue(x, y, value);
}

float FloatValueGridMap2D::GetMinValueInData() const {
  ReadLocker lock(access_);
  return min_value_in_data_;
}

float FloatValueGridMap2D::GetMaxValueInData() const {
  ReadLocker lock(access_);
  return max_value_in_data_;
}

std::vector<std::string> FloatValueGridMap2D::DebugString(
    const DynamicMapCellBound& bound, OverridePrintCellFunc func) const {
  PrintCellFunc _func = [&](const int& x, const int& y) {
    std::string msg;
    DataType value = default_value_;
    GetValue(x, y, value);
    auto limit_value =
        (max_value_in_data_ - min_value_in_data_) * 3 / 4 + min_value_in_data_;
    auto int_value =
        static_cast<int8_t>(round((value - limit_value) * 10.0 /
                                  (max_value_in_data_ - limit_value))) -
        1;
    int_value = Maximum(int_value, 0);
    if (func == nullptr) {
      if (int_value == 0) {
        msg += '.';
      } else if (FloatEqual(value, max_value_in_data_)) {
        msg += ZCOLOR_GREEN;
        msg += std::to_string(int_value);
        msg += ZCOLOR_NONE;
      } else if (int_value == 9) {
        msg += ZCOLOR_RED;
        msg += std::to_string(int_value);
        msg += ZCOLOR_NONE;
      } else {
        msg += std::to_string(int_value);
      }
      // if (value < predefine_default_value_) {
      //   msg += ZCOLOR_RED;
      //   msg += "@";
      //   msg += ZCOLOR_NONE;
      // } else if (value == predefine_default_value_) {
      //   msg += '.';
      // } else if (value >= predefine_min_space_value_ &&
      //            value <= predefine_medium_value_) {
      //   msg += '+';
      // } else if (value > predefine_medium_value_ &&
      //            value <= predefine_max_obstacle_value_) {
      //   msg += '@';
      // } else {
      //   msg += ZCOLOR_GREEN;
      //   msg += "@";
      //   msg += ZCOLOR_NONE;
      // }
    } else {
      msg += func(value);
    }
    return msg;
  };
  return DebugStringWrapper(bound, _func);
}

std::vector<std::string> FloatValueGridMap2D::DebugString(
    OverridePrintCellFunc func) const {
  return DebugString(data_bound_, func);
}

std::vector<std::string> FloatValueGridMap2D::DebugString(
    const DynamicMapCellBound& bound, const uint8_t& precision) const {
  ReadLocker lock(access_, false);
  if (!access_->IsLockedByThisThread()) {
    lock.Lock();
  }

  std::vector<std::string> msgs;
  std::string msg = BasicInfoString();
  // ZINFO << msg;
  if (range_x_ == 0 || range_y_ == 0 || !marked_) {
    // ZINFO;
    msgs.emplace_back(msg);
    return msgs;
  }

  msg += "\n";

  auto add_table = [](const int8_t& precision, std::string& str,
                      const int8_t& last_char_len) -> void {
    auto p = 2 * precision + 3;
    uint8_t table_count = 0;
    while (p > 0) {
      table_count++;
      p -= 7;
    }
    table_count++;

    auto l = last_char_len;
    while (l > 0) {
      if (table_count > 0) {
        table_count--;
      }
      l -= 7;
    }

    for (auto i = 0; i < table_count; i++) {
      str += "\t";
    }
  };

  {
    std::vector<std::string> str_vector;

    uint8_t max_num_size = 1;
    {
      auto _x = bound.GetMin().X();
      uint8_t num_size = _x < 0 ? 2 : 1;
      while (_x / 10 != 0) {
        num_size++;
        _x /= 10;
      }
      max_num_size = Maximum(max_num_size, num_size);
    }
    {
      auto _x = bound.GetMax().X();
      uint8_t num_size = _x < 0 ? 2 : 1;
      while (_x / 10 != 0) {
        num_size++;
        _x /= 10;
      }
      max_num_size = Maximum(max_num_size, num_size);
    }
    // Initialize for line count.
    while (str_vector.size() < max_num_size) {
      str_vector.emplace_back("\t");
    }

    for (auto x = bound.GetMin().X(); x <= bound.GetMax().X(); x++) {
      auto _x = x;
      uint8_t str_index = 0;
      // ZINFO << "Process for " << _x;
      do {
        str_vector.at(str_index) += std::to_string(abs(_x) % 10);
        add_table(precision, str_vector.at(str_index), 1);
        str_index++;
        _x /= 10;
      } while (_x != 0);

      if (x < 0) {
        str_vector.at(str_index) += "-";
        add_table(precision, str_vector.at(str_index), 1);
        str_index++;
      }

      while (str_index < str_vector.size()) {
        str_vector.at(str_index) += " ";
        add_table(precision, str_vector.at(str_index), 1);
        str_index++;
      }
    }

    std::reverse(str_vector.begin(), str_vector.end());
    for (auto&& str : str_vector) {
      msg += str;
      msg += "\n";
    }
  }
  msg += "\n";

  DataType value = default_value_;
  for (int y = bound.GetMax().Y(); y >= bound.GetMin().Y(); y--) {
    if (msg.length() > 25000) {
      msgs.emplace_back(msg);
      msg.clear();
      msg += "\n";
    }
    if (y >= 0) {
      msg += " " + std::to_string(y) + "\t";
    } else {
      msg += std::to_string(y) + "\t";
    }

    for (auto x = bound.GetMin().X(); x <= bound.GetMax().X(); x++) {
      GetValue(x, y, value);
      auto _value = FloatToString(value, precision);
      msg += _value;
      add_table(precision, msg, _value.length());
    }
    msg += "\n";
  }

  msgs.emplace_back(msg);
  return msgs;
}

std::vector<std::string> FloatValueGridMap2D::DebugString(
    const uint8_t& precision) const {
  return DebugString(data_bound_, precision);
}

void FloatValueGridMap2D::PrintWithPrecision(const DynamicMapCellBound& bound,
                                             const std::string& file,
                                             const std::string& function,
                                             const uint32_t& line,
                                             const uint8_t& precision) const {
  auto index = file.find_last_of('/');
  std::string file_name = file;
  if (index != file.npos) {
    file_name = file.substr(index + 1);
  }

  for (auto&& str : DebugString(bound, precision)) {
    ZINFO << file_name << ":" << line << " " << function << ":" << str;
  }
}

void FloatValueGridMap2D::PrintWithPrecision(const std::string& file,
                                             const std::string& function,
                                             const uint32_t& line,
                                             const uint8_t& precision) const {
  PrintWithPrecision(data_bound_, file, function, line, precision);
}

ZimaProto::Map::PCharGridMap2D DynamicMap2DSerializer::ToProto(
    const CharGridMap2D::SPtr& map) {
  ZimaProto::Map::PCharGridMap2D proto;
  if (map == nullptr) {
    ZERROR << "Map pointer empty.";
    return proto;
  }
  ReadLocker lock(map->GetLock());
  auto data_bound = map->GetDataBound();
  proto.mutable_map_info()->set_name(map->Name());
  proto.mutable_map_info()->mutable_x_min_y_max_cell()->set_x(
      data_bound.GetMin().X());
  proto.mutable_map_info()->mutable_x_min_y_max_cell()->set_y(
      data_bound.GetMax().Y());
  proto.mutable_map_info()->set_resolution(map->GetResolution());
  proto.mutable_map_info()->set_x_range(data_bound.GetMax().X() -
                                        data_bound.GetMin().X() + 1);
  proto.mutable_map_info()->set_y_range(data_bound.GetMax().Y() -
                                        data_bound.GetMin().Y() + 1);
  proto.mutable_map_info()->set_x_reserve_range(map->GetRangeX());
  proto.mutable_map_info()->set_y_reserve_range(map->GetRangeY());

  proto.clear_data();
  std::string data;
  for (auto y = data_bound.GetMax().Y(); y >= data_bound.GetMin().Y(); y--) {
    for (auto x = data_bound.GetMin().X(); x <= data_bound.GetMax().X(); x++) {
      CharGridMap2D::DataType value;
      map->GetValue(x, y, value);
      data += value;
    }
  }
  proto.set_data(data);
  return proto;
}

bool DynamicMap2DSerializer::FromProto(
    CharGridMap2D::SPtr& map, const ZimaProto::Map::PCharGridMap2D& proto) {
  if (map == nullptr) {
    ZERROR << "Map pointer empty.";
    return false;
  }
  // Convert proto map to char grid map
  map->ChangeName(proto.map_info().name());
  map->ResetMap(proto.map_info().x_reserve_range(),
                proto.map_info().y_reserve_range(),
                proto.map_info().resolution(), map->GetDefaultValue());
  map->SetResolution(proto.map_info().resolution());
  int map_x_min = proto.map_info().x_min_y_max_cell().x();
  int map_y_max = proto.map_info().x_min_y_max_cell().y();
  auto char_it = proto.data().begin();
  auto map_x = map_x_min;
  auto map_y = map_y_max;
  WriteLocker lock(map->GetLock());
  // ZINFO << map->GetDataBound().DebugString();
  while (char_it != proto.data().end()) {
    // ZINFO << "Set for " << map_x << ", " << map_y << " as: " << *char_it;
    if (!map->SetValue(map_x, map_y, *char_it)) {
      // ZERROR << "Error during loading value into map";
      ZERROR << "Error during loading value into map, set for " << map_x << ", "
             << map_y << " as: " << *char_it;
      return false;
    }
    map_x++;
    if (map_x - map_x_min >= proto.map_info().x_range()) {
      map_x = map_x_min;
      map_y--;
    }
    char_it++;
  }
  lock.Unlock();
  // ZINFO << map->GetDataBound().DebugString() << ", "
  //       << map->GetCurrentDataSize();
  return true;
}

ZimaProto::Map::PSlamValueGridMap2D DynamicMap2DSerializer::ToProto(
    const SlamValueGridMap2D::SPtr& map) {
  ZimaProto::Map::PSlamValueGridMap2D proto;
  if (map == nullptr) {
    ZERROR << "Map pointer empty.";
    return proto;
  }
  ReadLocker lock(map->GetLock());
  auto data_bound = map->GetDataBound();
  proto.mutable_map_info()->set_name(map->Name());
  proto.mutable_map_info()->mutable_x_min_y_max_cell()->set_x(
      data_bound.GetMin().X());
  // ZINFO << "Set x "
  //       << proto.mutable_map_info()->mutable_x_min_y_max_cell()->x();
  proto.mutable_map_info()->mutable_x_min_y_max_cell()->set_y(
      data_bound.GetMax().Y());
  proto.mutable_map_info()->set_resolution(map->GetResolution());
  proto.mutable_map_info()->set_x_range(data_bound.GetMax().X() -
                                        data_bound.GetMin().X() + 1);
  proto.mutable_map_info()->set_y_range(data_bound.GetMax().Y() -
                                        data_bound.GetMin().Y() + 1);
  proto.mutable_map_info()->set_x_reserve_range(map->GetRangeX());
  proto.mutable_map_info()->set_y_reserve_range(map->GetRangeY());

  proto.clear_data();
  for (auto y = data_bound.GetMax().Y(); y >= data_bound.GetMin().Y(); y--) {
    for (auto x = data_bound.GetMin().X(); x <= data_bound.GetMax().X(); x++) {
      SlamValueGridMap2D::DataType value =
          SlamValueGridMap2D::GetPredefineDefaultValue();
      map->GetValue(x, y, value);
      proto.add_data(value);
    }
  }

  return proto;
}

bool DynamicMap2DSerializer::FromProto(
    SlamValueGridMap2D::SPtr& map,
    const ZimaProto::Map::PSlamValueGridMap2D& proto) {
  if (map == nullptr) {
    ZERROR << "Map pointer empty.";
    return false;
  }
  // Convert proto map to slam value grid map
  map->ChangeName(proto.map_info().name());
  map->ResetMap(proto.map_info().x_reserve_range(),
                proto.map_info().y_reserve_range(),
                proto.map_info().resolution(), map->GetDefaultValue());
  int map_x_min = proto.map_info().x_min_y_max_cell().x();
  int map_y_max = proto.map_info().x_min_y_max_cell().y();
  auto int8_it = proto.data().begin();
  auto map_x = map_x_min;
  auto map_y = map_y_max;
  WriteLocker lock(map->GetLock());
  while (int8_it != proto.data().end()) {
    if (!map->SetValue(map_x, map_y, *int8_it)) {
      // ZINFO << "Set for " << map_x << ", " << map_y << " as: " << *int8_it;
      ZERROR << "Error during loading value into map, set for " << map_x << ", "
             << map_y << " as: " << *int8_it;
      return false;
    }
    map_x++;
    if (map_x - map_x_min >= proto.map_info().x_range()) {
      map_x = map_x_min;
      map_y--;
    }
    int8_it++;
  }
  return true;
}

bool CharGridMap2DLoader::LoadMap(CharGridMap2D::SPtr& map) {
  if (map == nullptr) {
    ZERROR << "Map pointer empty.";
    return false;
  }
  ZimaProto::Map::PCharGridMap2D proto;
  if (GetProtoFromBinaryFile(&proto) || GetProtoFromASCIIFile(&proto)) {
    if (DynamicMap2DSerializer::FromProto(map, proto)) {
      ZINFO << "Load " << file_name_ << " success.";
      return true;
    }
  }
  ZERROR << "Load " << file_name_ << " failed.";
  return false;
}

bool CharGridMap2DWriter::WriteMap(const CharGridMap2D::SPtr& map,
                                   const bool& is_binary) {
  if (map == nullptr) {
    ZERROR << "Map pointer empty.";
    return false;
  }
  auto proto = DynamicMap2DSerializer::ToProto(map);
  if (is_binary ? SetProtoToBinaryFile(proto) : SetProtoToASCIIFile(proto)) {
    ZINFO << "Write " << file_name_ << " success.";
    return true;
  } else {
    ZERROR << "Write " << file_name_ << " failed.";
    return false;
  }
}

bool SlamValueGridMap2DLoader::LoadMap(SlamValueGridMap2D::SPtr& map) {
  if (map == nullptr) {
    ZERROR << "Map pointer empty.";
    return false;
  }
  ZimaProto::Map::PSlamValueGridMap2D proto;
  if (GetProtoFromBinaryFile(&proto) || GetProtoFromASCIIFile(&proto)) {
    if (DynamicMap2DSerializer::FromProto(map, proto)) {
      ZINFO << "Load " << file_name_ << " success.";
      return true;
    }
  }
  ZERROR << "Load " << file_name_ << " failed.";
  return false;
}

bool SlamValueGridMap2DWriter::WriteMap(const SlamValueGridMap2D::SPtr& map,
                                        const bool& is_binary) {
  if (map == nullptr) {
    ZERROR << "Map pointer empty.";
    return false;
  }
  auto proto = DynamicMap2DSerializer::ToProto(map);
  if (is_binary ? SetProtoToBinaryFile(proto) : SetProtoToASCIIFile(proto)) {
    ZINFO << "Write " << file_name_ << " success.";
    return true;
  } else {
    ZERROR << "Write " << file_name_ << " failed.";
    return false;
  }
}

FloatValueGridStaticMap2D::FloatValueGridStaticMap2D(const std::string& name,
                                                     const float& resolution,
                                                     const float& default_value)
    : FloatGridStaticMap2D(name, resolution, default_value),
      min_value_in_data_(std::numeric_limits<float>::max()),
      max_value_in_data_(std::numeric_limits<float>::min()) {}

FloatValueGridStaticMap2D::FloatValueGridStaticMap2D(
    const FloatValueGridStaticMap2D& map)
    : FloatGridStaticMap2D(map),
      min_value_in_data_(std::numeric_limits<float>::max()),
      max_value_in_data_(std::numeric_limits<float>::min()) {}

FloatValueGridStaticMap2D::FloatValueGridStaticMap2D(
    const FloatValueGridStaticMap2D& map, const std::string& name)
    : FloatGridStaticMap2D(map, name),
      min_value_in_data_(std::numeric_limits<float>::max()),
      max_value_in_data_(std::numeric_limits<float>::min()) {}

FloatValueGridStaticMap2D& FloatValueGridStaticMap2D::operator=(
    const FloatValueGridStaticMap2D& map) {
  FloatGridStaticMap2D::operator=(map);
  this->min_value_in_data_ = map.min_value_in_data_;
  this->max_value_in_data_ = map.max_value_in_data_;
  return *this;
}

bool FloatValueGridStaticMap2D::SetValue(const int& x, const int& y,
                                         const DataType& value) {
  if (access_->InWriteByThisThread()) {
    min_value_in_data_ = Minimum(value, min_value_in_data_);
    max_value_in_data_ = Maximum(value, max_value_in_data_);
  }
  return FloatGridStaticMap2D::SetValue(x, y, value);
}

float FloatValueGridStaticMap2D::GetMinValueInData() const {
  ReadLocker lock(access_);
  return min_value_in_data_;
}

float FloatValueGridStaticMap2D::GetMaxValueInData() const {
  ReadLocker lock(access_);
  return max_value_in_data_;
}

std::vector<std::string> FloatValueGridStaticMap2D::DebugString(
    const DynamicMapCellBound& bound, OverridePrintCellFunc func) const {
  PrintCellFunc _func = [&](const int& x, const int& y) {
    std::string msg;
    DataType value = default_value_;
    GetValue(x, y, value);
    auto limit_value =
        (max_value_in_data_ - min_value_in_data_) * 3 / 4 + min_value_in_data_;
    auto int_value =
        static_cast<int8_t>(round((value - limit_value) * 10.0 /
                                  (max_value_in_data_ - limit_value))) -
        1;
    int_value = Maximum(int_value, 0);
    if (func == nullptr) {
      if (int_value == 0) {
        msg += '.';
      } else if (FloatEqual(value, max_value_in_data_)) {
        msg += ZCOLOR_GREEN;
        msg += std::to_string(int_value);
        msg += ZCOLOR_NONE;
      } else if (int_value == 9) {
        msg += ZCOLOR_RED;
        msg += std::to_string(int_value);
        msg += ZCOLOR_NONE;
      } else {
        msg += std::to_string(int_value);
      }
      // if (value < predefine_default_value_) {
      //   msg += ZCOLOR_RED;
      //   msg += "@";
      //   msg += ZCOLOR_NONE;
      // } else if (value == predefine_default_value_) {
      //   msg += '.';
      // } else if (value >= predefine_min_space_value_ &&
      //            value <= predefine_medium_value_) {
      //   msg += '+';
      // } else if (value > predefine_medium_value_ &&
      //            value <= predefine_max_obstacle_value_) {
      //   msg += '@';
      // } else {
      //   msg += ZCOLOR_GREEN;
      //   msg += "@";
      //   msg += ZCOLOR_NONE;
      // }
    } else {
      msg += func(value);
    }
    return msg;
  };
  return DebugStringWrapper(bound, _func);
}

std::vector<std::string> FloatValueGridStaticMap2D::DebugString(
    OverridePrintCellFunc func) const {
  return DebugString(data_bound_, func);
}

std::vector<std::string> FloatValueGridStaticMap2D::DebugString(
    const DynamicMapCellBound& bound, const uint8_t& precision) const {
  ReadLocker lock(access_, false);
  if (!access_->IsLockedByThisThread()) {
    lock.Lock();
  }

  std::vector<std::string> msgs;
  std::string msg = BasicInfoString();
  // ZINFO << msg;
  if (!marked_) {
    // ZINFO;
    msgs.emplace_back(msg);
    return msgs;
  }

  msg += "\n";

  auto add_table = [](const int8_t& precision, std::string& str,
                      const int8_t& last_char_len) -> void {
    auto p = 2 * precision + 3;
    uint8_t table_count = 0;
    while (p > 0) {
      table_count++;
      p -= 7;
    }
    table_count++;

    auto l = last_char_len;
    while (l > 0) {
      if (table_count > 0) {
        table_count--;
      }
      l -= 7;
    }

    for (auto i = 0; i < table_count; i++) {
      str += "\t";
    }
  };

  {
    std::vector<std::string> str_vector;

    uint8_t max_num_size = 1;
    {
      auto _x = bound.GetMin().X();
      uint8_t num_size = _x < 0 ? 2 : 1;
      while (_x / 10 != 0) {
        num_size++;
        _x /= 10;
      }
      max_num_size = Maximum(max_num_size, num_size);
    }
    {
      auto _x = bound.GetMax().X();
      uint8_t num_size = _x < 0 ? 2 : 1;
      while (_x / 10 != 0) {
        num_size++;
        _x /= 10;
      }
      max_num_size = Maximum(max_num_size, num_size);
    }
    // Initialize for line count.
    while (str_vector.size() < max_num_size) {
      str_vector.emplace_back("\t");
    }

    for (auto x = bound.GetMin().X(); x <= bound.GetMax().X(); x++) {
      auto _x = x;
      uint8_t str_index = 0;
      // ZINFO << "Process for " << _x;
      do {
        str_vector.at(str_index) += std::to_string(abs(_x) % 10);
        add_table(precision, str_vector.at(str_index), 1);
        str_index++;
        _x /= 10;
      } while (_x != 0);

      if (x < 0) {
        str_vector.at(str_index) += "-";
        add_table(precision, str_vector.at(str_index), 1);
        str_index++;
      }

      while (str_index < str_vector.size()) {
        str_vector.at(str_index) += " ";
        add_table(precision, str_vector.at(str_index), 1);
        str_index++;
      }
    }

    std::reverse(str_vector.begin(), str_vector.end());
    for (auto&& str : str_vector) {
      msg += str;
      msg += "\n";
    }
  }
  msg += "\n";

  DataType value = default_value_;
  for (int y = bound.GetMax().Y(); y >= bound.GetMin().Y(); y--) {
    if (msg.length() > 25000) {
      msgs.emplace_back(msg);
      msg.clear();
      msg += "\n";
    }
    if (y >= 0) {
      msg += " " + std::to_string(y) + "\t";
    } else {
      msg += std::to_string(y) + "\t";
    }

    for (auto x = bound.GetMin().X(); x <= bound.GetMax().X(); x++) {
      GetValue(x, y, value);
      auto _value = FloatToString(value, precision);
      msg += _value;
      add_table(precision, msg, _value.length());
    }
    msg += "\n";
  }

  msgs.emplace_back(msg);
  return msgs;
}

std::vector<std::string> FloatValueGridStaticMap2D::DebugString(
    const uint8_t& precision) const {
  return DebugString(data_bound_, precision);
}

void FloatValueGridStaticMap2D::PrintWithPrecision(
    const DynamicMapCellBound& bound, const std::string& file,
    const std::string& function, const uint32_t& line,
    const uint8_t& precision) const {
  auto index = file.find_last_of('/');
  std::string file_name = file;
  if (index != file.npos) {
    file_name = file.substr(index + 1);
  }

  for (auto&& str : DebugString(bound, precision)) {
    ZINFO << file_name << ":" << line << " " << function << ":" << str;
  }
}

void FloatValueGridStaticMap2D::PrintWithPrecision(
    const std::string& file, const std::string& function, const uint32_t& line,
    const uint8_t& precision) const {
  PrintWithPrecision(data_bound_, file, function, line, precision);
}

}  // namespace zima
