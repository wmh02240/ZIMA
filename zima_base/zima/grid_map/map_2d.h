/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_DYNAMIC_MAP_2D_H
#define ZIMA_DYNAMIC_MAP_2D_H

#include <functional>
#include <map>
#include <mutex>
#include <vector>

#include "zima/common/boundingbox.h"
#include "zima/common/gflags.h"
#include "zima/common/lock.h"
#include "zima/common/point_cell.h"
#include "zima/hal/system/file.h"
#include "zima/logger/logger.h"
#include "zima/proto/map.pb.h"

namespace zima {

using DynamicMapPointBound = BoundingBox<float>;

class DynamicMapCellBound : public BoundingBox<int> {
 public:
  DynamicMapCellBound() = delete;
  DynamicMapCellBound(const int& x_min, const int& x_max, const int& y_min,
                      const int& y_max)
      : BoundingBox<int>(x_min, x_max, y_min, y_max) {}
  ~DynamicMapCellBound() = default;

  DynamicMapCellBound& operator=(const DynamicMapCellBound& box) {
    BoundingBox<int>::operator=(box);
    return *this;
  }

  bool OnEastBound(const MapCell& cell) const {
    return (cell.X() == max_.X() && cell.Y() >= min_.Y() &&
            cell.Y() <= max_.Y());
  }

  bool OnSouthBound(const MapCell& cell) const {
    return (cell.Y() == min_.Y() && cell.X() >= min_.X() &&
            cell.X() <= max_.X());
  }

  bool OnWestBound(const MapCell& cell) const {
    return (cell.X() == min_.X() && cell.Y() >= min_.Y() &&
            cell.Y() <= max_.Y());
  }

  bool OnNorthBound(const MapCell& cell) const {
    return (cell.Y() == max_.Y() && cell.X() >= min_.X() &&
            cell.X() <= max_.X());
  }

  bool OnBound(const MapCell& cell) const {
    return OnEastBound(cell) || OnSouthBound(cell) || OnWestBound(cell) ||
           OnNorthBound(cell);
  }
};

class StaticMap2DBase : public DebugBase {
 public:
  using PrintCellFunc = std::function<std::string(const int& x, const int& y)>;

  StaticMap2DBase() = delete;

  /**
   * @brief  Constructor for static map
   * @param  name The name for this cell
   * @param  resolution The resolution of the map in meters/cell
   */
  StaticMap2DBase(const std::string& name, const float& resolution);

  StaticMap2DBase(const StaticMap2DBase& map);

  StaticMap2DBase(const StaticMap2DBase& map, const std::string& name);

  ~StaticMap2DBase();

  StaticMap2DBase& operator=(const StaticMap2DBase& map);

  std::string Name() const { return name_; }
  void ChangeName(const std::string& name) {
    WriteLocker lock(access_);
    name_ = name;
  }

  /**
   * @brief  Convert from map coordinates to world coordinates
   * @param  x The x map coordinate
   * @param  y The y map coordinate
   * @param  wx Will be set to the associated world x coordinate
   * @param  wy Will be set to the associated world y coordinate
   */
  // TODO(Austin): Stress test for map/world conversion.
  void MapToWorld(const int& x, const int& y, float& wx, float& wy) const;
  void MapToWorld(const MapCell& map_cell, MapPoint& map_point) const;

  /**
   * @brief  Convert from world coordinates to map coordinates
   * @param  wx The x world coordinate
   * @param  wy The y world coordinate
   * @param  x Will be set to the associated map x coordinate
   * @param  y Will be set to the associated map y coordinate
   * @return True if the conversion was successful (legal bounds) false
   * otherwise
   */
  // TODO(Austin): Stress test for map/world conversion.
  virtual bool WorldToMap(const float& wx, const float& wy, int& x,
                          int& y) const;
  virtual bool WorldToMap(const MapPoint& map_point, MapCell& map_cell) const;

  DynamicMapCellBound GetDataBound() const { return data_bound_; }

  DynamicMapPointBound GetDataPointBound() const {
    MapPoint min, max;
    MapToWorld(MapCell(data_bound_.GetMin().X(), data_bound_.GetMin().Y()),
               min);
    MapToWorld(MapCell(data_bound_.GetMax().X(), data_bound_.GetMax().Y()),
               max);
    return DynamicMapPointBound(min.X(), max.X(), min.Y(), max.Y());
  }

  void SetResolution(const float& resolution) {
    WriteLocker lock(access_);
    resolution_ = resolution;
  }
  float GetResolution() const { return resolution_; }
  bool IsMarked() const {
    ReadLocker lock(access_);
    return marked_;
  }

  MapCells GenerateCellsBetweenTwoCells(const MapCell& a,
                                        const MapCell& b) const;

  // For user using
  ReadWriteLock::SPtr GetLock() const { return access_; }

  virtual std::string BasicInfoString() const;

  // Glog limits the length of a string to 30000, so we will sepapate string
  // into strings which length is less than 30000.
  virtual std::vector<std::string> DebugStringWrapper(
      const DynamicMapCellBound& bound, PrintCellFunc func = nullptr) const;
  virtual std::vector<std::string> DebugStringWrapper(
      PrintCellFunc func = nullptr) const;

  void RemoveCopySuffix();

  static const std::string kCopySuffix_;

 protected:
  ReadWriteLock::SPtr access_ = nullptr;
  std::string name_;
  float resolution_;

  DynamicMapCellBound data_bound_;
  bool marked_;
};

class DynamicMap2DBase : public StaticMap2DBase {
 public:
  using PrintCellFunc = std::function<std::string(const int& x, const int& y)>;

  DynamicMap2DBase() = delete;

  /**
   * @brief  Constructor for dynamic map
   * @param  name The name for this cell
   * @param  cells_range_x The x size of the map in cells
   * @param  cells_range_y The y size of the map in cells
   * @param  resolution The resolution of the map in meters/cell
   */
  DynamicMap2DBase(const std::string& name, const uint16_t& cells_range_x,
                   const uint16_t& cells_range_y, const float& resolution);

  DynamicMap2DBase(const DynamicMap2DBase& map);

  DynamicMap2DBase(const DynamicMap2DBase& map, const std::string& name);

  ~DynamicMap2DBase();

  DynamicMap2DBase& operator=(const DynamicMap2DBase& map);

  /**
   * @brief  Convert from world coordinates to map coordinates
   * @param  wx The x world coordinate
   * @param  wy The y world coordinate
   * @param  x Will be set to the associated map x coordinate
   * @param  y Will be set to the associated map y coordinate
   * @return True if the conversion was successful (legal bounds) false
   * otherwise
   */
  // TODO(Austin): Stress test for map/world conversion.
  bool WorldToMap(const float& wx, const float& wy, int& x,
                  int& y) const override;
  bool WorldToMap(const MapPoint& map_point, MapCell& map_cell) const override;

  DynamicMapCellBound GetAvailableBound() const { return available_bound_; }

  uint16_t GetRangeX() const { return range_x_; }
  uint16_t GetRangeY() const { return range_y_; }

  uint32_t GetCurrentDataSize() const {
    ReadLocker lock(access_);
    return optimize_data_size_;
  }

  std::string BasicInfoString() const override;

  std::vector<std::string> DebugStringWrapper(
      const DynamicMapCellBound& bound,
      PrintCellFunc func = nullptr) const override;
  std::vector<std::string> DebugStringWrapper(
      PrintCellFunc func = nullptr) const override;

 protected:
  /**
   * @brief  Directly Get the index converted by x/y.
   *         ATTENTION!! It will not check for index
   *         validation.
   * @param x The x coordinate of the cell
   * @param y The y coordinate of the cell
   * @return Index for data_.
   */
  uint32_t GetIndex(const int& x, const int& y) const;
  uint32_t GetIndex(const int& x, const int& y, const int& x_range,
                    const int& y_range) const;

  uint8_t memory_optimize_factor_;
  uint16_t range_x_;
  uint16_t optimize_range_x_;
  uint16_t range_y_;
  uint16_t optimize_range_y_;

  uint32_t data_size_;
  uint32_t optimize_data_size_;
  DynamicMapCellBound available_bound_;
};

template <typename DataTypeT>
class DynamicMap2D : public DynamicMap2DBase {
 public:
  using DataType = DataTypeT;
  using OverridePrintCellFunc =
      std::function<std::string(const DataType& value)>;

  using SPtr = std::shared_ptr<DynamicMap2D<DataTypeT>>;

  DynamicMap2D() = delete;

  /**
   * @brief  Constructor for dynamic map
   * @param  name The name for this cell
   * @param  cells_range_x The x size of the map in cells
   * @param  cells_range_y The y size of the map in cells
   * @param  resolution The resolution of the map in meters/cell
   * @param  default_value Default Value
   */
  DynamicMap2D(const std::string& name, const uint16_t& cells_range_x,
               const uint16_t& cells_range_y, const float& resolution,
               const DataTypeT& default_value)
      : DynamicMap2DBase(name, cells_range_x, cells_range_y, resolution),
        default_value_(default_value) {
    // ZERROR << name_;
    default_value_ = default_value;
    ResetMap();
  }

  /**
   * @brief  Copy constructor for a dynamic map, creates a copy efficiently
   * @param map The dynamic map to copy
   */
  DynamicMap2D(const DynamicMap2D& map)
      : DynamicMap2DBase(map), default_value_(map.default_value_) {
    // ZERROR << name_;
    data_ = map.data_;
  }

  DynamicMap2D(const DynamicMap2D& map, const std::string& name,
               const bool& reserve = false)
      : DynamicMap2DBase(map, name),
        default_value_(map.default_value_) {
    // ZERROR << name_;
    if (reserve) {
      ResetMap();
    } else {
      data_ = map.data_;
    }
  }

  /**
   * @brief  Overloaded assignment operator
   * @param  map The dynamic map to copy
   * @return A reference to the map after the copy has finished
   */
  DynamicMap2D& operator=(const DynamicMap2D& map) {
    if (&map == this) {
      return *this;
    }

    DynamicMap2DBase::operator=(map);
    this->default_value_ = map.default_value_;
    this->data_ = map.data_;

    return *this;
  }

  ~DynamicMap2D() = default;

  /**
   * @brief  Reset map data.
   * @param  range_x The x size of map.
   * @param  range_y The y size of map.
   * @param  resolution The resolution of map.
   * @param  default_value The default value of map.
   */
  virtual void ResetMap(const uint16_t& range_x, const uint16_t& range_y,
                        const float& resolution,
                        const DataType& default_value) {
    WriteLocker lock(access_);

    range_x_ = range_x;
    range_y_ = range_y;
    resolution_ = resolution;
    data_size_ = range_x * range_y;
    if (range_x_ < memory_optimize_factor_ || memory_optimize_factor_ == 1) {
      optimize_range_x_ = range_x_;
    } else {
      // optimize_range_x_ = range_x_ / memory_optimize_factor_;
      optimize_range_x_ = 5;
    }
    if (range_y_ < memory_optimize_factor_ || memory_optimize_factor_ == 1) {
      optimize_range_y_ = range_y_;
    } else {
      // optimize_range_y_ = range_y_ / memory_optimize_factor_;
      optimize_range_y_ = 5;
    }

    optimize_data_size_ = Minimum(static_cast<uint32_t>(optimize_range_x_) *
                                      static_cast<uint32_t>(optimize_range_y_),
                                  data_size_);
    if (data_.size() >= optimize_data_size_) {
      data_.resize(optimize_data_size_, default_value);
      for (auto&& data : data_) {
        data = default_value;
      }
    } else {
      data_.clear();
      data_.resize(optimize_data_size_, default_value);
    }
    // ZINFO << name_ << " resize as " << std::to_string(optimize_data_size_)
    //       << ", max: " << std::to_string(data_size_);
    available_bound_.Reset(INT32_MIN, INT32_MAX, INT32_MIN, INT32_MAX);
    data_bound_.Reset(INT32_MAX, INT32_MIN, INT32_MAX, INT32_MIN);
    default_value_ = default_value;
    marked_ = false;
  }

  /**
   * @brief  Reset map data.
   */
  virtual void ResetMap() {
    ResetMap(range_x_, range_y_, resolution_, default_value_);
  }

  /**
   * @brief  Safely get the value of a cell in map
   * @param x The x coordinate of the cell
   * @param y The y coordinate of the cell
   * @param value The value of the cell
   * @return True for x/y in data bound.
   */
  bool GetValue(const int& x, const int& y, DataType& value) const {
    // Please lock layer outside this function.
    if (!access_->IsLockedByThisThread()) {
      ZWARN << "Map: " << name_ << " is not in read/write mode when accessing ("
            << x << ", " << y << ")";
      return false;
    }

    auto index = GetIndex(x, y);
    if (data_bound_.Contain(x, y) && index < data_.size()) {
      value = data_.at(index);
      return true;
    }

    value = default_value_;
    return false;
  }

  /**
   * @brief  Set the value of a cell in the map
   * @param x The x coordinate of the cell
   * @param y The y coordinate of the cell
   * @param value The value to set the cell to
   * @return True for x/y in available bound.
   */
  virtual bool SetValue(const int& x, const int& y, const DataType& value) {
    // Please lock layer outside this function.
    if (!access_->InWriteByThisThread()) {
      ZWARN << "Map: " << name_ << " is not in write mode when accessing ("
            << x << ", " << y << ")";
      return false;
    }

    if (!available_bound_.Contain(x, y)) {
      return false;
    }

    if (ExpendDataSizeIfNecessary(x, y)) {
      // ZINFO << x << ", " << y;
    }

    data_bound_.Expend(x, y);
    available_bound_.Reset(data_bound_.GetMax().X() - range_x_,
                           data_bound_.GetMin().X() + range_x_,
                           data_bound_.GetMax().Y() - range_y_,
                           data_bound_.GetMin().Y() + range_y_);

    // if (IsDebugFuncEnabled()) {
    //   CHECK(GetIndex(x, y) < data_size_);
    // }

    auto index = GetIndex(x, y);
    data_.at(index) = value;

    marked_ = true;

    return true;
  }

  DataType GetDefaultValue() const { return default_value_; }

  /**
   * @brief  Copy range data from target map.
   * @param  map The dynamic map to copy.
   * @param  select_bound The selected cell bound.
   */
  bool CopyRange(const DynamicMap2D& target_map,
                 const DynamicMapCellBound& select_bound) {
    ReadLocker read_lock(target_map.GetLock());
    WriteLocker write_lock(access_);
    for (auto x = select_bound.GetMin().X(); x <= select_bound.GetMax().X();
         x++) {
      for (auto y = select_bound.GetMin().Y(); y <= select_bound.GetMax().Y();
           y++) {
        DataType value;
        target_map.GetValue(x, y, value);
        SetValue(x, y, value);
      }
    }

    return true;
  }

  MapCell GradientMove(const int& init_x, const int& init_y,
                       const uint8_t& iteration_num, const bool& accending,
                       const MapCells& move_step_cells) const {
    ReadLocker lock(access_, false);
    if (!access_->IsLockedByThisThread()) {
      lock.Lock();
    }

    uint8_t iteration_count = 1;
    MapCell curr_cell(init_x, init_y);
    // ZINFO << "Curr: " << curr_cell.DebugString();
    DataType curr_value;
    if (!GetValue(curr_cell.X(), curr_cell.Y(), curr_value)) {
      ZWARN << "Init cell not in data range.";
      return curr_cell;
    }

    MapCell next_cell = curr_cell;
    DataType next_value = curr_value;
    DataType expend_cell_value = default_value_;
    for (; iteration_count <= iteration_num; iteration_count++) {
      for (auto&& expend_step_cell : move_step_cells) {
        auto expend_cell = curr_cell + expend_step_cell;
        if (!GetValue(expend_cell.X(), expend_cell.Y(), expend_cell_value)) {
          continue;
        }
        if (accending && expend_cell_value > next_value) {
          next_cell = expend_cell;
          next_value = expend_cell_value;
        }
        if (!accending && expend_cell_value < next_value) {
          next_cell = expend_cell;
          next_value = expend_cell_value;
        }
      }
      if (next_cell == curr_cell) {
        // Stepping on peak.
        // ZINFO << "Reach peak: " << curr_cell.DebugString();
        return curr_cell;
      }
      curr_cell = next_cell;
      // ZINFO << "Move to: " << curr_cell.DebugString();
    }
    // ZINFO << "Return: " << curr_cell.DebugString();
    return curr_cell;
  }

  MapCell GradientMoveFor4Direction(const int& init_x, const int& init_y,
                                    const uint8_t& iteration_num,
                                    const bool& accending) const {
    static const MapCells relative_expend_step_cells{
        MapCell(1, 0), MapCell(-1, 0), MapCell(0, 1), MapCell(0, -1)};
    return GradientMove(init_x, init_y, iteration_num, accending,
                        relative_expend_step_cells);
  }

  MapCell GradientMoveFor8Direction(const int& init_x, const int& init_y,
                                    const uint8_t& iteration_num,
                                    const bool& accending) const {
    static const MapCells relative_expend_step_cells{
        MapCell(1, 0), MapCell(-1, 0), MapCell(0, 1),  MapCell(0, -1),
        MapCell(1, 1), MapCell(-1, 1), MapCell(1, -1), MapCell(-1, -1)};
    return GradientMove(init_x, init_y, iteration_num, accending,
                        relative_expend_step_cells);
  }

  // Glog limits the length of a string to 30000, so we will sepapate string
  // into strings which length is less than 30000.
  virtual std::vector<std::string> DebugString(
      const DynamicMapCellBound& bound,
      OverridePrintCellFunc func = nullptr) const {
    PrintCellFunc _func = [&](const int& x, const int& y) {
      std::string msg;
      DataType value = default_value_;
      GetValue(x, y, value);
      if (func == nullptr) {
        if (value == 0) {
          msg += default_value_;
        } else {
          msg += value;
        }
      } else {
        msg += func(value);
      }
      return msg;
    };

    return DebugStringWrapper(bound, _func);
  }

  virtual std::vector<std::string> DebugString(
      OverridePrintCellFunc func = nullptr) const {
    return DebugString(data_bound_, func);
  }

  void Print(const DynamicMapCellBound& bound, const std::string& file,
             const std::string& function, const uint32_t& line) const {
    auto index = file.find_last_of('/');
    std::string file_name = file;
    if (index != file.npos) {
      file_name = file.substr(index + 1);
    }

    for (auto&& str : DebugString(bound)) {
      ZINFO << file_name << ":" << line << " " << function << ":" << str;
    }
  }

  void Print(const std::string& file, const std::string& function,
             const uint32_t& line) const {
    Print(data_bound_, file, function, line);
  }

 protected:
  DataType default_value_;
  std::vector<DataType> data_;

 private:
  bool ExpendDataSizeIfNecessary(const int16_t& x, const int16_t y) {
    if (!marked_) {
      return false;
    }
    bool expend_y_data_size = false;
    auto target_range_y = optimize_range_y_;
    while ((y <= data_bound_.GetMax().Y() - target_range_y ||
            y >= data_bound_.GetMin().Y() + target_range_y) &&
           target_range_y < range_y_) {
      target_range_y += range_y_ / memory_optimize_factor_ + 1;
      target_range_y = Minimum(range_y_, target_range_y);
      // ZINFO << "For y: " << std::to_string(y) << " target y range change to "
      //       << std::to_string(target_range_y)
      //       << " bound: " << data_bound_.DebugString();
      expend_y_data_size = true;
    }

    bool expend_x_data_size = false;
    auto target_range_x = optimize_range_x_;
    while ((x <= data_bound_.GetMax().X() - target_range_x ||
            x >= data_bound_.GetMin().X() + target_range_x) &&
           target_range_x < range_x_) {
      target_range_x += range_x_ / memory_optimize_factor_ + 1;
      target_range_x = Minimum(range_x_, target_range_x);
      // ZINFO << "For x: " << std::to_string(x) << " target x range change to "
      //       << std::to_string(target_range_x)
      //       << " bound: " << data_bound_.DebugString();
      expend_x_data_size = true;
    }

    if (expend_x_data_size || expend_y_data_size) {
      auto optimize_data_size = target_range_x * target_range_y;
      std::vector<DataType> new_data;
      new_data.clear();
      new_data.resize(optimize_data_size, default_value_);
      for (auto _x = data_bound_.GetMin().X(); _x <= data_bound_.GetMax().X();
           _x++) {
        for (auto _y = data_bound_.GetMin().Y(); _y <= data_bound_.GetMax().Y();
             _y++) {
          auto old_index = GetIndex(_x, _y);
          auto new_index = GetIndex(_x, _y, target_range_x, target_range_y);
          new_data[new_index] = data_[old_index];
        }
      }
      data_.swap(new_data);
      optimize_range_x_ = target_range_x;
      optimize_range_y_ = target_range_y;
      optimize_data_size_ = optimize_data_size;
    }

    return false;
  }
};

class CharGridMap2D : public DynamicMap2D<char> {
 public:
  CharGridMap2D(const std::string& name, const uint16_t& cells_range_x,
                const uint16_t& cells_range_y, const float& resolution,
                const DataType& default_value)
      : DynamicMap2D<char>(name, cells_range_x, cells_range_y, resolution,
                           default_value) {}
  CharGridMap2D(const std::string& name, const uint16_t& cells_range_x,
                const uint16_t& cells_range_y, const float& resolution)
      : DynamicMap2D<char>(name, cells_range_x, cells_range_y, resolution,
                           kCharGridMap2DDefaultValue_) {}

  CharGridMap2D(const CharGridMap2D& map) : DynamicMap2D<char>(map) {}

  CharGridMap2D(const CharGridMap2D& map, const std::string& name,
                const bool& reserve = false)
      : DynamicMap2D<char>(map, name, reserve) {}

  using SPtr = std::shared_ptr<CharGridMap2D>;

  CharGridMap2D& operator=(const CharGridMap2D& map) {
    DynamicMap2D<char>::operator=(map);
    return *this;
  }

  static void CellPathToPointPath(CharGridMap2D::SPtr map,
                                  const MapCellPath& cell_path,
                                  MapPointPath& point_path);
  static void PointPathToCellPath(CharGridMap2D::SPtr map,
                                  const MapPointPath& point_path,
                                  MapCellPath& cell_path);

  static const DataType kCharGridMap2DDefaultValue_;
};

using Int8GridMap2D = DynamicMap2D<int8_t>;
class SlamValueGridMap2D : public Int8GridMap2D {
 public:
  class Config {
   public:
    Config();
    explicit Config(const JsonSPtr& json);
    ~Config() = default;

    bool ParseFromJson(const JsonSPtr& json);

    bool config_valid_;

    static const std::string kConfigKey_;

    static const std::string kDefaultValueKey_;
    int default_value_;
    static const std::string kMinSpaceValueKey_;
    int min_space_value_;
    static const std::string kMediumValueKey_;
    int medium_value_;
    static const std::string kMaxObstacleValueKey_;
    int max_obstacle_value_;
  };

  SlamValueGridMap2D(const std::string& name, const uint16_t& cells_range_x,
                     const uint16_t& cells_range_y, const float& resolution);

  SlamValueGridMap2D(const SlamValueGridMap2D& map) : Int8GridMap2D(map) {}

  SlamValueGridMap2D(const SlamValueGridMap2D& map, const std::string& name,
                     const bool& reserve = false)
      : Int8GridMap2D(map, name, reserve) {}

  using SPtr = std::shared_ptr<SlamValueGridMap2D>;

  SlamValueGridMap2D& operator=(const SlamValueGridMap2D& map) {
    Int8GridMap2D::operator=(map);
    return *this;
  }

  static void LoadConfig();

  static DataType GetPredefineDefaultValue();
  static DataType GetPredefineMinSpaceValue();
  static DataType GetPredefineMediumValue();
  static DataType GetPredefineMaxObstacleValue();

  std::vector<std::string> DebugString(
      const DynamicMapCellBound& bound,
      OverridePrintCellFunc func = nullptr) const override;
  std::vector<std::string> DebugString(
      OverridePrintCellFunc func = nullptr) const override;

 private:
  static bool is_config_loaded_;
  static DataType predefine_default_value_;
  static DataType predefine_min_space_value_;
  static DataType predefine_medium_value_;
  static DataType predefine_max_obstacle_value_;
};

using FloatGridMap2D = DynamicMap2D<float>;
class FloatValueGridMap2D : public FloatGridMap2D {
 public:
  FloatValueGridMap2D(const std::string& name, const uint16_t& cells_range_x,
                      const uint16_t& cells_range_y, const float& resolution,
                      const float& default_value = 0);

  FloatValueGridMap2D(const FloatValueGridMap2D& map);

  FloatValueGridMap2D(const FloatValueGridMap2D& map, const std::string& name,
                      const bool& reserve = false);

  using SPtr = std::shared_ptr<FloatValueGridMap2D>;

  FloatValueGridMap2D& operator=(const FloatValueGridMap2D& map);

  bool SetValue(const int& x, const int& y, const DataType& value) override;

  float GetMinValueInData() const;
  float GetMaxValueInData() const;

  std::vector<std::string> DebugString(
      const DynamicMapCellBound& bound,
      OverridePrintCellFunc func = nullptr) const override;
  std::vector<std::string> DebugString(
      OverridePrintCellFunc func = nullptr) const override;
  std::vector<std::string> DebugString(const DynamicMapCellBound& bound,
                                       const uint8_t& precision) const;
  std::vector<std::string> DebugString(const uint8_t& precision) const;

  void PrintWithPrecision(const DynamicMapCellBound& bound,
                          const std::string& file, const std::string& function,
                          const uint32_t& line, const uint8_t& precision) const;
  void PrintWithPrecision(const std::string& file, const std::string& function,
                          const uint32_t& line, const uint8_t& precision) const;

 protected:
  float min_value_in_data_;
  float max_value_in_data_;
};

class DynamicMap2DSerializer {
 public:
  DynamicMap2DSerializer() = delete;

  static ZimaProto::Map::PCharGridMap2D ToProto(const CharGridMap2D::SPtr& map);
  static bool FromProto(CharGridMap2D::SPtr& map,
                        const ZimaProto::Map::PCharGridMap2D& proto);

  static ZimaProto::Map::PSlamValueGridMap2D ToProto(
      const SlamValueGridMap2D::SPtr& map);
  static bool FromProto(SlamValueGridMap2D::SPtr& map,
                        const ZimaProto::Map::PSlamValueGridMap2D& proto);
};

class CharGridMap2DLoader : public LocalProtoFileReader {
 public:
  CharGridMap2DLoader() = default;
  explicit CharGridMap2DLoader(const std::string& file_dir,
                               const std::string& file_name)
      : LocalProtoFileReader(file_dir, file_name, false){};
  ~CharGridMap2DLoader() = default;

  bool LoadMap(CharGridMap2D::SPtr& map);
};

class CharGridMap2DWriter : public LocalProtoFileWriter {
 public:
  CharGridMap2DWriter() = default;
  explicit CharGridMap2DWriter(const std::string& file_dir,
                               const std::string& file_name)
      : LocalProtoFileWriter(file_dir, file_name, false){};
  ~CharGridMap2DWriter() = default;

  bool WriteMap(const CharGridMap2D::SPtr& map, const bool& is_binary);
};

class SlamValueGridMap2DLoader : public LocalProtoFileReader {
 public:
  SlamValueGridMap2DLoader() = default;
  explicit SlamValueGridMap2DLoader(const std::string& file_dir,
                                    const std::string& file_name)
      : LocalProtoFileReader(file_dir, file_name, false){};
  ~SlamValueGridMap2DLoader() = default;

  bool LoadMap(SlamValueGridMap2D::SPtr& map);
};

class SlamValueGridMap2DWriter : public LocalProtoFileWriter {
 public:
  SlamValueGridMap2DWriter() = default;
  explicit SlamValueGridMap2DWriter(const std::string& file_dir,
                                    const std::string& file_name)
      : LocalProtoFileWriter(file_dir, file_name, false){};
  ~SlamValueGridMap2DWriter() = default;

  bool WriteMap(const SlamValueGridMap2D::SPtr& map, const bool& is_binary);
};

template <typename DataTypeT>
class StaticMap2D : public StaticMap2DBase {
 public:
  using DataType = DataTypeT;

  using OverridePrintCellFunc =
      std::function<std::string(const DataType& value)>;

  using SPtr = std::shared_ptr<StaticMap2D<DataTypeT>>;

  StaticMap2D() = delete;

  /**
   * @brief  Constructor for dynamic map
   * @param  name The name for this cell
   * @param  resolution The resolution of the map in meters/cell
   * @param  default_value Default Value
   */
  StaticMap2D(const std::string& name, const float& resolution,
              const DataTypeT& default_value)
      : StaticMap2DBase(name, resolution), default_value_(default_value) {
    // ZERROR << name_;
    default_value_ = default_value;
    ResetMap();
  }

  /**
   * @brief  Copy constructor for a dynamic map, creates a copy efficiently
   * @param map The dynamic map to copy
   */
  StaticMap2D(const StaticMap2D& map)
      : StaticMap2DBase(map), default_value_(map.default_value_) {
    // ZERROR << name_;
    data_ = map.data_;
  }

  StaticMap2D(const StaticMap2D& map, const std::string& name)
      : StaticMap2DBase(map, name), default_value_(map.default_value_) {
    // ZERROR << name_;
    data_ = map.data_;
  }

  /**
   * @brief  Overloaded assignment operator
   * @param  map The dynamic map to copy
   * @return A reference to the map after the copy has finished
   */
  StaticMap2D& operator=(const StaticMap2D& map) {
    if (&map == this) {
      return *this;
    }

    StaticMap2DBase::operator=(map);
    this->default_value_ = map.default_value_;
    this->data_ = map.data_;

    return *this;
  }

  ~StaticMap2D() = default;

  /**
   * @brief  Reset map data.
   * @param  resolution The resolution of map.
   * @param  default_value The default value of map.
   */
  virtual void ResetMap(const float& resolution,
                        const DataType& default_value) {
    WriteLocker lock(access_);

    resolution_ = resolution;
    data_.clear();
    // data_.resize(optimize_data_size_, default_value);
    // ZINFO << name_ << " resize as " << std::to_string(optimize_data_size_)
    //       << ", max: " << std::to_string(data_size_);
    data_bound_.Reset(INT32_MAX, INT32_MIN, INT32_MAX, INT32_MIN);
    default_value_ = default_value;
    marked_ = false;
  }

  /**
   * @brief  Reset map data.
   */
  virtual void ResetMap() {
    ResetMap(resolution_, default_value_);
  }

  /**
   * @brief  Safely get the value of a cell in map
   * @param x The x coordinate of the cell
   * @param y The y coordinate of the cell
   * @param value The value of the cell
   * @return True for x/y in data bound.
   */
  bool GetValue(const int& x, const int& y, DataType& value) const {
    // Please lock layer outside this function.
    if (!access_->IsLockedByThisThread()) {
      ZWARN << "Map: " << name_ << " is not in read/write mode when accessing ("
            << x << ", " << y << ")";
      return false;
    }

    MapCell cell(x, y);
    if (data_.count(cell) == 0) {
      value = default_value_;
      return false;
    }
    value = data_.at(cell);
    return true;
  }

  /**
   * @brief  Set the value of a cell in the map
   * @param x The x coordinate of the cell
   * @param y The y coordinate of the cell
   * @param value The value to set the cell to
   * @return True for x/y in available bound.
   */
  virtual bool SetValue(const int& x, const int& y, const DataType& value) {
    // Please lock layer outside this function.
    if (!access_->InWriteByThisThread()) {
      ZWARN << "Map: " << name_ << " is not in write mode when accessing ("
            << x << ", " << y << ")";
      return false;
    }

    data_bound_.Expend(x, y);
    data_[MapCell(x, y)] = value;
    marked_ = true;
    return true;
  }

  DataType GetDefaultValue() const { return default_value_; }

  size_t GetDataSize() const {
    ReadLocker lock(access_);
    return data_.size();
  }

  /**
   * @brief  Copy range data from target map.
   * @param  map The dynamic map to copy.
   * @param  select_bound The selected cell bound.
   */
  bool CopyRange(const StaticMap2D& target_map,
                 const DynamicMapCellBound& select_bound) {
    ReadLocker read_lock(target_map.GetLock());
    WriteLocker write_lock(access_);
    for (auto x = select_bound.GetMin().X(); x <= select_bound.GetMax().X();
         x++) {
      for (auto y = select_bound.GetMin().Y(); y <= select_bound.GetMax().Y();
           y++) {
        DataType value;
        target_map.GetValue(x, y, value);
        SetValue(x, y, value);
      }
    }

    return true;
  }

  // Glog limits the length of a string to 30000, so we will sepapate string
  // into strings which length is less than 30000.
  virtual std::vector<std::string> DebugString(
      const DynamicMapCellBound& bound,
      OverridePrintCellFunc func = nullptr) const {
    PrintCellFunc _func = [&](const int& x, const int& y) {
      std::string msg;
      DataType value = default_value_;
      GetValue(x, y, value);
      if (func == nullptr) {
        if (value == 0) {
          msg += default_value_;
        } else {
          msg += value;
        }
      } else {
        msg += func(value);
      }
      return msg;
    };

    return DebugStringWrapper(bound, _func);
  }

  virtual std::vector<std::string> DebugString(
      OverridePrintCellFunc func = nullptr) const {
    return DebugString(data_bound_, func);
  }

  void Print(const DynamicMapCellBound& bound, const std::string& file,
             const std::string& function, const uint32_t& line) const {
    auto index = file.find_last_of('/');
    std::string file_name = file;
    if (index != file.npos) {
      file_name = file.substr(index + 1);
    }

    for (auto&& str : DebugString(bound)) {
      ZINFO << file_name << ":" << line << " " << function << ":" << str;
    }
  }

  void Print(const std::string& file, const std::string& function,
             const uint32_t& line) const {
    Print(data_bound_, file, function, line);
  }

 protected:
  DataType default_value_;
  using DataElement = std::pair<MapCell, DataType>;
  std::map<MapCell, DataType> data_;
};

using FloatGridStaticMap2D = StaticMap2D<float>;
class FloatValueGridStaticMap2D : public FloatGridStaticMap2D {
 public:
  FloatValueGridStaticMap2D(const std::string& name, const float& resolution,
                            const float& default_value = 0);

  FloatValueGridStaticMap2D(const FloatValueGridStaticMap2D& map);

  FloatValueGridStaticMap2D(const FloatValueGridStaticMap2D& map,
                            const std::string& name);

  using SPtr = std::shared_ptr<FloatValueGridStaticMap2D>;

  FloatValueGridStaticMap2D& operator=(const FloatValueGridStaticMap2D& map);

  bool SetValue(const int& x, const int& y, const DataType& value) override;

  float GetMinValueInData() const;
  float GetMaxValueInData() const;

  std::vector<std::string> DebugString(
      const DynamicMapCellBound& bound,
      OverridePrintCellFunc func = nullptr) const override;
  std::vector<std::string> DebugString(
      OverridePrintCellFunc func = nullptr) const override;
  std::vector<std::string> DebugString(const DynamicMapCellBound& bound,
                                       const uint8_t& precision) const;
  std::vector<std::string> DebugString(const uint8_t& precision) const;

  void PrintWithPrecision(const DynamicMapCellBound& bound,
                          const std::string& file, const std::string& function,
                          const uint32_t& line, const uint8_t& precision) const;
  void PrintWithPrecision(const std::string& file, const std::string& function,
                          const uint32_t& line, const uint8_t& precision) const;

 protected:
  float min_value_in_data_;
  float max_value_in_data_;
};

}  // namespace zima

#endif  // ZIMA_DYNAMIC_MAP_2D_H
