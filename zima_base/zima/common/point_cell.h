/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_POINT_CELL_H
#define ZIMA_POINT_CELL_H

#include <cstdint>
#include <cstdio>
#include <deque>
#include <iomanip>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "zima/common/macro.h"
#include "zima/common/json.h"
#include "zima/common/util.h"
#include "zima/common/vector2.h"

namespace zima {

using CellBase = Vector2<int>;
using PointBase = Vector2<float>;

class MapCell;
using MapCells = std::deque<MapCell>;
using MapCellPath = std::deque<MapCell>;
using VMapCells = std::vector<MapCell>;
using MapCellPathIter = std::deque<MapCell>::const_iterator;
using MapCellsSet = std::set<MapCell>;
using MapTimeCell = std::tuple<double, MapCell>;

class MapCell : public CellBase {
 public:
  MapCell() : CellBase(){};

  MapCell(const JsonSPtr& json) : CellBase() { ParseFromJson(json); };

  MapCell(const int& x, const int& y) : CellBase(x, y){};

  ~MapCell() = default;

  using SPtr = std::shared_ptr<MapCell>;

  bool ParseFromJson(const JsonSPtr& json);

  inline const MapCell operator+(const MapCell& r_other) const {
    return MapCell(X() + r_other.X(), Y() + r_other.Y());
  }

  inline const MapCell operator-(const MapCell& r_other) const {
    return MapCell(X() - r_other.X(), Y() - r_other.Y());
  }

  inline const bool operator==(const MapCell& r_other) const {
    return X() == r_other.X() && Y() == r_other.Y();
  }

  MapCell& SetX(const int& x) {
    CellBase::X(x);
    return *this;
  }

  MapCell& AdjustX(const int& x) {
    CellBase::X(X() + x);
    return *this;
  }

  MapCell& SetY(const int& y) {
    CellBase::Y(y);
    return *this;
  }

  MapCell& AdjustY(const int& y) {
    CellBase::Y(Y() + y);
    return *this;
  }

  std::string DebugString() const {
    std::stringstream ss;
    ss << *this;
    return ss.str();
  }

  static std::string DebugString(MapCells& cells) {
    std::stringstream ss;
    for (auto&& cell : cells) {
      ss << cell;
    }
    return ss.str();
  }

  static std::string DebugString(MapCellsSet& cells) {
    std::stringstream ss;
    for (auto&& cell : cells) {
      ss << cell;
    }
    return ss.str();
  }
};

class MapPoint;
using MapPoints = std::deque<MapPoint>;
using MapPointPath = std::deque<MapPoint>;
using VMapPoints = std::vector<MapPoint>;
using MapPointPathIter = std::deque<MapPoint>::const_iterator;
using MapPointsSet = std::set<MapPoint>;

class MapPoint : public PointBase {
 public:
  MapPoint() : PointBase(), degree_(0){};

  MapPoint(const JsonSPtr& json) : PointBase(), degree_(0) {
    ParseFromJson(json);
  };

  MapPoint(const float& x, const float& y) : PointBase(x, y), degree_(0){};
  MapPoint(const float& x, const float& y, const float& degree)
      : PointBase(x, y), degree_(NormalizeDegree(degree)){};

  ~MapPoint() = default;

  using SPtr = std::shared_ptr<MapPoint>;

  bool ParseFromJson(const JsonSPtr& json);

  inline const MapPoint operator+(const MapPoint& r_other) const {
    return MapPoint(X() + r_other.X(), Y() + r_other.Y(),
                    NormalizeDegree(Degree() + r_other.Degree()));
  }

  inline const MapPoint operator-(const MapPoint& r_other) const {
    return MapPoint(X() - r_other.X(), Y() - r_other.Y(),
                    NormalizeDegree(Degree() - r_other.Degree()));
  }

  inline const bool operator==(const MapPoint& r_other) const {
    return FloatEqual(X(), r_other.X()) && FloatEqual(Y(), r_other.Y());
  }

  float Degree() const { return degree_; }
  float Radian() const { return DegreesToRadians(degree_); }

  MapPoint& SetX(const float& x) {
    PointBase::X(x);
    return *this;
  }

  MapPoint& AdjustX(const float& x) {
    PointBase::X(X() + x);
    return *this;
  }

  MapPoint& SetY(const float& y) {
    PointBase::Y(y);
    return *this;
  }

  MapPoint& AdjustY(const float& y) {
    PointBase::Y(Y() + y);
    return *this;
  }

  MapPoint& SetDegree(const float& degree) {
    degree_ = degree;
    return *this;
  }

  MapPoint& AdjustDegree(const float& degree) {
    degree_ = NormalizeDegree(degree_ + degree);
    return *this;
  }

  float AngleDiff(const MapPoint& r_point) const {
    return NormalizeDegree(r_point.Degree() - Degree());
  }

  std::string DebugString() const {
    std::stringstream ss;
    ss << *this;
    return ss.str();
  }

  static std::string DebugString(MapPoints& points) {
    std::stringstream ss;
    for (auto&& point : points) {
      ss << point;
    }
    return ss.str();
  }

  static std::string DebugString(MapPointsSet& points) {
    std::stringstream ss;
    for (auto&& point : points) {
      ss << point;
    }
    return ss.str();
  }

  friend inline std::ostream& operator<<(std::ostream& r_stream,
                                         const MapPoint& r_point) {
    r_stream << "(" << FloatToString(r_point.X(), 3) << ", "
             << FloatToString(r_point.Y(), 3) << ", "
             << FloatToString(r_point.Degree(), 1) << "),";
    return r_stream;
  }

  static MapPoint GetVector(const MapPoint& source, const MapPoint& target);

 private:
  float degree_;
};

class TimedMapPoint : public MapPoint {
 public:
  TimedMapPoint();
  TimedMapPoint(const double& timestamp);
  TimedMapPoint(const double& timestamp, const MapPoint& point);
  ~TimedMapPoint() = default;

  using SPtr = std::shared_ptr<TimedMapPoint>;

  DECLARE_DATA_GET_SET(double, Timestamp)

  std::string DebugString() const {
    std::string str =
        "[" + DoubleToString(timestamp_, 4) + "]" + MapPoint::DebugString();
    return str;
  }

 private:
  double timestamp_;
};

MapPoints GenerateInterpolationPoints(const MapPoint& a, const MapPoint& b,
                                      const float& interval);

}  // namespace zima

#endif  // ZIMA_POINT_CELL_H
