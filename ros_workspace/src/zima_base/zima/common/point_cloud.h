/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_POINT_CLOUD_H
#define ZIMA_POINT_CLOUD_H

#include <atomic>
#include <deque>
#include <functional>

#include "zima/common/macro.h"
#include "zima/common/lock.h"
#include "zima/common/point_cell.h"

namespace zima {

class PointCloud {
 public:
  PointCloud() = delete;
  PointCloud(const PointCloud& ref);
  explicit PointCloud(const std::string& name);
  ~PointCloud() = default;

  using SPtr = std::shared_ptr<PointCloud>;

  class Point {
   public:
    Point() = delete;
    Point(const double& timestamp, const float& polar_distance,
          const float& polar_degree, const float& intensity);
    Point(const double& timestamp, const MapPoint& cartesian_coordinate,
          const float& intensity);
    ~Point() = default;

    Point& operator=(const Point& point);

    double TimeStamp() const { return timestamp_; }
    float X() const { return cartesian_coordinate_.X(); }
    float Y() const { return cartesian_coordinate_.Y(); }
    float Distance() const { return polar_dis_; }
    float Degree() const { return polar_degree_; }
    float Intensity() const { return intensity_; }

    MapPoint ToMapPoint() const { return cartesian_coordinate_; }
    std::string DebugString() const;

   private:
    double timestamp_;
    float polar_dis_;
    float polar_degree_;
    MapPoint cartesian_coordinate_;
    float intensity_;
  };

  using Points = std::deque<Point>;

  std::string Name() const { return name_; }

  bool Empty() const;
  size_t Size() const;
  ReadWriteLock::SPtr GetLock() const { return lock_; }

  Points& GetPointsRef();
  const Points& GetPointsConstRef() const;

  bool FilterFromPointCloud(const PointCloud& source_point_cloud,
                            const float& filter_distance);

  void SetEqualDegreeInterval(const bool& val) {
    equal_degree_interval_.store(val);
  };

  bool IsEqualDegreeInterval() const { return equal_degree_interval_.load(); }

  void SetAscendDegree(const bool& val) { is_ascend_degree_.store(val); };

  bool IsAscendDegree() const { return is_ascend_degree_.load(); }

  DECLARE_DATA_GET_SET(double, TimeStamp);
  DECLARE_DATA_GET_SET(uint32_t, Seq);
  DECLARE_DATA_GET_SET(float, ScanTime);
  DECLARE_DATA_GET_SET(float, FilterDistance);

  using ProcessFunc = std::function<void(const Point& point)>;

  /*
   * @brief Get every point inside degree range in order.
   *        Range support both ascending and decending, so use it strictly.
   *        e.g. Start at 170°, end at -170°, it will go as 170° decending to
   *        -170° instead of 170° to 180°/-180° to -170°. Correct usage: start
   *        at 170°, end at 190°.
   * @param start_degree -180.0 to 360.0.
   * @param end_degree -180.0 to 360.0.
   * @return points inside range.
   */
  Points GetRangedPoints(const float& start_degree, const float& end_degree);

  /*
   * @brief Execute process function for every point inside degree range.
   *        Range support both ascending and decending, so use it strictly.
   *        e.g. Start at 170°, end at -170°, it will go as 170° decending to
   *        -170° instead of 170° to 180°/-180° to -170°. Correct usage: start
   *        at 170°, end at 190°.
   * @param start_degree -180.0 to 360.0.
   * @param end_degree -180.0 to 360.0.
   * @param process Call back function.
   */
  void ProcessRangedPoints(const float& start_degree, const float& end_degree,
                           ProcessFunc process);

  PointCloud::SPtr TransformBA(const std::string& name,
                               const MapPoint& transform_B_to_A);

  void Print(const float& map_resolution);

 private:
  std::string name_;
  double timestamp_;
  uint32_t seq_;
  double scan_time_;
  ReadWriteLock::SPtr lock_;
  Points points_;
  atomic_bool equal_degree_interval_;
  atomic_bool is_ascend_degree_;

  float filter_distance_;
};

using PointClouds = std::deque<PointCloud::SPtr>;

}  // namespace zima

#endif  // ZIMA_POINT_CLOUD_H
