/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_LIDAR_H
#define ZIMA_LIDAR_H

#include <atomic>
#include <memory>
#include <string>

#include "zima/common/point_cell.h"
#include "zima/common/point_cloud.h"
#include "zima/device/device.h"

namespace zima {

class Lidar : public DeviceBase {
 public:
  class Config : public DeviceBase::Config {
   public:
    Config();
    explicit Config(const JsonSPtr& json);
    ~Config() = default;

    bool ParseFromJson(const JsonSPtr& json);

    static const std::string kMaxRangeKey_;
    float max_range_;
    static const std::string kMinRangeKey_;
    float min_range_;
    static const std::string kFilterDistanceKey_;
    float filter_distance_;
  };

  Lidar() = delete;
  Lidar(const std::string name, const Config& config);
  ~Lidar() = default;

  using SPtr = std::shared_ptr<Lidar>;

  bool HasPointCloud() const;
  bool GetPointCloudTimeStamp(double& timestamp) const;
  bool GetPointCloudSeq(uint32_t& seq) const;
  bool CheckFresh(const double& limit) const;

  PointCloud::SPtr GetPointCloudInLidarFrame() const;
  PointCloud::SPtr GetPointCloudInChassisFrame() const;
  PointCloud::SPtr ConvertToChassisFrame(
      const PointCloud::SPtr& point_cloud_in_lidar_frame);
  void UpdatePointCloudInLidarFrame(const PointCloud::SPtr& new_point_cloud);

  static const std::string kPointCloudInLidarFrameName_;
  static const std::string kPointCloudInChassisFrameName_;

  static const std::string kNullName_;

 private:
  ReadWriteLock::SPtr lock_;
  Config config_;
  PointCloud::SPtr point_cloud_in_lidar_frame_;
  PointCloud::SPtr point_cloud_in_chassis_frame_;

  float max_range_;
  float min_range_;
};

}  // namespace zima

#endif  // ZIMA_LIDAR_H
