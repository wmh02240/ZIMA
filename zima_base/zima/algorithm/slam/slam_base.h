/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_SLAM_BASE_H
#define ZIMA_SLAM_BASE_H

#include <atomic>
#include <list>
#include <memory>

#include "zima/common/lock.h"
#include "zima/common/macro.h"
#include "zima/common/point_cloud.h"
#include "zima/common/ratio_sampler.h"
#include "zima/device/device_manager.h"
#include "zima/grid_map/map_2d.h"

namespace zima {

class SlamBase {
 public:
  class Config {
   public:
    Config();
    explicit Config(const JsonSPtr& json);
    ~Config() = default;

    bool ParseFromJson(const JsonSPtr& json);

    bool config_valid_;

    static const std::string kConfigKey_;

    static const std::string kOdomDataListMaxSizeKey_;
    uint32_t odom_data_list_max_size_;
    static const std::string kOdomDataListMaxTimeIntervalKey_;
    float odom_data_list_max_time_interval_;
    static const std::string kPointCloudListMaxSizeKey_;
    uint32_t point_cloud_list_max_size_;
    static const std::string kPointCloudListMaxTimeIntervalKey_;
    float point_cloud_list_max_time_interval_;

    static const std::string kOdomSampleRatioKey_;
    float odom_sample_ratio_;
    static const std::string kPointCloudSampleRatioKey_;
    float point_cloud_sample_ratio_;

    static const std::string kPoseFilterMaxLinearDistanceKey_;
    float pose_filter_max_linear_distance_;
    static const std::string kPoseFilterMaxAngleDegreeKey_;
    float pose_filter_max_angle_degree_;
    static const std::string kPoseFilterMaxTimeIntervalKey_;
    float pose_filter_max_time_interval_;
  };

  // Learn from cartographer.
  class PoseFilter {
   public:
    PoseFilter() = delete;
    PoseFilter(const float& max_linear_distance,
                 const float& max_angle_degree,
                 const float& max_time_interval_s);
    ~PoseFilter() = default;

    // If the accumulated motion (linear, rotation, or time) is above the
    // threshold, returns false. Otherwise the relative motion is accumulated
    // and true is returned.
    bool IsSimilar(const TimedMapPoint& pose);

    void UpdateMaxLinearDistance(const float& max_linear_distance);
    void UpdateMaxAngleDegree(const float& max_angle_degree);
    void UpdateMaxTimeInterval(const float& max_time_interval_s);

    void Reset();

    std::string DebugString() const;

   private:
    uint64_t num_total_ = 0;
    uint64_t num_different_ = 0;

    float max_linear_distance_;
    float max_angle_degree_;
    float max_time_interval_s_;

    TimedMapPoint::SPtr last_pose_;
  };

  using SPtr = std::shared_ptr<SlamBase>;

  bool IsRunning() const { return is_running_.load(); }
  bool IsReady() const { return is_ready_.load(); }

  virtual bool StartSlam(const MapPoint& init_pose,
                         const std::string& save_file_name,
                         const std::string& load_file_name);
  virtual bool StopSlam();

  virtual bool PauseSlam();

  virtual bool ResumeSlam(const MapPoint& resume_pose);

  virtual bool PushMergedOdomData(const MergedOdomData::SPtr& odom_data);
  virtual bool PushPointCloud(const PointCloud::SPtr& point_cloud);

  virtual SlamValueGridMap2D::SPtr GetSlamMap() const;
  virtual void SetSlamMap(const SlamValueGridMap2D::SPtr& slam_map);

  double GetStartOrResumeTimeStamp() const {
    ReadLocker lock(access_);
    return start_or_resume_timestamp_;
  }

  double GetStopOrPauseTimeStamp() const {
    ReadLocker lock(access_);
    return stop_or_pause_timestamp_;
  }

  TimedMapPoint GetLastWorldPose() const;

  PointCloud::SPtr GetLastMatchPointCloudInWorldFrame() const;

 protected:
  SlamBase() = delete;
  explicit SlamBase(const Config& config);
  ~SlamBase() = default;

  bool PrintRatioSamplersInfo(const float& interval);

  ReadWriteLock::SPtr access_;
  std::atomic_bool is_running_;
  std::atomic_bool is_ready_;
  std::string save_file_name_;

  const uint32_t odom_data_list_max_size_;
  const float odom_data_list_max_time_interval_;
  const uint32_t point_cloud_list_max_size_;
  const float point_cloud_list_max_time_interval_;

  std::deque<MergedOdomData::SPtr> odom_data_list_;
  std::deque<PointCloud::SPtr> point_cloud_data_list_;

  RatioSampler odom_ratio_sampler_;
  RatioSampler point_cloud_ratio_sampler_;

  uint32_t odom_frequency_count_;
  uint32_t point_cloud_frequency_count_;
  double print_sensor_frequency_time_;

  SlamValueGridMap2D::SPtr slam_map_;
  MapPoint correction_;

  double start_or_resume_timestamp_;
  double stop_or_pause_timestamp_;

  TimedMapPoint last_world_pose_;
  PointCloud::SPtr last_match_point_cloud_in_world_frame_;
};

}  // namespace zima

#endif  // ZIMA_SLAM_BASE_H
