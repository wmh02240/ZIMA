/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_SIMPLE_SLAM_H
#define ZIMA_SIMPLE_SLAM_H

#include "zima/algorithm/point_cloud_matcher.h"
#include "zima/algorithm/pose_interpolator.h"
#include "zima/algorithm/slam/probability_map.h"
#include "zima/algorithm/slam/slam_base.h"
#include "zima/common/point_cloud.h"
#include "zima/common/thread.h"

namespace zima {

class SimpleSlam : public SlamBase {
 public:
  class Config : public SlamBase::Config {
   public:
    Config();
    Config(const JsonSPtr& json);
    ~Config() = default;

    using SPtr = std::shared_ptr<Config>;

    bool ParseFromJson(const JsonSPtr& json);

    bool config_valid_;

    static const std::string kConfigKey_;

    ProbabilityIndexGridMap2D::Config probability_map_config_;

    static const std::string kSlamLinearRangeKey_;
    float slam_linear_range_;
    static const std::string kSlamResolutionKey_;
    float slam_resolution_;

    static const std::string kMinRequiredMatchScoreKey_;
    float min_required_match_score_;
    static const std::string kMaxRequiredMatchScoreKey_;
    float max_required_match_score_;
    static const std::string kMinSearchLinearRangeKey_;
    float min_search_linear_range_;
    static const std::string kMaxSearchLinearRangeKey_;
    float max_search_linear_range_;
    static const std::string kMinSearchDegreeRangeKey_;
    float min_search_degree_range_;
    static const std::string kMaxSearchDegreeRangeKey_;
    float max_search_degree_range_;
    static const std::string kRawSearchLinearStepKey_;
    float raw_search_linear_step_;
    static const std::string kRawSearchDegreeStepKey_;
    float raw_search_degree_step_;
    static const std::string kPreciseSearchLinearRangeKey_;
    float precise_search_linear_range_;
    static const std::string kPreciseSearchDegreeRangeKey_;
    float precise_search_degree_range_;
    static const std::string kPreciseSearchLinearStepKey_;
    float precise_search_linear_step_;
    static const std::string kPreciseSearchDegreeStepKey_;
    float precise_search_degree_step_;

    static const std::string kRawMatcherConfigKey_;
    PointCloudMatcher::Config raw_matcher_config_;
    static const std::string kPreciseMatcherConfigKey_;
    PointCloudMatcher::Config precise_matcher_config_;

    static const std::string kMotionDetectorTimeInterval_;
    float motion_detector_time_interval_;
    static const std::string kMotionDetectorDegreeLimit_;
    float motion_detector_degree_limit_;
  };

  class OdomMotionDetector {
    public:
    OdomMotionDetector() = default;
    ~OdomMotionDetector() = default;

    enum MotionState {
      kUnknown,
      kNotMoving,
      kMoving,
      kNotTurning,
      kTurning,
    };

    static MotionState GetMovingState(
        const std::deque<MergedOdomData::SPtr>& odom_data_list,
        const double& start_time, const double& end_time,
        const float& distance_limit);
    static MotionState GetTurningState(
        const std::deque<MergedOdomData::SPtr>& odom_data_list,
        const double& start_time, const double& end_time,
        const float& degree_limit);
  };

  SimpleSlam() = delete;
  SimpleSlam(const Config& config, const MapPoint& lidar_tf_to_chassis);
  ~SimpleSlam() = default;

  bool StartSlam(const MapPoint& init_pose, const std::string& save_file_name,
                 const std::string& load_file_name) override;

  bool StopSlam() override;

  bool PauseSlam() override;

  bool ResumeSlam(const MapPoint& resume_pose) override;

  bool PushMergedOdomData(const MergedOdomData::SPtr& odom_data) override;
  bool PushPointCloud(const PointCloud::SPtr& point_cloud) override;

  bool UpdateScanIntoMap(const MapPoint& pose,
                         const PointCloud::SPtr& point_cloud_in_chassis_frame);

  void PrintMap() const;

  ProbabilityIndexGridMap2D::SPtr GetProbabilityMap() const;
  bool UpdateProbabilityMap(const ProbabilityIndexGridMap2D::SPtr& map);

  static PointCloud::SPtr PointCloudInLidarFrameMotionCompensate(
      const std::deque<MergedOdomData::SPtr>& odom_data_list,
      const MapPoint& lidar_tf_to_chassis,
      const PointCloud::SPtr& point_cloud_in_lidar_frame);

  PointCloud::SPtr PointCloudInLidarFrameMotionCompensate(
      const MapPoint& lidar_tf_to_chassis,
      const PointCloud::SPtr& point_cloud_in_lidar_frame);

 private:
  void OperateThread(const ZimaThreadWrapper::ThreadParam& param);

  TimedMapPoint::SPtr InterpolateWorldPose(
      const std::deque<MergedOdomData::SPtr>& odom_data_list,
      const TimedMapPoint& last_world_pose,
      const double& point_cloud_timestamp) const;

  TimedMapPoint::SPtr TryMatch(
      const PointCloud::SPtr& point_cloud_in_chassis_frame,
      const MapPoint& interpolate_pose);

  bool UpdateScanIntoMap(const ProbabilityIndexGridMap2D::SPtr& map,
                         const MapPoint& pose,
                         const PointCloud::SPtr& point_cloud_in_chassis_frame);

  ZimaThreadWrapper::ThreadParam thread_param_;
  std::atomic_bool stop_thread_;

  ProbabilityIndexGridMap2D::SPtr probability_grid_map_;
  FloatValueGridStaticMap2D::SPtr cached_probability_map_;
  PointCloudMatcher::SPtr raw_matcher_;
  PointCloudMatcher::SPtr precise_matcher_;

  PoseFilter pose_filter_;
  bool cached_is_odom_not_turning_;

  Config config_;
  MapPoint lidar_tf_to_chassis_;
  float dynamic_required_score_;
};

}  // namespace zima

#endif  // ZIMA_SIMPLE_SLAM_H
