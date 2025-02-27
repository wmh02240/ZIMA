/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/algorithm/slam/simple_slam.h"

#include "zima/common/config.h"
#include "zima/common/time.h"
#include "zima/common/transform.h"
#include "zima/grid_map/map_util.h"
#include "zima/logger/logger.h"

namespace zima {

SimpleSlam::Config::Config() : Config(nullptr){};

SimpleSlam::Config::Config(const JsonSPtr& json)
    : SlamBase::Config(json), config_valid_(false) {
  // Load default setting.
  slam_linear_range_ = 20;
  slam_resolution_ = 0.07;
  min_required_match_score_ = 0.4;
  max_required_match_score_ = 0.51;
  // min_search_linear_range_ = slam_resolution_ * 2 + 0.01;
  min_search_linear_range_ = 0.15;
  // max_search_linear_range_ = min_search_linear_range_ * 2;
  max_search_linear_range_ = 0.3;
  min_search_degree_range_ = 6;
  max_search_degree_range_ = 25;
  // raw_search_linear_step_ = slam_resolution_;
  raw_search_linear_step_ = 0.07;
  // raw_search_degree_step_ = atan(slam_resolution_ / 2 / lidar_max_distance);
  raw_search_degree_step_ = 0.5;
  // precise_search_linear_range_ = slam_resolution_;
  precise_search_linear_range_ = 0.07;
  precise_search_degree_range_ = 0.5;
  precise_search_linear_step_ = 0.01;
  precise_search_degree_step_ = 0.2;
  motion_detector_time_interval_ = 0.1;
  motion_detector_degree_limit_ = 4;
  raw_matcher_config_.translation_delta_cost_weight_ = 3;
  raw_matcher_config_.rotation_degree_delta_cost_weight_ = 0.1;
  precise_matcher_config_.translation_delta_cost_weight_ = 5;
  precise_matcher_config_.rotation_degree_delta_cost_weight_ = 10;

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

bool SimpleSlam::Config::ParseFromJson(const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }

  if (!SlamBase::Config::ParseFromJson(json)) {
    ZERROR << "Config " << SlamBase::Config::kConfigKey_ << " invalid.";
    return false;
  }

  {
    JsonSPtr _json(new Json());
    if (!JsonHelper::GetObject(
            *json, ProbabilityIndexGridMap2D::Config::kConfigKey_, *_json)) {
      ZGERROR << "Config " << ProbabilityIndexGridMap2D::Config::kConfigKey_
              << " not found.";
      // return false;
    }
    probability_map_config_ = ProbabilityIndexGridMap2D::Config(_json);
    if (!probability_map_config_.config_valid_) {
      ZGERROR << "Config " << ProbabilityIndexGridMap2D::Config::kConfigKey_
              << " invalid.";
      // return false;
    }
  }

  {
    JsonSPtr _json(new Json());
    if (!JsonHelper::GetObject(*json, kRawMatcherConfigKey_, *_json)) {
      ZGERROR << "Config " << kRawMatcherConfigKey_ << " not found.";
      // return false;
    } else {
      raw_matcher_config_ = PointCloudMatcher::Config(_json);
      if (!raw_matcher_config_.config_valid_) {
        ZGERROR << "Config " << kRawMatcherConfigKey_ << " invalid.";
        // return false;
        raw_matcher_config_.translation_delta_cost_weight_ = 3;
        raw_matcher_config_.rotation_degree_delta_cost_weight_ = 0.1;
      }
    }
  }

  {
    JsonSPtr _json(new Json());
    if (!JsonHelper::GetObject(*json, kPreciseMatcherConfigKey_, *_json)) {
      ZGERROR << "Config " << kPreciseMatcherConfigKey_ << " not found.";
      // return false;
    } else {
      precise_matcher_config_ = PointCloudMatcher::Config(_json);
      if (!precise_matcher_config_.config_valid_) {
        ZGERROR << "Config " << kPreciseMatcherConfigKey_ << " invalid.";
        // return false;
        precise_matcher_config_.translation_delta_cost_weight_ = 5;
        precise_matcher_config_.rotation_degree_delta_cost_weight_ = 10;
      }
    }
  }

  if (!JsonHelper::GetFloat(*json, kSlamLinearRangeKey_, slam_linear_range_)) {
    ZGERROR << "Config " << kSlamLinearRangeKey_ << " not found.";
    // return false;
  }
  if (slam_linear_range_ < 0) {
    ZERROR << "Config " << kSlamLinearRangeKey_
           << " invalid: " << FloatToString(slam_linear_range_, 3) << ".";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kSlamResolutionKey_, slam_resolution_)) {
    ZGERROR << "Config " << kSlamResolutionKey_ << " not found.";
    // return false;
  }
  if (slam_resolution_ < 0) {
    ZERROR << "Config " << kSlamResolutionKey_
           << " invalid: " << FloatToString(slam_resolution_, 3) << ".";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kMinRequiredMatchScoreKey_,
                            min_required_match_score_)) {
    ZGERROR << "Config " << kMinRequiredMatchScoreKey_ << " not found.";
    // return false;
  }
  if (min_required_match_score_ < 0) {
    ZERROR << "Config " << kMinRequiredMatchScoreKey_
           << " invalid: " << FloatToString(min_required_match_score_, 3)
           << ".";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kMaxRequiredMatchScoreKey_,
                            max_required_match_score_)) {
    ZGERROR << "Config " << kMaxRequiredMatchScoreKey_ << " not found.";
    // return false;
  }
  if (max_required_match_score_ < min_required_match_score_) {
    ZERROR << "Config " << kMaxRequiredMatchScoreKey_
           << " invalid: " << FloatToString(max_required_match_score_, 3)
           << ".";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kMinSearchLinearRangeKey_,
                            min_search_linear_range_)) {
    ZGERROR << "Config " << kMinSearchLinearRangeKey_ << " not found.";
    // return false;
  }
  if (min_search_linear_range_ < 0) {
    ZERROR << "Config " << kMinSearchLinearRangeKey_
           << " invalid: " << FloatToString(min_search_linear_range_, 3) << ".";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kMaxSearchLinearRangeKey_,
                            max_search_linear_range_)) {
    ZGERROR << "Config " << kMaxSearchLinearRangeKey_ << " not found.";
    // return false;
  }
  if (max_search_linear_range_ <= min_search_linear_range_) {
    ZERROR << "Config " << kMaxSearchLinearRangeKey_
           << " invalid: " << FloatToString(max_search_linear_range_, 3) << ".";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kMinSearchDegreeRangeKey_,
                            min_search_degree_range_)) {
    ZGERROR << "Config " << kMinSearchDegreeRangeKey_ << " not found.";
    // return false;
  }
  if (min_search_degree_range_ < 0) {
    ZERROR << "Config " << kMinSearchDegreeRangeKey_
           << " invalid: " << FloatToString(min_search_degree_range_, 3) << ".";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kMaxSearchDegreeRangeKey_,
                            max_search_degree_range_)) {
    ZGERROR << "Config " << kMaxSearchDegreeRangeKey_ << " not found.";
    // return false;
  }
  if (max_search_degree_range_ <= min_search_degree_range_) {
    ZERROR << "Config " << kMaxSearchDegreeRangeKey_
           << " invalid: " << FloatToString(max_search_degree_range_, 3) << ".";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kRawSearchLinearStepKey_,
                            raw_search_linear_step_)) {
    ZGERROR << "Config " << kRawSearchLinearStepKey_ << " not found.";
    // return false;
  }
  if (raw_search_linear_step_ < 0) {
    ZERROR << "Config " << kRawSearchLinearStepKey_
           << " invalid: " << FloatToString(raw_search_linear_step_, 3) << ".";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kRawSearchDegreeStepKey_,
                            raw_search_degree_step_)) {
    ZGERROR << "Config " << kRawSearchDegreeStepKey_ << " not found.";
    // return false;
  }
  if (raw_search_degree_step_ < 0) {
    ZERROR << "Config " << kRawSearchDegreeStepKey_
           << " invalid: " << FloatToString(raw_search_degree_step_, 3) << ".";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kPreciseSearchLinearRangeKey_,
                            precise_search_linear_range_)) {
    ZGERROR << "Config " << kPreciseSearchLinearRangeKey_ << " not found.";
    // return false;
  }
  if (precise_search_linear_range_ < 0) {
    ZERROR << "Config " << kPreciseSearchLinearRangeKey_
           << " invalid: " << FloatToString(precise_search_linear_range_, 3)
           << ".";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kPreciseSearchDegreeRangeKey_,
                            precise_search_degree_range_)) {
    ZGERROR << "Config " << kPreciseSearchDegreeRangeKey_ << " not found.";
    // return false;
  }
  if (precise_search_degree_range_ < 0) {
    ZERROR << "Config " << kPreciseSearchDegreeRangeKey_
           << " invalid: " << FloatToString(precise_search_degree_range_, 3)
           << ".";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kPreciseSearchLinearStepKey_,
                            precise_search_linear_step_)) {
    ZGERROR << "Config " << kPreciseSearchLinearStepKey_ << " not found.";
    // return false;
  }
  if (precise_search_linear_step_ < 0) {
    ZERROR << "Config " << kPreciseSearchLinearStepKey_
           << " invalid: " << FloatToString(precise_search_linear_step_, 3)
           << ".";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kPreciseSearchDegreeStepKey_,
                            precise_search_degree_step_)) {
    ZGERROR << "Config " << kPreciseSearchDegreeStepKey_ << " not found.";
    // return false;
  }
  if (precise_search_degree_step_ < 0) {
    ZERROR << "Config " << kPreciseSearchDegreeStepKey_
           << " invalid: " << FloatToString(precise_search_degree_step_, 3)
           << ".";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kMotionDetectorTimeInterval_,
                            motion_detector_time_interval_)) {
    ZGERROR << "Config " << kMotionDetectorTimeInterval_ << " not found.";
    // return false;
  }
  if (motion_detector_time_interval_ < 0) {
    ZERROR << "Config " << kMotionDetectorTimeInterval_
           << " invalid: " << FloatToString(motion_detector_time_interval_, 3)
           << ".";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kMotionDetectorDegreeLimit_,
                            motion_detector_degree_limit_)) {
    ZGERROR << "Config " << kMotionDetectorDegreeLimit_ << " not found.";
    // return false;
  }
  if (motion_detector_degree_limit_ < 0) {
    ZERROR << "Config " << kMotionDetectorDegreeLimit_
           << " invalid: " << FloatToString(motion_detector_degree_limit_, 3)
           << ".";
    return false;
  }

  return true;
}

SimpleSlam::OdomMotionDetector::MotionState
SimpleSlam::OdomMotionDetector::GetMovingState(
    const std::deque<MergedOdomData::SPtr>& odom_data_list,
    const double& start_time, const double& end_time,
    const float& distance_limit) {
  auto start_odom_pose =
      PoseInterpolatorBase::InterpolatePose(odom_data_list, start_time);
  auto end_odom_pose =
      PoseInterpolatorBase::InterpolatePose(odom_data_list, end_time);
  if (start_odom_pose == nullptr || end_odom_pose == nullptr) {
    ZWARN << "start(" << std::to_string(start_odom_pose == nullptr) << ") end("
          << std::to_string(end_odom_pose == nullptr) << ")";
    return kUnknown;
  }

  if (start_odom_pose->GetPose().Distance(end_odom_pose->GetPose()) >
      distance_limit) {
    return kMoving;
  }
  return kNotMoving;
}

SimpleSlam::OdomMotionDetector::MotionState
SimpleSlam::OdomMotionDetector::GetTurningState(
    const std::deque<MergedOdomData::SPtr>& odom_data_list,
    const double& start_time, const double& end_time, const float& degree_limit) {
  auto start_odom_pose =
      PoseInterpolatorBase::InterpolatePose(odom_data_list, start_time);
  auto end_odom_pose =
      PoseInterpolatorBase::InterpolatePose(odom_data_list, end_time);
  if (start_odom_pose == nullptr || end_odom_pose == nullptr) {
    ZGWARN << "start(" << std::to_string(start_odom_pose == nullptr) << ") end("
           << std::to_string(end_odom_pose == nullptr) << ")";
    return kUnknown;
  }

  if (fabs(start_odom_pose->GetPose().AngleDiff(end_odom_pose->GetPose())) >
      degree_limit) {
    return kTurning;
  }
  return kNotTurning;
}

SimpleSlam::SimpleSlam(const Config& config,
                       const MapPoint& lidar_tf_to_chassis)
    : SlamBase(config),
      stop_thread_(false),
      probability_grid_map_(nullptr),
      cached_probability_map_(nullptr),
      pose_filter_(config.pose_filter_max_linear_distance_,
                   config.pose_filter_max_angle_degree_,
                   config.pose_filter_max_time_interval_),
      cached_is_odom_not_turning_(true),
      config_(config),
      lidar_tf_to_chassis_(lidar_tf_to_chassis),
      dynamic_required_score_(config.probability_map_config_
                                  .min_probability_for_obstacle_in_cell_) {
  raw_matcher_.reset(new PointCloudMatcher(config.raw_matcher_config_));
  precise_matcher_.reset(new PointCloudMatcher(config.precise_matcher_config_));
  // precise_matcher_->EnableDebugFunc();

  thread_param_ = ZimaThreadWrapper::ThreadParam(
      "Simple slam thread", ZimaThreadManager::kSlamThreadIndex_, 100, 1, 2);

  is_ready_.store(true);
};

bool SimpleSlam::StartSlam(const MapPoint& init_pose,
                           const std::string& save_file_name,
                           const std::string& load_file_name) {
  if (IsRunning()) {
    ZINFO << "Slam is already running.";
    return true;
  }

  ZINFO << "Start slam.";

  {
    WriteLocker lock(access_);
    if (probability_grid_map_ == nullptr) {
      probability_grid_map_.reset(new ProbabilityIndexGridMap2D(
          "simple slam probability map",
          config_.slam_linear_range_ / config_.slam_resolution_,
          config_.slam_linear_range_ / config_.slam_resolution_,
          config_.slam_resolution_, config_.probability_map_config_));
      ZGINFO << "Create new probability grid map.";
      cached_probability_map_.reset(new FloatValueGridStaticMap2D(
          "cached " + probability_grid_map_->Name(),
          config_.precise_search_linear_step_,
          probability_grid_map_->GetMediumProbabilityForObstacleInCell()));
    }
    dynamic_required_score_ =
        probability_grid_map_->GetMinProbabilityForObstacleInCell();
    point_cloud_data_list_.clear();
    odom_data_list_.clear();
  }
  stop_thread_.store(false);
  last_world_pose_ = TimedMapPoint(Time::Now(), init_pose);

  auto thread_manager = ZimaThreadManager::Instance();
  if (!thread_manager->IsThreadRunning(thread_param_.thread_name_)) {
    thread_manager->RegisterThread(
        std::thread(&SimpleSlam::OperateThread, this, thread_param_),
        thread_param_);

    while (!IsRunning()) {
      Time::SleepMSec(2);
    }
  } else {
    is_running_.store(true);
  }

  {
    WriteLocker lock(access_);
    start_or_resume_timestamp_ = Time::Now();
    pose_filter_.Reset();
  }

  ZINFO << "Start slam finish.";
  return true;
}

bool SimpleSlam::StopSlam() {
  {
    ReadLocker lock(access_);
    if (!IsRunning() && probability_grid_map_ == nullptr) {
      ZGINFO << "Slam is already stopped.";
      return true;
    }
  }

  ZINFO << "Stop slam.";
  stop_thread_.store(true);

  while (stop_thread_.load() || IsRunning()) {
    Time::SleepMSec(2);
  }
  ZINFO << "Stop slam finish.";

  {
    WriteLocker lock(access_);
    odom_data_list_.clear();
    point_cloud_data_list_.clear();
    probability_grid_map_.reset();
    cached_probability_map_.reset();
    slam_map_.reset();
    stop_or_pause_timestamp_ = Time::Now();
  }

  return true;
}

bool SimpleSlam::PauseSlam() {
  ZINFO << "Pause slam.";
  {
    WriteLocker lock(access_);
    stop_or_pause_timestamp_ = Time::Now();
  }
  is_running_.store(false);
  return true;
}

bool SimpleSlam::ResumeSlam(const MapPoint& resume_pose) {
  ZINFO << "Resume slam.";
  {
    WriteLocker lock(access_);
    start_or_resume_timestamp_ = Time::Now();
    correction_.SetX(0);
    correction_.SetY(0);
    correction_.SetDegree(0);
    last_world_pose_.SetX(resume_pose.X());
    last_world_pose_.SetY(resume_pose.Y());
    last_world_pose_.SetDegree(resume_pose.Degree());
  }
  auto& tf = *TransformManager::Instance();
  Transform odom_tf(tf.kOdomFrame_, tf.kRobotFrame_, resume_pose.X(),
                          resume_pose.Y(), resume_pose.Degree());
  tf.UpdateTransform(odom_tf);
  Transform correction_tf(tf.kWorldFrame_, tf.kOdomFrame_, 0,
                          0, 0);
  tf.UpdateTransform(correction_tf);

  is_running_.store(true);
  return true;
}

bool SimpleSlam::PushMergedOdomData(const MergedOdomData::SPtr& odom_data) {
  if (!IsRunning()) {
    return false;
  }
  return SlamBase::PushMergedOdomData(odom_data);
}

bool SimpleSlam::PushPointCloud(const PointCloud::SPtr& point_cloud) {
  if (!IsRunning()) {
    return false;
  }
  return SlamBase::PushPointCloud(point_cloud);
}

bool SimpleSlam::UpdateScanIntoMap(
    const MapPoint& pose,
    const PointCloud::SPtr& point_cloud_in_chassis_frame) {
  return UpdateScanIntoMap(probability_grid_map_, pose,
                           point_cloud_in_chassis_frame);
}

void SimpleSlam::PrintMap() const {
  probability_grid_map_->Print(__FILE__, __FUNCTION__, __LINE__);
}

ProbabilityIndexGridMap2D::SPtr SimpleSlam::GetProbabilityMap() const {
  ReadLocker lock(access_);
  return probability_grid_map_;
};

bool SimpleSlam::UpdateProbabilityMap(
    const ProbabilityIndexGridMap2D::SPtr& map) {
  if (map == nullptr) {
    ZERROR << "Input ptr empty.";
    return false;
  }
  WriteLocker lock(access_);
  probability_grid_map_ = map;
  if (cached_probability_map_ == nullptr) {
    cached_probability_map_.reset(new FloatValueGridStaticMap2D(
        "cached " + probability_grid_map_->Name(),
        config_.precise_search_linear_step_,
        probability_grid_map_->GetMediumProbabilityForObstacleInCell()));
  } else {
    cached_probability_map_->ResetMap();
  }
  return true;
}

PointCloud::SPtr SimpleSlam::PointCloudInLidarFrameMotionCompensate(
    const std::deque<MergedOdomData::SPtr>& odom_data_list,
    const MapPoint& lidar_tf_to_chassis,
    const PointCloud::SPtr& point_cloud_in_lidar_frame) {
  if (odom_data_list.size() < 2) {
    ZERROR << "Odom data list invalid: "
           << std::to_string(odom_data_list.size());
    return point_cloud_in_lidar_frame;
  }
  if (point_cloud_in_lidar_frame == nullptr) {
    ZERROR << "Point cloud pointer invalid.";
    return point_cloud_in_lidar_frame;
  }
  {
    ReadLocker lock(point_cloud_in_lidar_frame->GetLock());
    if (point_cloud_in_lidar_frame->Empty()) {
      ZERROR << "Point cloud size invalid:"
             << std::to_string(point_cloud_in_lidar_frame->Size());
      return point_cloud_in_lidar_frame;
    }
  }
  auto point_cloud_start_time = point_cloud_in_lidar_frame->GetTimeStamp();
  if (point_cloud_start_time < odom_data_list.front()->GetTimeStamp()) {
    ZERROR << "Point cloud time "
           << DoubleToString(point_cloud_start_time, 4)
           << " before odom first data time "
           << DoubleToString(odom_data_list.front()->GetTimeStamp(), 4);
    return point_cloud_in_lidar_frame;
  }
  auto point_cloud_end_time =
      point_cloud_start_time + point_cloud_in_lidar_frame->GetScanTime();
  if (point_cloud_end_time > odom_data_list.back()->GetTimeStamp()) {
    ZERROR << "Point cloud end time " << DoubleToString(point_cloud_end_time, 4)
           << " newer than odom last data time "
           << DoubleToString(odom_data_list.back()->GetTimeStamp(), 4);
    return point_cloud_in_lidar_frame;
  }

  auto point_cloud_start_time_chassis_pose =
      PoseInterpolatorBase::InterpolatePose(odom_data_list,
                                            point_cloud_start_time);
  if (point_cloud_start_time_chassis_pose == nullptr) {
    ZWARN << "Interpolate failed.";
    return point_cloud_in_lidar_frame;
  }

  auto chassis_pose_to_lidar_pose_in_odom_frame =
      [](const MapPoint& chassis_pose,
         const MapPoint& lidar_tf_to_chassis) -> MapPoint {
    double x, y;
    Transform::CoordinateTransformationBA(
        lidar_tf_to_chassis.X(), lidar_tf_to_chassis.Y(), chassis_pose.X(),
        chassis_pose.Y(), DegreesToRadians(chassis_pose.Degree()), x, y);
    MapPoint lidar_pose;
    lidar_pose.SetX(x);
    lidar_pose.SetY(y);
    lidar_pose.SetDegree(
        NormalizeDegree(chassis_pose.Degree() + lidar_tf_to_chassis.Degree()));
    return lidar_pose;
  };

  auto compensate_chassis_pose = point_cloud_start_time_chassis_pose->GetPose();
  ZINFO << compensate_chassis_pose.DebugString();
  auto compensate_lidar_pose = chassis_pose_to_lidar_pose_in_odom_frame(
      compensate_chassis_pose, lidar_tf_to_chassis);
  ZINFO << compensate_lidar_pose.DebugString();

  PointCloud::SPtr compensated_point_cloud_in_lidar_frame =
      std::make_shared<PointCloud>(*point_cloud_in_lidar_frame);

  WriteLocker lock(compensated_point_cloud_in_lidar_frame->GetLock());
  auto& points = compensated_point_cloud_in_lidar_frame->GetPointsRef();
  for (uint i = 0; i < points.size(); i++) {
    auto point = points.at(i);
    auto point_time_chassis_pose = PoseInterpolatorBase::InterpolatePose(
        odom_data_list, point.TimeStamp());
    if (point_cloud_start_time_chassis_pose == nullptr) {
      ZWARN << "Interpolate failed for points.at(" << std::to_string(i) << "), "
            << point.DebugString();
      return point_cloud_in_lidar_frame;
    }
    auto chassis_pose = point_time_chassis_pose->GetPose();
    // ZINFO << chassis_pose.DebugString();
    auto lidar_pose = chassis_pose_to_lidar_pose_in_odom_frame(
        chassis_pose, lidar_tf_to_chassis);
    // ZINFO << lidar_pose.DebugString();
    double x, y;
    Transform::CoordinateTransformationBA(
        point.X(), point.Y(), lidar_pose.X(), lidar_pose.Y(),
        DegreesToRadians(lidar_pose.Degree()), x, y);
    Transform::CoordinateTransformationAB(
        x, y, compensate_lidar_pose.X(), compensate_lidar_pose.Y(),
        DegreesToRadians(compensate_lidar_pose.Degree()), x, y);
    // ZINFO << "Point " << std::to_string(i) << " " << point.DebugString();
    points.at(i) =
        PointCloud::Point(point.TimeStamp(), MapPoint(x, y), point.Intensity());
    // ZINFO << "Point after compensate " << std::to_string(i) << " "
    //       << point.DebugString();
    // ZINFO << "Point after compensate " << std::to_string(i) << " "
    //       << points.at(i).DebugString();
  }
  compensated_point_cloud_in_lidar_frame->SetEqualDegreeInterval(false);
  // ZINFO << "Point 20 after compensate : "
  //       << compensated_point_cloud_in_lidar_frame->GetPointsConstRef()
  //              .at(20)
  //              .DebugString();

  return compensated_point_cloud_in_lidar_frame;
}

PointCloud::SPtr SimpleSlam::PointCloudInLidarFrameMotionCompensate(
    const MapPoint& lidar_tf_to_chassis,
    const PointCloud::SPtr& point_cloud_in_lidar_frame) {
  return PointCloudInLidarFrameMotionCompensate(
      odom_data_list_, lidar_tf_to_chassis, point_cloud_in_lidar_frame);
}

void SimpleSlam::OperateThread(const ZimaThreadWrapper::ThreadParam& param) {
  ZGINFO << "Thread \"" << param.thread_name_ << "\" start.";
  auto thread_manager = ZimaThreadManager::Instance();
  if (param.bind_cpu_id_ >= 0) {
    thread_manager->BindCPUCore(param.thread_name_, param.bind_cpu_id_);
  }

  is_running_.store(true);
  auto& tf = *TransformManager::Instance();

  // std::shared_ptr<StopWatch> calculate_process_speed_stop_watch(
  //     new StopWatch("Calculate process speed stop_watch", true, true, true));
  while (!stop_thread_.load()) {
    thread_manager->UpdateThreadCycle(param.thread_name_);
    const auto sleep_ms = 5;
    Time::SleepMSec(sleep_ms);

    if (!IsRunning()) {
      // Slam in pause state.
      continue;
    }

    TimedMapPoint::SPtr match_time_world_pose = nullptr;
    PointCloud::SPtr point_cloud = nullptr;

    if (PrintRatioSamplersInfo(30)) {
      ZGINFO << pose_filter_.DebugString();
    }

    double point_cloud_start_time = 0;
    // StopWatch slam_operate_stop_watch("Slam operate stop watch", true, true);
    {
      WriteLocker lock(access_);
      if (point_cloud_data_list_.empty()) {
        continue;
      }
      point_cloud_start_time = point_cloud_data_list_.front()->GetTimeStamp();
      if (odom_data_list_.empty() ||
          odom_data_list_.front()->GetTimeStamp() > point_cloud_start_time) {
        // Point cloud data time not interpolatable.
        point_cloud_data_list_.pop_front();
        continue;
      }
      auto point_cloud_end_time = point_cloud_start_time +
                                  point_cloud_data_list_.front()->GetScanTime();
      if (odom_data_list_.empty() ||
          odom_data_list_.back()->GetTimeStamp() < point_cloud_end_time) {
        // Insufficient odom data.
        continue;
      }

      // Trim odom data list.
      while (odom_data_list_.size() > 2) {
        if (odom_data_list_.at(1)->GetTimeStamp() <
            last_world_pose_.GetTimestamp()) {
          odom_data_list_.pop_front();
        } else {
          break;
        }
      }

      // Use interpolate pose as input pose of matching.
      auto point_cloud_start_time_world_pose = InterpolateWorldPose(
          odom_data_list_, last_world_pose_, point_cloud_start_time);
      lock.Unlock();

      point_cloud = point_cloud_data_list_.front();
      // point_cloud = PointCloudInLidarFrameMotionCompensate(
      //     odom_data_list_, lidar_tf_to_chassis_,
      //     point_cloud_data_list_.front());

      point_cloud = point_cloud->TransformBA("point cloud in chassis frame",
                                             lidar_tf_to_chassis_);

      if (point_cloud_start_time_world_pose != nullptr) {
        // ZINFO << "Interpolate pose: " <<
        // point_cloud_start_time_world_pose->DebugString(); Match.
        match_time_world_pose =
            TryMatch(point_cloud, *point_cloud_start_time_world_pose);
        if (match_time_world_pose != nullptr) {
          // ZINFO << "Match as " << match_time_world_pose->DebugString()
          //       << ", cost time" << slam_operate_stop_watch.DebugString();
          lock.Lock();
          last_world_pose_ = *match_time_world_pose;
          // Update transform.
          auto match_time_odom_pose = PoseInterpolatorBase::InterpolatePose(
              odom_data_list_, point_cloud_start_time);
          lock.Unlock();
          if (match_time_odom_pose != nullptr) {
            // We hav P(world) as robot world pose, P(odom) as robot odom
            // pose. And we need to calculate transform between world frame
            // and odom frame.
            // In order to use Transform::CoordinateTransformationXX
            // function and for more comprehensible, firstly we inversly
            // calculate the coordinate of origin point of odom frme on
            // "pose frame". Then we get full transform chain from 'world'
            // to 'pose' to 'odom'.
            auto match_odom_pose = match_time_odom_pose->GetPose();
            MapPoint odom_origin_on_odom_pose_coordinate;
            {
              odom_origin_on_odom_pose_coordinate.SetDegree(
                  -match_odom_pose.Degree());
              double x, y;
              Transform::CoordinateTransformationAB(
                  0, 0, match_odom_pose.X(), match_odom_pose.Y(),
                  DegreesToRadians(match_odom_pose.Degree()), x, y);
              odom_origin_on_odom_pose_coordinate.SetX(x);
              odom_origin_on_odom_pose_coordinate.SetY(y);
            }
            MapPoint odom_origin_on_world_coordinate;
            {
              odom_origin_on_world_coordinate.SetDegree(
                  match_time_world_pose->Degree() +
                  odom_origin_on_odom_pose_coordinate.Degree());
              double x, y;
              Transform::CoordinateTransformationBA(
                  odom_origin_on_odom_pose_coordinate.X(),
                  odom_origin_on_odom_pose_coordinate.Y(),
                  match_time_world_pose->X(), match_time_world_pose->Y(),
                  DegreesToRadians(match_time_world_pose->Degree()), x, y);
              odom_origin_on_world_coordinate.SetX(x);
              odom_origin_on_world_coordinate.SetY(y);
            }

            lock.Lock();
            correction_ = odom_origin_on_world_coordinate;
            // ZINFO << "correction: " << correction_.DebugString();
            Transform correction_tf(tf.kWorldFrame_, tf.kOdomFrame_,
                                    correction_.X(), correction_.Y(),
                                    correction_.Degree());
            lock.Unlock();
            tf.UpdateTransform(correction_tf);
          } else {
            ZERROR << "Failed to interpolate odom pose at match time: "
                   << DoubleToString(point_cloud_start_time, 4) << ".";
          }
        } else {
          lock.Lock();
          last_world_pose_ = *point_cloud_start_time_world_pose;
          lock.Unlock();
        }

        lock.Lock();
        // ZINFO;
        // Pop this point cloud if it is not poped.
        if (!point_cloud_data_list_.empty() &&
            point_cloud_data_list_.front()->GetTimeStamp() ==
                point_cloud_start_time) {
          point_cloud_data_list_.pop_front();
        }
        lock.Unlock();
      }
    }

    if (match_time_world_pose != nullptr) {
      // ZINFO << "Match as " << match_time_world_pose->DebugString()
      //       << ", cost time" << slam_operate_stop_watch.DebugString();

      {
        WriteLocker lock(access_);
        last_match_point_cloud_in_world_frame_ = point_cloud->TransformBA(
            "match point cloud", *match_time_world_pose);
      }

      {
        ReadLocker lock(probability_grid_map_->GetLock());
        MapCell match_cell;
        probability_grid_map_->WorldToMap(*match_time_world_pose, match_cell);
        const float middle_probability =
            probability_grid_map_->GetMediumProbabilityForObstacleInCell();
        const float probability_limit =
            probability_grid_map_->GetMinProbabilityForObstacleInCell() +
            0.3 * (middle_probability -
                   probability_grid_map_->GetMinProbabilityForObstacleInCell());
        float probability = 0;
        if (probability_grid_map_->GetProbability(
                match_cell.X(), match_cell.Y(), probability)) {
          if (probability < probability_limit) {
            pose_filter_.UpdateMaxLinearDistance(
                config_.pose_filter_max_linear_distance_ * 3);
            pose_filter_.UpdateMaxAngleDegree(
                config_.pose_filter_max_angle_degree_ * 3);
            // ZGINFO << "Enlarge pose filter limit.";
          } else {
            pose_filter_.UpdateMaxLinearDistance(
                config_.pose_filter_max_linear_distance_);
            pose_filter_.UpdateMaxAngleDegree(
                config_.pose_filter_max_angle_degree_);
            // ZGINFO << "Restore pose filter limit.";
          }
        }
      }
      if (!pose_filter_.IsSimilar(*match_time_world_pose)) {
        // ZINFO << "Not similar.";
        bool odom_not_turning = true;
        {
          ReadLocker lock(access_);
          odom_not_turning = OdomMotionDetector::GetTurningState(
                                 odom_data_list_, point_cloud_start_time,
                                 point_cloud_start_time +
                                     config_.motion_detector_time_interval_,
                                 config_.motion_detector_degree_limit_) !=
                             OdomMotionDetector::kTurning;
          // ZINFO << "Odom turning " << (odom_not_turning ? "0" : "1");

          // if (cached_is_odom_not_turning_ ^ odom_not_turning) {
          //   ZINFO << "Odom turning " << (odom_not_turning ? "0" : "1");
          // }
          cached_is_odom_not_turning_ = odom_not_turning;
        }

        if (odom_not_turning) {
          if (!UpdateScanIntoMap(*match_time_world_pose, point_cloud)) {
            ZWARN;
          } else {
            MapConverter map_converter;
            WriteLocker lock(access_);
            if (slam_map_ == nullptr) {
              slam_map_.reset(new SlamValueGridMap2D(
                  "slam value grid map", 1, 1,
                  probability_grid_map_->GetResolution()));
            }
            map_converter.ConvertProbabilityGridMap2DToSlamValueGridMap(
                probability_grid_map_, slam_map_);

            // ZGINFO << "Reset " << cached_probability_map_->Name()
            //        << " last size " << cached_probability_map_->GetDataSize();
            cached_probability_map_->ResetMap();
            // BicubicInterpolator::Interpolate(probability_grid_map_, 3);
            // ZGINFO << "Update map.";
          }
        }
      }
    }
  }

  if (stop_thread_.load()) {
    ZGINFO << "Handle for stop thread cmd.";
    stop_thread_.store(false);
  }

  is_running_.store(false);

  ZGINFO << "Thread \"" << param.thread_name_ << "\" exiting.";
  thread_manager->MarkThreadExited(param.thread_name_, __FILE__, __FUNCTION__,
                                   __LINE__);
  ZGINFO << "Thread \"" << param.thread_name_ << "\" exit.";
}

TimedMapPoint::SPtr SimpleSlam::InterpolateWorldPose(
    const std::deque<MergedOdomData::SPtr>& odom_data_list,
    const TimedMapPoint& last_world_pose,
    const double& point_cloud_timestamp) const {
  if (last_world_pose.GetTimestamp() > point_cloud_timestamp) {
    ZERROR << "Last pose timestamp "
           << DoubleToString(last_world_pose.GetTimestamp(), 6)
           << " is later than point cloud timestamp "
           << DoubleToString(point_cloud_timestamp, 6);
    return nullptr;
  }

  ReadLocker read_lock(access_, false);
  if (!access_->IsLockedByThisThread()) {
    ZWARN << "Data is not in read/write mode.";
    read_lock.Lock();
  }

  auto point_cloud_timestamp_interpolate_odom_pose =
      PoseInterpolatorBase::InterpolatePose(odom_data_list,
                                            point_cloud_timestamp);

  if (point_cloud_timestamp_interpolate_odom_pose == nullptr) {
    return nullptr;
  }

  auto last_odom_pose = PoseInterpolatorBase::InterpolatePose(
      odom_data_list, last_world_pose.GetTimestamp());
  if (last_odom_pose == nullptr) {
    // If point_cloud_timestamp_interpolate_odom_pose is not nullptr,
    // odom_data_list is not empty.
    if (last_world_pose.GetTimestamp() <=
        odom_data_list.front()->GetTimeStamp()) {
      last_odom_pose = odom_data_list.front();
    } else {
      ZERROR << "Should never run here, last pose timestamp "
             << DoubleToString(last_world_pose.GetTimestamp(), 4)
             << ", first odom data timestamp "
             << DoubleToString(odom_data_list.front()->GetTimeStamp(), 4);
      return nullptr;
    }
  }

  double x, y;
  Transform::CoordinateTransformationAB(
      point_cloud_timestamp_interpolate_odom_pose->GetPose().X(),
      point_cloud_timestamp_interpolate_odom_pose->GetPose().Y(),
      last_odom_pose->GetPose().X(), last_odom_pose->GetPose().Y(),
      DegreesToRadians(last_odom_pose->GetPose().Degree()), x, y);
  MapPoint offset_point(
      x, y,
      point_cloud_timestamp_interpolate_odom_pose->GetPose().Degree() -
          last_odom_pose->GetPose().Degree());
  Transform::CoordinateTransformationBA(
      offset_point.X(), offset_point.Y(), last_world_pose.X(),
      last_world_pose.Y(), DegreesToRadians(last_world_pose.Degree()), x, y);
  auto predict_pose = std::make_shared<TimedMapPoint>(
      point_cloud_timestamp,
      MapPoint(x, y, last_world_pose.Degree() + offset_point.Degree()));
  // ZINFO << "Offset " << offset_point.DebugString() << ", last match "
  //       << last_world_pose.DebugString() << ", predict as "
  //       << predict_pose->DebugString();
  return predict_pose;
}

TimedMapPoint::SPtr SimpleSlam::TryMatch(
    const PointCloud::SPtr& point_cloud_in_chassis_frame,
    const MapPoint& interpolate_pose) {
  const int8_t kScoreStepCount = 5;
  const auto score_increase_step =
      (config_.max_required_match_score_ - config_.min_required_match_score_) /
      (kScoreStepCount - 1);
  const int8_t kSearchRangeStepCount = 3;
  auto linear_range_increase_step =
      (config_.max_search_linear_range_ - config_.min_search_linear_range_) /
      (kSearchRangeStepCount - 1);
  auto degree_range_increase_step =
      (config_.max_search_degree_range_ - config_.min_search_degree_range_) /
      (kSearchRangeStepCount - 1);
  auto search_linear_range = config_.min_search_linear_range_;
  auto search_degree_range = config_.min_search_degree_range_;
  PointCloudMatcher::MatchResult raw_match_result;
  for (; search_linear_range < config_.max_search_linear_range_ &&
         search_degree_range < config_.max_search_degree_range_;) {
    {
      // Raw match
      DynamicMapPointBound search_bound(
          interpolate_pose.X() - search_linear_range / 2,
          interpolate_pose.X() + search_linear_range / 2,
          interpolate_pose.Y() - search_linear_range / 2,
          interpolate_pose.Y() + search_linear_range / 2);

      PointCloudMatcher::NormalSearchParameter parameter(
          interpolate_pose, search_bound, config_.raw_search_linear_step_,
          search_degree_range / 2, config_.raw_search_degree_step_, 0);
      // ZINFO << "Raw match.";
      raw_matcher_->NormalSearch(probability_grid_map_,
                                 point_cloud_in_chassis_frame, parameter,
                                 raw_match_result);
      // ZGWARN << "Raw match finish." << raw_match_result.DebugString();
    }
    // ZERROR << "dynamic_required_score_: "
    //        << FloatToString(dynamic_required_score_, 4);
    if (raw_match_result.score_ > dynamic_required_score_) {
    // if (true) {
      // Precise match
      DynamicMapPointBound search_bound(
          raw_match_result.pose_.X() - config_.precise_search_linear_range_ / 2,
          raw_match_result.pose_.X() + config_.precise_search_linear_range_ / 2,
          raw_match_result.pose_.Y() - config_.precise_search_linear_range_ / 2,
          raw_match_result.pose_.Y() +
              config_.precise_search_linear_range_ / 2);
      if (config_.precise_search_linear_step_ >
          config_.precise_search_linear_range_) {
        ZERROR << "precise_search_linear_step_ "
               << FloatToString(config_.precise_search_linear_step_, 3)
               << " is larget than search linear range "
               << FloatToString(config_.precise_search_linear_range_, 3);
      }

      PointCloudMatcher::MatchResult precise_match_result;
      // ZINFO << "Precise match.";
      if (false) {
        PointCloudMatcher::NormalSearchParameter parameter(
            raw_match_result.pose_, search_bound,
            config_.precise_search_linear_step_,
            config_.precise_search_degree_range_,
            config_.precise_search_degree_step_, 0);
        precise_matcher_->NormalSearch(probability_grid_map_,
                                       point_cloud_in_chassis_frame, parameter,
                                       precise_match_result);
        // ZWARN << "Precise match: " << precise_match_result.DebugString();
      }
      PointCloudMatcher::GradientSearchParameter parameter(
          raw_match_result.pose_, search_bound,
          config_.precise_search_linear_step_,
          config_.precise_search_degree_range_,
          config_.precise_search_degree_step_, 10, 0);
      precise_matcher_->GradientSearch(
          probability_grid_map_, point_cloud_in_chassis_frame, parameter,
          precise_match_result, true, cached_probability_map_);
      // ZGWARN << "Precise match finish." <<
      // precise_match_result.DebugString();

      // if (precise_match_result.score_ < raw_match_result.score_) {
      //   ZERROR << "Raw match " << raw_match_result.DebugString()
      //          << ", precise match " << precise_match_result.DebugString();
      // } else
      {
        dynamic_required_score_ =
            std::min(dynamic_required_score_ + score_increase_step,
                     precise_match_result.score_);
        dynamic_required_score_ =
            Clip(dynamic_required_score_, config_.min_required_match_score_,
                 config_.max_required_match_score_);
        // ZWARN << "Precise match: " << precise_match_result.DebugString();
        // if (precise_match_result.pose_ != raw_match_result.pose_ ||
        //     !FloatEqual(precise_match_result.pose_.Degree(),
        //                 raw_match_result.pose_.Degree())) {
        //   ZWARN << "Raw match: " << raw_match_result.DebugString();
        //   ZWARN << "Precise match: " << precise_match_result.DebugString();
        // }
        return std::make_shared<TimedMapPoint>(
            point_cloud_in_chassis_frame->GetTimeStamp(),
            precise_match_result.pose_);
      }
    }

    search_linear_range += linear_range_increase_step;
    search_degree_range += degree_range_increase_step;
  }

  dynamic_required_score_ -= score_increase_step;
  dynamic_required_score_ =
      Clip(dynamic_required_score_, config_.min_required_match_score_,
           config_.max_required_match_score_);

  if (raw_match_result.score_ > config_.min_required_match_score_) {
    // Directly precise match
    DynamicMapPointBound search_bound(
        raw_match_result.pose_.X() - config_.precise_search_linear_range_ / 2,
        raw_match_result.pose_.X() + config_.precise_search_linear_range_ / 2,
        raw_match_result.pose_.Y() - config_.precise_search_linear_range_ / 2,
        raw_match_result.pose_.Y() + config_.precise_search_linear_range_ / 2);
    if (config_.precise_search_linear_step_ >
        config_.precise_search_linear_range_) {
      ZERROR << "precise_search_linear_step_ "
             << FloatToString(config_.precise_search_linear_step_, 3)
             << " is larget than search linear range "
             << FloatToString(config_.precise_search_linear_range_, 3);
    }

    PointCloudMatcher::MatchResult precise_match_result;
    if (false) {
      PointCloudMatcher::NormalSearchParameter parameter(
          raw_match_result.pose_, search_bound,
          config_.precise_search_linear_step_,
          config_.precise_search_degree_range_,
          config_.precise_search_degree_step_, 0);
      precise_matcher_->NormalSearch(probability_grid_map_,
                                     point_cloud_in_chassis_frame, parameter,
                                     precise_match_result);
      // ZWARN << "Precise match: " << precise_match_result.DebugString();
    }
    PointCloudMatcher::GradientSearchParameter parameter(
        raw_match_result.pose_, search_bound,
        config_.precise_search_linear_step_,
        config_.precise_search_degree_range_,
        config_.precise_search_degree_step_, 10, 0);
    precise_matcher_->GradientSearch(probability_grid_map_,
                                     point_cloud_in_chassis_frame, parameter,
                                     precise_match_result);
    if (precise_match_result.score_ < raw_match_result.score_) {
      ZERROR << "Raw match " << raw_match_result.DebugString()
             << ", precise match " << precise_match_result.DebugString();
    } else {
      // ZWARN << "Precise match finish." << precise_match_result.DebugString();
      return std::make_shared<TimedMapPoint>(
          point_cloud_in_chassis_frame->GetTimeStamp(),
          precise_match_result.pose_);
    }
  }

  // ZWARN << "dynamic_required_score_: "
  //        << FloatToString(dynamic_required_score_, 4);

  return nullptr;
}

bool SimpleSlam::UpdateScanIntoMap(
    const ProbabilityIndexGridMap2D::SPtr& map, const MapPoint& pose,
    const PointCloud::SPtr& point_cloud_in_chassis_frame) {
  if (map == nullptr) {
    ZERROR << "Map pointer invalid.";
    return false;
  }
  if (point_cloud_in_chassis_frame == nullptr) {
    ZERROR << "Point cloud pointer invalid.";
    return false;
  }

  WriteLocker lock(map->GetLock());

  if (!map->PrepareForUpdate()) {
    ZERROR << "Map not ready for update.";
    return false;
  }

  auto point_cloud_in_map =
      point_cloud_in_chassis_frame->TransformBA("point cloud in map", pose);

  MapPoints hit_points;
  {
    ReadLocker lock(point_cloud_in_map->GetLock());
    for (auto&& point : point_cloud_in_map->GetPointsConstRef()) {
      hit_points.emplace_back(point.ToMapPoint());
    }
  }

  // Update for hit points.
  MapCell update_cell;
  for (auto&& point : hit_points) {
    map->WorldToMap(point, update_cell);
    if (!map->UpdateForHit(update_cell.X(), update_cell.Y())) {
      ZERROR << "Update hit point " << point.DebugString() << "("
             << update_cell.DebugString() << ") failed.";
      return false;
    }
  }

  // Update for space between hit point and robot.
  for (auto&& hit_point : hit_points) {
    auto space_points =
        GenerateInterpolationPoints(pose, hit_point, map->GetResolution() / 2);
    for (auto&& space_point : space_points) {
      map->WorldToMap(space_point, update_cell);
      if (!map->UpdateForMiss(update_cell.X(), update_cell.Y())) {
        ZERROR << "Update miss point " << hit_point.DebugString() << "("
               << update_cell.DebugString() << ") failed.";
        return false;
      }
    }
  }

  // map->Print(__FILE__, __FUNCTION__, __LINE__);

  return true;
}

}  // namespace zima
