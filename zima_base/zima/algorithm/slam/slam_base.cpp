/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/algorithm/slam/slam_base.h"

#include "zima/common/config.h"
#include "zima/common/time.h"
#include "zima/logger/logger.h"

namespace zima {

SlamBase::Config::Config() : Config(nullptr){};

SlamBase::Config::Config(const JsonSPtr& json) : config_valid_(false) {
  // Load default setting.
  odom_data_list_max_size_ = 100;
  odom_data_list_max_time_interval_ = 2;
  point_cloud_list_max_size_ = 10;
  point_cloud_list_max_time_interval_ = 2;
  odom_sample_ratio_ = 0.5;
  point_cloud_sample_ratio_ = 1;
  pose_filter_max_linear_distance_ = 0.3;
  pose_filter_max_angle_degree_ = 20;
  pose_filter_max_time_interval_ = 9999;

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
      ZGWARN << "Global config not found.";
    }
  }
};

bool SlamBase::Config::ParseFromJson(const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }

  // Override.
  if (!JsonHelper::GetUInt(*json, kOdomDataListMaxSizeKey_,
                           odom_data_list_max_size_)) {
    ZGERROR << "Config " << kOdomDataListMaxSizeKey_ << " not found.";
    // return false;
  }

  if (!JsonHelper::GetFloat(*json, kOdomDataListMaxTimeIntervalKey_,
                            odom_data_list_max_time_interval_)) {
    ZGERROR << "Config " << kOdomDataListMaxTimeIntervalKey_ << " not found.";
    // return false;
  }
  if (odom_data_list_max_time_interval_ < 0) {
    ZERROR << "Config " << kOdomDataListMaxTimeIntervalKey_ << " invalid: "
           << FloatToString(odom_data_list_max_time_interval_, 3) << ".";
    return false;
  }

  if (!JsonHelper::GetUInt(*json, kPointCloudListMaxSizeKey_,
                           point_cloud_list_max_size_)) {
    ZGERROR << "Config " << kPointCloudListMaxSizeKey_ << " not found.";
    // return false;
  }

  if (!JsonHelper::GetFloat(*json, kPointCloudListMaxTimeIntervalKey_,
                            point_cloud_list_max_time_interval_)) {
    ZGERROR << "Config " << kPointCloudListMaxTimeIntervalKey_ << " not found.";
    // return false;
  }
  if (point_cloud_list_max_time_interval_ < 0) {
    ZERROR << "Config " << kPointCloudListMaxTimeIntervalKey_ << " invalid: "
           << FloatToString(point_cloud_list_max_time_interval_, 3) << ".";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kOdomSampleRatioKey_, odom_sample_ratio_)) {
    ZGERROR << "Config " << kOdomSampleRatioKey_ << " not found.";
    // return false;
  }
  if (odom_sample_ratio_ < 0 || odom_sample_ratio_ > 1) {
    ZERROR << "Config " << kOdomSampleRatioKey_
           << " invalid: " << FloatToString(odom_sample_ratio_, 3) << ".";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kPointCloudSampleRatioKey_,
                            point_cloud_sample_ratio_)) {
    ZGERROR << "Config " << kPointCloudSampleRatioKey_ << " not found.";
    // return false;
  }
  if (point_cloud_sample_ratio_ < 0 || point_cloud_sample_ratio_ > 1) {
    ZERROR << "Config " << kPointCloudSampleRatioKey_
           << " invalid: " << FloatToString(point_cloud_sample_ratio_, 3)
           << ".";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kPoseFilterMaxLinearDistanceKey_,
                            pose_filter_max_linear_distance_)) {
    ZGERROR << "Config " << kPoseFilterMaxLinearDistanceKey_ << " not found.";
    // return false;
  }
  if (pose_filter_max_linear_distance_ < 0) {
    ZERROR << "Config " << kPoseFilterMaxLinearDistanceKey_
           << " invalid: " << FloatToString(pose_filter_max_linear_distance_, 3)
           << ".";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kPoseFilterMaxAngleDegreeKey_,
                            pose_filter_max_angle_degree_)) {
    ZGERROR << "Config " << kPoseFilterMaxAngleDegreeKey_ << " not found.";
    // return false;
  }
  if (pose_filter_max_angle_degree_ < 0) {
    ZERROR << "Config " << kPoseFilterMaxAngleDegreeKey_
           << " invalid: " << FloatToString(pose_filter_max_angle_degree_, 3)
           << ".";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kPoseFilterMaxTimeIntervalKey_,
                            pose_filter_max_time_interval_)) {
    ZGERROR << "Config " << kPoseFilterMaxTimeIntervalKey_ << " not found.";
    // return false;
  }
  if (pose_filter_max_time_interval_ < 0) {
    ZERROR << "Config " << kPoseFilterMaxTimeIntervalKey_
           << " invalid: " << FloatToString(pose_filter_max_time_interval_, 3)
           << ".";
    return false;
  }

  return true;
}

SlamBase::PoseFilter::PoseFilter(const float& max_linear_distance,
                                 const float& max_angle_degree,
                                 const float& max_time_interval_s)
    : num_total_(0),
      num_different_(0),
      max_linear_distance_(max_linear_distance),
      max_angle_degree_(max_angle_degree),
      max_time_interval_s_(max_time_interval_s),
      last_pose_(nullptr) {}

bool SlamBase::PoseFilter::IsSimilar(const TimedMapPoint& pose) {
  ++num_total_;
  if (num_total_ > 1 && last_pose_ != nullptr &&
      pose.GetTimestamp() - last_pose_->GetTimestamp() <=
          max_time_interval_s_ &&
      pose.Distance(*last_pose_) <= max_linear_distance_ &&
      fabs(pose.AngleDiff(*last_pose_)) <= max_angle_degree_) {
    return true;
  }
  last_pose_.reset(new TimedMapPoint(pose));
  ++num_different_;
  return false;
}

void SlamBase::PoseFilter::UpdateMaxLinearDistance(
    const float& max_linear_distance) {
  max_linear_distance_ = max_linear_distance;
}

void SlamBase::PoseFilter::UpdateMaxAngleDegree(const float& max_angle_degree) {
  max_angle_degree_ = max_angle_degree;
}

void SlamBase::PoseFilter::UpdateMaxTimeInterval(
    const float& max_time_interval_s) {
  max_time_interval_s_ = max_time_interval_s;
}

void SlamBase::PoseFilter::Reset() {
  num_total_ = 0;
  num_different_ = 0;
  last_pose_ = nullptr;
}

std::string SlamBase::PoseFilter::DebugString() const {
  return "Filter reduced to " +
         FloatToString(100. * num_different_ / num_total_, 2) + "%.";
}

SlamBase::SlamBase(const Config& config)
    : access_(std::make_shared<ReadWriteLock>()),
      is_running_(false),
      is_ready_(false),
      save_file_name_(""),
      odom_data_list_max_size_(config.odom_data_list_max_size_),
      odom_data_list_max_time_interval_(
          config.odom_data_list_max_time_interval_),
      point_cloud_list_max_size_(config.point_cloud_list_max_size_),
      point_cloud_list_max_time_interval_(
          config.point_cloud_list_max_time_interval_),
      odom_ratio_sampler_(config.odom_sample_ratio_),
      point_cloud_ratio_sampler_(config.point_cloud_sample_ratio_),
      odom_frequency_count_(0),
      point_cloud_frequency_count_(0),
      print_sensor_frequency_time_(0),
      slam_map_(nullptr),
      correction_(),
      start_or_resume_timestamp_(0),
      stop_or_pause_timestamp_(0),
      last_world_pose_(),
      last_match_point_cloud_in_world_frame_(nullptr) {}

bool SlamBase::StartSlam(const MapPoint& init_pose,
                         const std::string& save_file_name,
                         const std::string& load_file_name) {
  ZERROR << "Never call this function, it should be override.";
  return false;
}

bool SlamBase::StopSlam() {
  ZERROR << "Never call this function, it should be override.";
  return false;
}

bool SlamBase::PauseSlam() {
  ZERROR << "Never call this function, it should be override.";
  return false;
}

bool SlamBase::ResumeSlam(const MapPoint& resume_pose) {
  ZERROR << "Never call this function, it should be override.";
  return false;
}

bool SlamBase::PushMergedOdomData(const MergedOdomData::SPtr& odom_data) {
  if (odom_data != nullptr) {
    WriteLocker lock(access_);
    if (odom_ratio_sampler_.Pulse()) {
      odom_data_list_.emplace_back(odom_data);
      odom_frequency_count_++;
    }
    while (!odom_data_list_.empty()) {
      if (odom_data_list_.front()->GetTimeStamp() <
          odom_data->GetTimeStamp() - odom_data_list_max_time_interval_) {
        odom_data_list_.pop_front();
      } else if (odom_data_list_.size() > odom_data_list_max_size_) {
        odom_data_list_.pop_front();
      } else {
        break;
      }
    }
    return true;
  } else {
    ZERROR << "Input empty data.";
  }
  return false;
}

bool SlamBase::PushPointCloud(const PointCloud::SPtr& point_cloud) {
  if (point_cloud != nullptr) {
    WriteLocker lock(access_);
    if (point_cloud_ratio_sampler_.Pulse()) {
      point_cloud_data_list_.emplace_back(point_cloud);
      point_cloud_frequency_count_++;
    }
    ReadLocker point_cloud_lock(point_cloud->GetLock());
    while (!point_cloud_data_list_.empty()) {
      ReadLocker first_point_cloud_lock(
          point_cloud_data_list_.front()->GetLock());
      if (point_cloud_data_list_.front()->GetTimeStamp() <
          point_cloud->GetTimeStamp() - point_cloud_list_max_time_interval_) {
        point_cloud_data_list_.pop_front();
      } else if (point_cloud_data_list_.size() > point_cloud_list_max_size_) {
        point_cloud_data_list_.pop_front();
      } else {
        break;
      }
    }
    return true;
  } else {
    ZERROR << "Input empty data.";
  }
  return false;
}

SlamValueGridMap2D::SPtr SlamBase::GetSlamMap() const {
  ReadLocker lock(access_);
  return slam_map_;
}

void SlamBase::SetSlamMap(const SlamValueGridMap2D::SPtr& slam_map) {
  WriteLocker lock(access_);
  if (slam_map_ == nullptr) {
    ZINFO << "First time set slam map.";
  }
  slam_map_ = slam_map;
}

TimedMapPoint SlamBase::GetLastWorldPose() const {
  ReadLocker lock(access_);
  return last_world_pose_;
}

PointCloud::SPtr SlamBase::GetLastMatchPointCloudInWorldFrame() const {
  ReadLocker lock(access_);
  return last_match_point_cloud_in_world_frame_;
}

bool SlamBase::PrintRatioSamplersInfo(const float& interval) {
  auto now = Time::Now();
  if (now - print_sensor_frequency_time_ > interval) {
    WriteLocker lock(access_);
    auto time_interval = now - print_sensor_frequency_time_;
    auto odom_frequency = odom_frequency_count_ / time_interval;
    auto point_cloud_frequency = point_cloud_frequency_count_ / time_interval;
    ZINFO << "Odom frequency: " << FloatToString(odom_frequency, 1) << "Hz("
          << odom_ratio_sampler_.DebugString() << "), point cloud frequency: "
          << FloatToString(point_cloud_frequency, 1) << "Hz("
          << point_cloud_ratio_sampler_.DebugString() << ").";
    odom_frequency_count_ = 0;
    point_cloud_frequency_count_ = 0;
    print_sensor_frequency_time_ = now;
    return true;
  }
  return false;
}

}  // namespace zima
