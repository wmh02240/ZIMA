/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/algorithm/pose_interpolator.h"

#include "zima/common/config.h"
#include "zima/common/time.h"
#include "zima/common/transform.h"
#include "zima/logger/logger.h"

namespace zima {

MapPoint::SPtr PoseInterpolatorBase::CalculateVelocity(
    const MergedOdomData& a, const MergedOdomData& b) {
  if (DoubleEqual(a.GetTimeStamp(), b.GetTimeStamp())) {
    ZGWARN << "Input with same time stamp "
           << DoubleToString(a.GetTimeStamp(), 4);
    return std::make_shared<MapPoint>(0, 0, 0);
  }
  if (fabs(a.GetTimeStamp() - b.GetTimeStamp()) > 1) {
    ZGERROR
        << "time stamp " << DoubleToString(a.GetTimeStamp(), 4) << " and "
        << DoubleToString(b.GetTimeStamp(), 4)
        << " are too far(more than 1s), angular velocity will be incorrect.";
    return nullptr;
  }

  auto a_pose = a.GetPose();
  auto b_pose = b.GetPose();
  auto time_diff = b.GetTimeStamp() - a.GetTimeStamp();
  if (a_pose == b_pose) {
    return std::make_shared<MapPoint>(
        0, 0, (b_pose.Degree() - a_pose.Degree()) / time_diff);
  } else {
    double x_offset_for_a_pose, y_offset_for_a_pose;
    Transform::CoordinateTransformationAB(
        b_pose.X(), b_pose.Y(), a_pose.X(), a_pose.Y(),
        DegreesToRadians(a_pose.Degree()), x_offset_for_a_pose,
        y_offset_for_a_pose);
    return std::make_shared<MapPoint>(
        x_offset_for_a_pose / time_diff, y_offset_for_a_pose / time_diff,
        (b_pose.Degree() - a_pose.Degree()) / time_diff);
  }
}

MergedOdomData::SPtr PoseInterpolatorBase::PredictPose(
    const std::deque<MergedOdomData::SPtr> buffer, const double& timestamp) {
  if (buffer.size() < 2) {
    ZGWARN << "Insufficient buffer data, buffer size: " << buffer.size();
    return nullptr;
  }
  if (timestamp < buffer.back()->GetTimeStamp()) {
    ZGWARN << "Timestamp " << DoubleToString(timestamp, 4)
           << " is before buffer last timestamp: "
           << DoubleToString(buffer.back()->GetTimeStamp(), 4);
    return nullptr;
  }

  auto time_diff = timestamp - buffer.back()->GetTimeStamp();
  MapPoint velocity;
  auto buffer_back_point = buffer.back()->GetPose();
  if (buffer.back()->IsVelocityValid()) {
    velocity = buffer.back()->GetVelocity();
  } else {
    auto _velocity =
        CalculateVelocity(**(buffer.end() - 2), **(buffer.end() - 1));
    if (_velocity != nullptr) {
      velocity = *_velocity;
    } else {
      return nullptr;
    }
  }
  ZGINFO << "velocity: " << velocity.DebugString();

  MapPoint future_point_offset;
  future_point_offset.SetX(velocity.X() * time_diff);
  future_point_offset.SetY(velocity.Y() * time_diff);
  future_point_offset.SetDegree(velocity.Degree() * time_diff);
  double future_point_x, future_point_y;
  Transform::CoordinateTransformationBA(
      future_point_offset.X(), future_point_offset.Y(), buffer_back_point.X(),
      buffer_back_point.Y(), DegreesToRadians(buffer_back_point.Degree()),
      future_point_x, future_point_y);
  double future_point_degree = NormalizeDegree(buffer_back_point.Degree() +
                                               future_point_offset.Degree());
  return std::make_shared<MergedOdomData>(
      timestamp, MapPoint(future_point_x, future_point_y, future_point_degree));
}

bool PoseInterpolatorBase::IsInterpolatable(
    const std::deque<MergedOdomData::SPtr> buffer, const double& timestamp) {
  if (buffer.size() < 2) {
    ZGWARN << "Insufficient buffer data, buffer size: " << buffer.size();
    return false;
  }
  if (timestamp > buffer.back()->GetTimeStamp()) {
    // ZWARN << "Timestamp " << DoubleToString(timestamp, 6)
    //       << " is after buffer last timestamp: "
    //       << DoubleToString(buffer.back()->GetTimeStamp(), 6);
    return false;
  }

  if (timestamp < buffer.front()->GetTimeStamp()) {
    ZGWARN << "Timestamp " << DoubleToString(timestamp, 6)
           << " is before buffer first timestamp: "
           << DoubleToString(buffer.front()->GetTimeStamp(), 6);
    return false;
  }

  return true;
}

MergedOdomData::SPtr PoseInterpolatorBase::InterpolatePose(
    const std::deque<MergedOdomData::SPtr> buffer, const double& timestamp) {
  if (!IsInterpolatable(buffer, timestamp)) {
    return nullptr;
  }

  auto first_data_after_timestamp =
      std::find_if(buffer.begin(), buffer.end(),
                   [timestamp](const MergedOdomData::SPtr& ptr) {
                     return ptr->GetTimeStamp() >= timestamp;
                   });
  auto last_data_before_timestamp = first_data_after_timestamp == buffer.begin()
                                        ? first_data_after_timestamp
                                        : first_data_after_timestamp - 1;

  auto velocity = CalculateVelocity(**last_data_before_timestamp,
                                    **first_data_after_timestamp);
  if (velocity == nullptr) {
    return nullptr;
  }
  // ZINFO << "velocity: " << velocity->DebugString();

  auto time_diff = timestamp - (*last_data_before_timestamp)->GetTimeStamp();
  auto pose = (*last_data_before_timestamp)->GetPose();
  MapPoint interpolate_point_offset;
  interpolate_point_offset.SetX(velocity->X() * time_diff);
  interpolate_point_offset.SetY(velocity->Y() * time_diff);
  interpolate_point_offset.SetDegree(velocity->Degree() * time_diff);
  double interpolate_point_x, interpolate_point_y;
  Transform::CoordinateTransformationBA(
      interpolate_point_offset.X(), interpolate_point_offset.Y(), pose.X(),
      pose.Y(), DegreesToRadians(pose.Degree()), interpolate_point_x,
      interpolate_point_y);
  double interpolate_point_degree =
      NormalizeDegree(pose.Degree() + interpolate_point_offset.Degree());
  return std::make_shared<MergedOdomData>(
      timestamp, MapPoint(interpolate_point_x, interpolate_point_y,
                          interpolate_point_degree));
}

PoseInterpolator::Config::Config() : Config(nullptr) {}

PoseInterpolator::Config::Config(const JsonSPtr& json) : config_valid_(false) {
  // Load default setting.
  buffer_time_limit_ = 3;
  buffer_count_limit_ = 150;

  // Override if json config is provided.
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
      ZERROR;
    }
  }
}

bool PoseInterpolator::Config::ParseFromJson(const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }

  // Override.
  if (!JsonHelper::GetFloat(*json, kBufferTimeLimitKey_, buffer_time_limit_)) {
    ZERROR << "Config " << kBufferTimeLimitKey_ << " not found.";
    return false;
  }

  if (!JsonHelper::GetUInt(*json, kBufferCountLimitKey_, buffer_count_limit_)) {
    ZERROR << "Config " << kBufferCountLimitKey_ << " not found.";
    return false;
  }

  return true;
}

PoseInterpolator::PoseInterpolator(const Config& config)
    : access_(std::make_shared<ReadWriteLock>()),
      buffer_time_limit_(config.buffer_time_limit_),
      buffer_count_limit_(config.buffer_count_limit_) {
  ZINFO << "Initialize interpolator with time limit: "
        << FloatToString(buffer_time_limit_, 1)
        << " count limit: " << buffer_count_limit_;
}

void PoseInterpolator::AddData(const MergedOdomData::SPtr& data) {
  if (data == nullptr) {
    ZERROR << "Input empty data.";
    return;
  }

  WriteLocker lock(access_);
  while (!buffer_.empty()) {
    if (buffer_.size() > buffer_count_limit_) {
      buffer_.pop_front();
    } else if (Time::Now() - buffer_.front()->GetTimeStamp() <
               buffer_time_limit_) {
      buffer_.pop_front();
    } else {
      break;
    }
  }

  if (!buffer_.empty() &&
      data->GetTimeStamp() < buffer_.back()->GetTimeStamp()) {
    ZGERROR << "Input data timestamp: "
            << DoubleToString(data->GetTimeStamp(), 4)
            << ", newest timestamp in buffer: "
            << DoubleToString(buffer_.back()->GetTimeStamp(), 4);
    return;
  }

  buffer_.emplace_back(data);
}

MergedOdomData::SPtr PoseInterpolator::PredictPose(
    const double& timestamp) const {
  ReadLocker lock(access_);
  return PoseInterpolatorBase::PredictPose(buffer_, timestamp);
}

bool PoseInterpolator::IsInterpolatable(const double& timestamp) const {
  ReadLocker lock(access_);
  return PoseInterpolatorBase::IsInterpolatable(buffer_, timestamp);
}

MergedOdomData::SPtr PoseInterpolator::InterpolatePose(
    const double& timestamp) const {
  if (!IsInterpolatable(timestamp)) {
    return nullptr;
  }

  ReadLocker lock(access_);
  return PoseInterpolatorBase::InterpolatePose(buffer_, timestamp);
}

}  // namespace zima
