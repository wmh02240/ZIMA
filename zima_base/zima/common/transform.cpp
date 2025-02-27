/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/common/transform.h"
#include "zima/logger/logger.h"
#include "zima/zima_base_version.h"

namespace zima {

std::string Transform::DebugString() const {
  std::string msg;
  msg += "TF " + source_frame_ + " to " + target_frame_ + ":\n";
  msg += "(" + std::to_string(x_) + ", " + std::to_string(y_) + ", " +
         std::to_string(degree_) + ")";

  return msg;
}

TransformManager::TransformManager()
    : access_(std::make_shared<ReadWriteLock>()) {
  Transform lidar_into_robot(kRobotFrame_, kLidarFrame_);
  UpdateTransform(lidar_into_robot);
  Transform robot_into_odom(kOdomFrame_, kRobotFrame_);
  UpdateTransform(robot_into_odom);
  Transform odom_into_world(kWorldFrame_, kOdomFrame_);
  UpdateTransform(odom_into_world);
}

bool TransformManager::UpdateTransform(const Transform& transform) {
  WriteLocker lock(access_);
  if (transforms_.count(transform.SourceFrame()) != 0) {
    if (transforms_.at(transform.SourceFrame()).TargetFrame() !=
        transform.TargetFrame()) {
      ZERROR << "Changing target frame of " << transform.SourceFrame()
             << "(" << transform.TargetFrame() << ") is not allowed";
      return false;
    }
  } else {
    transforms_.emplace(transform.SourceFrame(), transform);
    ZGINFO << "Create tf from " << transform.SourceFrame() << " into "
           << transform.TargetFrame();
  }
  WriteLocker transform_lock(transforms_.at(transform.SourceFrame()).GetLock());
  transforms_.at(transform.SourceFrame()) = transform;
  return true;
}

bool TransformManager::UpdateTransform(const std::string& target_frame,
                                       const std::string& source_frame,
                                       const MapPoint& offset_in_source_frame) {
  WriteLocker lock(access_);
  if (transforms_.count(source_frame) != 0) {
    if (transforms_.at(source_frame).TargetFrame() != target_frame) {
      ZERROR << "Changing target frame of " << source_frame << "("
             << target_frame << ") is not allowed";
      return false;
    }
  }
  const auto& tf = transforms_.at(source_frame);

  double offset_in_target_frame_x, offset_in_target_frame_y;
  Transform::CoordinateTransformationBA(
      offset_in_source_frame.X(), offset_in_source_frame.Y(), tf.X(), tf.Y(),
      DegreesToRadians(tf.Degree()), offset_in_target_frame_x,
      offset_in_target_frame_y);

  Transform new_transform(
      target_frame, source_frame, offset_in_target_frame_x,
      offset_in_target_frame_y,
      NormalizeDegree(tf.Degree() + offset_in_source_frame.Degree()));
  transforms_.at(source_frame) = new_transform;
  return true;
}

bool TransformManager::GetTransform(const std::string& target_frame,
                                    const std::string& source_frame,
                                    Transform& out) const {
  ReadLocker lock(access_);
  if (transforms_.count(source_frame) == 0) {
    ZWARN << "Transform " << source_frame << " is not registered.";
    return false;
  }

  ReadLocker transform_lock(transforms_.at(source_frame).GetLock());
  std::string _target_frame = transforms_.at(source_frame).TargetFrame();
  Transform _out(target_frame, source_frame,
                 transforms_.at(source_frame).X(),
                 transforms_.at(source_frame).Y(),
                 transforms_.at(source_frame).Degree());
  transform_lock.Unlock();

  while (_target_frame != target_frame) {
    // Find source frame.
    if (transforms_.count(_target_frame) == 0) {
      ZWARN << "Can not find transform from " << source_frame << " into "
            << target_frame << ".";
      return false;
    }

    double new_x, new_y;

    ReadLocker transform_lock(transforms_.at(_target_frame).GetLock());
    Transform::CoordinateTransformationBA(
        _out.X(), _out.Y(), transforms_.at(_target_frame).X(),
        transforms_.at(_target_frame).Y(),
        DegreesToRadians(transforms_.at(_target_frame).Degree()), new_x, new_y);
    _out.X(new_x);
    _out.Y(new_y);
    _out.Degree(NormalizeDegree(transforms_.at(_target_frame).Degree() +
                                _out.Degree()));

    _target_frame = transforms_.at(_target_frame).TargetFrame();
  }

  out = _out;
  return true;
}

}  // namespace zima
