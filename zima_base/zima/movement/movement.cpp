/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/movement/movement.h"

namespace zima {

MovementBase::MovementBase()
    : access_(std::make_shared<ReadWriteLock>()),
      name_("Unknown"),
      state_(MotionBase::State::kReady),
      obstacle_degree_(0),
      map_obstacle_degree_(0) {
  step_on_past_path_.store(false);
}

uint32_t MovementBase::GetRestPathPointCount() const { return UINT32_MAX; }

MapPointPath MovementBase::GetRestPath() const {
  return {};
}
bool MovementBase::ExtendPath(const MapPointPath& path) {
  return false;
}

MotionBase::State MovementBase::GetState() const {
  ReadLocker lock(access_);
  return state_;
}

Steps MovementBase::GetSteps() const {
  ReadLocker lock(access_);
  return steps_recorder_.GetPath();
}

bool MovementBase::IsSteppedOnPastPath() const {
  return step_on_past_path_.load();
}

std::string MovementBase::Name() const {
  ReadLocker lock(access_);
  return name_;
}

void MovementBase::Stop() {
  WriteLocker lock(access_);
  if (state_ == MotionBase::State::kReady ||
      state_ == MotionBase::State::kRunning ||
      state_ == MotionBase::State::kNearTarget) {
    state_ = MotionBase::State::kStop;
  }
}

bool EncircleMovementBase::OutOfBound(const MapPoint& world_pose,
                                      const NavMap::SPtr& map,
                                      const MarkerPoints& marker_points) {
  // if (!dead_bound_.Contain(world_pose)) {
  //   return true;
  // }
  // if (!limit_bound_.Contain(world_pose)) {
  //   auto target_bound_obstacle_distance =
  //       NavMap::kRobotCellWidth_2_ * map->GetResolution();
  //   for (auto&& marker_point : marker_points) {
  //     double x, y;
  //     Transform::CoordinateTransformationBA(
  //         marker_point.Point().X(), marker_point.Point().Y(), world_pose.X(),
  //         world_pose.Y(), world_pose.Radian(), x, y);
  //     MapPoint marker_point_in_world(x, y);
  //     if (!obs_bound_.Contain(marker_point_in_world)) {
  //       // Marker should block path in bound.
  //       return true;
  //     }
  //   }
  // }

  auto degree = NormalizeDegree(world_pose.Degree());
  if (world_pose.X() < limit_bound_.GetMin().X() &&
      (degree > 90 || degree < -90)) {
    return true;
  } else if (world_pose.Y() < limit_bound_.GetMin().Y() && degree < 0) {
    return true;
  } else if (world_pose.X() > limit_bound_.GetMax().X() &&
             InRange(degree, 90.0f, -90.0f)) {
    return true;
  } else if (world_pose.Y() > limit_bound_.GetMax().Y() && degree > 0) {
    return true;
  }

  return false;
}

bool EncircleMovementBase::NearBound(const MapPoint& world_pose) {
  const uint8_t kMaxPrintCount = 250;
  auto degree = NormalizeDegree(world_pose.Degree());
  if (world_pose.X() - limit_bound_.GetMin().X() < NavMap::GetResolution() &&
      (degree > 90 || degree < -90)) {
    ++near_bound_print_count_;
    if (near_bound_print_count_ == 1) {
      ZGINFO;
    }
    if (near_bound_print_count_ > kMaxPrintCount) {
      near_bound_print_count_ = 0;
    }
    return true;
  } else if (world_pose.Y() - limit_bound_.GetMin().Y() <
                 NavMap::GetResolution() &&
             degree < 0) {
    ++near_bound_print_count_;
    if (near_bound_print_count_ == 1) {
      ZGINFO;
    }
    if (near_bound_print_count_ > kMaxPrintCount) {
      near_bound_print_count_ = 0;
    }
    return true;
  } else if (limit_bound_.GetMax().X() - world_pose.X() <
                 NavMap::GetResolution() &&
             InRange(degree, 90.0f, -90.0f)) {
    ++near_bound_print_count_;
    if (near_bound_print_count_ == 1) {
      ZGINFO;
    }
    if (near_bound_print_count_ > kMaxPrintCount) {
      near_bound_print_count_ = 0;
    }
    return true;
  } else if (limit_bound_.GetMax().Y() - world_pose.Y() <
                 NavMap::GetResolution() &&
             degree > 0) {
    ++near_bound_print_count_;
    if (near_bound_print_count_ == 1) {
      ZGINFO;
    }
    if (near_bound_print_count_ > kMaxPrintCount) {
      near_bound_print_count_ = 0;
    }
    return true;
  }

  if (near_bound_print_count_ > 0) {
    --near_bound_print_count_;
  }

  return false;
}

}  // namespace zima
