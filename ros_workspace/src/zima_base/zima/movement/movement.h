/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_MOVEMENT_H
#define ZIMA_MOVEMENT_H

#include "zima/common/debug.h"
#include "zima/common/steps_recorder.h"
#include "zima/grid_map/nav_map_2d.h"
#include "zima/motion/motion.h"
#include "zima/robot/chassis.h"

namespace zima {

class MovementBase : public DebugBase {
 public:
  MovementBase();
  virtual ~MovementBase() = default;

  using UPtr = std::unique_ptr<MovementBase>;

  virtual void Run(const Chassis::SPtr& chassis, const MapPoint& world_pose,
                   const MapPoint& odom_pose, const NavMap::SPtr& map) = 0;

  virtual uint32_t GetRestPathPointCount() const;
  virtual MapPointPath GetRestPath() const;
  virtual bool ExtendPath(const MapPointPath& path);

  MotionBase::State GetState() const;
  Steps GetSteps() const;
  bool IsSteppedOnPastPath() const;

  std::string Name() const;

  void Stop();

 protected:
  ReadWriteLock::SPtr access_;
  std::string name_;
  MotionBase::State state_;
  StepsRecorder steps_recorder_;
  atomic_bool step_on_past_path_;

  float obstacle_degree_;
  float map_obstacle_degree_;

  bool chassis_has_center_bumper_;
};

class EncircleMovementBase : public MovementBase {
 public:
  EncircleMovementBase() = delete;
  explicit EncircleMovementBase(const DynamicMapPointBound& limit_bound,
                                const float& map_resolution)
      : limit_bound_(limit_bound),
        dead_bound_(limit_bound),
        obs_bound_(INT_MAX, INT_MIN, INT_MAX, INT_MIN),
        near_bound_print_count_(0) {
    dead_bound_.Expend(limit_bound_.GetMin() -
                       MapPoint(map_resolution, map_resolution));
    dead_bound_.Expend(limit_bound_.GetMax() +
                       MapPoint(map_resolution, map_resolution));
    obs_bound_.Expend(limit_bound_.GetMin() +
                      MapPoint(NavMap::kRobotCellWidth_2_ * map_resolution,
                               NavMap::kRobotCellWidth_2_ * map_resolution));
    obs_bound_.Expend(limit_bound_.GetMax() -
                      MapPoint(NavMap::kRobotCellWidth_2_ * map_resolution,
                               NavMap::kRobotCellWidth_2_ * map_resolution));
  }
  ~EncircleMovementBase() = default;

 protected:
  bool OutOfBound(const MapPoint& world_pose, const NavMap::SPtr& map,
                  const MarkerPoints& marker_points);
  bool NearBound(const MapPoint& world_pose);

  DynamicMapPointBound limit_bound_;
  DynamicMapPointBound dead_bound_;
  DynamicMapPointBound obs_bound_;

  uint8_t near_bound_print_count_;
};

}  // namespace zima

#endif  // ZIMA_MOVEMENT_H
