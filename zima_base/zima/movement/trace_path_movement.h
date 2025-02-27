/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_TRACE_PATH_MOVEMENT_H
#define ZIMA_TRACE_PATH_MOVEMENT_H

#include <memory>

#include "zima/common/config.h"
#include "zima/grid_map/nav_map_2d.h"
#include "zima/motion/retreat_motion.h"
#include "zima/motion/rotate_motion.h"
#include "zima/motion/trace_path_motion.h"
#include "zima/movement/movement.h"
#include "zima/robot/chassis.h"

namespace zima {

class TracePathMovement : public MovementBase {
 public:
  class Config {
   public:
    Config() = delete;
    Config(const Chassis::SPtr& chassis, const NavMap::SPtr& map,
           const JsonSPtr& json = nullptr);
    ~Config() = default;

    bool ParseFromJson(const float& chassis_radius, const float& track_length,
                       const float& wheel_max_speed, const JsonSPtr& json);

    bool config_valid_;

    static const std::string kConfigKey_;

    TracePathMotion::Config::SPtr trace_path_motion_config_;
    RetreatMotion::Config::SPtr retreat_motion_config_;
    RotateMotion::Config::SPtr rotate_motion_config_;

    static const std::string kRetreatDistanceKey_;
    float retreat_distance_;
    static const std::string kTracePathObstacleSlowDownDistanceKey_;
    float trace_path_obstacle_slow_down_distance_;
    static const std::string kTracePathObstacleSlowDownSpeedKey_;
    float trace_path_obstacle_slow_down_speed_;
    static const std::string kTracePathObstacleSlowDownSpeedStepKey_;
    float trace_path_obstacle_slow_down_speed_step_;
  };

  TracePathMovement() = delete;
  explicit TracePathMovement(
      const Chassis::SPtr& chassis, const MapPointPath& path,
      const NavMap::SPtr& map,
      const CharGridMap2D::DataType& current_room_index = NavMap::kUnknown_,
      const bool& enable_user_select_area_checking = false,
      const bool& enable_room_value_checking = false,
      const bool& trace_for_edge_path = false);
  ~TracePathMovement();

  enum Stage {
    kStop,
    kTracePath,
    kTurnForTarget,
    kRetreat,
    kFinish,
  };

  enum StopReason {
    kStopForException,
    kStopForObstacle,
    kStopForRoomEdge,
    kStopForForbidArea,
    kStopForUserUnSelectArea,
    kStopForCleaningAreaEdge,
  };

  void Run(const Chassis::SPtr& chassis, const MapPoint& world_pose,
           const MapPoint& odom_pose, const NavMap::SPtr& map) override;

  float GetObstacleDegree() const;
  float GetMapObstacleDegree() const;

  StopReason GetStopReason() const;

  uint32_t GetRestPathPointCount() const override;
  MapPointPath GetRestPath() const override;

  bool ExtendPath(const MapPointPath& path) override;

  bool IsTraceForEdgePath() const { return trace_for_edge_path_; };

 private:
  void SwitchStageAndClearMotion(const Stage& stage);
  bool SwitchToRotateMotion(const Chassis::SPtr& chassis,
                            const MapPoint& odom_pose,
                            const float& rotate_degree);
  bool SwitchToRetreatMotion(const Chassis::SPtr& chassis,
                             const MapPoint& odom_pose);
  bool SwitchToTracePathMotion(const Chassis::SPtr& chassis,
                               const NavMap::SPtr& map,
                               const MapPointPath& path);

  PointCloud::Point GetChassisFrontNearestPointCloudObs(
      const Chassis::SPtr& chassis);

  bool HandleMainLidar(const Chassis::SPtr& chassis, const MapPoint& world_pose,
                       const NavMap::SPtr& map,
                       const PointCloud::Point& nearest_point);
  bool HandleBumper(const Chassis::SPtr& chassis, const MapPoint& world_pose,
                    const NavMap::SPtr& map);
  bool HandleMapValue(const Chassis::SPtr& chassis, const MapPoint& world_pose,
                      const NavMap::SPtr& map);

  Config config_;
  Stage stage_;
  StopReason stop_reason_;
  CharGridMap2D::DataType current_room_index_;
  MapPointPath path_;
  TracePathMotion::SPtr p_trace_path_motion_;
  RetreatMotion::SPtr p_retreat_motion_;
  RotateMotion::SPtr p_rotate_motion_;

  std::vector<float> left_bumper_mark_degrees_;
  const bool enable_user_select_area_checking_;
  const bool enable_room_value_checking_;
  const bool trace_for_edge_path_;
};

}  // namespace zima

#endif  // ZIMA_TRACE_PATH_MOVEMENT_H
