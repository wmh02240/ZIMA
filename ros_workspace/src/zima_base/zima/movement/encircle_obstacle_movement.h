/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_ENCIRCLE_OBSTACLE_MOVEMENT_H
#define ZIMA_ENCIRCLE_OBSTACLE_MOVEMENT_H

#include <memory>

#include "zima/grid_map/nav_map_2d.h"
#include "zima/motion/encircle_obstacle_motion.h"
#include "zima/motion/move_forward_motion.h"
#include "zima/motion/retreat_motion.h"
#include "zima/motion/rotate_motion.h"
#include "zima/movement/movement.h"
#include "zima/robot/chassis.h"

namespace zima {

class EncircleObstacleMovement : public EncircleMovementBase {
 public:
  class Config {
   public:
    Config() = delete;
    Config(const Chassis::SPtr& chassis, const float& map_resolution,
           const JsonSPtr& json = nullptr);
    ~Config() = default;

    bool ParseFromJson(const float& track_length, const float& chassis_radius,
                       const float& wheel_max_speed,
                       const MapPoint& wall_sensor_tf, const JsonSPtr& json);

    bool config_valid_;

    static const std::string kConfigKey_;

    EncircleObstacleMotion::Config::SPtr encircle_obstacle_motion_config_;
    MoveForwardMotion::Config::SPtr move_forward_motion_config_;
    RetreatMotion::Config::SPtr retreat_motion_config_;
    RotateMotion::Config::SPtr rotate_motion_config_;

    static const std::string kMoveForwardDistanceKey_;
    float move_forward_distance_;
    static const std::string kMoveForwardTimeKey_;
    float move_forward_time_;
    static const std::string kRetreatDistanceKey_;
    float retreat_distance_;

    // For tof lidar, near distance measurement will not be that accurate.
    static const std::string kRotateLidarCalDegreeCompensateKey_;
    float rotate_lidar_cal_degree_compensate_;

    DynamicMapPointBound left_point_cloud_select_range_;
    DynamicMapPointBound right_point_cloud_select_range_;
  };

  EncircleObstacleMovement() = delete;
  explicit EncircleObstacleMovement(
      const Chassis::SPtr& chassis, const float& map_resolution,
      const DynamicMapPointBound& limit_bound, const MapPoint& start_world_pose,
      const MapPoint& start_odom_pose, const bool& on_left,
      const float& init_rotate_degree = 0,
      const CharGridMap2D::DataType& current_room_index = NavMap::kUnknown_,
      const bool& turn_with_less_degree = false);
  ~EncircleObstacleMovement();

  enum Stage {
    kStop,
    kEncircleObs,
    kTurnForObs,
    kMoveForward,
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
    kStopForLoopDetected,
  };

  void Run(const Chassis::SPtr& chassis, const MapPoint& world_pose,
           const MapPoint& odom_pose, const NavMap::SPtr& map) override;

  float GetMapObstacleDegree() const;

  StopReason GetStopReason() const;

 private:
  void SwitchStageAndClearMotion(const Stage& state);
  bool SwitchToRotateMotion(const Chassis::SPtr& chassis,
                            const MapPoint& odom_pose,
                            const float& rotate_degree);
  bool SwitchToRetreatMotion(const Chassis::SPtr& chassis,
                             const MapPoint& odom_pose);
  bool SwitchToEncircleObstacleMotion(const Chassis::SPtr& chassis);
  bool SwitchToMoveForwardMotion(const Chassis::SPtr& chassis,
                                 const MapPoint& world_pose,
                                 const float& move_distance,
                                 const float& move_time);

  MapPointPath GetPathFromPointCloud(
      const Chassis::SPtr& chassis, const MapPoint& odom_pose,
      const PointCloud::SPtr& point_cloud_in_chassis_frame,
      const float& distance);

  float GetWallDistanceFromPointCloud(
      const Chassis::SPtr& chassis,
      const PointCloud::SPtr& point_cloud_in_chassis_frame);

  PointCloud::Point GetChassisFrontNearestPointCloudObs(
      const Chassis::SPtr& chassis);
  bool HandleMainLidar(const Chassis::SPtr& chassis, const MapPoint& world_pose,
                       const NavMap::SPtr& map,
                       const PointCloud::Point& nearest_point);
  bool HandleBumper(const Chassis::SPtr& chassis, const MapPoint& world_pose,
                    const NavMap::SPtr& map);
  bool HandleMapValue(const Chassis::SPtr& chassis, const MapPoint& world_pose,
                      const NavMap::SPtr& map);

  float CalculateRotateDegree(const MapPoint& odom_pose,
                              const Chassis::SPtr& chassis);
  float BumperEventRotateDegree(const Chassis::SPtr& chassis);
  float LidarOptimizedRotateDegree(const Chassis::SPtr& chassis,
                                   const float& min_degree);

  Config config_;
  Stage stage_;
  bool on_left_;
  MapPoint start_world_pose_;
  CharGridMap2D::DataType current_room_index_;
  StopReason stop_reason_;
  EncircleObstacleMotion::SPtr p_encircle_obstacle_motion_;
  MoveForwardMotion::SPtr p_move_forward_motion_;
  RetreatMotion::SPtr p_retreat_motion_;
  RotateMotion::SPtr p_rotate_motion_;
  bool turn_with_less_degree_;

  uint32_t last_process_point_cloud_seq_;
};

}  // namespace zima

#endif  // ZIMA_ENCIRCLE_OBSTACLE_MOVEMENT_H
