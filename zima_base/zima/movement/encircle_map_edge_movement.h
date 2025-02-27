/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_ENCIRCLE_MAP_EDGE_MOVEMENT_H
#define ZIMA_ENCIRCLE_MAP_EDGE_MOVEMENT_H

#include <memory>

#include "zima/grid_map/nav_map_2d.h"
#include "zima/motion/encircle_map_edge_motion.h"
#include "zima/motion/move_forward_motion.h"
#include "zima/motion/retreat_motion.h"
#include "zima/motion/rotate_motion.h"
#include "zima/movement/movement.h"
#include "zima/robot/chassis.h"

namespace zima {

class EncircleMapEdgeMovement : public EncircleMovementBase {
 public:
  class Config {
   public:
    Config() = delete;
    Config(const Chassis::SPtr& chassis, const float& map_resolution,
           const JsonSPtr& json = nullptr);
    ~Config() = default;

    bool ParseFromJson(const float& track_length, const float& wheel_max_speed,
                       const float& map_resolution, const JsonSPtr& json);

    bool config_valid_;

    static const std::string kConfigKey_;

    EncircleMapEdgeMotion::Config::SPtr encircle_map_edge_motion_config_;
    RetreatMotion::Config::SPtr retreat_motion_config_;
    RotateMotion::Config::SPtr rotate_motion_config_;

    static const std::string kRetreatDistanceKey_;
    float retreat_distance_;
  };

  EncircleMapEdgeMovement() = delete;
  explicit EncircleMapEdgeMovement(
      const Chassis::SPtr& chassis, const float& map_resolution,
      const DynamicMapPointBound& limit_bound,
      const CharGridMap2D::DataType& current_room_index,
      const MapPoint& start_world_pose, const MapPoint& start_odom_pose,
      const bool& on_left, const float& init_rotate_degree = 0);
  ~EncircleMapEdgeMovement() = default;

  enum Stage {
    kStop,
    kEncircleMapEdge,
    kTurnForMapEdge,
    kRetreat,
    kFinish,
  };

  void Run(const Chassis::SPtr& chassis, const MapPoint& world_pose,
           const MapPoint& odom_pose, const NavMap::SPtr& map) override;

  float GetObstacleDegree() const;

 private:
  void SwitchStageAndClearMotion(const Stage& stage);
  bool SwitchToRotateMotion(const Chassis::SPtr& chassis,
                            const MapPoint& odom_pose,
                            const float& rotate_degree);
  bool SwitchToRetreatMotion(const Chassis::SPtr& chassis,
                             const MapPoint& odom_pose);
  bool SwitchToEncircleRoomEdgeMotion(const Chassis::SPtr& chassis);

  bool HandleBumper(const Chassis::SPtr& chassis, const MapPoint& world_pose,
                    const NavMap::SPtr& map);

  bool CheckMapEdgeDistance(const Chassis::SPtr& chassis, const float& radian,
                            const MapPoint& world_pose,
                            const NavMap::SPtr& map, float& distance);

  Config config_;
  Stage stage_;
  bool on_left_;
  MapPoint start_world_pose_;
  CharGridMap2D::DataType current_room_index_;
  EncircleMapEdgeMotion::SPtr p_encircle_map_edge_motion_;
  RetreatMotion::SPtr p_retreat_motion_;
  RotateMotion::SPtr p_rotate_motion_;
};

}  // namespace zima

#endif  // ZIMA_ENCIRCLE_MAP_EDGE_MOVEMENT_H
