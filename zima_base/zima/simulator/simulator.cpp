/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/simulator/simulator.h"

#include "zima/common/time.h"
#include "zima/logger/logger.h"

namespace zima {

void Simulator::Run(Chassis::SPtr chassis, TransformManager& tf_manager,
                    const float& duration, NavMap::SPtr map) {
  Transform chassis_tf("", "", 0, 0, 0);
  tf_manager.GetTransform(tf_manager.kOdomFrame_, tf_manager.kRobotFrame_,
                          chassis_tf);

  auto left_speed = chassis->GetWheel(chassis->kLeftWheel_)->TargetSpeed();
  auto right_speed = chassis->GetWheel(chassis->kRightWheel_)->TargetSpeed();
  // ZINFO << "speed: " << left_speed << ", " << right_speed;
  chassis->GetWheel(chassis->kLeftWheel_)->SetCurrentSpeed(left_speed);
  chassis->GetWheel(chassis->kRightWheel_)->SetCurrentSpeed(right_speed);

  auto distance = (left_speed + right_speed) / 2 * duration;

  auto radian_speed = (right_speed - left_speed) / chassis->GetTrackLength();

  auto new_radian = NormalizeRadian(
      DegreesToRadians(chassis->GetGyro(chassis->kGyro_)->GetDegree()) +
      radian_speed * duration);

  auto new_x = chassis_tf.X() + distance * cos(new_radian);
  auto new_y = chassis_tf.Y() + distance * sin(new_radian);

  Transform new_chassis_tf(tf_manager.kOdomFrame_, tf_manager.kRobotFrame_,
                           new_x, new_y, RadiansToDegrees(new_radian));
  tf_manager.UpdateTransform(new_chassis_tf);

  chassis->GetGyro(chassis->kGyro_)
      ->SetDegree(RadiansToDegrees(new_radian), Time::Now());

  MapPoint pose(new_x, new_y, RadiansToDegrees(new_radian));
  {
    // Update for bumper.

    bool left_bumper_triggered = false;
    bool right_bumper_triggered = false;
    float radian;
    auto sensor_map = map->GetSensorLayer();
    ReadLocker read_lock(sensor_map->GetLock());

    double x, y;
    int map_x, map_y;
    CharGridMap2D::DataType value;
    auto left_bumper = chassis->GetBumper(chassis->kLeftBumper_);
    // ZERROR << "left bumper sensor:" << left_bumper->GetTf().DebugString();
    auto right_bumper = chassis->GetBumper(chassis->kRightBumper_);

    radian = DegreesToRadians(left_bumper->GetCoverRangeMinDegree());
    for (; radian < DegreesToRadians(left_bumper->GetCoverRangeMaxDegree());
         radian += DegreesToRadians(10)) {
      Transform::CoordinateTransformationBA(chassis->GetRadius() * cos(radian),
                                            chassis->GetRadius() * sin(radian),
                                            pose.X(), pose.Y(), pose.Radian(),
                                            x, y);
      if (sensor_map->WorldToMap(x, y, map_x, map_y) &&
          sensor_map->GetValue(map_x, map_y, value) &&
          value != map->kUnknown_) {
        ZINFO << "Left bumper.";
        // ZERROR << "Pose" << pose.DebugString();
        // ZERROR << "Angle: (" << RadiansToDegrees(radian) << ")";
        // ZERROR << "Bumper point: (" << x << ", " << y << ")";
        // ZERROR << "map: (" << map_x << ", " << map_y << ")";
        // map->GetSensorLayer()->Print(__FILE__, __FUNCTION__, __LINE__);
        left_bumper_triggered = true;
        break;
      }
    }
    radian = DegreesToRadians(right_bumper->GetCoverRangeMinDegree());
    for (; radian < DegreesToRadians(right_bumper->GetCoverRangeMaxDegree());
         radian += DegreesToRadians(10)) {
      Transform::CoordinateTransformationBA(chassis->GetRadius() * cos(radian),
                                            chassis->GetRadius() * sin(radian),
                                            pose.X(), pose.Y(), pose.Radian(),
                                            x, y);
      if (sensor_map->WorldToMap(x, y, map_x, map_y) &&
          sensor_map->GetValue(map_x, map_y, value) &&
          value != map->kUnknown_) {
        ZINFO << "Right bumper.";
        // ZERROR << "Pose" << pose.DebugString();
        // ZERROR << "Angle: (" << RadiansToDegrees(radian) << ")";
        // ZERROR << "Bumper point: (" << x << ", " << y << ")";
        // ZERROR << "map: (" << map_x << ", " << map_y << ")";
        // map->GetSensorLayer()->Print(__FILE__, __FUNCTION__, __LINE__);
        right_bumper_triggered = true;
        break;
      }
    }
    left_bumper->UpdateRealtimeTriggeredState(left_bumper_triggered);
    right_bumper->UpdateRealtimeTriggeredState(right_bumper_triggered);
  }
  {
    // Update for wall sensor.
    auto sensor_map = map->GetSensorLayer();
    ReadLocker read_lock(sensor_map->GetLock());
    double x, y;
    int map_x, map_y;
    CharGridMap2D::DataType value;
    float check_distance;
    float radian;

    auto left_wall_sensor = chassis->GetWallSensor(chassis->kLeftWallSensor_);
    auto right_wall_sensor = chassis->GetWallSensor(chassis->kRightWallSensor_);
    // left
    auto install_tf = left_wall_sensor->GetTf();
    // ZERROR << "left wall sensor:" << install_tf.DebugString();
    radian = install_tf.Radian();

    for (check_distance = chassis->GetRadius();
         check_distance < chassis->GetRadius() + 0.3; check_distance += 0.005) {
      // Change coordinate to map frame.
      Transform::CoordinateTransformationBA(
          check_distance * cos(radian), check_distance * sin(radian), pose.X(),
          pose.Y(), pose.Radian(), x, y);
      if (sensor_map->WorldToMap(x, y, map_x, map_y) &&
          sensor_map->GetValue(map_x, map_y, value) &&
          value != map->kUnknown_) {
        break;
      }
    }
    // ZERROR << "left distance:" << check_distance - chassis->GetRadius();
    left_wall_sensor->UpdateDistance(check_distance - chassis->GetRadius());

    // right
    install_tf = right_wall_sensor->GetTf();
    radian = install_tf.Radian();

    for (check_distance = chassis->GetRadius();
         check_distance < chassis->GetRadius() + 0.1; check_distance += 0.005) {
      // Change coordinate to map frame.
      Transform::CoordinateTransformationBA(
          check_distance * cos(radian), check_distance * sin(radian), pose.X(),
          pose.Y(), pose.Radian(), x, y);
      // ZERROR << "Check: " << check_distance * cos(radian) << ", "
      //        << check_distance * sin(radian);
      // ZERROR << "Pose" << pose.DebugString();
      // ZERROR << "Angle: (" << RadiansToDegrees(radian) << ")";
      // ZERROR << "Wall point: (" << x << ", " << y << ")";
      if (sensor_map->WorldToMap(x, y, map_x, map_y) &&
          sensor_map->GetValue(map_x, map_y, value) &&
          value != map->kUnknown_) {
        break;
      }
    }
    // ZERROR << "right distance:" << check_distance - chassis->GetRadius();
    right_wall_sensor->UpdateDistance(check_distance - chassis->GetRadius());
  }
}

}  // namespace zima
