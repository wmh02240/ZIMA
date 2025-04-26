/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_DEVICE_MANAGER_H
#define ZIMA_DEVICE_MANAGER_H

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "zima/common/macro.h"
#include "zima/device/battery.h"
#include "zima/device/bumper.h"
#include "zima/device/button.h"
#include "zima/device/gyro.h"
#include "zima/device/lidar.h"
#include "zima/device/wall_sensor.h"
#include "zima/device/wheel.h"

namespace zima {

class MergedOdomData {
 public:
  MergedOdomData() = delete;
  MergedOdomData(const double& timestamp, const MapPoint& pose);
  MergedOdomData(const double& timestamp, const MapPoint& pose,
               const MapPoint& velocity);
  ~MergedOdomData() = default;

  using SPtr = std::shared_ptr<MergedOdomData>;

  double GetTimeStamp() const { return timestamp_; }
  MapPoint GetPose() const { return pose_; }
  bool IsVelocityValid() const { return is_velocity_valid_; }
  MapPoint GetVelocity() const { return velocity_; }

  std::string DebugString() const;

 private:
  const double timestamp_;
  const MapPoint pose_;
  const bool is_velocity_valid_;
  // Velocity is for per second.
  const MapPoint velocity_;
};

class DeviceManager {
 public:
  using SPtr = std::shared_ptr<DeviceManager>;

  Battery::SPtr GetBattery(const std::string& name) const;
  Bumper::SPtr GetBumper(const std::string& name) const;
  Button::SPtr GetButton(const std::string& name) const;
  Wheel::SPtr GetWheel(const std::string& name) const;
  WallSensor::SPtr GetWallSensor(const std::string& name) const;
  Gyro::SPtr GetGyro(const std::string& name) const;
  Lidar::SPtr GetLidar(const std::string& name) const;

  bool IsDeviceRegistered(const std::string& name) const;

  DECLARE_DATA_GET_SET(MergedOdomData::SPtr, MergedOdomData);
  double GetMergedOdomDataTimeStamp() const;
  bool CheckMergedOdomDataFresh(const double& limit) const;

 protected:
  DeviceManager();
  ~DeviceManager() = default;

  void InitializeBattery(const std::vector<Battery::SPtr>& batteries_info);
  void InitializeBumper(const std::vector<Bumper::SPtr>& bumpers_info);
  void InitializeButton(const std::vector<Button::SPtr>& buttons_info);
  void InitializeWheel(const std::vector<Wheel::SPtr>& wheels_info);
  void InitializeWallSensor(
      const std::vector<WallSensor::SPtr>& wall_sensors_info);
  void InitializeGyro(const std::vector<Gyro::SPtr>& gyros_info);
  void InitializeLidar(const std::vector<Lidar::SPtr>& lidars_info);

  ReadWriteLock::SPtr access_;

  std::map<std::string, Battery::SPtr> batteries_;
  std::map<std::string, Bumper::SPtr> bumpers_;
  std::map<std::string, Button::SPtr> buttons_;
  std::map<std::string, Wheel::SPtr> wheels_;
  std::map<std::string, WallSensor::SPtr> wall_sensors_;
  std::map<std::string, Gyro::SPtr> gyros_;
  std::map<std::string, Lidar::SPtr> lidars_;

  MergedOdomData::SPtr merged_odom_data_;
};

}  // namespace zima

#endif  // ZIMA_DEVICE_MANAGER_H
