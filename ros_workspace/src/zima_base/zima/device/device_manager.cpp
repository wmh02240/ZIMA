/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/device/device_manager.h"

#include "zima/common/time.h"
#include "zima/logger/logger.h"

namespace zima {

MergedOdomData::MergedOdomData(const double& timestamp, const MapPoint& pose)
    : timestamp_(timestamp),
      pose_(pose),
      is_velocity_valid_(false),
      velocity_(MapPoint()) {}

MergedOdomData::MergedOdomData(const double& timestamp, const MapPoint& pose,
                               const MapPoint& velocity)
    : timestamp_(timestamp),
      pose_(pose),
      is_velocity_valid_(true),
      velocity_(velocity) {}

std::string MergedOdomData::DebugString() const {
  return "[" + DoubleToString(timestamp_, 4) + "]" + pose_.DebugString() +
         (is_velocity_valid_ ? (" velocity: " + velocity_.DebugString()) : "");
}

DeviceManager::DeviceManager()
    : access_(std::make_shared<ReadWriteLock>()), merged_odom_data_(nullptr){};

void DeviceManager::InitializeBattery(
    const std::vector<Battery::SPtr>& batteries_info) {
  WriteLocker lock(access_);
  for (auto&& battery_info : batteries_info) {
    batteries_.emplace(battery_info->Name(), battery_info);
  }
}

void DeviceManager::InitializeBumper(
    const std::vector<Bumper::SPtr>& bumpers_info) {
  WriteLocker lock(access_);
  for (auto&& bumper_info : bumpers_info) {
    bumpers_.emplace(bumper_info->Name(), bumper_info);
  }
}

void DeviceManager::InitializeButton(
    const std::vector<Button::SPtr>& buttons_info) {
  WriteLocker lock(access_);
  for (auto&& button_info : buttons_info) {
    buttons_.emplace(button_info->Name(), button_info);
  }
}

void DeviceManager::InitializeWheel(
    const std::vector<Wheel::SPtr>& wheels_info) {
  WriteLocker lock(access_);
  for (auto&& wheel_info : wheels_info) {
    wheels_.emplace(wheel_info->Name(), wheel_info);
  }
}

void DeviceManager::InitializeWallSensor(
    const std::vector<WallSensor::SPtr>& wall_sensors_info) {
  WriteLocker lock(access_);
  for (auto&& wall_sensor_info : wall_sensors_info) {
    wall_sensors_.emplace(wall_sensor_info->Name(), wall_sensor_info);
  }
}

void DeviceManager::InitializeGyro(const std::vector<Gyro::SPtr>& gyros_info) {
  WriteLocker lock(access_);
  for (auto&& gyro_info : gyros_info) {
    gyros_.emplace(gyro_info->Name(), gyro_info);
  }
}

void DeviceManager::InitializeLidar(
    const std::vector<Lidar::SPtr>& lidars_info) {
  WriteLocker lock(access_);
  for (auto&& lidar_info : lidars_info) {
    lidars_.emplace(lidar_info->Name(), lidar_info);
  }
}

Battery::SPtr DeviceManager::GetBattery(const std::string& name) const {
  ReadLocker lock(access_);
  if (batteries_.count(name) == 0) {
    ZERROR << "Battery " << name << " was not registered.";
    Battery::Config config;
    config.fully_charged_state_voltage_ = 0;
    config.low_state_voltage_ = 0;
    config.dangerous_state_voltage_ = 0;
    static Battery::SPtr null_battery(
        new Battery(Battery::kNullName_, config));
    return null_battery;
  }
  return batteries_.at(name);
}

Bumper::SPtr DeviceManager::GetBumper(const std::string& name) const {
  ReadLocker lock(access_);
  if (bumpers_.count(name) == 0) {
    ZERROR << "Bumper " << name << " was not registered.";
    Bumper::Config config(0, 0, 0);
    static Bumper::SPtr null_bumper(new Bumper(Bumper::kNullName_, config));
    return null_bumper;
  }
  return bumpers_.at(name);
}

Button::SPtr DeviceManager::GetButton(const std::string& name) const {
  ReadLocker lock(access_);
  if (buttons_.count(name) == 0) {
    ZERROR << "Button " << name << " was not registered.";
    Button::Config config;
    static Button::SPtr null_button(new Button(Button::kNullName_, config));
    return null_button;
  }
  return buttons_.at(name);
}

Wheel::SPtr DeviceManager::GetWheel(const std::string& name) const {
  ReadLocker lock(access_);
  if (wheels_.count(name) == 0) {
    ZERROR << "Wheel " << name << " was not registered.";
    Wheel::Config config;
    static Wheel::SPtr null_wheel(new Wheel(Wheel::kNullName_, config));
    return null_wheel;
  }
  return wheels_.at(name);
}

WallSensor::SPtr DeviceManager::GetWallSensor(const std::string& name) const {
  ReadLocker lock(access_);
  if (wall_sensors_.count(name) == 0) {
    ZERROR << "Wall sensor " << name << " was not registered.";
    WallSensor::Config config(0);
    static WallSensor::SPtr null_wall_sensor(
        new WallSensor(WallSensor::kNullName_, config));
    return null_wall_sensor;
  }
  return wall_sensors_.at(name);
}

Gyro::SPtr DeviceManager::GetGyro(const std::string& name) const {
  ReadLocker lock(access_);
  if (gyros_.count(name) == 0) {
    ZERROR << "Gyro " << name << " was not registered.";
    Gyro::Config config;
    static Gyro::SPtr null_gyro(new Gyro(Gyro::kNullName_, config));
    return null_gyro;
  }
  return gyros_.at(name);
}

Lidar::SPtr DeviceManager::GetLidar(const std::string& name) const {
  ReadLocker lock(access_);
  if (lidars_.count(name) == 0) {
    ZERROR << "Lidar " << name << " was not registered.";
    Lidar::Config config;
    static Lidar::SPtr null_lidar(new Lidar(Lidar::kNullName_, config));
    return null_lidar;
  }
  return lidars_.at(name);
}

bool DeviceManager::IsDeviceRegistered(const std::string& name) const {
  ReadLocker lock(access_);
  if (batteries_.count(name) != 0 || bumpers_.count(name) != 0 ||
      buttons_.count(name) != 0 || wheels_.count(name) != 0 ||
      wall_sensors_.count(name) != 0 || gyros_.count(name) != 0 ||
      lidars_.count(name) != 0) {
    return true;
  }
  return false;
}

MergedOdomData::SPtr DeviceManager::GetMergedOdomData() const {
  ReadLocker lock(access_);
  return merged_odom_data_;
}

void DeviceManager::SetMergedOdomData(const MergedOdomData::SPtr& data) {
  WriteLocker lock(access_);
  merged_odom_data_ = data;
}

double DeviceManager::GetMergedOdomDataTimeStamp() const {
  ReadLocker lock(access_);
  if (merged_odom_data_ == nullptr) {
    return 0;
  }
  return merged_odom_data_->GetTimeStamp();
}

bool DeviceManager::CheckMergedOdomDataFresh(const double& limit) const {
  ReadLocker lock(access_);
  if (merged_odom_data_ == nullptr) {
    return 0;
  }
  return (Time::Now() - merged_odom_data_->GetTimeStamp()) < limit;
}

}  // namespace zima
