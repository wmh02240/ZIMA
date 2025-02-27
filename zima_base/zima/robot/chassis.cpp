/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/robot/chassis.h"

#include <memory>

#include "zima/grid_map/nav_map_2d.h"

namespace zima {

Chassis::Config::Config() : Config(nullptr) {}

Chassis::Config::Config(const JsonSPtr& json) : config_valid_(true) {
  // Load default setting.
  device_config_ = nullptr;
  track_length_ = 0;
  radius_ = 0;
  encircle_obstacle_on_left_ = false;

  // Override if json config is provided.
  if (json != nullptr) {
    // ZINFO << "Load config from json.";
    config_valid_ = ParseFromJson(json);
  } else {
    JsonSPtr config;
    if (GlobalJsonConfig::Instance()->GetGlobalConfigObject(kChassisConfigKey_,
                                                            config)) {
      // ZINFO << "Load config from global json config.";
      config_valid_ = ParseFromJson(config);
    } else {
      ZERROR;
      config_valid_ = false;
    }
  }
}

bool Chassis::Config::ParseFromJson(const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }

  // Override.
  device_config_.reset(new Json());
  if (!JsonHelper::GetObject(*json, kDeviceConfigKey_,
                            *device_config_)) {
    ZERROR << "Config " << kDeviceConfigKey_ << " not found.";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kTrackLengthKey_, track_length_)) {
    ZERROR << "Config " << kTrackLengthKey_ << " not found.";
    return false;
  }

  if (!JsonHelper::GetFloat(*json, kRadiusKey_, radius_)) {
    ZERROR << "Config " << kRadiusKey_ << " not found.";
    return false;
  }

  if (!JsonHelper::GetBoolean(*json, kEncircleObstacleOnLeftKey_,
                              encircle_obstacle_on_left_)) {
    ZERROR << "Config " << kEncircleObstacleOnLeftKey_ << " not found.";
    return false;
  }

  return true;
}

// Chassis::Chassis() : config_(nullptr) {}

Chassis::Chassis(const Config& config)
    : config_(config), ready_(false), stall_test_running_(false) {}

void Chassis::Initialize() {
  ZGINFO;
  if (!config_.config_valid_) {
    ZERROR << "Config not valid.";
    return;
  }

  track_length_ = config_.track_length_;
  radius_ = config_.radius_;
  encircle_obstacle_on_left_ = config_.encircle_obstacle_on_left_;

  std::vector<Wheel::SPtr> wheel_infos;
  std::vector<Bumper::SPtr> bumper_infos;
  std::vector<WallSensor::SPtr> wall_sensor_infos;
  std::vector<Gyro::SPtr> gyro_infos;
  std::vector<Lidar::SPtr> lidar_infos;
  std::vector<Battery::SPtr> battery_infos;
  for (auto&& item : config_.device_config_->items()) {
    if (!item.value().is_object()) {
      ZERROR << "Invalid config:\n"
             << item.key() << ": " << item.value().dump();
      continue;
    }
    if (item.key() == kLeftWheel_ || item.key() == kRightWheel_) {
      Wheel::Config config(std::make_shared<Json>(item.value()));
      if (!config.config_valid_) {
        ZERROR << "Invalid config:\n"
               << item.key() << ": " << item.value().dump();
        continue;
      }
      wheel_infos.emplace_back(std::make_shared<Wheel>(item.key(), config));
    }
    if (item.key() == kLeftBumper_ || item.key() == kCenterBumper_ ||
        item.key() == kRightBumper_) {
      Bumper::Config config(NavMap::GetResolution(), NavMap::kRobotCellWidth_2_,
                            radius_, std::make_shared<Json>(item.value()));
      if (!config.config_valid_) {
        ZERROR << "Invalid config:\n"
               << item.key() << ": " << item.value().dump();
        continue;
      }
      bumper_infos.emplace_back(std::make_shared<Bumper>(item.key(), config));
    }
    if (item.key() == kLeftWallSensor_ || item.key() == kRightWallSensor_) {
      WallSensor::Config config(radius_, std::make_shared<Json>(item.value()));
      if (!config.config_valid_) {
        ZERROR << "Invalid config:\n"
               << item.key() << ": " << item.value().dump();
        continue;
      }
      wall_sensor_infos.emplace_back(
          std::make_shared<WallSensor>(item.key(), config));
    }
    if (item.key() == kGyro_) {
      Gyro::Config config(std::make_shared<Json>(item.value()));
      if (!config.config_valid_) {
        ZERROR << "Invalid config:\n"
               << item.key() << ": " << item.value().dump();
        continue;
      }
      gyro_infos.emplace_back(std::make_shared<Gyro>(item.key(), config));
    }
    if (item.key() == kLidar_) {
      Lidar::Config config(std::make_shared<Json>(item.value()));
      if (!config.config_valid_) {
        ZERROR << "Invalid config:\n"
               << item.key() << ": " << item.value().dump();
        continue;
      }
      lidar_infos.emplace_back(std::make_shared<Lidar>(item.key(), config));
    }
    if (item.key() == kBattery_) {
      Battery::Config config(std::make_shared<Json>(item.value()));
      if (!config.config_valid_) {
        ZERROR << "Invalid config:\n"
               << item.key() << ": " << item.value().dump();
        continue;
      }
      battery_infos.emplace_back(std::make_shared<Battery>(item.key(), config));
    }
  }
  InitializeWheel(wheel_infos);
  InitializeBumper(bumper_infos);
  InitializeWallSensor(wall_sensor_infos);
  InitializeGyro(gyro_infos);
  InitializeLidar(lidar_infos);
  InitializeBattery(battery_infos);
}

bool Chassis::GetBumperEvent(std::vector<std::string>& bumper_names) {
  bumper_names.clear();
  bool ret = false;
  for (auto&& bumper_pair : bumpers_) {
    if (bumper_pair.second->TriggeredEvent()) {
      bumper_names.emplace_back(bumper_pair.second->Name());
      ret = true;
    }
  }

  return ret;
}

bool Chassis::ClearBumperEvent() {
  for (auto&& bumper_pair : bumpers_) {
    bumper_pair.second->ClearEvent();
  }
  return true;
}

void Chassis::StopWheels() {
  GetWheel(kLeftWheel_)->SetTargetSpeed(0);
  GetWheel(kRightWheel_)->SetTargetSpeed(0);
}

void Chassis::EnableStallTest() {
  ZINFO << "Enable stall test.";
  stall_test_running_.store(true);
}

void Chassis::DisableStallTest() {
  ZINFO << "Disable stall test.";
  stall_test_running_.store(false);
}

}  // namespace zima
