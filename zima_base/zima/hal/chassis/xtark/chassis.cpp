/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/hal/chassis/xtark/chassis.h"

#include "zima/grid_map/nav_map_2d.h"

namespace zima {

XTarkChassisSerial::~XTarkChassisSerial() {
  ZINFO;
  SetVelocityCmd(0, 0, 0);
  PackWriteFrame();
  Serial::BytesFrames write_frames;
  if (GetWriteFrames(write_frames)) {
    for (auto& frame : write_frames) {
      Write(frame);
    }
  }
}

bool XTarkChassisSerial::ParseFrame(const uint8_t& recv_byte) {
  WriteLocker lock(data_access_);
  return protocal_parser_.ParseReadFrame(recv_byte, cache_bytes_,
                                         valid_frames_data_);
};

bool XTarkChassisSerial::ParseReadData(const Serial::BytesFrame& valid_frame) {
  return data_parser_.ParseReadData(valid_frame, chassis_data_);
}

bool XTarkChassisSerial::PackWriteFrame() {
  static bool sent_pid = false;
  if (!sent_pid) {
    Serial::BytesFrame data;
    data.emplace_back(XTarkChassisDataParser::kCPR2PIDCommand_);
    const uint16_t p = 8400;
    const uint16_t d = 15600;
    data.emplace_back((int16_t)(p) >> 8);
    data.emplace_back((int16_t)(p));
    data.emplace_back((int16_t)(d) >> 8);
    data.emplace_back((int16_t)(d));

    CacheWriteFrames(protocal_parser_.PackWriteFrame(data));
    sent_pid = true;
    ZINFO << "send p: " << p << ", d: " << d;
    return true;
  }
  WriteLocker lock(access_);
  CacheWriteFrames(protocal_parser_.PackWriteFrame(
      data_parser_.PackVelocityCommandFrameData(velocity_command_)));
  return true;
}

void XTarkChassisSerial::SetVelocityCmdByWheel(const float& left_wheel_speed,
                                               const float& right_wheel_speed,
                                               const float& track_length) {
  WriteLocker lock(access_);
  velocity_command_.SetVelocityCmdByWheel(
      left_wheel_speed, right_wheel_speed, track_length);
}

void XTarkChassisSerial::SetVelocityCmd(const float& linear_x,
                                        const float& linear_y,
                                        const float& angular_z) {
  WriteLocker lock(access_);
  velocity_command_.SetVelocityCmd(linear_x, linear_y, angular_z);
}

XTarkChassisDataParser::ImuData XTarkChassisSerial::GetImuData() {
  ReadLocker lock(access_);
  return chassis_data_.imu_data_;
}
XTarkChassisDataParser::PoseData XTarkChassisSerial::GetOdomData() {
  ReadLocker lock(access_);
  return chassis_data_.pose_data_;
}
XTarkChassisDataParser::VelocityData XTarkChassisSerial::GetVelocityData() {
  ReadLocker lock(access_);
  return chassis_data_.velocity_data_;
}
void XTarkChassisSerial::GetVelocityDataForWheel(float& left_wheel_speed,
                                                 float& right_wheel_speed,
                                                 const float& track_length) {
  auto velocity_data = GetVelocityData();
  auto tmp = velocity_data.angular_z_ * track_length / 2;
  left_wheel_speed = velocity_data.linear_x_ - tmp;
  right_wheel_speed = velocity_data.linear_x_ + tmp;
}
XTarkChassisDataParser::BatteryData XTarkChassisSerial::GetBatteryData() {
  ReadLocker lock(access_);
  return chassis_data_.battery_data_;
}

XTarkChassis::XTarkChassis() : Chassis(Chassis::Config()) {}

void XTarkChassis::Initialize() {
  ZINFO;
  // track_length_ = 0.172;
  // radius_ = 0.16;
  // std::make_shared<Battery>(kBattery_, 16.5, 14, 13.5);

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

}  // namespace zima
