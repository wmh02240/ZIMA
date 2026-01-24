/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_HAL_CHASSIS_XTARK_CHASSIS_H_
#define ZIMA_HAL_CHASSIS_XTARK_CHASSIS_H_

#include "zima/hal/chassis/xtark/protocal.h"
#include "zima/robot/chassis.h"
#include "zima/hal/io/serial.h"

namespace zima {

class XTarkChassisSerial : public Serial {
 public:
  XTarkChassisSerial() = delete;
  XTarkChassisSerial(const std::string& serial_port, const int& baud)
      : Serial(serial_port, baud), access_(std::make_shared<ReadWriteLock>()){
    velocity_command_.linear_x_ = 0;
    velocity_command_.linear_y_ = 0;
    velocity_command_.angular_z_ = 0;
  };
  ~XTarkChassisSerial();

  bool ParseFrame(const uint8_t& recv_byte) override;
  bool ParseReadData(const Serial::BytesFrame& valid_frame);
  bool PackWriteFrame();

  void SetVelocityCmdByWheel(const float& left_wheel_speed,
                             const float& right_wheel_speed,
                             const float& track_length);
  void SetVelocityCmd(const float& linear_x, const float& linear_y,
                      const float& angular_z);

  XTarkChassisDataParser::ImuData GetImuData();
  XTarkChassisDataParser::PoseData GetOdomData();
  XTarkChassisDataParser::VelocityData GetVelocityData();
  void GetVelocityDataForWheel(float& left_wheel_speed,
                               float& right_wheel_speed,
                               const float& track_length);
  XTarkChassisDataParser::BatteryData GetBatteryData();

 private:
  XTarkChassisSerialProtocalParser protocal_parser_;
  XTarkChassisDataParser data_parser_;

  ReadWriteLock::SPtr access_;
  // Read data
  XTarkChassisDataParser::ChassisData chassis_data_;
  // Write data
  XTarkChassisDataParser::VelocityData velocity_command_;
};


class XTarkChassis : public Chassis {
 public:
  XTarkChassis();
  ~XTarkChassis() = default;

  void Initialize() override;
};


}  // namespace zima

#endif  // ZIMA_HAL_CHASSIS_XTARK_CHASSIS_H_
