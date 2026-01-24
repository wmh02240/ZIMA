/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_HAL_CHASSIS_XTARK_PROTOCAL_H_
#define ZIMA_HAL_CHASSIS_XTARK_PROTOCAL_H_

// #include "zima/common/lock.h"
// #include "zima/common/util.h"
#include "zima/hal/io/serial.h"

namespace zima {

class XTarkChassisSerialProtocalParser {
 public:
  XTarkChassisSerialProtocalParser()
      : progress_(kForHeader1), expect_data_len_(0), cache_data_len_(0){};
  ~XTarkChassisSerialProtocalParser() = default;

  void InitForReadBuffer(Serial::BytesFrame& frame_buf);
  bool ParseReadFrame(const uint8_t& recv_byte, Serial::BytesFrame& frame_buf,
                      Serial::BytesFrames& valid_frames);
  Serial::BytesFrame PackWriteFrame(const Serial::BytesFrame& frame_data);

 private:
  static const uint8_t kHeader1_;
  static const uint8_t kHeader2_;
  static const uint8_t kFixProtocalPrefixLen_;
  static const uint8_t kFixProtocalSuffixLen_;

  static uint32_t kBufMaxSize_;

  // Data contains frame type and data value.
  enum Progress {
    kForHeader1,
    kForHeader2,
    kForFrameLen,
    kForFrameData,
    kForCheckSum,
  };

  Progress progress_;
  uint8_t expect_data_len_;
  uint8_t cache_data_len_;
};

class XTarkChassisDataParser {
 public:
  XTarkChassisDataParser() = default;
  ~XTarkChassisDataParser() = default;

  class DataBase {
   public:
    virtual std::string DebugString() = 0;
  };

  class ImuData : public DataBase {
   public:
    std::string DebugString() override;

    float acc_x_{0};
    float acc_y_{0};
    float acc_z_{0};

    float gyro_x_{0};
    float gyro_y_{0};
    float gyro_z_{0};
  };

  class ImuOrientationData : public DataBase {
   public:
    std::string DebugString() override;

    float w_{0};
    float x_{0};
    float y_{0};
    float z_{0};
  };

  //机器人速度数据结构体
  class VelocityData : public DataBase {
   public:
    std::string DebugString() override;
    void SetVelocityCmdByWheel(const float& left_wheel_speed,
                               const float& right_wheel_speed,
                               const float& track_length);
    void SetVelocityCmd(const float& linear_x, const float& linear_y,
                        const float& angular_z);

    float linear_x_{0};
    float linear_y_{0};
    float angular_z_{0};
  };

  //机器人位置数据结构体
  class PoseData : public DataBase {
   public:
    std::string DebugString() override;

    float pose_x_{0};
    float pose_y_{0};
    float angular_z_{0};
  };

  class BatteryData : public DataBase {
   public:
    std::string DebugString() override;

    double battery_voltage_{0};
  };

  class ChassisData {
   public:
    ImuData imu_data_;
    ImuOrientationData imu_orientation_data_;
    VelocityData velocity_data_;
    BatteryData battery_data_;
    PoseData pose_data_;
  };

  static uint32_t kCPR2StatusFrameSize_;
  static uint32_t kFrameMaxSize_;

  bool ParseReadData(const Serial::BytesFrame& valid_frame,
                     ChassisData& chassis_data);

  Serial::BytesFrame PackVelocityCommandFrameData(const VelocityData& velocity_data);

  static const uint8_t kCPR2Status_;
  static const uint8_t kCPR2PIDCommand_;
  static const uint8_t kCPR2VelocityCommand_;
  static const uint8_t kCPR2ImuCommand_;
};

}  // namespace zima

#endif  // ZIMA_HAL_CHASSIS_XTARK_PROTOCAL_H_
