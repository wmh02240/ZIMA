/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/hal/chassis/xtark/protocal.h"

#include "zima/common/maths.h"
#include "zima/common/util.h"
#include "zima/logger/logger.h"

namespace zima {

const uint8_t XTarkChassisSerialProtocalParser::kHeader1_ = 0xAA;
const uint8_t XTarkChassisSerialProtocalParser::kHeader2_ = 0x55;
const uint8_t XTarkChassisSerialProtocalParser::kFixProtocalPrefixLen_ = 3;
const uint8_t XTarkChassisSerialProtocalParser::kFixProtocalSuffixLen_ = 1;
uint32_t XTarkChassisSerialProtocalParser::kBufMaxSize_ = 60;

const uint8_t XTarkChassisDataParser::kCPR2Status_ = 0x10;
const uint8_t XTarkChassisDataParser::kCPR2PIDCommand_ = 0x30;
const uint8_t XTarkChassisDataParser::kCPR2VelocityCommand_ = 0x50;
const uint8_t XTarkChassisDataParser::kCPR2ImuCommand_ = 0x51;
uint32_t XTarkChassisDataParser::kCPR2StatusFrameSize_ = 25;
// Should be max size of all type.
uint32_t XTarkChassisDataParser::kFrameMaxSize_ = 25;

void XTarkChassisSerialProtocalParser::InitForReadBuffer(
    Serial::BytesFrame& frame_buf) {
  frame_buf.clear();
  frame_buf.resize(kBufMaxSize_, 0);
  ZINFO << "Frame buffer size " << frame_buf.size();
}

bool XTarkChassisSerialProtocalParser::ParseReadFrame(
    const uint8_t& recv_byte, Serial::BytesFrame& frame_buf,
    Serial::BytesFrames& valid_frames_data) {
  if (frame_buf.size() < kBufMaxSize_) {
    InitForReadBuffer(frame_buf);
  }

  switch (progress_) {
    case kForHeader1: {
      cache_data_len_ = 0;
      if (recv_byte == kHeader1_) {
        // ZINFO << "Match header 1.";
        frame_buf.at(cache_data_len_++) = recv_byte;
        progress_ = kForHeader2;
      }
      break;
    }
    case kForHeader2: {
      if (recv_byte == kHeader2_) {
        // ZINFO << "Match header 2.";
        frame_buf.at(cache_data_len_++) = recv_byte;
        progress_ = kForFrameLen;
      } else {
        progress_ = kForHeader1;
      }
      break;
    }
    case kForFrameLen: {
      // Expect data length means length of WHOLE pack.
      expect_data_len_ = recv_byte;
      if (expect_data_len_ > kBufMaxSize_) {
        ZWARN << "Insufficient buf size.";
        kBufMaxSize_ = expect_data_len_;
        progress_ = kForHeader1;
        break;
      }
      // ZINFO << "Expect data len: " << expect_data_len_;
      frame_buf.at(cache_data_len_++) = recv_byte;
      progress_ = kForFrameData;
      break;
    }
    case kForFrameData: {
      frame_buf.at(cache_data_len_++) = recv_byte;
      if (cache_data_len_ >= expect_data_len_ - kFixProtocalSuffixLen_) {
        progress_ = kForCheckSum;
      }
      break;
    }
    case kForCheckSum: {
      frame_buf.at(cache_data_len_) = recv_byte;
      uint32_t check_sum = 0;
      for (auto i = 0; i < expect_data_len_ - kFixProtocalSuffixLen_; i++) {
        check_sum += frame_buf.at(i);
      }
      // ZINFO << "Cal checksum: " << UintToHexString(check_sum & 0xff);
      // ZINFO << "Receive checksum: " << UintToHexString(recv_byte);
      auto pack_data_end = expect_data_len_ - kFixProtocalSuffixLen_;
      if ((check_sum & 0xff) == recv_byte) {
        // Valid frame.
        valid_frames_data.emplace_back(Serial::BytesFrame(
            frame_buf.begin() + kFixProtocalPrefixLen_,
            frame_buf.begin() + pack_data_end));
      } else {
        ZWARN << "Cal checksum: " << UintToHexString(check_sum & 0xff);
        ZWARN << "Receive checksum: " << UintToHexString(recv_byte);
        ZWARN << Serial::DebugString(
            Serial::BytesFrame(frame_buf.begin(),
                               frame_buf.begin() + expect_data_len_));
      }
      progress_ = kForHeader1;
      break;
    }
    default: {
      ZERROR;
      progress_ = kForHeader1;
      break;
    }
  }

  return true;
}

Serial::BytesFrame XTarkChassisSerialProtocalParser::PackWriteFrame(
    const Serial::BytesFrame& frame_data) {
  if (frame_data.size() >
      UINT8_MAX - kFixProtocalPrefixLen_ - kFixProtocalSuffixLen_) {
    ZERROR << "Invalid frame len: " << frame_data.size() << "Max support len: "
           << UINT8_MAX - kFixProtocalPrefixLen_ - kFixProtocalSuffixLen_;
    return {};
  }
  Serial::BytesFrame frame;
  frame.emplace_back(kHeader1_);
  frame.emplace_back(kHeader2_);
  frame.emplace_back(frame_data.size() + kFixProtocalPrefixLen_ +
                     kFixProtocalSuffixLen_);
  frame.insert(frame.end(), frame_data.begin(), frame_data.end());

  uint32_t check_sum = 0;
  for (auto&& byte : frame) {
    check_sum += byte;
  }
  frame.emplace_back(check_sum & 0xFF);
  return frame;
}

std::string XTarkChassisDataParser::ImuData::DebugString() {
  return "acc_x: " + FloatToString(acc_x_, 4) + ", " +
         "acc_y: " + FloatToString(acc_y_, 4) + ", " +
         "acc_z: " + FloatToString(acc_z_, 4) + ", " +
         "gyro_x: " + FloatToString(gyro_x_, 4) + ", " +
         "gyro_y: " + FloatToString(gyro_y_, 4) + ", " +
         "gyro_z: " + FloatToString(gyro_z_, 4);
}

std::string XTarkChassisDataParser::ImuOrientationData::DebugString() {
  return "w: " + FloatToString(w_, 4) + ", " + "x: " + FloatToString(x_, 4) +
         ", " + "y: " + FloatToString(y_, 4) + ", " +
         "z: " + FloatToString(z_, 4);
}

std::string XTarkChassisDataParser::VelocityData::DebugString() {
  return "linear x: " + FloatToString(linear_x_, 4) + ", " +
         "linear y: " + FloatToString(linear_y_, 4) + ", " +
         "angular z: " + FloatToString(angular_z_, 4) + "(" +
         FloatToString(RadiansToDegrees(angular_z_), 2) + ").";
}

void XTarkChassisDataParser::VelocityData::SetVelocityCmdByWheel(
    const float& left_wheel_speed, const float& right_wheel_speed,
    const float& track_length) {
  linear_x_ = (left_wheel_speed + right_wheel_speed) / 2;
  linear_y_ = 0;
  angular_z_ = (right_wheel_speed - left_wheel_speed) / track_length;
}

void XTarkChassisDataParser::VelocityData::SetVelocityCmd(
    const float& linear_x, const float& linear_y, const float& angular_z) {
  linear_x_ = linear_x;
  linear_y_ = linear_y;
  angular_z_ = angular_z;
}

std::string XTarkChassisDataParser::PoseData::DebugString() {
  return "pose x: " + FloatToString(pose_x_, 4) + ", " +
         "pose y: " + FloatToString(pose_y_, 4) + ", " +
         "angular z: " + FloatToString(angular_z_, 4) + "(" +
         FloatToString(RadiansToDegrees(angular_z_), 2) + ").";
}

std::string XTarkChassisDataParser::BatteryData::DebugString() {
  return "Battery: " + FloatToString(battery_voltage_, 2) + "V";
}

bool XTarkChassisDataParser::ParseReadData(
    const Serial::BytesFrame& valid_frame, ChassisData& chassis_data) {
  if (valid_frame.size() > kFrameMaxSize_) {
    ZERROR << "Over flow data, frame size: " << valid_frame.size()
           << ", max size is " << kFrameMaxSize_;
    return false;
  }

  uint32_t start_index = 0;
  if (valid_frame.at(start_index) == kCPR2Status_) {
    if (valid_frame.size() < kCPR2StatusFrameSize_) {
      ZERROR << "Insufficient data, frame size: " << valid_frame.size()
             << ", required size is " << kCPR2StatusFrameSize_;
      return false;
    }

    // IMU加速度计量程±2g，对应数据范围±32768
    //加速度计原始数据转换位m/s^2单位，32768/2g=32768/19.6=1671.84
    static constexpr double kAccRatio = (2 * 9.8 / 32768);

    chassis_data.imu_data_.acc_x_ =
        ((float)((int16_t)(valid_frame.at(start_index + 1) * 256 +
                           valid_frame.at(start_index + 2))) *
         kAccRatio);
    chassis_data.imu_data_.acc_y_ =
        ((float)((int16_t)(valid_frame.at(start_index + 3) * 256 +
                           valid_frame.at(start_index + 4))) *
         kAccRatio);
    chassis_data.imu_data_.acc_z_ =
        ((float)((int16_t)(valid_frame.at(start_index + 5) * 256 +
                           valid_frame.at(start_index + 6))) *
         kAccRatio);

    // IMU陀螺仪量程±500°，对应数据范围±32768
    //陀螺仪原始数据转换位弧度(rad)单位
    static constexpr float kGyroRatio = ((500 * M_PI / 180) / 32768);
    chassis_data.imu_data_.gyro_x_ =
        ((float)((int16_t)(valid_frame.at(start_index + 7) * 256 +
                           valid_frame.at(start_index + 8))) *
         kGyroRatio);
    chassis_data.imu_data_.gyro_y_ =
        ((float)((int16_t)(valid_frame.at(start_index + 9) * 256 +
                           valid_frame.at(start_index + 10))) *
         kGyroRatio);
    chassis_data.imu_data_.gyro_z_ =
        ((float)((int16_t)(valid_frame.at(start_index + 11) * 256 +
                           valid_frame.at(start_index + 12))) *
         kGyroRatio);
    // ZINFO << imu_data_.DebugString();

    chassis_data.velocity_data_.linear_x_ =
        ((float)((int16_t)(valid_frame.at(start_index + 13) * 256 +
                           valid_frame.at(start_index + 14))) /
         1000);
    chassis_data.velocity_data_.linear_y_ =
        ((float)((int16_t)(valid_frame.at(start_index + 15) * 256 +
                           valid_frame.at(start_index + 16))) /
         1000);
    chassis_data.velocity_data_.angular_z_ =
        ((float)((int16_t)(valid_frame.at(start_index + 17) * 256 +
                           valid_frame.at(start_index + 18))) /
         1000);
    // ZINFO << velocity_data_.DebugString();

    chassis_data.battery_data_.battery_voltage_ =
        (float)(((valid_frame.at(start_index + 19) << 8) +
                 valid_frame.at(start_index + 20))) /
        100;
    // ZINFO << battery_data_.DebugString();

    static constexpr double kDataPeriod = 0.02;
    //计算里程计数据
    chassis_data.pose_data_.pose_x_ +=
        (chassis_data.velocity_data_.linear_x_ *
             cos(chassis_data.pose_data_.angular_z_) -
         chassis_data.velocity_data_.linear_y_ *
             sin(chassis_data.pose_data_.angular_z_)) *
        kDataPeriod;
    chassis_data.pose_data_.pose_y_ +=
        (chassis_data.velocity_data_.linear_x_ *
             sin(chassis_data.pose_data_.angular_z_) +
         chassis_data.velocity_data_.linear_y_ *
             cos(chassis_data.pose_data_.angular_z_)) *
        kDataPeriod;
    chassis_data.pose_data_.angular_z_ +=
        chassis_data.velocity_data_.angular_z_ *
        kDataPeriod;  //绕Z轴的角位移，单位：rad
    // ZINFO << pose_data_.DebugString();
  } else {
    ZERROR << "Frame type: " << valid_frame.at(start_index) << " invalid.";
    return false;
  }

  return true;
}

Serial::BytesFrame XTarkChassisDataParser::PackVelocityCommandFrameData(
    const VelocityData& velocity_data) {
  Serial::BytesFrame data;
  data.emplace_back(kCPR2VelocityCommand_);
  data.emplace_back((int16_t)(velocity_data.linear_x_ * 1000) >> 8);
  data.emplace_back((int16_t)(velocity_data.linear_x_ * 1000));
  data.emplace_back((int16_t)(velocity_data.linear_y_ * 1000) >> 8);
  data.emplace_back((int16_t)(velocity_data.linear_y_ * 1000));
  data.emplace_back((int16_t)(velocity_data.angular_z_ * 1000) >> 8);
  data.emplace_back((int16_t)(velocity_data.angular_z_ * 1000));

  return data;
}

}  // namespace zima
