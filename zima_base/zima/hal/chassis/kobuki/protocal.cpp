/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/hal/chassis/kobuki/protocal.h"
#include "zima/logger/logger.h"
#include "zima/common/util.h"
#include "zima/common/maths.h"

namespace zima {

const uint8_t KobukiChassisSerialProtocalParser::kHeader1_ = 0xAA;
const uint8_t KobukiChassisSerialProtocalParser::kHeader2_ = 0x55;
const uint8_t KobukiChassisSerialProtocalParser::kFixProtocalPrefixLen_ = 3;
const uint8_t KobukiChassisSerialProtocalParser::kFixProtocalSuffixLen_ = 1;
uint32_t KobukiChassisSerialProtocalParser::kBufMaxSize_ = 1024;

using BaseControlCmdData = KobukiChassisDataParser::BaseControlCmdData;
const uint8_t BaseControlCmdData::kCmdHeader_ = 0x01;
const uint8_t BaseControlCmdData::kLength_ = 0x04;
using SoundCmdData = KobukiChassisDataParser::SoundCmdData;
const uint8_t SoundCmdData::kCmdHeader_ = 0x03;
const uint8_t SoundCmdData::kLength_ = 0x03;
const double SoundCmdData::kA_ = 0.00000275;
using SoundSequenceCmdData = KobukiChassisDataParser::SoundSequenceCmdData;
const uint8_t SoundSequenceCmdData::kCmdHeader_ = 0x04;
const uint8_t SoundSequenceCmdData::kLength_ = 0x01;
const uint8_t SoundSequenceCmdData::kOnSound_ = 0x01;
const uint8_t SoundSequenceCmdData::kOffSound_ = 0x02;
const uint8_t SoundSequenceCmdData::kRechargeSound_ = 0x03;
const uint8_t SoundSequenceCmdData::kButtonSound_ = 0x04;
const uint8_t SoundSequenceCmdData::kErrorSound_ = 0x05;
const uint8_t SoundSequenceCmdData::kCleaningStartSound_ = 0x06;
const uint8_t SoundSequenceCmdData::kCleaningEndSound_ = 0x07;
using RequestExtraCmdData = KobukiChassisDataParser::RequestExtraCmdData;
const uint8_t RequestExtraCmdData::kCmdHeader_ = 0x09;
const uint8_t RequestExtraCmdData::kLength_ = 0x02;
const uint16_t RequestExtraCmdData::kHardwareVersion_ = 0x01;
const uint16_t RequestExtraCmdData::kFirmwareVersion_ = 0x02;
const uint16_t RequestExtraCmdData::kUDID_ = 0x08;
using GeneralPurposeOutputCmdData =
    KobukiChassisDataParser::GeneralPurposeOutputCmdData;
const uint8_t GeneralPurposeOutputCmdData::kCmdHeader_ = 0x0C;
const uint8_t GeneralPurposeOutputCmdData::kLength_ = 0x02;
const uint16_t GeneralPurposeOutputCmdData::kDigChannel0_ = 0x0001;
const uint16_t GeneralPurposeOutputCmdData::kDigChannel1_ = 0x0002;
const uint16_t GeneralPurposeOutputCmdData::kDigChannel2_ = 0x0004;
const uint16_t GeneralPurposeOutputCmdData::kDigChannel3_ = 0x0008;
const uint16_t GeneralPurposeOutputCmdData::kPower3p3V_ = 0x0010;
const uint16_t GeneralPurposeOutputCmdData::kPower5V_ = 0x0020;
const uint16_t GeneralPurposeOutputCmdData::kPower12V5A_ = 0x0040;
const uint16_t GeneralPurposeOutputCmdData::kPower12V1p5A_ = 0x0080;
const uint16_t GeneralPurposeOutputCmdData::kLed1Red_ = 0x0100;
const uint16_t GeneralPurposeOutputCmdData::kLed1Green_ = 0x0200;
const uint16_t GeneralPurposeOutputCmdData::kLed2Red_ = 0x0400;
const uint16_t GeneralPurposeOutputCmdData::kLed2Green_ = 0x0800;
using SetControllerGainCmdData =
    KobukiChassisDataParser::SetControllerGainCmdData;
const uint8_t SetControllerGainCmdData::kCmdHeader_ = 0x0D;
const uint8_t SetControllerGainCmdData::kLength_ = 0x0D;
const uint8_t SetControllerGainCmdData::kDefaultPID_ = 0x00;
const uint8_t SetControllerGainCmdData::kCustomPID_ = 0x01;
using GetControllerGainCmdData =
    KobukiChassisDataParser::GetControllerGainCmdData;
const uint8_t GetControllerGainCmdData::kCmdHeader_ = 0x0E;
const uint8_t GetControllerGainCmdData::kLength_ = 0x01;

using BasicSensorRspData = KobukiChassisDataParser::BasicSensorRspData;
const uint8_t BasicSensorRspData::kRspHeader_ = 0x01;
const uint8_t BasicSensorRspData::kLength_ = 0x0F;
const uint8_t BasicSensorRspData::kRightBumperPressed_ = 0x01;
const uint8_t BasicSensorRspData::kCentralBumperPressed_ = 0x02;
const uint8_t BasicSensorRspData::kLeftBumperPressed_ = 0x04;
const uint8_t BasicSensorRspData::kRightWheelDrop_ = 0x01;
const uint8_t BasicSensorRspData::kLeftWheelDrop_ = 0x02;
const uint8_t BasicSensorRspData::kRightCliffDetected_ = 0x01;
const uint8_t BasicSensorRspData::kCentralCliffDetected_ = 0x02;
const uint8_t BasicSensorRspData::kLeftCliffDetected_ = 0x04;
const uint8_t BasicSensorRspData::kButton0Pressed_ = 0x01;
const uint8_t BasicSensorRspData::kButton1Pressed_ = 0x02;
const uint8_t BasicSensorRspData::kButton2Pressed_ = 0x04;
const uint8_t BasicSensorRspData::kDisCharging_ = 0x00;
const uint8_t BasicSensorRspData::kDockCharged_ = 0x02;
const uint8_t BasicSensorRspData::kDockCharging_ = 0x06;
const uint8_t BasicSensorRspData::kAdapterCharged_ = 0x12;
const uint8_t BasicSensorRspData::kAdapterCharging_ = 0x16;
const uint8_t BasicSensorRspData::kLeftWheelOC_ = 0x01;
const uint8_t BasicSensorRspData::kRightWheelOC_ = 0x02;
using DockIRRspData = KobukiChassisDataParser::DockIRRspData;
const uint8_t DockIRRspData::kRspHeader_ = 0x03;
const uint8_t DockIRRspData::kLength_ = 0x03;
const uint8_t DockIRRspData::kNearLeft_ = 0x01;
const uint8_t DockIRRspData::kNearCenter_ = 0x02;
const uint8_t DockIRRspData::kNearRight_ = 0x04;
const uint8_t DockIRRspData::kFarLeft_ = 0x08;
const uint8_t DockIRRspData::kFarCenter_ = 0x10;
const uint8_t DockIRRspData::kFarRight_ = 0x20;
using InertialSensorRspData = KobukiChassisDataParser::InertialSensorRspData;
const uint8_t InertialSensorRspData::kRspHeader_ = 0x04;
const uint8_t InertialSensorRspData::kLength_ = 0x07;
using CliffSensorRspData = KobukiChassisDataParser::CliffSensorRspData;
const uint8_t CliffSensorRspData::kRspHeader_ = 0x05;
const uint8_t CliffSensorRspData::kLength_ = 0x06;
const uint16_t CliffSensorRspData::kAdcMin_ = 0;
const uint16_t CliffSensorRspData::kAdcMax_ = 0x0FFF;
const float CliffSensorRspData::kDistanceMin_ = 2.0;
const float CliffSensorRspData::kDistanceMax_ = 15.0;
using WheelCurrentRspData = KobukiChassisDataParser::WheelCurrentRspData;
const uint8_t WheelCurrentRspData::kRspHeader_ = 0x06;
const uint8_t WheelCurrentRspData::kLength_ = 0x02;
using HardwareVersionRspData = KobukiChassisDataParser::HardwareVersionRspData;
const uint8_t HardwareVersionRspData::kRspHeader_ = 0x0A;
const uint8_t HardwareVersionRspData::kLength_ = 0x04;
using FirmwareVersionRspData = KobukiChassisDataParser::FirmwareVersionRspData;
const uint8_t FirmwareVersionRspData::kRspHeader_ = 0x0B;
const uint8_t FirmwareVersionRspData::kLength_ = 0x04;
using GyroRawRspData = KobukiChassisDataParser::GyroRawRspData;
const uint8_t GyroRawRspData::kRspHeader_ = 0x0D;
const uint8_t GyroRawRspData::kLength_ = 0x0E;
using GeneralPurposeInputRspData =
    KobukiChassisDataParser::GeneralPurposeInputRspData;
const uint8_t GeneralPurposeInputRspData::kRspHeader_ = 0x10;
const uint8_t GeneralPurposeInputRspData::kLength_ = 0x10;
const uint16_t GeneralPurposeInputRspData::kDigChannel0Enable_ = 0x01;
const uint16_t GeneralPurposeInputRspData::kDigChannel1Enable_ = 0x02;
const uint16_t GeneralPurposeInputRspData::kDigChannel2Enable_ = 0x04;
const uint16_t GeneralPurposeInputRspData::kDigChannel3Enable_ = 0x08;
const uint16_t GeneralPurposeInputRspData::kAdcMin_ = 0x00;
const uint16_t GeneralPurposeInputRspData::kAdcMax_ = 0x0FFF;
const float GeneralPurposeInputRspData::kVoltageMin_ = 0.0;
const float GeneralPurposeInputRspData::kVoltageMax_ = 3.3;
using UDIDRspData = KobukiChassisDataParser::UDIDRspData;
const uint8_t UDIDRspData::kRspHeader_ = 0x13;
const uint8_t UDIDRspData::kLength_ = 0x0C;
using ControllerGainRspData = KobukiChassisDataParser::ControllerGainRspData;
const uint8_t ControllerGainRspData::kRspHeader_ = 0x15;
const uint8_t ControllerGainRspData::kLength_ = 0x0D;
const uint8_t ControllerGainRspData::kDefaultPID_ = 0x00;
const uint8_t ControllerGainRspData::kCustomPID_ = 0x01;
using RspDataPack = KobukiChassisDataParser::RspDataPack;
const uint8_t RspDataPack::kMaxDequeSize_ = 50;

// Should be max size of all type.
uint32_t KobukiChassisDataParser::kFrameMaxSize_ = 1024;

void KobukiChassisSerialProtocalParser::InitForReadBuffer(
    Serial::BytesFrame& frame_buf) {
  frame_buf.clear();
  frame_buf.resize(kBufMaxSize_, 0);
  ZINFO << "Frame buffer size " << frame_buf.size();
}

bool KobukiChassisSerialProtocalParser::ParseReadFrame(
    const uint8_t& recv_byte, Serial::BytesFrame& frame_buf,
    Serial::BytesFrames& valid_frames) {
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
      // Expect data length means length of DATA part.
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
      if (cache_data_len_ >= expect_data_len_ + kFixProtocalPrefixLen_) {
        progress_ = kForCheckSum;
      }
      break;
    }
    case kForCheckSum: {
      frame_buf.at(cache_data_len_) = recv_byte;
      unsigned char check_sum = 0;
      // for (auto i = 2; i < expect_data_len_ - kFixProtocalSuffixLen_; i++) {
      //   check_sum ^= frame_buf.at(i);
      // }
      for (auto i = 2; i < expect_data_len_ + kFixProtocalPrefixLen_; i++) {
        check_sum ^= frame_buf.at(i);
      }
      // ZINFO << "Cal checksum: " << UintToHexString(check_sum);
      // ZINFO << "Receive checksum: " << UintToHexString(recv_byte);
      auto pack_data_end = kFixProtocalPrefixLen_ + expect_data_len_;
      if (check_sum == recv_byte) {
        // Valid frame.
        valid_frames.emplace_back(
            Serial::BytesFrame(frame_buf.begin() + kFixProtocalPrefixLen_,
                               frame_buf.begin() + pack_data_end));
        // ZINFO << Serial::DebugString(Serial::BytesFrame(
        //     frame_buf.begin() + kFixProtocalPrefixLen_, frame_buf.begin() +
        //     pack_data_end));
      } else {
        ZINFO << "Cal checksum: " << UintToHexString(check_sum);
        ZINFO << "Receive checksum: " << UintToHexString(recv_byte);
        ZWARN << Serial::DebugString(Serial::BytesFrame(
            frame_buf.begin(), frame_buf.begin() + pack_data_end));
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

Serial::BytesFrame KobukiChassisSerialProtocalParser::PackWriteFrame(
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
  frame.emplace_back(frame_data.size());
  frame.insert(frame.end(), frame_data.begin(), frame_data.end());

  char check_sum = 0;
  for (auto&& byte : frame) {
    check_sum ^= byte;
  }
  frame.emplace_back(check_sum & 0xFF);
  return frame;
}

std::string KobukiChassisDataParser::BaseControlCmdData::DebugString() {
  return std::string(ZCOLOR_GREEN) +
         "left: " + FloatToString(left_wheel_speed_, 2) +
         ", right: " + FloatToString(right_wheel_speed_, 2) +
         std::string(ZCOLOR_NONE) + ", linear x: " + FloatToString(linear_x_, 4) +
         ", angular z: " + FloatToString(angular_z_, 4) + "(" +
         FloatToString(RadiansToDegrees(angular_z_), 2) +
         "), speed_mm: " + FloatToString(speed_mm_, 1) +
         ", radius_mm: " + FloatToString(radius_mm_, 1);
}

void KobukiChassisDataParser::BaseControlCmdData::SetVelocityCmdByWheel(
    const float& left_wheel_speed, const float& right_wheel_speed,
    const float& track_length) {
  left_wheel_speed_ = left_wheel_speed;
  right_wheel_speed_ = right_wheel_speed;
  linear_x_ = (left_wheel_speed + right_wheel_speed) / 2;
  angular_z_ = (right_wheel_speed - left_wheel_speed) / track_length;
  CalSpeedNRadius(track_length);
}

void KobukiChassisDataParser::BaseControlCmdData::SetVelocityCmd(
    const float& linear_x, const float& angular_z, const float& track_length) {
  linear_x_ = linear_x;
  angular_z_ = angular_z;
  auto tmp = angular_z_ * track_length / 2;
  left_wheel_speed_ = linear_x_ - tmp;
  left_wheel_speed_ = linear_x_ + tmp;
  CalSpeedNRadius(track_length);
}

void KobukiChassisDataParser::BaseControlCmdData::CalSpeedNRadius(
    const float& track_length) {
  if (FloatEqual(angular_z_, 0)) {
    radius_mm_ = 0;
    speed_mm_ = linear_x_ * 1000;
    return;
  }
  radius_mm_ = linear_x_ * 1000 / angular_z_;
  if (FloatEqual(linear_x_, 0) || fabs(radius_mm_) <= 1) {
    speed_mm_ = 1000 * track_length * angular_z_ / 2;
    radius_mm_ = 1;
    return;
  }
  if (radius_mm_ > 0) {
    speed_mm_ = (radius_mm_ + 1000 * track_length / 2) * angular_z_;
  } else {
    speed_mm_ = (radius_mm_ - 1000 * track_length / 2) * angular_z_;
  }
}

std::string KobukiChassisDataParser::SoundCmdData::DebugString() { return ""; }

std::string KobukiChassisDataParser::SoundSequenceCmdData::DebugString() {
  return "";
}

std::string KobukiChassisDataParser::RequestExtraCmdData::DebugString() {
  std::string str;
  str += "Request for: ";
  str += (request_flag_ & kHardwareVersion_) ? "hardware version / " : "";
  str += (request_flag_ & kFirmwareVersion_) ? "firmware version / " : "";
  str += (request_flag_ & kHardwareVersion_) ? "UUID / " : "";
  return str;
}

std::string
KobukiChassisDataParser::GeneralPurposeOutputCmdData::DebugString() {
  std::string str;

  str += "Set for: ";
  str += (config_ & kDigChannel0_) ? "enable " : "disable ";
  str += "channel 0 / ";
  str += (config_ & kDigChannel1_) ? "enable " : "disable ";
  str += "channel 1 / ";
  str += (config_ & kDigChannel2_) ? "enable " : "disable ";
  str += "channel 2 / ";
  str += (config_ & kDigChannel3_) ? "enable " : "disable ";
  str += "channel 3 / ";

  str += (config_ & kPower3p3V_) ? "enable " : "disable ";
  str += "3.3V power / ";
  str += (config_ & kPower5V_) ? "enable " : "disable ";
  str += "5V power / ";
  str += (config_ & kPower12V5A_) ? "enable " : "disable ";
  str += "12V5A power / ";
  str += (config_ & kPower12V1p5A_) ? "enable " : "disable ";
  str += "12V1.5A power / ";

  str += "LED1 ";
  if ((config_ & kLed1Green_) && (config_ & kLed1Red_)) {
    str += "yellow ";
  } else if (config_ & kLed1Green_) {
    str += "green ";
  } else if (config_ & kLed1Red_) {
    str += "red ";
  } else {
    str += "off ";
  }
  str += "/ ";

  str += "LED2 ";
  if ((config_ & kLed2Green_) && (config_ & kLed2Red_)) {
    str += "yellow ";
  } else if (config_ & kLed2Green_) {
    str += "green ";
  } else if (config_ & kLed2Red_) {
    str += "red ";
  } else {
    str += "off ";
  }

  return str;
}

std::string KobukiChassisDataParser::SetControllerGainCmdData::DebugString() {
  std::string str;
  str += "Set PID: ";
  str += "p " + FloatToString(p_x1000_ / 1000.0, 1) + ", ";
  str += "i " + FloatToString(i_x1000_ / 1000.0, 1) + ", ";
  str += "d " + FloatToString(d_x1000_ / 1000.0, 1) + ".";
  return str;
}

std::string KobukiChassisDataParser::GetControllerGainCmdData::DebugString() {
  return "Get PID request.";
}

std::string KobukiChassisDataParser::BasicSensorRspData::DebugString() {
  std::string str;
  str += "Timestamp: " + std::to_string(timestamp_) + ", ";

  str += "bumper left(";
  str += (bumper_pressed_ & kLeftBumperPressed_) ? "1" : "0";
  str += "), ";
  str += "central(";
  str += (bumper_pressed_ & kCentralBumperPressed_) ? "1" : "0";
  str += "), ";
  str += "right(";
  str += (bumper_pressed_ & kRightBumperPressed_) ? "1" : "0";
  str += "), ";

  str += "wheel drop left(";
  str += (wheel_drop_ & kLeftWheelDrop_) ? "1" : "0";
  str += "), ";
  str += "right(";
  str += (wheel_drop_ & kRightWheelDrop_) ? "1" : "0";
  str += "), ";

  str += "cliff left(";
  str += (cliff_detected_ & kLeftCliffDetected_) ? "1" : "0";
  str += "), ";
  str += "central(";
  str += (cliff_detected_ & kCentralCliffDetected_) ? "1" : "0";
  str += "), ";
  str += "right(";
  str += (cliff_detected_ & kRightCliffDetected_) ? "1" : "0";
  str += "), ";

  str += "encoder left(" + std::to_string(left_encoder_) + "), ";
  str += "right(" + std::to_string(right_encoder_) + "), ";

  str += "pwm left(" + std::to_string(left_pwm_) + "), ";
  str += "right(" + std::to_string(right_pwm_) + "), ";

  str += "button 0(";
  str += (button_pressed_ & kButton0Pressed_) ? "1" : "0";
  str += "), ";
  str += "1(";
  str += (button_pressed_ & kButton1Pressed_) ? "1" : "0";
  str += "), ";
  str += "2(";
  str += (button_pressed_ & kButton2Pressed_) ? "1" : "0";
  str += "), ";

  switch (charging_state_) {
    case kDisCharging_: {
      str += "not charging, ";
      break;
    }
    case kDockCharged_: {
      str += "dock charge finished, ";
      break;
    }
    case kDockCharging_: {
      str += "dock charging, ";
      break;
    }
    case kAdapterCharged_: {
      str += "adapter charge finished, ";
      break;
    }
    case kAdapterCharging_: {
      str += "adapter charging, ";
      break;
    }
    default: {
      str += "unknown charging state " + std::to_string(charging_state_) + ", ";
    }
  }

  str += "battery " + FloatToString(battery_p1V_ / 10.0f, 1) + " V, ";

  str += "wheel oc left(";
  str += (wheel_OC_ & kLeftWheelOC_) ? "1" : "0";
  str += "), ";
  str += "right(";
  str += (wheel_OC_ & kRightWheelOC_) ? "1" : "0";
  str += "), ";

  return str;
}

void KobukiChassisDataParser::BasicSensorRspData::PrintOnTriggered() {
  if (bumper_pressed_ != 0x00 || wheel_drop_ != 0x00 ||
      cliff_detected_ != 0x00 || button_pressed_ != 0x00 || wheel_OC_ != 0x00) {
    ZINFO << DebugString();
  }
}

std::string KobukiChassisDataParser::DockIRRspData::DebugString() {
  std::string str;
  str += "IR: ";
  auto signal_str = [&](const uint8_t& signal) -> std::string {
    std::string str;
    if (signal & kNearLeft_) {
      str += "near left, ";
    }
    if (signal & kNearCenter_) {
      str += "near center, ";
    }
    if (signal & kNearRight_) {
      str += "near right, ";
    }
    if (signal & kFarLeft_) {
      str += "far left, ";
    }
    if (signal & kFarCenter_) {
      str += "far center, ";
    }
    if (signal & kFarRight_) {
      str += "far right, ";
    }
    return str;
  };

  if (right_ir_received_ != 0) {
    str += "right ir receive: ";
    str += signal_str(right_ir_received_);
  }
  if (center_ir_received_ != 0) {
    str += "center ir receive: ";
    str += signal_str(center_ir_received_);
  }
  if (left_ir_received_ != 0) {
    str += "left ir receive: ";
    str += signal_str(left_ir_received_);
  }
  return str;
}

std::string KobukiChassisDataParser::InertialSensorRspData::DebugString() {
  std::string str;
  str += "Angle: " + std::to_string(angle_);
  str += ", angle rate: " + std::to_string(angle_rate_);
  return str;
}

std::string KobukiChassisDataParser::CliffSensorRspData::DebugString() {
  std::string str;
  str += "Cliff ";
  str += "left: " + std::to_string(left_cliff_adc_) + "(" +
         FloatToString(left_cliff_distance_, 1) + "cm), ";
  str += "center: " + std::to_string(central_cliff_adc_) + "(" +
         FloatToString(central_cliff_distance_, 1) + "cm), ";
  str += "right: " + std::to_string(right_cliff_adc_) + "(" +
         FloatToString(right_cliff_distance_, 1) + "cm).";
  return str;
}

float KobukiChassisDataParser::CliffSensorRspData::AdcToCm(
    const uint16_t& adc_value) {
  static const uint16_t kAdcRange = kAdcMax_ - kAdcMin_;
  static const float kDistanceRange = kDistanceMax_ - kDistanceMin_;
  return kDistanceMin_ +
         ((adc_value - kAdcMin_) * 1.0 / (kAdcRange)) * kDistanceRange;
}

std::string KobukiChassisDataParser::WheelCurrentRspData::DebugString() {
  std::string str;
  str += "Current: ";
  str += "left (" + std::to_string(left_wheel_current_10mA_ * 10) + "mA), ";
  str += "right (" + std::to_string(right_wheel_current_10mA_ * 10) + "mA). ";
  return str;
}

std::string KobukiChassisDataParser::HardwareVersionRspData::DebugString() {
  std::string str;
  str += "Hardware version: ";
  str += std::to_string(major_) + "." + std::to_string(minor_) + "." +
         std::to_string(patch_);
  return str;
}

std::string KobukiChassisDataParser::FirmwareVersionRspData::DebugString() {
  std::string str;
  str += "Firmware version: ";
  str += std::to_string(major_) + "." + std::to_string(minor_) + "." +
         std::to_string(patch_);
  return str;
}

std::string KobukiChassisDataParser::GyroRawRspData::DebugString() {
  std::string str;
  int index = 0;
  str += "Frame " + std::to_string(frame_id_);
  str += "(data " + std::to_string(data_len_) + " bytes): ";
  for (auto&& gyro : gyros_) {
    str += "Gyro" + std::to_string(index) + " ";
    str += "x_axis " + std::to_string(gyro.x_axis_);
    str += ", y_axis " + std::to_string(gyro.y_axis_);
    str += ", z_axis " + std::to_string(gyro.z_axis_) + ". ";
    index++;
  }
  return str;
}

std::string KobukiChassisDataParser::GeneralPurposeInputRspData::DebugString() {
  std::string str;
  str += "Channel ";
  bool enabled = false;
  if (digital_channel_enabled_flags_ & kDigChannel0Enable_) {
    str += "0";
    enabled = true;
  }
  if (digital_channel_enabled_flags_ & kDigChannel1Enable_) {
    str += "1";
    enabled = true;
  }
  if (digital_channel_enabled_flags_ & kDigChannel2Enable_) {
    str += "2";
    enabled = true;
  }
  if (digital_channel_enabled_flags_ & kDigChannel3Enable_) {
    str += "3";
    enabled = true;
  }
  if (!enabled) {
    str += "disabled, ";
  } else {
    str += "enabled, ";
  }
  str += " channel ";
  // if (digital_channel_enabled_flags_ & kDigChannel0Enable_) {
  str += "0: " + FloatToString(channel_0_voltage_, 2) + "V, ";
  // }
  // if (digital_channel_enabled_flags_ & kDigChannel1Enable_) {
  str += "1: " + FloatToString(channel_1_voltage_, 2) + "V, ";
  // }
  // if (digital_channel_enabled_flags_ & kDigChannel2Enable_) {
  str += "2: " + FloatToString(channel_2_voltage_, 2) + "V, ";
  // }
  // if (digital_channel_enabled_flags_ & kDigChannel3Enable_) {
  str += "3: " + FloatToString(channel_3_voltage_, 2) + "V, ";
  // }

  return str;
}

float KobukiChassisDataParser::GeneralPurposeInputRspData::AdcToVoltage(
    const uint16_t& adc) {
  static const uint16_t kAdcRange = kAdcMax_ - kAdcMin_;
  static const float kVoltageRange = kVoltageMax_ - kVoltageMin_;
  return kVoltageMin_ + ((adc - kAdcMin_) * 1.0 / (kAdcRange)) * kVoltageRange;
}

std::string KobukiChassisDataParser::UDIDRspData::DebugString() {
  std::string str;
  str += "UDID: ";
  str += UintToHexString(UDID0_) + " ";
  str += UintToHexString(UDID1_) + " ";
  str += UintToHexString(UDID2_) + ".";

  return str;
}

std::string KobukiChassisDataParser::ControllerGainRspData::DebugString() {
  std::string str;
  str += "PID: ";
  str += (type_ == kDefaultPID_) ? "(default) " : "(custom) ";
  // str += "p: " + std::to_string(p_x1000_) + ", ";
  // str += "i: " + std::to_string(i_x1000_) + ", ";
  // str += "d: " + std::to_string(d_x1000_) + ".";
  str += "p: " + FloatToString(p_x1000_ / 1000.0, 1) + ", ";
  str += "i: " + FloatToString(i_x1000_ / 1000.0, 1) + ", ";
  str += "d: " + FloatToString(d_x1000_ / 1000.0, 1) + ".";
  return str;
}

bool KobukiChassisDataParser::ParseReadData(
    const Serial::BytesFrame& valid_frame_data, RspDataPack& data_pack) {
  if (valid_frame_data.size() > kFrameMaxSize_) {
    ZERROR << "Over flow data, frame size: " << valid_frame_data.size()
           << ", max size is " << kFrameMaxSize_;
    return false;
  }
  uint32_t read_index = 0;
  while (read_index < valid_frame_data.size()) {
    // ZINFO << "For " << UintToHexString(valid_frame_data.at(read_index));
    switch (valid_frame_data.at(read_index)) {
      case BasicSensorRspData::kRspHeader_: {
        read_index =
            ParseBasicSensorData(valid_frame_data, read_index, data_pack);
        break;
      }
      case DockIRRspData::kRspHeader_: {
        read_index = ParseDockIR(valid_frame_data, read_index, data_pack);
        break;
      }
      case InertialSensorRspData::kRspHeader_: {
        read_index =
            ParseInertialSensor(valid_frame_data, read_index, data_pack);
        break;
      }
      case CliffSensorRspData::kRspHeader_: {
        read_index = ParseCliffSensor(valid_frame_data, read_index, data_pack);
        break;
      }
      case WheelCurrentRspData::kRspHeader_: {
        read_index = ParseWheelCurrent(valid_frame_data, read_index, data_pack);
        break;
      }
      case HardwareVersionRspData::kRspHeader_: {
        read_index =
            ParseHardwareVersion(valid_frame_data, read_index, data_pack);
        break;
      }
      case FirmwareVersionRspData::kRspHeader_: {
        read_index =
            ParseFirmwareVersion(valid_frame_data, read_index, data_pack);
        break;
      }
      case GyroRawRspData::kRspHeader_: {
        read_index = ParseGyro(valid_frame_data, read_index, data_pack);
        break;
      }
      case GeneralPurposeInputRspData::kRspHeader_: {
        read_index =
            ParseGeneralPurposeInput(valid_frame_data, read_index, data_pack);
        break;
      }
      case UDIDRspData::kRspHeader_: {
        read_index = ParseUDID(valid_frame_data, read_index, data_pack);
        break;
      }
      case ControllerGainRspData::kRspHeader_: {
        // ZERROR << "For PID.";
        read_index =
            ParseControllerGain(valid_frame_data, read_index, data_pack);
        break;
      }
      default: {
        ZERROR << "Invalid id: "
               << UintToHexString(valid_frame_data.at(read_index));
        return false;
      }
    }
  }

  return true;
}

Serial::BytesFrame KobukiChassisDataParser::PackCmdFrameData(
    CmdDataPack& cmd_pack) {
  Serial::BytesFrame frame;
  for (auto&& base_control_cmd : cmd_pack.base_control_cmd_data_) {
    if (base_control_cmd == nullptr) {
      ZWARN << "base_control_cmd is nullptr";
      continue;
    }
    frame.emplace_back(BaseControlCmdData::kCmdHeader_);
    frame.emplace_back(BaseControlCmdData::kLength_);
    auto speed_mm = static_cast<uint16_t>(
        static_cast<int16_t>(base_control_cmd->speed_mm_));
    // ZINFO << "Speed mm " << speed_mm;
    frame.emplace_back(speed_mm & 0xFF);
    frame.emplace_back((speed_mm >> 8) & 0xFF);
    auto radius_mm = static_cast<uint16_t>(
        static_cast<int16_t>(base_control_cmd->radius_mm_));
    // ZINFO << "Radius mm " << radius_mm;
    frame.emplace_back(radius_mm & 0xFF);
    frame.emplace_back((radius_mm >> 8) & 0xFF);
    // ZINFO << "Pack for " << base_control_cmd->DebugString();
  }
  cmd_pack.base_control_cmd_data_.clear();

  for (auto&& request_extra_cmd : cmd_pack.request_extra_cmd_data_) {
    if (request_extra_cmd == nullptr) {
      ZWARN << "request_extra_cmd is nullptr";
      continue;
    }
    frame.emplace_back(RequestExtraCmdData::kCmdHeader_);
    frame.emplace_back(RequestExtraCmdData::kLength_);
    frame.emplace_back(request_extra_cmd->request_flag_ & 0xFF);
    frame.emplace_back((request_extra_cmd->request_flag_ >> 8) & 0xFF);
    ZINFO << "Pack for " << request_extra_cmd->DebugString();
  }
  cmd_pack.request_extra_cmd_data_.clear();

  for (auto&& set_pid_cmd : cmd_pack.set_controller_gain_cmd_data_) {
    if (set_pid_cmd == nullptr) {
      ZWARN << "set_pid_cmd is nullptr";
      continue;
    }
    frame.emplace_back(SetControllerGainCmdData::kCmdHeader_);
    frame.emplace_back(SetControllerGainCmdData::kLength_);
    frame.emplace_back(set_pid_cmd->type_);
    auto p = static_cast<uint32_t>(static_cast<int32_t>(set_pid_cmd->p_x1000_));
    frame.emplace_back(p & 0xFF);
    frame.emplace_back((p >> 8) & 0xFF);
    frame.emplace_back((p >> 16) & 0xFF);
    frame.emplace_back((p >> 24) & 0xFF);
    auto i = static_cast<uint32_t>(static_cast<int32_t>(set_pid_cmd->i_x1000_));
    frame.emplace_back(i & 0xFF);
    frame.emplace_back((i >> 8) & 0xFF);
    frame.emplace_back((i >> 16) & 0xFF);
    frame.emplace_back((i >> 24) & 0xFF);
    auto d = static_cast<uint32_t>(static_cast<int32_t>(set_pid_cmd->d_x1000_));
    frame.emplace_back(d & 0xFF);
    frame.emplace_back((d >> 8) & 0xFF);
    frame.emplace_back((d >> 16) & 0xFF);
    frame.emplace_back((d >> 24) & 0xFF);
    ZINFO << "Pack for " << set_pid_cmd->DebugString();
  }
  cmd_pack.set_controller_gain_cmd_data_.clear();

  for (auto&& get_pid_cmd : cmd_pack.get_controller_gain_cmd_data_) {
    if (get_pid_cmd == nullptr) {
      ZWARN << "get_pid_cmd is nullptr";
      continue;
    }
    frame.emplace_back(GetControllerGainCmdData::kCmdHeader_);
    frame.emplace_back(GetControllerGainCmdData::kLength_);
    frame.emplace_back(0x00);
    ZINFO << "Pack for " << get_pid_cmd->DebugString();
  }
  cmd_pack.get_controller_gain_cmd_data_.clear();

  // if (frame.empty()) {
  //   ZWARN << "Pack for no thing.";
  // }

  return frame;
}

bool KobukiChassisDataParser::SubPayloadCheck(
    const Serial::BytesFrame& data_frame, const uint32_t& header_index,
    const uint8_t& expect_data_length, const std::string& name) {
  auto required_bytes_length = 1 + expect_data_length;
  if (header_index + required_bytes_length >= data_frame.size()) {
    ZWARN << name << " insufficient data at " << std::to_string(header_index)
          << ", need " << std::to_string(required_bytes_length)
          << " more data but data size was "
          << std::to_string(data_frame.size());
    return false;
  }
  if (data_frame.at(header_index + 1) != expect_data_length) {
    ZWARN << name << " invalid sub payload length "
          << std::to_string(data_frame.at(header_index + 1)) << ", expect "
          << std::to_string(expect_data_length);
    return false;
  }
  return true;
}

uint32_t KobukiChassisDataParser::ParseBasicSensorData(
    const Serial::BytesFrame& data_frame, const uint32_t& header_index,
    RspDataPack& rsp_data_pack) {
  if (!SubPayloadCheck(data_frame, header_index, BasicSensorRspData::kLength_,
                       typeid(BasicSensorRspData).name())) {
    return data_frame.size();
  }

  while (rsp_data_pack.basic_sensor_data_.size() >
         RspDataPack::kMaxDequeSize_) {
    rsp_data_pack.basic_sensor_data_.pop_front();
  }
  auto data = std::make_unique<BasicSensorRspData>();
  auto index = header_index + 2;
  data->timestamp_ = data_frame.at(index++);
  data->timestamp_ += (data_frame.at(index++) << 8);
  data->bumper_pressed_ = data_frame.at(index++);
  data->wheel_drop_ = data_frame.at(index++);
  data->cliff_detected_ = data_frame.at(index++);
  data->left_encoder_ = data_frame.at(index++);
  data->left_encoder_ += (data_frame.at(index++) << 8);
  data->right_encoder_ = data_frame.at(index++);
  data->right_encoder_ += (data_frame.at(index++) << 8);
  data->left_pwm_ = data_frame.at(index++);
  data->right_pwm_ = data_frame.at(index++);
  data->button_pressed_ = data_frame.at(index++);
  data->charging_state_ = data_frame.at(index++);
  data->battery_p1V_ = data_frame.at(index++);
  data->wheel_OC_ = data_frame.at(index++);
  // data->PrintOnTriggered();
  // ZINFO << data->DebugString();
  rsp_data_pack.basic_sensor_data_.emplace_back(std::move(data));

  return index;
}

uint32_t KobukiChassisDataParser::ParseDockIR(
    const Serial::BytesFrame& data_frame, const uint32_t& header_index,
    RspDataPack& rsp_data_pack) {
  if (!SubPayloadCheck(data_frame, header_index, DockIRRspData::kLength_,
                       typeid(DockIRRspData).name())) {
    return data_frame.size();
  }

  while (rsp_data_pack.Dock_IR_data_.size() > RspDataPack::kMaxDequeSize_) {
    rsp_data_pack.Dock_IR_data_.pop_front();
  }
  auto data = std::make_unique<DockIRRspData>();
  auto index = header_index + 2;
  data->right_ir_received_ = data_frame.at(index++);
  data->center_ir_received_ = data_frame.at(index++);
  data->left_ir_received_ = data_frame.at(index++);
  // ZINFO << data->DebugString();
  rsp_data_pack.Dock_IR_data_.emplace_back(std::move(data));

  return index;
}

uint32_t KobukiChassisDataParser::ParseInertialSensor(
    const Serial::BytesFrame& data_frame, const uint32_t& header_index,
    RspDataPack& rsp_data_pack) {
  if (!SubPayloadCheck(data_frame, header_index,
                       InertialSensorRspData::kLength_,
                       typeid(InertialSensorRspData).name())) {
    return data_frame.size();
  }

  while (rsp_data_pack.inertial_sensor_data_.size() >
         RspDataPack::kMaxDequeSize_) {
    rsp_data_pack.inertial_sensor_data_.pop_front();
  }
  auto data = std::make_unique<InertialSensorRspData>();
  auto index = header_index + 2;
  data->angle_ = data_frame.at(index++);
  data->angle_ += (data_frame.at(index++) << 8);
  data->angle_rate_ = data_frame.at(index++);
  data->angle_rate_ += (data_frame.at(index++) << 8);
  // ZINFO << data->DebugString();
  rsp_data_pack.inertial_sensor_data_.emplace_back(std::move(data));
  index += 3;  // Skip unused data.

  return index;
}

uint32_t KobukiChassisDataParser::ParseCliffSensor(
    const Serial::BytesFrame& data_frame, const uint32_t& header_index,
    RspDataPack& rsp_data_pack) {
  if (!SubPayloadCheck(data_frame, header_index, CliffSensorRspData::kLength_,
                       typeid(CliffSensorRspData).name())) {
    return data_frame.size();
  }

  while (rsp_data_pack.cliff_sensor_data_.size() >
         RspDataPack::kMaxDequeSize_) {
    rsp_data_pack.cliff_sensor_data_.pop_front();
  }
  auto data = std::make_unique<CliffSensorRspData>();
  auto index = header_index + 2;
  data->right_cliff_adc_ = data_frame.at(index++);
  data->right_cliff_adc_ += (data_frame.at(index++) << 8);
  data->right_cliff_distance_ =
      CliffSensorRspData::AdcToCm(data->right_cliff_adc_);
  data->central_cliff_adc_ = data_frame.at(index++);
  data->central_cliff_adc_ += (data_frame.at(index++) << 8);
  data->central_cliff_distance_ =
      CliffSensorRspData::AdcToCm(data->central_cliff_adc_);
  data->left_cliff_adc_ = data_frame.at(index++);
  data->left_cliff_adc_ += (data_frame.at(index++) << 8);
  data->left_cliff_distance_ =
      CliffSensorRspData::AdcToCm(data->left_cliff_adc_);
  // ZINFO << data->DebugString();
  rsp_data_pack.cliff_sensor_data_.emplace_back(std::move(data));

  return index;
}

uint32_t KobukiChassisDataParser::ParseWheelCurrent(
    const Serial::BytesFrame& data_frame, const uint32_t& header_index,
    RspDataPack& rsp_data_pack) {
  if (!SubPayloadCheck(data_frame, header_index, WheelCurrentRspData::kLength_,
                       typeid(WheelCurrentRspData).name())) {
    return data_frame.size();
  }

  while (rsp_data_pack.wheel_current_data_.size() >
         RspDataPack::kMaxDequeSize_) {
    rsp_data_pack.wheel_current_data_.pop_front();
  }
  auto data = std::make_unique<WheelCurrentRspData>();
  auto index = header_index + 2;
  data->left_wheel_current_10mA_ = data_frame.at(index++);
  data->right_wheel_current_10mA_ = data_frame.at(index++);
  // ZINFO << data->DebugString();
  rsp_data_pack.wheel_current_data_.emplace_back(std::move(data));

  return index;
}

uint32_t KobukiChassisDataParser::ParseHardwareVersion(
    const Serial::BytesFrame& data_frame, const uint32_t& header_index,
    RspDataPack& rsp_data_pack) {
  if (!SubPayloadCheck(data_frame, header_index,
                       HardwareVersionRspData::kLength_,
                       typeid(HardwareVersionRspData).name())) {
    return data_frame.size();
  }

  while (rsp_data_pack.hardware_version_.size() > RspDataPack::kMaxDequeSize_) {
    rsp_data_pack.hardware_version_.pop_front();
  }
  auto data = std::make_unique<HardwareVersionRspData>();
  auto index = header_index + 2;
  data->patch_ = data_frame.at(index++);
  data->minor_ = data_frame.at(index++);
  data->major_ = data_frame.at(index++);
  ZINFO << data->DebugString();
  rsp_data_pack.hardware_version_.emplace_back(std::move(data));
  index += 1;  // Skip unused data.

  return index;
}

uint32_t KobukiChassisDataParser::ParseFirmwareVersion(
    const Serial::BytesFrame& data_frame, const uint32_t& header_index,
    RspDataPack& rsp_data_pack) {
  if (!SubPayloadCheck(data_frame, header_index,
                       FirmwareVersionRspData::kLength_,
                       typeid(FirmwareVersionRspData).name())) {
    return data_frame.size();
  }

  while (rsp_data_pack.firmware_version_.size() > RspDataPack::kMaxDequeSize_) {
    rsp_data_pack.firmware_version_.pop_front();
  }
  auto data = std::make_unique<FirmwareVersionRspData>();
  auto index = header_index + 2;
  data->patch_ = data_frame.at(index++);
  data->minor_ = data_frame.at(index++);
  data->major_ = data_frame.at(index++);
  ZINFO << data->DebugString();
  rsp_data_pack.firmware_version_.emplace_back(std::move(data));
  index += 1;  // Skip unused data.

  return index;
}

uint32_t KobukiChassisDataParser::ParseGyro(
    const Serial::BytesFrame& data_frame, const uint32_t& header_index,
    RspDataPack& rsp_data_pack) {
  if (header_index + 1 >= data_frame.size()) {
    ZWARN << typeid(GyroRawRspData).name() << " insufficient data at "
          << std::to_string(header_index) << ", need " << std::to_string(1)
          << " more data but data size was "
          << std::to_string(data_frame.size());
    return data_frame.size();
  }
  if (((data_frame.at(header_index + 1) - 2) % 6) != 0) {
    ZWARN << typeid(GyroRawRspData).name() << " invalid sub payload length "
          << std::to_string(data_frame.at(header_index + 1))
          << ", expect to be 6N+2.";
    return data_frame.size();
  }
  auto gyro_count = (data_frame.at(header_index + 1) - 2) / 6;
  if (gyro_count == 0) {
    ZWARN << typeid(GyroRawRspData).name() << " invalid gyro count "
          << std::to_string(gyro_count);
    return data_frame.size();
  }

  while (rsp_data_pack.gyro_raw_data_.size() > RspDataPack::kMaxDequeSize_) {
    rsp_data_pack.gyro_raw_data_.pop_front();
  }
  // ZINFO << "Data length: " << std::to_string(data_frame.at(header_index + 1))
  // << ", gyro count "
  //       << gyro_count;
  auto data = std::make_unique<GyroRawRspData>();
  auto index = header_index + 2;
  data->frame_id_ = data_frame.at(index++);
  data->data_len_ = data_frame.at(index++);
  for (auto gyro_index = 0; gyro_index < gyro_count; gyro_index++) {
    GyroRawRspData::GyroData gyro;
    gyro.x_axis_ = data_frame.at(index++);
    gyro.x_axis_ += (data_frame.at(index++) << 8);
    gyro.y_axis_ = data_frame.at(index++);
    gyro.y_axis_ += (data_frame.at(index++) << 8);
    gyro.z_axis_ = data_frame.at(index++);
    gyro.z_axis_ += (data_frame.at(index++) << 8);
    data->gyros_.emplace_back(gyro);
  }
  // ZINFO << data->DebugString();
  rsp_data_pack.gyro_raw_data_.emplace_back(std::move(data));

  return index;
}

uint32_t KobukiChassisDataParser::ParseGeneralPurposeInput(
    const Serial::BytesFrame& data_frame, const uint32_t& header_index,
    RspDataPack& rsp_data_pack) {
  if (!SubPayloadCheck(data_frame, header_index,
                       GeneralPurposeInputRspData::kLength_,
                       typeid(GeneralPurposeInputRspData).name())) {
    return data_frame.size();
  }

  while (rsp_data_pack.general_purpose_input_.size() >
         RspDataPack::kMaxDequeSize_) {
    rsp_data_pack.general_purpose_input_.pop_front();
  }
  auto data = std::make_unique<GeneralPurposeInputRspData>();
  auto index = header_index + 2;
  data->digital_channel_enabled_flags_ = data_frame.at(index++);
  data->digital_channel_enabled_flags_ += (data_frame.at(index++) << 8);
  data->channel_0_adc_ = data_frame.at(index++);
  data->channel_0_adc_ += (data_frame.at(index++) << 8);
  data->channel_0_voltage_ =
      GeneralPurposeInputRspData::AdcToVoltage(data->channel_0_adc_);
  data->channel_1_adc_ = data_frame.at(index++);
  data->channel_1_adc_ += (data_frame.at(index++) << 8);
  data->channel_1_voltage_ =
      GeneralPurposeInputRspData::AdcToVoltage(data->channel_1_adc_);
  data->channel_2_adc_ = data_frame.at(index++);
  data->channel_2_adc_ += (data_frame.at(index++) << 8);
  data->channel_2_voltage_ =
      GeneralPurposeInputRspData::AdcToVoltage(data->channel_2_adc_);
  data->channel_3_adc_ = data_frame.at(index++);
  data->channel_3_adc_ += (data_frame.at(index++) << 8);
  data->channel_3_voltage_ =
      GeneralPurposeInputRspData::AdcToVoltage(data->channel_3_adc_);
  // ZINFO << data->DebugString();
  rsp_data_pack.general_purpose_input_.emplace_back(std::move(data));
  index += 6;  // Skip unused data.

  return index;
}

uint32_t KobukiChassisDataParser::ParseUDID(
    const Serial::BytesFrame& data_frame, const uint32_t& header_index,
    RspDataPack& rsp_data_pack) {
  if (!SubPayloadCheck(data_frame, header_index, UDIDRspData::kLength_,
                       typeid(UDIDRspData).name())) {
    return data_frame.size();
  }

  while (rsp_data_pack.UDID_.size() > RspDataPack::kMaxDequeSize_) {
    rsp_data_pack.UDID_.pop_front();
  }
  auto data = std::make_unique<UDIDRspData>();
  auto index = header_index + 2;
  data->UDID0_ = data_frame.at(index++);
  data->UDID0_ += (data_frame.at(index++) << 8);
  data->UDID0_ += (data_frame.at(index++) << 16);
  data->UDID0_ += (data_frame.at(index++) << 24);
  data->UDID1_ = data_frame.at(index++);
  data->UDID1_ += (data_frame.at(index++) << 8);
  data->UDID1_ += (data_frame.at(index++) << 16);
  data->UDID1_ += (data_frame.at(index++) << 24);
  data->UDID2_ = data_frame.at(index++);
  data->UDID2_ += (data_frame.at(index++) << 8);
  data->UDID2_ += (data_frame.at(index++) << 16);
  data->UDID2_ += (data_frame.at(index++) << 24);
  ZINFO << data->DebugString();
  rsp_data_pack.UDID_.emplace_back(std::move(data));

  return index;
}

uint32_t KobukiChassisDataParser::ParseControllerGain(
    const Serial::BytesFrame& data_frame, const uint32_t& header_index,
    RspDataPack& rsp_data_pack) {
  if (!SubPayloadCheck(data_frame, header_index,
                       ControllerGainRspData::kLength_,
                       typeid(ControllerGainRspData).name())) {
    return data_frame.size();
  }

  // ZINFO << Serial::DebugString(Serial::BytesFrame(
  //     data_frame.begin() + header_index,
  //     data_frame.begin() + header_index + 2 +
  //     ControllerGainRspData::kLength_));
  while (rsp_data_pack.controller_gain_data_.size() >
         RspDataPack::kMaxDequeSize_) {
    rsp_data_pack.controller_gain_data_.pop_front();
  }
  auto data = std::make_unique<ControllerGainRspData>();
  auto index = header_index + 2;
  data->type_ = data_frame.at(index++);
  data->p_x1000_ = static_cast<uint32_t>(data_frame.at(index++));
  data->p_x1000_ += (static_cast<uint32_t>(data_frame.at(index++)) << 8);
  data->p_x1000_ += (static_cast<uint32_t>(data_frame.at(index++)) << 16);
  data->p_x1000_ += (static_cast<uint32_t>(data_frame.at(index++)) << 24);
  data->i_x1000_ = static_cast<uint32_t>(data_frame.at(index++));
  data->i_x1000_ += (static_cast<uint32_t>(data_frame.at(index++)) << 8);
  data->i_x1000_ += (static_cast<uint32_t>(data_frame.at(index++)) << 16);
  data->i_x1000_ += (static_cast<uint32_t>(data_frame.at(index++)) << 24);
  data->d_x1000_ = static_cast<uint32_t>(data_frame.at(index++));
  data->d_x1000_ += (static_cast<uint32_t>(data_frame.at(index++)) << 8);
  data->d_x1000_ += (static_cast<uint32_t>(data_frame.at(index++)) << 16);
  data->d_x1000_ += (static_cast<uint32_t>(data_frame.at(index++)) << 24);

  ZINFO << data->DebugString();
  rsp_data_pack.controller_gain_data_.emplace_back(std::move(data));

  return index;
}

}  // namespace zima
