/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_HAL_CHASSIS_KOBUKI_PROTOCAL_H_
#define ZIMA_HAL_CHASSIS_KOBUKI_PROTOCAL_H_

#include "zima/hal/io/serial.h"

namespace zima {

class KobukiChassisSerialProtocalParser {
 public:
  KobukiChassisSerialProtocalParser()
      : progress_(kForHeader1), expect_data_len_(0), cache_data_len_(0){};
  ~KobukiChassisSerialProtocalParser() = default;

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

class KobukiChassisDataParser {
 public:
  KobukiChassisDataParser() = default;
  ~KobukiChassisDataParser() = default;

  class DataBase {
   public:
    virtual std::string DebugString() = 0;
  };

  class BaseControlCmdData : public DataBase {
   public:
    std::string DebugString() override;
    using UPtr = std::unique_ptr<BaseControlCmdData>;

    void SetVelocityCmdByWheel(const float& left_wheel_speed,
                               const float& right_wheel_speed,
                               const float& track_length);
    void SetVelocityCmd(const float& linear_x, const float& angular_z,
                        const float& track_length);

    void CalSpeedNRadius(const float& track_length);

    static const uint8_t kCmdHeader_;
    static const uint8_t kLength_;
    float left_wheel_speed_{0};
    float right_wheel_speed_{0};
    float linear_x_{0};
    float angular_z_{0};
    float speed_mm_{0};
    float radius_mm_{0};
  };

  class SoundCmdData : public DataBase {
   public:
    std::string DebugString() override;
    using UPtr = std::unique_ptr<SoundCmdData>;

    // note_ = 1 / (frequency * kA_)
    void CalNote(const float& frequency);

    static const uint8_t kCmdHeader_;
    static const uint8_t kLength_;
    float note_{0};
    float duration_ms_{0};
    static const double kA_;
  };

  class SoundSequenceCmdData : public DataBase {
   public:
    std::string DebugString() override;
    using UPtr = std::unique_ptr<SoundSequenceCmdData>;

    static const uint8_t kCmdHeader_;
    static const uint8_t kLength_;
    static const uint8_t kOnSound_;
    static const uint8_t kOffSound_;
    static const uint8_t kRechargeSound_;
    static const uint8_t kButtonSound_;
    static const uint8_t kErrorSound_;
    static const uint8_t kCleaningStartSound_;
    static const uint8_t kCleaningEndSound_;
    uint8_t sequence_num_{0};
  };

  class RequestExtraCmdData : public DataBase {
   public:
    std::string DebugString() override;
    using UPtr = std::unique_ptr<RequestExtraCmdData>;

    static const uint8_t kCmdHeader_;
    static const uint8_t kLength_;
    static const uint16_t kHardwareVersion_;
    static const uint16_t kFirmwareVersion_;
    static const uint16_t kUDID_;
    uint16_t request_flag_{0};
  };

  class GeneralPurposeOutputCmdData : public DataBase {
   public:
    std::string DebugString() override;
    using UPtr = std::unique_ptr<GeneralPurposeOutputCmdData>;

    static const uint8_t kCmdHeader_;
    static const uint8_t kLength_;
    static const uint16_t kDigChannel0_;
    static const uint16_t kDigChannel1_;
    static const uint16_t kDigChannel2_;
    static const uint16_t kDigChannel3_;
    static const uint16_t kPower3p3V_;
    static const uint16_t kPower5V_;
    static const uint16_t kPower12V5A_;
    static const uint16_t kPower12V1p5A_;
    static const uint16_t kLed1Red_;
    static const uint16_t kLed1Green_;
    static const uint16_t kLed2Red_;
    static const uint16_t kLed2Green_;
    uint16_t config_{0};
  };

  class SetControllerGainCmdData : public DataBase {
   public:
    std::string DebugString() override;
    using UPtr = std::unique_ptr<SetControllerGainCmdData>;

    static const uint8_t kCmdHeader_;
    static const uint8_t kLength_;
    static const uint8_t kDefaultPID_;
    static const uint8_t kCustomPID_;
    uint8_t type_{0};
    float p_x1000_{100000};  // default as 100*1000
    float i_x1000_{100};     // default as 0.1*1000
    float d_x1000_{2000};    // default as 2*1000
  };

  class GetControllerGainCmdData : public DataBase {
   public:
    std::string DebugString() override;
    using UPtr = std::unique_ptr<GetControllerGainCmdData>;

    static const uint8_t kCmdHeader_;
    static const uint8_t kLength_;
    uint8_t data_{0};  // No use.
  };

  class CmdDataPack {
   public:
    using UPtr = std::unique_ptr<CmdDataPack>;

    std::deque<BaseControlCmdData::UPtr> base_control_cmd_data_;
    std::deque<SoundCmdData::UPtr> sound_cmd_data_;
    std::deque<SoundSequenceCmdData::UPtr> sound_sequence_cmd_data_;
    std::deque<RequestExtraCmdData::UPtr> request_extra_cmd_data_;
    std::deque<GeneralPurposeOutputCmdData::UPtr>
        general_purpose_output_cmd_data_;
    std::deque<SetControllerGainCmdData::UPtr> set_controller_gain_cmd_data_;
    std::deque<GetControllerGainCmdData::UPtr> get_controller_gain_cmd_data_;
  };

  class BasicSensorRspData : public DataBase {
   public:
    std::string DebugString() override;
    void PrintOnTriggered();
    using UPtr = std::unique_ptr<BasicSensorRspData>;

    static const uint8_t kRspHeader_;
    static const uint8_t kLength_;
    uint16_t timestamp_{0};
    static const uint8_t kRightBumperPressed_;
    static const uint8_t kCentralBumperPressed_;
    static const uint8_t kLeftBumperPressed_;
    uint8_t bumper_pressed_{0};
    static const uint8_t kRightWheelDrop_;
    static const uint8_t kLeftWheelDrop_;
    uint8_t wheel_drop_{0};
    static const uint8_t kRightCliffDetected_;
    static const uint8_t kCentralCliffDetected_;
    static const uint8_t kLeftCliffDetected_;
    uint8_t cliff_detected_{0};
    uint16_t left_encoder_{0};
    uint16_t right_encoder_{0};
    int8_t left_pwm_{0};
    int8_t right_pwm_{0};
    static const uint8_t kButton0Pressed_;
    static const uint8_t kButton1Pressed_;
    static const uint8_t kButton2Pressed_;
    uint8_t button_pressed_{0};
    static const uint8_t kDisCharging_;
    static const uint8_t kDockCharged_;
    static const uint8_t kDockCharging_;
    static const uint8_t kAdapterCharged_;
    static const uint8_t kAdapterCharging_;
    uint8_t charging_state_{0};
    uint8_t battery_p1V_{0};
    static const uint8_t kLeftWheelOC_;
    static const uint8_t kRightWheelOC_;
    uint8_t wheel_OC_{0};
  };

  class DockIRRspData : public DataBase {
   public:
    std::string DebugString() override;
    using UPtr = std::unique_ptr<DockIRRspData>;

    static const uint8_t kRspHeader_;
    static const uint8_t kLength_;
    static const uint8_t kNearLeft_;
    static const uint8_t kNearCenter_;
    static const uint8_t kNearRight_;
    static const uint8_t kFarLeft_;
    static const uint8_t kFarCenter_;
    static const uint8_t kFarRight_;

    uint8_t right_ir_received_{0};
    uint8_t center_ir_received_{0};
    uint8_t left_ir_received_{0};
  };

  class InertialSensorRspData : public DataBase {
   public:
    std::string DebugString() override;
    using UPtr = std::unique_ptr<InertialSensorRspData>;

    static const uint8_t kRspHeader_;
    static const uint8_t kLength_;
    int16_t angle_{0};
    int16_t angle_rate_{0};
  };

  class CliffSensorRspData : public DataBase {
   public:
    std::string DebugString() override;
    using UPtr = std::unique_ptr<CliffSensorRspData>;

    static float AdcToCm(const uint16_t& adc_value);

    static const uint8_t kRspHeader_;
    static const uint8_t kLength_;
    static const uint16_t kAdcMin_;
    static const uint16_t kAdcMax_;
    static const float kDistanceMin_;
    static const float kDistanceMax_;
    uint16_t right_cliff_adc_{0};
    float right_cliff_distance_{0};
    uint16_t central_cliff_adc_{0};
    float central_cliff_distance_{0};
    uint16_t left_cliff_adc_{0};
    float left_cliff_distance_{0};
  };

  class WheelCurrentRspData : public DataBase {
   public:
    std::string DebugString() override;
    using UPtr = std::unique_ptr<WheelCurrentRspData>;

    static const uint8_t kRspHeader_;
    static const uint8_t kLength_;
    uint8_t left_wheel_current_10mA_{0};
    uint8_t right_wheel_current_10mA_{0};
  };

  class HardwareVersionRspData : public DataBase {
   public:
    std::string DebugString() override;
    using UPtr = std::unique_ptr<HardwareVersionRspData>;

    static const uint8_t kRspHeader_;
    static const uint8_t kLength_;
    uint8_t patch_{0};
    uint8_t minor_{0};
    uint8_t major_{0};
  };

  class FirmwareVersionRspData : public DataBase {
   public:
    std::string DebugString() override;
    using UPtr = std::unique_ptr<FirmwareVersionRspData>;

    static const uint8_t kRspHeader_;
    static const uint8_t kLength_;
    uint8_t patch_{0};
    uint8_t minor_{0};
    uint8_t major_{0};
  };

  class GyroRawRspData : public DataBase {
   public:
    std::string DebugString() override;
    using UPtr = std::unique_ptr<GyroRawRspData>;

    static const uint8_t kRspHeader_;
    static const uint8_t kLength_;
    uint8_t frame_id_;
    uint8_t data_len_;
    class GyroData {
     public:
      int16_t x_axis_;
      int16_t y_axis_;
      int16_t z_axis_;
    };

    std::deque<GyroData> gyros_;
  };

  class GeneralPurposeInputRspData : public DataBase {
   public:
    std::string DebugString() override;
    using UPtr = std::unique_ptr<GeneralPurposeInputRspData>;

    static float AdcToVoltage(const uint16_t& adc);

    static const uint8_t kRspHeader_;
    static const uint8_t kLength_;
    static const uint16_t kDigChannel0Enable_;
    static const uint16_t kDigChannel1Enable_;
    static const uint16_t kDigChannel2Enable_;
    static const uint16_t kDigChannel3Enable_;
    static const uint16_t kAdcMin_;
    static const uint16_t kAdcMax_;
    static const float kVoltageMin_;
    static const float kVoltageMax_;
    uint16_t digital_channel_enabled_flags_{0};
    uint16_t channel_0_adc_{0};
    float channel_0_voltage_{0};
    uint16_t channel_1_adc_{0};
    float channel_1_voltage_{0};
    uint16_t channel_2_adc_{0};
    float channel_2_voltage_{0};
    uint16_t channel_3_adc_{0};
    float channel_3_voltage_{0};
  };

  class UDIDRspData : public DataBase {
   public:
    std::string DebugString() override;
    using UPtr = std::unique_ptr<UDIDRspData>;

    static const uint8_t kRspHeader_;
    static const uint8_t kLength_;
    uint32_t UDID0_{0};
    uint32_t UDID1_{0};
    uint32_t UDID2_{0};
  };

  class ControllerGainRspData : public DataBase {
   public:
    std::string DebugString() override;
    using UPtr = std::unique_ptr<ControllerGainRspData>;

    static const uint8_t kRspHeader_;
    static const uint8_t kLength_;
    static const uint8_t kDefaultPID_;
    static const uint8_t kCustomPID_;
    uint8_t type_{0};
    uint32_t p_x1000_{100000};  // default as 100*1000
    uint32_t i_x1000_{100};     // default as 0.1*1000
    uint32_t d_x1000_{2000};    // default as 2*1000
  };

  class RspDataPack {
   public:
    void MoveFrom(RspDataPack& source);

    using UPtr = std::unique_ptr<RspDataPack>;

    static const uint8_t kMaxDequeSize_;
    std::deque<BasicSensorRspData::UPtr> basic_sensor_data_;
    std::deque<DockIRRspData::UPtr> Dock_IR_data_;
    std::deque<InertialSensorRspData::UPtr> inertial_sensor_data_;
    std::deque<CliffSensorRspData::UPtr> cliff_sensor_data_;
    std::deque<WheelCurrentRspData::UPtr> wheel_current_data_;
    std::deque<HardwareVersionRspData::UPtr> hardware_version_;
    std::deque<FirmwareVersionRspData::UPtr> firmware_version_;
    std::deque<GyroRawRspData::UPtr> gyro_raw_data_;
    std::deque<GeneralPurposeInputRspData::UPtr> general_purpose_input_;
    std::deque<UDIDRspData::UPtr> UDID_;
    std::deque<ControllerGainRspData::UPtr> controller_gain_data_;
  };

  static uint32_t kFrameMaxSize_;

  bool ParseReadData(const Serial::BytesFrame& valid_frame,
                     RspDataPack& rsp_data_pack);
  Serial::BytesFrame PackCmdFrameData(CmdDataPack& cmd_pack);

 private:
  bool SubPayloadCheck(const Serial::BytesFrame& data_frame,
                       const uint32_t& header_index,
                       const uint8_t& expect_data_length,
                       const std::string& name);

  uint32_t ParseBasicSensorData(const Serial::BytesFrame& data_frame,
                                const uint32_t& header_index,
                                RspDataPack& rsp_data_pack);
  uint32_t ParseDockIR(const Serial::BytesFrame& data_frame,
                          const uint32_t& header_index,
                          RspDataPack& rsp_data_pack);
  uint32_t ParseInertialSensor(const Serial::BytesFrame& data_frame,
                               const uint32_t& header_index,
                               RspDataPack& rsp_data_pack);
  uint32_t ParseCliffSensor(const Serial::BytesFrame& data_frame,
                            const uint32_t& header_index,
                            RspDataPack& rsp_data_pack);
  uint32_t ParseWheelCurrent(const Serial::BytesFrame& data_frame,
                             const uint32_t& header_index,
                             RspDataPack& rsp_data_pack);
  uint32_t ParseHardwareVersion(const Serial::BytesFrame& data_frame,
                                const uint32_t& header_index,
                                RspDataPack& rsp_data_pack);
  uint32_t ParseFirmwareVersion(const Serial::BytesFrame& data_frame,
                                const uint32_t& header_index,
                                RspDataPack& rsp_data_pack);
  uint32_t ParseGyro(const Serial::BytesFrame& data_frame,
                     const uint32_t& header_index, RspDataPack& rsp_data_pack);
  uint32_t ParseGeneralPurposeInput(const Serial::BytesFrame& data_frame,
                                    const uint32_t& header_index,
                                    RspDataPack& rsp_data_pack);
  uint32_t ParseUDID(const Serial::BytesFrame& data_frame,
                     const uint32_t& header_index, RspDataPack& rsp_data_pack);
  uint32_t ParseControllerGain(const Serial::BytesFrame& data_frame,
                               const uint32_t& header_index,
                               RspDataPack& rsp_data_pack);
};

}  // namespace zima

#endif  // ZIMA_HAL_CHASSIS_KOBUKI_PROTOCAL_H_
