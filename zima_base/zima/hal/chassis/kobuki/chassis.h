/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_HAL_CHASSIS_KOBUKI_CHASSIS_H_
#define ZIMA_HAL_CHASSIS_KOBUKI_CHASSIS_H_

#include "zima/common/thread.h"
#include "zima/hal/chassis/kobuki/protocal.h"
#include "zima/hal/io/serial.h"
#include "zima/robot/chassis.h"

namespace zima {

class KobukiChassisSerial : public Serial {
 public:
  KobukiChassisSerial() = delete;
  KobukiChassisSerial(const std::string& serial_port, const int& baud);
  ~KobukiChassisSerial();

  using SPtr = std::shared_ptr<KobukiChassisSerial>;

  bool ParseFrame(const uint8_t& recv_byte) override;
  bool ParseReadData(const Serial::BytesFrame& valid_frame);
  bool PackWriteFrame();

  void SetVelocityCmdByWheel(const float& left_wheel_speed,
                             const float& right_wheel_speed,
                             const float& track_length);
  void SetVelocityCmd(const float& linear_x, const float& linear_y,
                      const float& angular_z);
  void RequestExtraData();
  void GetPID();
  bool SetPID(const float& p, const float& i, const float& d);

  bool GetBasicSensorData(
      KobukiChassisDataParser::BasicSensorRspData::UPtr& data);
  bool GetInertialSensorData(
      KobukiChassisDataParser::InertialSensorRspData::UPtr& data);

  bool StartThread(const float& p, const float& i, const float& d);
  bool ShutdownThread();

 private:
  void CommReadThread(const ZimaThreadWrapper::ThreadParam& param);
  void CommWriteThread(const ZimaThreadWrapper::ThreadParam& param,
                       const float& p, const float& i, const float& d);

  ReadWriteLock::SPtr access_;

  KobukiChassisSerialProtocalParser protocal_parser_;
  KobukiChassisDataParser data_parser_;

  // Read data
  KobukiChassisDataParser::RspDataPack cached_rsp_data_pack_;

  // Write data
  KobukiChassisDataParser::CmdDataPack cached_cmd_data_pack_;
  KobukiChassisDataParser::GeneralPurposeOutputCmdData
      cached_general_purpose_output_cmd_;

  zima::ZimaThreadWrapper::ThreadParam comm_read_thread_param_;
  zima::ZimaThreadWrapper::ThreadParam comm_write_thread_param_;
  bool shutdown_request_;
};

class KobukiChassis : public Chassis, public DebugBase {
 public:
  KobukiChassis() = delete;
  KobukiChassis(const std::string& port);
  ~KobukiChassis() = default;

  void Initialize() override;

  KobukiChassisSerial::SPtr GetSerialRef() const { return serial_; };

  bool StartThread(const MergedOdomDataCb& call_back_func);
  bool ShutdownThread();

 private:
  void DataProcessThread(const ZimaThreadWrapper::ThreadParam& param,
                         const MergedOdomDataCb& call_back_func);

  zima::ZimaThreadWrapper::ThreadParam data_process_thread_param_;

  KobukiChassisSerial::SPtr serial_;
  bool shutdown_request_;
};

}  // namespace zima

#endif  // ZIMA_HAL_CHASSIS_KOBUKI_CHASSIS_H_
