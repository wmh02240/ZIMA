/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/hal/chassis/kobuki/chassis.h"

#include <memory>

#include "zima/common/time.h"
#include "zima/grid_map/nav_map_2d.h"

namespace zima {

KobukiChassisSerial::KobukiChassisSerial(const std::string& serial_port,
                                         const int& baud)
    : Serial(serial_port, baud),
      access_(std::make_shared<ReadWriteLock>()),
      shutdown_request_(false) {
  cached_general_purpose_output_cmd_.config_ |=
      KobukiChassisDataParser::GeneralPurposeOutputCmdData::kDigChannel0_;
  cached_general_purpose_output_cmd_.config_ |=
      KobukiChassisDataParser::GeneralPurposeOutputCmdData::kDigChannel1_;
  cached_general_purpose_output_cmd_.config_ |=
      KobukiChassisDataParser::GeneralPurposeOutputCmdData::kDigChannel2_;
  cached_general_purpose_output_cmd_.config_ |=
      KobukiChassisDataParser::GeneralPurposeOutputCmdData::kDigChannel3_;

  cached_general_purpose_output_cmd_.config_ |=
      KobukiChassisDataParser::GeneralPurposeOutputCmdData::kPower3p3V_;
  cached_general_purpose_output_cmd_.config_ |=
      KobukiChassisDataParser::GeneralPurposeOutputCmdData::kPower5V_;
  cached_general_purpose_output_cmd_.config_ |=
      KobukiChassisDataParser::GeneralPurposeOutputCmdData::kPower12V5A_;
  cached_general_purpose_output_cmd_.config_ |=
      KobukiChassisDataParser::GeneralPurposeOutputCmdData::kPower12V1p5A_;

  cached_general_purpose_output_cmd_.config_ |=
      KobukiChassisDataParser::GeneralPurposeOutputCmdData::kLed1Green_;
  cached_general_purpose_output_cmd_.config_ &=
      ~KobukiChassisDataParser::GeneralPurposeOutputCmdData::kLed1Red_;
  cached_general_purpose_output_cmd_.config_ |=
      KobukiChassisDataParser::GeneralPurposeOutputCmdData::kLed2Green_;
  cached_general_purpose_output_cmd_.config_ |=
      ~KobukiChassisDataParser::GeneralPurposeOutputCmdData::kLed2Red_;
  ZINFO << "Initialize general purpose output cmd as: "
        << cached_general_purpose_output_cmd_.DebugString();

  comm_read_thread_param_ = zima::ZimaThreadWrapper::ThreadParam(
      "Kobuki comm read thread",
      zima::ZimaThreadManager::kRealtimeThreadIndex_, 100, 0.2, 1);

  comm_write_thread_param_ = zima::ZimaThreadWrapper::ThreadParam(
      "Kobuki comm write thread",
      zima::ZimaThreadManager::kRealtimeThreadIndex_, 100, 0.2, 1);
};

KobukiChassisSerial::~KobukiChassisSerial() {
  ZINFO;
  SetVelocityCmd(0, 0, 0);
  PackWriteFrame();
  Serial::BytesFrames write_frames;
  if (GetWriteFrames(write_frames)) {
    for (auto& frame : write_frames) {
      Write(frame);
    }
  }

  Close();
}

bool KobukiChassisSerial::ParseFrame(const uint8_t& recv_byte) {
  WriteLocker lock(data_access_);
  return protocal_parser_.ParseReadFrame(recv_byte, cache_bytes_,
                                         valid_frames_data_);
};

bool KobukiChassisSerial::ParseReadData(
    const Serial::BytesFrame& valid_frame_data) {
  return data_parser_.ParseReadData(valid_frame_data, cached_rsp_data_pack_);
}

bool KobukiChassisSerial::PackWriteFrame() {
  WriteLocker lock(access_);
  auto data = data_parser_.PackCmdFrameData(cached_cmd_data_pack_);
  if (!data.empty()) {
    CacheWriteFrames(protocal_parser_.PackWriteFrame(data));
  }
  return true;
}

void KobukiChassisSerial::SetVelocityCmdByWheel(const float& left_wheel_speed,
                                                const float& right_wheel_speed,
                                                const float& track_length) {
  WriteLocker lock(access_);
  auto base_control_cmd =
      std::make_unique<KobukiChassisDataParser::BaseControlCmdData>();
  base_control_cmd->SetVelocityCmdByWheel(left_wheel_speed, right_wheel_speed,
                                          track_length);
  cached_cmd_data_pack_.base_control_cmd_data_.emplace_back(
      std::move(base_control_cmd));
}

void KobukiChassisSerial::SetVelocityCmd(const float& linear_x,
                                         const float& angular_z,
                                         const float& track_length) {
  WriteLocker lock(access_);
  auto base_control_cmd =
      std::make_unique<KobukiChassisDataParser::BaseControlCmdData>();
  base_control_cmd->SetVelocityCmd(linear_x, angular_z, track_length);
  cached_cmd_data_pack_.base_control_cmd_data_.emplace_back(
      std::move(base_control_cmd));
}

void KobukiChassisSerial::RequestExtraData() {
  WriteLocker lock(access_);
  auto cmd = std::make_unique<KobukiChassisDataParser::RequestExtraCmdData>();
  cmd->request_flag_ |=
      KobukiChassisDataParser::RequestExtraCmdData::kHardwareVersion_;
  cmd->request_flag_ |=
      KobukiChassisDataParser::RequestExtraCmdData::kFirmwareVersion_;
  cmd->request_flag_ |= KobukiChassisDataParser::RequestExtraCmdData::kUDID_;
  cached_cmd_data_pack_.request_extra_cmd_data_.emplace_back(std::move(cmd));
}

void KobukiChassisSerial::GetPID() {
  WriteLocker lock(access_);
  auto cmd =
      std::make_unique<KobukiChassisDataParser::GetControllerGainCmdData>();
  cached_cmd_data_pack_.get_controller_gain_cmd_data_.emplace_back(
      std::move(cmd));
}

bool KobukiChassisSerial::SetPID(const float& p, const float& i,
                                 const float& d) {
  if (p < 0 || i < 0 || d < 0) {
    ZWARN << "pid param invalid: p " << FloatToString(p, 1) << ", i "
          << FloatToString(i, 1) << ", d " << FloatToString(d, 1);
    return false;
  }
  auto cmd =
      std::make_unique<KobukiChassisDataParser::SetControllerGainCmdData>();
  cmd->type_ = KobukiChassisDataParser::SetControllerGainCmdData::kCustomPID_;
  cmd->p_x1000_ = p * 1000;
  cmd->i_x1000_ = i * 1000;
  cmd->d_x1000_ = d * 1000;
  cached_cmd_data_pack_.set_controller_gain_cmd_data_.emplace_back(
      std::move(cmd));
  return true;
}

bool KobukiChassisSerial::GetBasicSensorData(
    KobukiChassisDataParser::BasicSensorRspData::UPtr& data) {
  WriteLocker lock(access_);
  if (cached_rsp_data_pack_.basic_sensor_data_.empty()) {
    return false;
  }
  data = std::move(cached_rsp_data_pack_.basic_sensor_data_.front());
  cached_rsp_data_pack_.basic_sensor_data_.pop_front();
  return true;
}

bool KobukiChassisSerial::GetInertialSensorData(
    KobukiChassisDataParser::InertialSensorRspData::UPtr& data) {
  WriteLocker lock(access_);
  if (cached_rsp_data_pack_.inertial_sensor_data_.empty()) {
    return false;
  }
  data = std::move(cached_rsp_data_pack_.inertial_sensor_data_.front());
  cached_rsp_data_pack_.inertial_sensor_data_.pop_front();
  return true;
}

bool KobukiChassisSerial::StartThread(const float& p, const float& i,
                                      const float& d) {
  auto thread_manager = zima::ZimaThreadManager::Instance();
  if (!thread_manager->IsThreadRunning(comm_read_thread_param_.thread_name_)) {
    thread_manager->RegisterThread(
        std::thread(&KobukiChassisSerial::CommReadThread, this,
                    comm_read_thread_param_),
        comm_read_thread_param_);
  }
  if (!thread_manager->IsThreadRunning(comm_write_thread_param_.thread_name_)) {
    thread_manager->RegisterThread(
        std::thread(&KobukiChassisSerial::CommWriteThread, this,
                    comm_write_thread_param_, p, i, d),
        comm_write_thread_param_);
  }

  return true;
}

bool KobukiChassisSerial::ShutdownThread() {
  shutdown_request_ = true;
  auto thread_manager = zima::ZimaThreadManager::Instance();
  auto ret = true;
  if (!thread_manager->WaitForThreadExit(comm_read_thread_param_.thread_name_,
                                         1)) {
    ZERROR << "Comm read thread not exit.";
    ret = false;
  }
  if (!thread_manager->WaitForThreadExit(comm_write_thread_param_.thread_name_,
                                         1)) {
    ZERROR << "Comm write thread not exit.";
    ret = false;
  }

  return ret;
}

void KobukiChassisSerial::CommReadThread(
    const ZimaThreadWrapper::ThreadParam &param) {
  ZINFO << "Thread \"" << param.thread_name_ << "\" start.";
  auto thread_manager = ZimaThreadManager::Instance();
  if (param.bind_cpu_id_ >= 0) {
    thread_manager->BindCPUCore(param.thread_name_, param.bind_cpu_id_);
  }

  if (!Open()) {
    ZERROR;
    ZINFO << "Thread \"" << param.thread_name_ << "\" exiting.";
    thread_manager->MarkThreadExited(param.thread_name_, __FILE__, __FUNCTION__,
                                     __LINE__);
    ZINFO << "Thread \"" << param.thread_name_ << "\" exit.";
    return;
  }

  ZINFO << "Ready.";
  while (!shutdown_request_) {
    thread_manager->UpdateThreadCycle(param.thread_name_);
    Read();
  }
  Time::SleepMSec(100);

  ZINFO << "Thread \"" << param.thread_name_ << "\" exiting.";
  thread_manager->MarkThreadExited(param.thread_name_, __FILE__, __FUNCTION__,
                                   __LINE__);
  ZINFO << "Thread \"" << param.thread_name_ << "\" exit.";
}

void KobukiChassisSerial::CommWriteThread(
    const ZimaThreadWrapper::ThreadParam& param, const float& p, const float& i,
    const float& d) {
  ZINFO << "Thread \"" << param.thread_name_ << "\" start.";
  auto thread_manager = ZimaThreadManager::Instance();
  if (param.bind_cpu_id_ >= 0) {
    thread_manager->BindCPUCore(param.thread_name_, param.bind_cpu_id_);
  }

  if (!Open()) {
    ZERROR;

    ZINFO << "Thread \"" << param.thread_name_ << "\" exiting.";
    thread_manager->MarkThreadExited(param.thread_name_, __FILE__, __FUNCTION__,
                                     __LINE__);
    ZINFO << "Thread \"" << param.thread_name_ << "\" exit.";

    return;
  }

  ZINFO << "Ready.";
  RequestExtraData();

  SetPID(p / 10.0, i / 10.0, d / 10.0);

  Serial::BytesFrames write_frames;
  while (!shutdown_request_) {
    thread_manager->UpdateThreadCycle(param.thread_name_);
    PackWriteFrame();
    if (GetWriteFrames(write_frames)) {
      if (write_frames.size() > 1) {
        ZWARN << "Write buffer jamed.";
      }
      for (auto &frame : write_frames) {
        Write(frame);
      }
    }
    Time::SleepMSec(20);
  }
  Time::SleepMSec(100);

  ZINFO << "Thread \"" << param.thread_name_ << "\" exiting.";
  thread_manager->MarkThreadExited(param.thread_name_, __FILE__, __FUNCTION__,
                                   __LINE__);
  ZINFO << "Thread \"" << param.thread_name_ << "\" exit.";
}

// const double KobukiChassis::kWheelTickToDistance_ =
//     0.035 * 0.002436916871363930187454f;

KobukiChassis::KobukiChassis(const std::string& port)
    : Chassis(Chassis::Config()), shutdown_request_(false) {
  serial_.reset(new KobukiChassisSerial(port, 115200));
  data_process_thread_param_ = zima::ZimaThreadWrapper::ThreadParam(
      "Kobuki data process thread",
      zima::ZimaThreadManager::kRealtimeThreadIndex_, 100, 0.2, 1);
}

void KobukiChassis::Initialize() {
  ZINFO;
  // std::make_shared<Battery>(kBattery_, 16.5, 15, 14.5);
  // track_length_ = 0.23;
  // Fake radius as a real cleaner robot.
  // radius_ = 0.18;
  if (!config_.config_valid_) {
    ZERROR << "Config not valid.";
    return;
  }

  track_length_ = config_.track_length_;
  radius_ = config_.radius_;
  encircle_obstacle_on_left_ = config_.encircle_obstacle_on_left_;

  std::vector<Wheel::SPtr> wheel_infos;
  std::vector<Bumper::SPtr> bumper_infos;
  std::vector<Button::SPtr> button_infos;
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
    if (item.key() == kButton1_) {
      Button::Config config(std::make_shared<Json>(item.value()));
      if (!config.config_valid_) {
        ZERROR << "Invalid config:\n"
               << item.key() << ": " << item.value().dump();
        continue;
      }
      button_infos.emplace_back(std::make_shared<Button>(item.key(), config));
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
  InitializeButton(button_infos);
  InitializeWallSensor(wall_sensor_infos);
  InitializeGyro(gyro_infos);
  InitializeLidar(lidar_infos);
  InitializeBattery(battery_infos);
}

bool KobukiChassis::StartThread(const MergedOdomDataCb& call_back_func) {
  auto thread_manager = zima::ZimaThreadManager::Instance();
  if (!thread_manager->IsThreadRunning(
          data_process_thread_param_.thread_name_)) {
    thread_manager->RegisterThread(
        std::thread(&KobukiChassis::DataProcessThread, this,
                    data_process_thread_param_, call_back_func),
        data_process_thread_param_);
  }
  serial_->StartThread(GetWheel(kLeftWheel_)->GetSpeedControlP(),
                       GetWheel(kLeftWheel_)->GetSpeedControlI(),
                       GetWheel(kLeftWheel_)->GetSpeedControlD());
  return true;
}

bool KobukiChassis::ShutdownThread() {
  shutdown_request_ = true;
  auto thread_manager = zima::ZimaThreadManager::Instance();
  auto ret = true;
  if (!serial_->ShutdownThread()) {
    ret = false;
  }
  if (!thread_manager->WaitForThreadExit(
          data_process_thread_param_.thread_name_, 1)) {
    ret = false;
  }
  return ret;
}

void KobukiChassis::DataProcessThread(
    const ZimaThreadWrapper::ThreadParam& param,
    const MergedOdomDataCb& merged_odom_data_cb_func) {
  ZINFO << "Thread \"" << param.thread_name_ << "\" start.";
  auto thread_manager = ZimaThreadManager::Instance();
  if (param.bind_cpu_id_ >= 0) {
    thread_manager->BindCPUCore(param.thread_name_, param.bind_cpu_id_);
  }

  if (!serial_->Open()) {
    ZERROR;
    ZINFO << "Thread \"" << param.thread_name_ << "\" exiting.";
    thread_manager->MarkThreadExited(param.thread_name_, __FILE__, __FUNCTION__,
                                     __LINE__);
    ZINFO << "Thread \"" << param.thread_name_ << "\" exit.";
    return;
  }

  auto &tf = *TransformManager::Instance();
  ZINFO << "Ready.";
  Serial::BytesFrames valid_frames_data;

  KobukiChassisDataParser::BasicSensorRspData::UPtr last_basic_sensor_data =
      nullptr;
  // auto button_filter_start_time = Time::Now();
  while (!shutdown_request_) {
    thread_manager->UpdateThreadCycle(param.thread_name_);
    if (serial_->GetReadValidFramesData(valid_frames_data)) {
      if (valid_frames_data.size() > 1) {
        ZWARN << "Read buffer jamed.";
      }
      for (auto &frame_data : valid_frames_data) {
        if (!serial_->ParseReadData(frame_data)) {
          ZINFO << Serial::DebugString(frame_data);
        }
        // ZINFO << Serial::DebugString(frame_data);
        KobukiChassisDataParser::InertialSensorRspData::UPtr
            inertial_sensor_data = nullptr;
        while (serial_->GetInertialSensorData(inertial_sensor_data)) {
          auto degree = inertial_sensor_data->angle_ / 100.0;
          GetGyro(kGyro_)->SetDegree(degree, Time::Now());
          // Transform odom_tf("", "");
          // tf.GetTransform(tf.kOdomFrame_, tf.kRobotFrame_, odom_tf);
          // MapPoint new_odom_pose(odom_tf.X(), odom_tf.Y(), degree);
          // ZINFO << new_odom_pose.DebugString();
          // auto old_merged_odom_data = GetMergedOdomData();
          // if (old_merged_odom_data == nullptr) {
          //   auto merged_odom_data = std::make_shared<MergedOdomData>(
          //       Time::Now(), new_odom_pose, MapPoint());
          //   SetMergedOdomData(merged_odom_data);
          //   merged_odom_data_cb_func(merged_odom_data);
          // } else {
          //   auto velocity = old_merged_odom_data->GetVelocity();

          //   auto merged_odom_data = std::make_shared<MergedOdomData>(
          //       Time::Now(), new_odom_pose, velocity);
          //   SetMergedOdomData(merged_odom_data);
          //   merged_odom_data_cb_func(merged_odom_data);
          // }

          // Transform new_odom_tf(tf.kOdomFrame_, tf.kRobotFrame_,
          //                       new_odom_pose.X(), new_odom_pose.Y(),
          //                       new_odom_pose.Degree());
          // tf.UpdateTransform(new_odom_tf);
        }

        KobukiChassisDataParser::BasicSensorRspData::UPtr basic_sensor_data =
            nullptr;
        while (serial_->GetBasicSensorData(basic_sensor_data)) {
          if (last_basic_sensor_data == nullptr) {
            last_basic_sensor_data =
                std::make_unique<KobukiChassisDataParser::BasicSensorRspData>(
                    *basic_sensor_data);
            ZINFO << "Init for last data "
                  << last_basic_sensor_data->DebugString();
            continue;
          }
          auto left_wheel_distance_m =
              static_cast<int16_t>(basic_sensor_data->left_encoder_ -
                                   last_basic_sensor_data->left_encoder_) *
              GetWheel(kLeftWheel_)
                  ->GetWheelTickToDistance();
          auto right_wheel_distance_m =
              static_cast<int16_t>(basic_sensor_data->right_encoder_ -
                                   last_basic_sensor_data->right_encoder_) *
              GetWheel(kRightWheel_)
                  ->GetWheelTickToDistance();
          // ZINFO << "Distance Left: " << left_wheel_distance_m
          //       << ", right: " << right_wheel_distance_m;

          auto distance_diff =
              (right_wheel_distance_m + left_wheel_distance_m) / 2;
          // auto radian_diff =
          //     NormalizeRadian((right_wheel_distance_m -
          //     left_wheel_distance_m) /
          //                     GetTrackLength());
          auto diff_time_ms =
              static_cast<int16_t>(basic_sensor_data->timestamp_ -
                                   last_basic_sensor_data->timestamp_);
          // ZINFO << "Diff ms: " << diff_time_ms;
          auto diff_time_s = diff_time_ms / 1000.0;

          float left_speed = left_wheel_distance_m / diff_time_s;
          float right_speed = right_wheel_distance_m / diff_time_s;
          // auto target_speed = 0.3;
          // if (left_speed < 0) {
          //   target_speed = -0.3;
          // }
          // ZINFO << PIDPrint::DebugString("wheel speed", target_speed,
          //                                target_speed - 0.3, target_speed +
          //                                0.3, left_speed);
          // ZINFO << ZCOLOR_YELLOW << "left: " << FloatToString(left_speed, 2)
          //       << " right: " << FloatToString(right_speed, 2) <<
          //       ZCOLOR_NONE;
          GetWheel(kLeftWheel_)->SetCurrentSpeed(left_speed);
          GetWheel(kRightWheel_)->SetCurrentSpeed(right_speed);

          Transform odom_tf("", "");
          tf.GetTransform(tf.kOdomFrame_, tf.kRobotFrame_, odom_tf);
          MapPoint odom_pose(odom_tf.X(), odom_tf.Y(), odom_tf.Degree());
          // MapPoint odom_diff_pose(
          //     cos(odom_pose.Radian() + radian_diff) * distance_diff,
          //     sin(odom_pose.Radian() + radian_diff) * distance_diff,
          //     RadiansToDegrees(radian_diff));
          MapPoint odom_diff_pose(cos(odom_pose.Radian()) * distance_diff,
                                  sin(odom_pose.Radian()) * distance_diff, 0);
          auto new_odom_pose = odom_pose + odom_diff_pose;
          new_odom_pose.SetDegree(GetGyro(kGyro_)->GetDegree());
          auto degree_diff = new_odom_pose.Degree() - odom_pose.Degree();
          // ZINFO << odom_pose.DebugString();
          // ZINFO << odom_diff_pose.DebugString();
          // ZINFO << new_odom_pose.DebugString();
          // if (new_odom_pose.Distance(MapPoint(0, 0,0)) > 0.8) {
          //   stop = true;
          //   ZWARN << "Stop.";
          // }

          auto old_merged_odom_data = GetMergedOdomData();
          if (old_merged_odom_data == nullptr) {
            MapPoint velocity;
            if (diff_time_ms != 0) {
              auto merged_odom_data = std::make_shared<MergedOdomData>(
                  Time::Now(), new_odom_pose,
                  MapPoint(distance_diff / diff_time_s, 0, degree_diff / diff_time_s));
              SetMergedOdomData(merged_odom_data);
              merged_odom_data_cb_func(merged_odom_data);
            } else {
              auto merged_odom_data = std::make_shared<MergedOdomData>(
                  Time::Now(), new_odom_pose, MapPoint());
              SetMergedOdomData(merged_odom_data);
              merged_odom_data_cb_func(merged_odom_data);
            }
          } else {
            auto velocity = old_merged_odom_data->GetVelocity();
            if (diff_time_ms != 0) {
              velocity.SetX(distance_diff / diff_time_s);
              velocity.SetY(0);
              velocity.SetDegree(degree_diff / diff_time_s);
            }

            auto merged_odom_data = std::make_shared<MergedOdomData>(
                Time::Now(), new_odom_pose, velocity);
            SetMergedOdomData(merged_odom_data);
            merged_odom_data_cb_func(merged_odom_data);
          }

          Transform new_odom_tf(tf.kOdomFrame_, tf.kRobotFrame_,
                                new_odom_pose.X(), new_odom_pose.Y(),
                                new_odom_pose.Degree());
          tf.UpdateTransform(new_odom_tf);

          GetBumper(kLeftBumper_)
              ->UpdateRealtimeTriggeredState(
                  (basic_sensor_data->bumper_pressed_ &
                   KobukiChassisDataParser::BasicSensorRspData::
                       kLeftBumperPressed_) > 0);
          GetBumper(kCenterBumper_)
              ->UpdateRealtimeTriggeredState(
                  (basic_sensor_data->bumper_pressed_ &
                   KobukiChassisDataParser::BasicSensorRspData::
                       kCentralBumperPressed_) > 0);
          GetBumper(kRightBumper_)
              ->UpdateRealtimeTriggeredState(
                  (basic_sensor_data->bumper_pressed_ &
                   KobukiChassisDataParser::BasicSensorRspData::
                       kRightBumperPressed_) > 0);

          GetBattery(kBattery_)
              ->UpdateBatteryVoltage(basic_sensor_data->battery_p1V_ / 10.0);
          if (basic_sensor_data->charging_state_ ==
              KobukiChassisDataParser::BasicSensorRspData::kDisCharging_) {
            GetBattery(kBattery_)->SetDisChargingState();
          } else if (basic_sensor_data->charging_state_ ==
                         KobukiChassisDataParser::BasicSensorRspData::
                             kDockCharging_ ||
                     basic_sensor_data->charging_state_ ==
                         KobukiChassisDataParser::BasicSensorRspData::
                             kAdapterCharging_) {
            GetBattery(kBattery_)->SetChargingState();
          } else {
            GetBattery(kBattery_)->SetFullyChargedState();
          }

          GetButton(kButton1_)->UpdateRealtimeTriggeredState(
              basic_sensor_data->button_pressed_);

          last_basic_sensor_data = std::move(basic_sensor_data);
          if (IsDebugLogEnabled()) {
            last_basic_sensor_data->PrintOnTriggered();
          }
        }
      }
    }
    Time::SleepMSec(5);
  }
  Time::SleepMSec(100);

  ZINFO << "Thread \"" << param.thread_name_ << "\" exiting.";
  thread_manager->MarkThreadExited(param.thread_name_, __FILE__, __FUNCTION__,
                                   __LINE__);
  ZINFO << "Thread \"" << param.thread_name_ << "\" exit.";
}

}  // namespace zima
