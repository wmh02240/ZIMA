/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_AUTO_SCAN_HOUSE_MODE_H
#define ZIMA_AUTO_SCAN_HOUSE_MODE_H

#include "zima/algorithm/point_cloud_matcher.h"
#include "zima/algorithm/slam/slam_base.h"
#include "zima/auto_cleaning/auto_scan_house.h"
#include "zima/mode/mode_base.h"

namespace zima {

class AutoScanHouseMode : public OperationMode {
 public:
  class Config : public OperationMode::Config {
   public:
    Config();
    Config(const JsonSPtr& json);
    ~Config() = default;

    using SPtr = std::shared_ptr<Config>;

    static const std::string kConfigKey_;
  };

  // Entrance type:
  // 1. New scanning.
  //   1. Init for sensor.
  //   2. Init for slam.
  // 2. Resume scanning.
  //   1. Init for sensor.
  //   2. Init for relocation, change pose and relocate again if needed.
  //   3. Init for slam.

  enum EntranceType {
    kNewScanning,
    kResumeScanning,
  };

  static std::string DebugString(const EntranceType& type) {
    switch (type) {
      case kNewScanning: {
        return "New scanning";
      }
      case kResumeScanning: {
        return "Resume scanning";
      }
      default: {
        return "Unknown type";
      }
    }
  };

  AutoScanHouseMode(const Chassis::SPtr& chassis,
                    const ChassisController::SPtr& chassis_controller,
                    const SlamBase::SPtr& slam_wrapper,
                    const EntranceType& entrance_type = kNewScanning,
                    const AutoScanHouse::AutoScanningInfo::SPtr&
                        cached_scanning_info = nullptr);
  ~AutoScanHouseMode();

  using SPtr = std::shared_ptr<AutoScanHouseMode>;

  void Run(const OperationData::SPtr& operation_data) override;

  void Start();
  void Stop();
  void Pause();

  enum State {
    kReady,
    kInitializing,
    kRunning,
    kPaused,
    kHandlingException,
    kAutoFinished,
    kStopped,
  };

  State GetState() const;

  bool IsReady() const;
  bool IsInitializing() const;
  bool IsRunning() const;
  bool IsPaused() const;
  bool IsHandlingException() const;
  bool IsAutoFinished() const;
  bool IsStopped() const;

  AutoScanHouse::AutoScanningInfo::SPtr GetScanningInfo();

 private:

  Config::SPtr config_;

  void ReadyStateRun(const OperationData::SPtr& operation_data);
  void InitializingStateRun(const OperationData::SPtr& operation_data);
  void RunningStateRun(const OperationData::SPtr& operation_data);
  void PausedStateRun(const OperationData::SPtr& operation_data);
  void HandlingExceptionStateRun(const OperationData::SPtr& operation_data);
  void AutoFinishedStateRun(const OperationData::SPtr& operation_data);

  const EntranceType entrance_type_;

  AutoScanHouse::SPtr auto_scan_house_;
  AutoScanHouse::AutoScanningInfo::SPtr cached_scanning_info_;

  SlamBase::SPtr slam_wrapper_;
  State state_;

  void SwitchToReadyState();
  void SwitchToInitializingState();
  void SwitchToRunningState(const OperationData::SPtr& operation_data);
  void SwitchToPauseState(const OperationData::SPtr& operation_data);
  void SwitchToAutoFinishedState(const OperationData::SPtr& operation_data);
  void SwitchToStoppedState(const OperationData::SPtr& operation_data);

  // Initializing state
  bool InitializeForNewScanning(const OperationData::SPtr& operation_data);
  bool InitializeForResumeScanning(const OperationData::SPtr& operation_data);

  enum InitializeState {
    kNull,

    kInitForSensor,
    kWaitForLidarAndOdomData,
    kInitForRelocation,
    kWaitForRelocation,
    kChangePoseForRelocation,
    kInitForSlam,
    kWaitForSlam,

    kInitFinish,
  };

  bool InitializeForSensor(const OperationData::SPtr& operation_data);
  bool WaitForSensor(const OperationData::SPtr& operation_data);

  InitializeState initialize_state_;
};

}  // namespace zima

#endif  // ZIMA_AUTO_SCAN_HOUSE_MODE_H
