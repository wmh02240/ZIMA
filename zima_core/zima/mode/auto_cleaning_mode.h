/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_AUTO_CLEANING_MODE_H
#define ZIMA_AUTO_CLEANING_MODE_H

#include "zima/algorithm/point_cloud_matcher.h"
#include "zima/algorithm/slam/slam_base.h"
#include "zima/auto_cleaning/auto_cleaning.h"
#include "zima/mode/mode_base.h"

namespace zima {

class AutoCleaningMode : public OperationMode {
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
  // 1. Totally new cleaning.
  //   1. Init for sensor.
  //   2. Init for slam.
  // 2. New cleaning base on selected operation_data.
  //   1. Init for sensor.
  //   2. Init for relocation, change pose and relocate again if needed.
  //   3. Init for slam.
  // 3. Resume cleaning with robot quiescent state.
  //   1. Init for sensor.
  //   2. Init for relocation, change pose and relocate again if needed.
  //   3. Init for slam.
  // 4. Resume cleaning with robot active state.
  //   1. Init for sensor.
  enum EntranceType {
    kTotallyNewCleaning,
    kNewCleaningBaseOnSelectedNavData,
    kResumeCleaningWithRobotQuiescentState,
    kResumeCleaningWithRobotActiveState,
  };

  static std::string DebugString(const EntranceType& type) {
    switch (type) {
      case kTotallyNewCleaning: {
        return "Totally new cleaning";
      }
      case kNewCleaningBaseOnSelectedNavData: {
        return "New cleaning base on selected nav data";
      }
      case kResumeCleaningWithRobotQuiescentState: {
        return "Resume cleaning with robot quiescent state";
      }
      case kResumeCleaningWithRobotActiveState: {
        return "Resume cleaning with robot active state";
      }
      default: {
        return "Unknown type";
      }
    }
  };

  AutoCleaningMode(const Chassis::SPtr& chassis,
                   const ChassisController::SPtr& chassis_controller,
                   const SlamBase::SPtr& slam_wrapper,
                   const EntranceType& entrance_type = kTotallyNewCleaning,
                   const AutoCleaning::AutoCleaningInfo::SPtr&
                       cached_cleaning_info = nullptr,
                   const bool& use_simple_slam = true);
  ~AutoCleaningMode();

  using SPtr = std::shared_ptr<AutoCleaningMode>;

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

  AutoCleaning::AutoCleaningInfo::SPtr GetCleaningInfo();

 private:

  Config::SPtr config_;

  void ReadyStateRun(const OperationData::SPtr& operation_data);
  void InitializingStateRun(const OperationData::SPtr& operation_data);
  void RunningStateRun(const OperationData::SPtr& operation_data);
  void PausedStateRun(const OperationData::SPtr& operation_data);
  void HandlingExceptionStateRun(const OperationData::SPtr& operation_data);
  void AutoFinishedStateRun(const OperationData::SPtr& operation_data);

  const EntranceType entrance_type_;

  AutoCleaning::SPtr auto_cleaning_;
  AutoCleaning::AutoCleaningInfo::SPtr cached_cleaning_info_;

  SlamBase::SPtr slam_wrapper_;
  State state_;

  void SwitchToReadyState();
  void SwitchToInitializingState();
  void SwitchToRunningState(const OperationData::SPtr& operation_data);
  void SwitchToPauseState(const OperationData::SPtr& operation_data);
  void SwitchToAutoFinishedState(const OperationData::SPtr& operation_data);
  void SwitchToStoppedState(const OperationData::SPtr& operation_data);

  // Initializing state
  bool InitializeForTotallyNewCleaning(
      const OperationData::SPtr& operation_data);
  bool InitializeForNewCleaningBaseOnSelectedNavData(
      const OperationData::SPtr& operation_data);
  bool InitializeForResumeCleaningWithRobotQuiescentState(
      const OperationData::SPtr& operation_data);
  bool InitializeForResumeCleaningWithRobotActiveState(
      const OperationData::SPtr& operation_data);

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

#endif  // ZIMA_AUTO_CLEANING_MODE_H
