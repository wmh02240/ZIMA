/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_PAUSE_MODE_WRAPPER_H
#define ZIMA_PAUSE_MODE_WRAPPER_H

#include "zima/mode/mode_event_wrapper_base.h"
#include "zima/mode/pause_mode.h"

namespace zima {

class PauseModeWrapper : public ModeEventWrapperBase {
 public:
  PauseModeWrapper(const Chassis::SPtr& chassis,
                   const ChassisController::SPtr& chassis_controller,
                   const SlamBase::SPtr& slam_wrapper,
                   const OperationData::SPtr& operation_data,
                   const bool& use_simple_slam = true);
  ~PauseModeWrapper();

  void Run(OperationData::SPtr& operation_data,
           const SlamBase::SPtr& slam_wrapper) override;

 private:
  void InitializeForCallBack();

  void PrintInfo();

  ModeLabel GenerateNextMode(const OperationData::SPtr& operation_data);

  PauseMode::SPtr pause_mode_;
};

}  // namespace zima

#endif  // ZIMA_PAUSE_MODE_WRAPPER_H
