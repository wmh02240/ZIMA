/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_STANDBY_MODE_WRAPPER_H
#define ZIMA_STANDBY_MODE_WRAPPER_H

#include "zima/mode/mode_event_wrapper_base.h"
#include "zima/mode/standby_mode.h"

namespace zima {

class StandbyModeWrapper : public ModeEventWrapperBase {
 public:
  StandbyModeWrapper(const Chassis::SPtr& chassis,
                     const ChassisController::SPtr& chassis_controller,
                     const SlamBase::SPtr& slam_wrapper);
  ~StandbyModeWrapper();

  void Run(OperationData::SPtr& operation_data,
           const SlamBase::SPtr& slam_wrapper) override;

 private:
  void InitializeForCallBack();

  void PrintInfo();

  StandbyMode::SPtr standby_mode_;
};

}  // namespace zima

#endif  // ZIMA_STANDBY_MODE_WRAPPER_H
