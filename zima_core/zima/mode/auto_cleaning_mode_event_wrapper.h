/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_AUTO_CLEANING_MODE_WRAPPER_H
#define ZIMA_AUTO_CLEANING_MODE_WRAPPER_H

#include "zima/mode/auto_cleaning_mode.h"
#include "zima/mode/mode_event_wrapper_base.h"

namespace zima {

class AutoCleaningModeWrapper : public ModeEventWrapperBase {
 public:
  AutoCleaningModeWrapper(const Chassis::SPtr& chassis,
                          const ChassisController::SPtr& chassis_controller,
                          const SlamBase::SPtr& slam_wrapper,
                          OperationData::SPtr& operation_data,
                          const AutoCleaning::AutoCleaningInfo::SPtr&
                              cached_cleaning_info = nullptr,
                          const bool& use_simple_slam = true);
  ~AutoCleaningModeWrapper();

  void Run(OperationData::SPtr& operation_data, const SlamBase::SPtr&
               slam_wrapper) override;

  AutoCleaning::AutoCleaningInfo::SPtr GetCleaningInfo();

 private:
  void InitializeForCallBack();

  void PrintInfo();

  AutoCleaningMode::SPtr auto_cleaning_mode_;
};

}  // namespace zima

#endif  // ZIMA_AUTO_CLEANING_MODE_WRAPPER_H
