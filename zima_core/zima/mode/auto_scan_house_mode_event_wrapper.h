/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_AUTO_SCANNING_MODE_WRAPPER_H
#define ZIMA_AUTO_SCANNING_MODE_WRAPPER_H

#include "zima/mode/auto_scan_house_mode.h"
#include "zima/mode/mode_event_wrapper_base.h"

namespace zima {

class AutoScanHouseModeWrapper : public ModeEventWrapperBase {
 public:
  AutoScanHouseModeWrapper(const Chassis::SPtr& chassis,
                           const ChassisController::SPtr& chassis_controller,
                           const SlamBase::SPtr& slam_wrapper,
                           OperationData::SPtr& operation_data,
                           const AutoScanHouse::AutoScanningInfo::SPtr&
                               cached_scanning_info = nullptr,
                           const bool& use_simple_slam = true);
  ~AutoScanHouseModeWrapper();

  void Run(OperationData::SPtr& operation_data,
           const SlamBase::SPtr& slam_wrapper) override;

  AutoScanHouse::AutoScanningInfo::SPtr GetScanningInfo();

 private:
  void InitializeForCallBack();

  void PrintInfo();

  AutoScanHouseMode::SPtr auto_scan_house_mode_;
};

}  // namespace zima

#endif  // ZIMA_AUTO_SCANNING_MODE_WRAPPER_H
