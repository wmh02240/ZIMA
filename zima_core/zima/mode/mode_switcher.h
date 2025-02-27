/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_MODE_SWITCHER_H
#define ZIMA_MODE_SWITCHER_H

#include "zima/common/debug.h"
#include "zima/mode/auto_cleaning_mode.h"
#include "zima/mode/auto_scan_house_mode.h"
#include "zima/mode/mode_event_wrapper_base.h"

namespace zima {

class ModeSwitcher : public DebugBase {
 public:
  ModeSwitcher();
  ~ModeSwitcher() = default;

  using SPtr = std::shared_ptr<ModeSwitcher>;

  bool Run(ModeEventWrapperBase::SPtr& mode_wrapper,
           const ChassisController::SPtr& chassis_controller,
           const Chassis::SPtr& chassis, OperationData::SPtr& operation_data,
           const SlamBase::SPtr slam_wrapper);

  AutoCleaning::AutoCleaningInfo::SPtr auto_cleaning_info_;
  AutoScanHouse::AutoScanningInfo::SPtr auto_scanning_info_;
};

}  // namespace zima

#endif  // ZIMA_MODE_SWITCHER_H
