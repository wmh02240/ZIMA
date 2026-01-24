/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_AUTO_SCAN_HOUSE_H
#define ZIMA_AUTO_SCAN_HOUSE_H

#include <atomic>
#include <memory>

#include "zima/auto_cleaning/delemma_handler.h"
#include "zima/auto_cleaning/house_scanning.h"
#include "zima/robot_data/operation_data.h"

namespace zima {

class AutoScanHouse {
 public:
  enum State {
    kAutoScanHouse,
    kEscapingDelemma,
    kFinished,
  };

  class AutoScanningInfo {
   public:
    AutoScanningInfo();
    AutoScanningInfo(const AutoScanningInfo& ref);
    ~AutoScanningInfo() = default;

    using SPtr = std::shared_ptr<AutoScanningInfo>;

    State state_;

    DelemmaHandler::DelemmaHandlerInfo::SPtr delemma_handler_info_;
  };

  AutoScanHouse() = delete;
  AutoScanHouse(const OperationData::SPtr& operation_data,
        const AutoScanningInfo::SPtr& cached_auto_scanning_info = nullptr);
  ~AutoScanHouse();

  using SPtr = std::shared_ptr<AutoScanHouse>;

  bool Pause(const Chassis::SPtr& chassis,
             const ChassisController::SPtr& chassis_controller);

  bool ChassisSupervise(const Chassis::SPtr& chassis,
                        const ChassisController::SPtr& chassis_controller);

  State GetState() const {
    ReadLocker lock(access_);
    return info_.state_;
  }

  MapCellPath GetCurrentPath() const;

  AutoScanningInfo GetScanningInfo();

 protected:
  void CheckCachedAutoScanningInfo();

  AutoScanningInfo info_;
  AutoScanningInfo::SPtr cached_auto_scanning_info_;
  OperationData::SPtr operation_data_;
  HouseScanning::UPtr house_scanning_;
  DelemmaHandler::SPtr delemma_handler_;

  ReadWriteLock::SPtr access_;
  MapCellPath current_path_;
};

}  // namespace zima

#endif  // ZIMA_AUTO_SCAN_HOUSE_H
