/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_AUTO_CLEANING_H
#define ZIMA_AUTO_CLEANING_H

#include <atomic>
#include <memory>

#include "zima/auto_cleaning/delemma_handler.h"
#include "zima/auto_cleaning/house_cleaning.h"
#include "zima/robot_data/operation_data.h"

namespace zima {

class AutoCleaning {
 public:

  enum State {
    kAutoCleaning,
    kEscapingDelemma,
    kFinished,
  };

  class AutoCleaningInfo {
   public:
    AutoCleaningInfo();
    AutoCleaningInfo(const AutoCleaningInfo& ref);
    ~AutoCleaningInfo() = default;

    using SPtr = std::shared_ptr<AutoCleaningInfo>;

    State state_;

    HouseCleaning::HouseCleaningInfo::SPtr house_cleaning_info_;
    DelemmaHandler::DelemmaHandlerInfo::SPtr delemma_handler_info_;
  };

  AutoCleaning() = delete;
  AutoCleaning(const OperationData::SPtr& operation_data,
               const AutoCleaningInfo::SPtr&
                   cached_auto_cleaning_info = nullptr);
  ~AutoCleaning();

  using SPtr = std::shared_ptr<AutoCleaning>;
  bool Pause(const Chassis::SPtr& chassis,
             const ChassisController::SPtr& chassis_controller);

  bool ChassisSupervise(const Chassis::SPtr& chassis,
                        const ChassisController::SPtr& chassis_controller);

  State GetState() const {
    ReadLocker lock(access_);
    return info_.state_;
  }

  MapCellPath GetCurrentPath() const;

  AutoCleaningInfo GetCleaningInfo();

 protected:
  void CheckCachedAutoCleaningInfo();

  AutoCleaningInfo info_;
  AutoCleaningInfo::SPtr cached_auto_cleaning_info_;

  OperationData::SPtr operation_data_;
  HouseCleaning::SPtr house_cleaning_;
  DelemmaHandler::SPtr delemma_handler_;

  ReadWriteLock::SPtr access_;
  MapCellPath current_path_;
};

}  // namespace zima

#endif  // ZIMA_AUTO_CLEANING_H
