/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/auto_cleaning/auto_scan_house.h"

#include <algorithm>

#include "zima/common/time.h"
#include "zima/common/util.h"
#include "zima/logger/logger.h"
#include "zima/zima_core_version.h"

namespace zima {

AutoScanHouse::AutoScanningInfo::AutoScanningInfo()
    : state_(kAutoScanHouse), delemma_handler_info_(nullptr) {}

AutoScanHouse::AutoScanningInfo::AutoScanningInfo(const AutoScanningInfo& ref)
    : state_(ref.state_), delemma_handler_info_(ref.delemma_handler_info_) {}

AutoScanHouse::AutoScanHouse(
    const OperationData::SPtr& operation_data,
    const AutoScanningInfo::SPtr& cached_auto_scanning_info)
    : cached_auto_scanning_info_(cached_auto_scanning_info),
      operation_data_(operation_data),
      house_scanning_(nullptr),
      delemma_handler_(nullptr),
      access_(std::make_shared<ReadWriteLock>()),
      current_path_({}) {
  ZINFO << GetCoreVersionInfo();
}

AutoScanHouse::~AutoScanHouse() {}

bool AutoScanHouse::Pause(const Chassis::SPtr& chassis,
                         const ChassisController::SPtr& chassis_controller) {
  WriteLocker lock(access_);
  if (chassis_controller == nullptr) {
    ZERROR << "Chassis controller invalid.";
    return false;
  }
  if (!chassis_controller->IsThreadRunning()) {
    ZERROR << "Chassis controller thread not running.";
    return false;
  }

  ZGINFO << "Exec pause.";

  switch (info_.state_) {
    case kAutoScanHouse: {
      if (house_scanning_ == nullptr) {
        ZERROR << "Ptr empty, it should never run here.";
        break;
      }
      house_scanning_->Pause(chassis, chassis_controller,
                             operation_data_->GetNavMapRef());
      break;
    }
    case kEscapingDelemma:
    default: {
      if (delemma_handler_ == nullptr) {
        ZERROR << "Ptr empty, it should never run here.";
        break;
      }
      delemma_handler_->Pause(chassis, chassis_controller,
                              operation_data_->GetNavMapRef());
      break;
    }
  }

  ZGINFO << "Exec pause finish.";
  return true;
}

bool AutoScanHouse::ChassisSupervise(
    const Chassis::SPtr& chassis,
    const ChassisController::SPtr& chassis_controller) {
  WriteLocker lock(access_);
  if (chassis_controller == nullptr) {
    ZERROR << "Chassis controller invalid.";
    return false;
  }
  if (!chassis_controller->IsThreadRunning()) {
    ZERROR << "Chassis controller thread not running.";
    return false;
  }

  CheckCachedAutoScanningInfo();

  switch (info_.state_) {
    case kAutoScanHouse: {
      if (house_scanning_ == nullptr) {
        house_scanning_.reset(new HouseScanning());
      }
      house_scanning_->ChassisSupervise(chassis, chassis_controller,
                                        operation_data_->GetNavMapRef());
      auto cleaning_state = house_scanning_->GetState();
      if (cleaning_state == HouseScanning::kFinished) {
        ZGINFO << "Enter delemma handling.";
        info_.state_ = kEscapingDelemma;
        house_scanning_.reset();
      }

      break;
    }
    case kEscapingDelemma: {
      if (delemma_handler_ == nullptr) {
        delemma_handler_.reset(new DelemmaHandler());
      }
      delemma_handler_->ChassisSupervise(chassis, chassis_controller,
                                         operation_data_);
      auto delemma_state = delemma_handler_->GetState();
      if (delemma_state == DelemmaHandler::kNotInDelemma) {
        ZGINFO << "Finish.";
        info_.state_ = kFinished;
      } else if (delemma_state == DelemmaHandler::kEscapeFromDelemma) {
        ZGINFO << "Resume.";
        info_.state_ = kAutoScanHouse;
        delemma_handler_.reset();
      }

      break;
    }
    case kFinished: {
      ZGINFO << "Finished.";
      break;
    }
    default: {
      ZERROR << "Should never run here.";
      return false;
    }
  }

  return true;
}

MapCellPath AutoScanHouse::GetCurrentPath() const {
  ReadLocker lock(access_);
  switch (info_.state_) {
    case kAutoScanHouse: {
      return current_path_;
    }
    default: {
      MapCellPath path;
      return path;
    }
  }
}

AutoScanHouse::AutoScanningInfo AutoScanHouse::GetScanningInfo() {
  ReadLocker lock(access_);

  // Get cleaning info only when auto cleaning info is needed.
  switch (info_.state_) {
    case kAutoScanHouse: {
      if (house_scanning_ != nullptr) {
        info_.delemma_handler_info_.reset();
      }
      break;
    }
    default: {
      if (delemma_handler_ != nullptr) {
        info_.delemma_handler_info_ =
            std::make_shared<DelemmaHandler::DelemmaHandlerInfo>(
                delemma_handler_->GetDelemmaHandlerInfo());
      }
      break;
    }
  }

  return info_;
}

void AutoScanHouse::CheckCachedAutoScanningInfo() {
  if (cached_auto_scanning_info_ != nullptr) {
    if (cached_auto_scanning_info_->delemma_handler_info_ != nullptr) {
      house_scanning_.reset();
      ZGINFO << "Switch to delemma handler for cached info.";
      info_.state_ = kEscapingDelemma;
      delemma_handler_.reset(new DelemmaHandler(
          cached_auto_scanning_info_->delemma_handler_info_));
    } else {
      delemma_handler_.reset();
      ZGINFO << "Switch to house scanning for cached info.";
      info_.state_ = kAutoScanHouse;
      house_scanning_.reset(new HouseScanning());
    }
    cached_auto_scanning_info_.reset();
  }
}

}  // namespace zima
