/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/auto_cleaning/auto_cleaning.h"

#include <algorithm>

#include "zima/common/time.h"
#include "zima/common/util.h"
#include "zima/hal/system/process.h"
#include "zima/logger/logger.h"
#include "zima/zima_core_version.h"

namespace zima {

AutoCleaning::AutoCleaningInfo::AutoCleaningInfo()
    : state_(kAutoCleaning),
      house_cleaning_info_(nullptr),
      delemma_handler_info_(nullptr) {}

AutoCleaning::AutoCleaningInfo::AutoCleaningInfo(const AutoCleaningInfo& ref)
    : state_(ref.state_),
      house_cleaning_info_(nullptr),
      delemma_handler_info_(nullptr) {
  if (ref.house_cleaning_info_ != nullptr) {
    house_cleaning_info_ = std::make_shared<HouseCleaning::HouseCleaningInfo>(
        *ref.house_cleaning_info_);
  }
  if (ref.delemma_handler_info_ != nullptr) {
    delemma_handler_info_ =
        std::make_shared<DelemmaHandler::DelemmaHandlerInfo>(
            *ref.delemma_handler_info_);
  }
}

AutoCleaning::AutoCleaning(
    const OperationData::SPtr& operation_data,
    const AutoCleaningInfo::SPtr& cached_auto_cleaning_info)
    : cached_auto_cleaning_info_(cached_auto_cleaning_info),
      operation_data_(operation_data),
      house_cleaning_(nullptr),
      delemma_handler_(nullptr),
      access_(std::make_shared<ReadWriteLock>()),
      current_path_({}) {
  ZINFO << GetCoreVersionInfo();
}

AutoCleaning::~AutoCleaning() {}

bool AutoCleaning::Pause(const Chassis::SPtr& chassis,
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
    case kAutoCleaning: {
      if (house_cleaning_ == nullptr) {
        ZERROR << "Ptr empty, it should never run here.";
        break;
      }
      house_cleaning_->Pause(chassis, chassis_controller,
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

bool AutoCleaning::ChassisSupervise(
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

  CheckCachedAutoCleaningInfo();

  switch (info_.state_) {
    case kAutoCleaning: {
      if (house_cleaning_ == nullptr) {
        auto rooms_info = operation_data_->GetSelectedRoomsInfo();
        if (rooms_info.empty()) {
          rooms_info = operation_data_->GetRoomsInfo();
        }
        auto selected_rooms = false;
        if (!operation_data_->GetSelectedRoomsInfo().empty()) {
          selected_rooms = true;
        }
        auto consider_slam_map = selected_rooms;
        if (operation_data_->GetNavMapConstRef()
                ->GetUserSelectAreaLayer()
                ->IsMarked()) {
          consider_slam_map = true;
        }

        house_cleaning_.reset(
            new HouseCleaning(rooms_info, consider_slam_map, selected_rooms));
      }
      house_cleaning_->ChassisSupervise(chassis, chassis_controller,
                                        operation_data_->GetNavMapRef());
      auto cleaning_state = house_cleaning_->GetState();
      if (cleaning_state == HouseCleaning::kFinished) {
        ZGINFO << "Enter delemma handling.";
        info_.state_ = kEscapingDelemma;
        house_cleaning_.reset();
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
        ZINFO << "Finish cleaning.";
        info_.state_ = kFinished;
        ZGINFO << "Current memory usage: "
               << std::to_string(ZimaGetProcessMemoryUsageInKB()) << " Kb.";
      } else if (delemma_state == DelemmaHandler::kEscapeFromDelemma) {
        ZGINFO << "Resume auto cleaning.";
        info_.state_ = kAutoCleaning;
        delemma_handler_.reset();
      } else if (delemma_state == DelemmaHandler::kTimeout) {
        ZINFO << "Timeout.";
        info_.state_ = kFinished;
        ZGINFO << "Current memory usage: "
               << std::to_string(ZimaGetProcessMemoryUsageInKB()) << " Kb.";
      }

      break;
    }
    case kFinished: {
      ZGINFO << "Auto cleaning finished.";
      break;
    }
    default: {
      ZERROR << "Should never run here.";
      return false;
    }
  }

  return true;
}

MapCellPath AutoCleaning::GetCurrentPath() const {
  ReadLocker lock(access_);
  switch (info_.state_) {
    case kAutoCleaning: {
      return current_path_;
    }
    default: {
      MapCellPath path;
      return path;
    }
  }
}

AutoCleaning::AutoCleaningInfo AutoCleaning::GetCleaningInfo() {
  ReadLocker lock(access_);

  // Get cleaning info only when auto cleaning info is needed.
  switch (info_.state_) {
    case kAutoCleaning: {
      if (house_cleaning_ != nullptr) {
        info_.house_cleaning_info_ =
            std::make_shared<HouseCleaning::HouseCleaningInfo>(
                house_cleaning_->GetCleaningInfo());
        info_.delemma_handler_info_.reset();
      }
      break;
    }
    default: {
      if (delemma_handler_ != nullptr) {
        info_.delemma_handler_info_ =
            std::make_shared<DelemmaHandler::DelemmaHandlerInfo>(
                delemma_handler_->GetDelemmaHandlerInfo());
        info_.house_cleaning_info_.reset();
      }
      break;
    }
  }

  return info_;
}

void AutoCleaning::CheckCachedAutoCleaningInfo() {
  if (cached_auto_cleaning_info_ != nullptr) {
    if (cached_auto_cleaning_info_->house_cleaning_info_ != nullptr &&
        cached_auto_cleaning_info_->house_cleaning_info_->room_cleaning_info_ !=
            nullptr) {
      delemma_handler_.reset();
      auto rooms_info = operation_data_->GetSelectedRoomsInfo();
      if (rooms_info.empty()) {
        rooms_info = operation_data_->GetRoomsInfo();
      }
      auto selected_rooms = false;
      if (!operation_data_->GetSelectedRoomsInfo().empty()) {
        selected_rooms = true;
      }
      auto consider_slam_map = selected_rooms;
      if (operation_data_->GetNavMapConstRef()
              ->GetUserSelectAreaLayer()
              ->IsMarked()) {
        consider_slam_map = true;
      }

      ZGINFO << "Switch to auto cleaning for cached info.";
      info_.state_ = kAutoCleaning;
      house_cleaning_.reset(
          new HouseCleaning(rooms_info, consider_slam_map, selected_rooms,
                            cached_auto_cleaning_info_->house_cleaning_info_
                                ->room_cleaning_info_));
    } else if (cached_auto_cleaning_info_->delemma_handler_info_ != nullptr) {
      house_cleaning_.reset();
      ZGINFO << "Switch to delemma handler for cached info.";
      info_.state_ = kEscapingDelemma;
      delemma_handler_.reset(new DelemmaHandler(
          cached_auto_cleaning_info_->delemma_handler_info_));
    }

    cached_auto_cleaning_info_.reset();
  }
}

}  // namespace zima
