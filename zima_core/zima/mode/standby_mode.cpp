/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/mode/standby_mode.h"

#include "zima/algorithm/slam/slam_base.h"
#include "zima/event/event_manager.h"

namespace zima {

StandbyMode::StandbyMode(const Chassis::SPtr& chassis,
                         const ChassisController::SPtr& chassis_controller,
                         const SlamBase::SPtr& slam_wrapper)
    : ModeBase("Standby mode", chassis, chassis_controller),
      start_request_(false),
      slam_wrapper_(slam_wrapper) {
  if (slam_wrapper == nullptr) {
    ZERROR << "Slam wrapper empty.";
    valid_ = false;
  }
  chassis_controller->StopThread();

  auto& event_manager = *EventManager::Instance();
  if (chassis_ == nullptr) {
    event_manager.PushCommonNotice(std::make_shared<FatalErrorNotice>());
    ZERROR << "Chassis pointer empty.";
  }
}

StandbyMode::~StandbyMode() {}

void StandbyMode::Run(const OperationData::SPtr& operation_data) {
  WriteLocker lock(access_);
  if (start_request_) {
    ZINFO << "Handle for start request.";
    start_request_ = false;
  }

  // Do nothing.
}

void StandbyMode::Start() {
  WriteLocker lock(access_);
  start_request_ = true;
  ZINFO << "Receive start request.";
}

}  // namespace zima
