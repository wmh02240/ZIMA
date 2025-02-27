/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/mode/pause_mode.h"

#include "zima/algorithm/slam/slam_base.h"
#include "zima/event/event_manager.h"

namespace zima {

PauseMode::PauseMode(const Chassis::SPtr& chassis,
            const ChassisController::SPtr& chassis_controller,
            const SlamBase::SPtr& slam_wrapper)
    : ModeBase("Pause mode", chassis, chassis_controller),
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

PauseMode::~PauseMode() {}

void PauseMode::Run(const OperationData::SPtr& operation_data) {
  WriteLocker lock(access_);
  auto& event_manager = *EventManager::Instance();
  if (operation_data == nullptr) {
    event_manager.PushCommonNotice(std::make_shared<FatalErrorNotice>());
    ZERROR << "Nav data pointer empty.";
  }

  if (start_request_) {
    ZINFO << "Handle for start request.";
    start_request_ = false;
  }

  // Do nothing.
}

void PauseMode::Start() {
  WriteLocker lock(access_);
  start_request_ = true;
  ZINFO << "Receive start request.";
}

}  // namespace zima
