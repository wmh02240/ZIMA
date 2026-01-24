/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_PAUSE_MODE_H
#define ZIMA_PAUSE_MODE_H

#include "zima/algorithm/slam/slam_base.h"
#include "zima/mode/mode_base.h"

namespace zima {

class PauseMode : public ModeBase {
 public:
  PauseMode(const Chassis::SPtr& chassis,
            const ChassisController::SPtr& chassis_controller,
            const SlamBase::SPtr& slam_wrapper);
  ~PauseMode();

  using SPtr = std::shared_ptr<PauseMode>;

  void Run(const OperationData::SPtr& operation_data) override;

  void Start();

 private:
  bool start_request_;

  SlamBase::SPtr slam_wrapper_;
};

}  // namespace zima

#endif  // ZIMA_PAUSE_MODE_H
