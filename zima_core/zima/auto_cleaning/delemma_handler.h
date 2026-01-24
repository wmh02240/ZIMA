/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_DELEMMA_HANDLER_H
#define ZIMA_DELEMMA_HANDLER_H

#include <atomic>
#include <memory>

#include "zima/path_planner/quick_scan_house_path_planner.h"
#include "zima/path_planner/return_home_planner.h"
#include "zima/path_planner/switch_room_section_planner.h"
#include "zima/robot/chassis_controller.h"
#include "zima/robot_data/operation_data.h"

namespace zima {

class DelemmaHandler {
 public:
  enum State {
    kInDelemma,
    kEscapeFromDelemma,
    kNotInDelemma,
    kTimeout,
  };

  class DelemmaHandlerInfo {
   public:
    DelemmaHandlerInfo();
    DelemmaHandlerInfo(const DelemmaHandlerInfo& ref);
    ~DelemmaHandlerInfo() = default;

    using SPtr = std::shared_ptr<DelemmaHandlerInfo>;

    State state_;
    std::string current_movement_name_;
    MapCellPath current_cell_path_;
    MapPoint::SPtr current_world_pose_;
    float run_duration_;
  };

  DelemmaHandler();
  DelemmaHandler(const DelemmaHandlerInfo::SPtr& cached_info);
  ~DelemmaHandler();

  using SPtr = shared_ptr<DelemmaHandler>;

  bool Pause(const Chassis::SPtr& chassis,
             const ChassisController::SPtr& chassis_controller,
             const NavMap::SPtr& nav_map);

  bool ChassisSupervise(const Chassis::SPtr& chassis,
                        const ChassisController::SPtr& chassis_controller,
                        const OperationData::SPtr& operation_data);

  State GetState() const {
    ReadLocker lock(access_);
    return info_.state_;
  }

  MapCellPath GetCurrentPath() const;

  DelemmaHandlerInfo GetDelemmaHandlerInfo();

 protected:
  bool CheckIsInDelemma(const Chassis::SPtr& chassis,
                        const MapPoint& world_pose,
                        const OperationData::SPtr& operation_data);

  std::shared_ptr<StopWatch> escape_delemma_stop_watch_;
  DelemmaHandlerInfo info_;
  MovementBase::UPtr next_movement_;
  double last_check_time_;

  ReadWriteLock::SPtr access_;
  MapCellPath current_path_;

  ReturnHomePlanner return_home_planner_;
  SwitchSectionPlanner switch_section_planner_;
  SwitchRoomPlanner switch_room_planner_;
  QuickScanHousePlanner quick_scan_house_planner_;
};

}  // namespace zima

#endif  // ZIMA_DELEMMA_HANDLER_H
