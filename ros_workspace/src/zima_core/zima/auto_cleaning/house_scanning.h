/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_HOUSE_SCANNING_H
#define ZIMA_HOUSE_SCANNING_H

#include <atomic>
#include <memory>

#include "zima/movement/trace_path_movement.h"
#include "zima/path_planner/quick_scan_house_path_planner.h"
#include "zima/robot/chassis_controller.h"
#include "zima/robot_data/nav_data.h"

namespace zima {

class HouseScanning {
 public:
  HouseScanning();
  ~HouseScanning();

  using UPtr = std::unique_ptr<HouseScanning>;

  enum State {
    kHouseScanning,
    kFinished,
  };

  bool Pause(const Chassis::SPtr& chassis,
             const ChassisController::SPtr& chassis_controller,
             const NavMap::SPtr& nav_map);

  bool ChassisSupervise(const Chassis::SPtr& chassis,
                        const ChassisController::SPtr& chassis_controller,
                        const NavMap::SPtr& nav_map);

  State GetState() const {
    ReadLocker lock(access_);
    return state_;
  }

  MapCellPath GetCurrentPath() const {
    ReadLocker lock(access_);
    return current_path_;
  }

 protected:
  bool GenerateNextMovement(const Chassis::SPtr& chassis,
                            const MapPoint& world_pose,
                            const NavMap::SPtr& nav_map,
                            MovementBase::UPtr& next_movement);

  State state_;
  atomic_bool shutdown_planner_thread_;
  atomic_bool generate_path_request_;
  MovementBase::UPtr movement_;
  MovementBase::UPtr next_movement_;

  ReadWriteLock::SPtr access_;
  MapCellPath current_path_;

  QuickScanHousePlanner quick_scan_house_planner_;
};

}  // namespace zima

#endif  // ZIMA_HOUSE_SCANNING_H
