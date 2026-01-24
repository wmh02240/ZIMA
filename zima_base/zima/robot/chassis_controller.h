/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_CHASSIS_CONTROLLER_H
#define ZIMA_CHASSIS_CONTROLLER_H

#include <thread>

#include "zima/common/macro.h"
#include "zima/common/thread.h"
#include "zima/grid_map/nav_map_2d.h"
#include "zima/movement/movement.h"
#include "zima/robot/chassis.h"

namespace zima {

class ChassisController {
 public:
  ChassisController() = delete;
  ChassisController(const Chassis::SPtr& chassis,
                    const float& speed_times = 1.0f);
  ~ChassisController();

  using SPtr = std::shared_ptr<ChassisController>;

  void RunThread();
  void StopThread();
  bool IsThreadRunning();
  void Thread(const ZimaThreadWrapper::ThreadParam& param);

  class InfoWrapper {
   public:
    enum ControllerState {
      kStop,
      kStandby,
      kRunning,
    };

    InfoWrapper();
    InfoWrapper(const InfoWrapper& info) = delete;
    ~InfoWrapper() = default;

    std::string DebugString();

    void GlanceFrom(const InfoWrapper& info);

    void MoveFrom(InfoWrapper& info);

    DECLARE_DATA_GET_SET(ControllerState, ControllerState)
    DECLARE_DATA_GET_SET(MapPoint, WorldPose)
    DECLARE_DATA_GET_SET(MapPoint, OdomPose)
    DECLARE_DATA_GET_SET(MotionBase::State, MovementState)
    DECLARE_DATA_GET_SET(std::string, CurrentMovementName)
    DECLARE_DATA_GET_SET(uint32_t, TargetsLeft)
    DECLARE_DATA_GET_SET(bool, IsSteppedOnPastPath)
    MovementBase::UPtr GetLastMovement();
    void SetLastMovement(MovementBase::UPtr& curr_movement);

    void InitializeForMovement(const MovementBase::UPtr& movement);

   private:
    void Reset() {
      WriteLocker lock(access_);
      controller_state_ = kStandby;
      movement_state_ = MotionBase::State::kStop;
      current_movement_name_ = "";
      targets_left_ = 0;
      is_stepped_on_past_path_ = false;
      last_movement_.reset();
    }

    ReadWriteLock::SPtr access_;

    ControllerState controller_state_;

    MapPoint world_pose_;
    MapPoint odom_pose_;

    MotionBase::State movement_state_;
    std::string current_movement_name_;
    uint32_t targets_left_;
    bool is_stepped_on_past_path_;

    MovementBase::UPtr last_movement_;
  };

  class PlanInterface {
   public:
    PlanInterface()
        : access_(std::make_shared<ReadWriteLock>()), is_executed_(false){};
    ~PlanInterface() = default;

    MovementBase::UPtr GetNextMovement();
    void SetNextMovement(MovementBase::UPtr& next_movement);
    DECLARE_DATA_GET_SET(bool, IsExecuted)

   private:
    ReadWriteLock::SPtr access_;
    MovementBase::UPtr next_movement_;
    bool is_executed_;
  };

  class MapInterface {
   public:
    MapInterface()
        : access_(std::make_shared<ReadWriteLock>()), map_(nullptr){};
    ~MapInterface() = default;

    NavMap::SPtr GetMap();
    void SetMap(const NavMap::SPtr& map);

   private:
    ReadWriteLock::SPtr access_;
    NavMap::SPtr map_;
  };

  // Interface functions.
  /*
   * @brief function for getting info other than 'last_movement'.
   */
  void GlanceInfo(InfoWrapper& info) const;

  /*
   * @brief function for extracting info, it will take control for
   * 'last_movement'.
   */
  void ExtractInfo(InfoWrapper& info);

  /*
   * @brief function for requiring path execution info, including past path and
   * targets left.
   * @return true for predicted path reliable, false for otherwise.
   */
  bool GetCurrentPathExecutionInfo(Steps& past_path,
                                   MapPointPath& targets_left);

  /*
   * @brief function for passing movement to controller.
   */
  void SetPlan(MovementBase::UPtr& next_movement);

  /*
   * @brief function for extending path, it will not remove origin path.
   */
  bool ExtendPath(const MapPointPath& path);

  /*
   * @brief function for updating cached nav map.
   */
  void UpdateCachedNavMap(const NavMap::SPtr& map);

  /*
   * @brief function for force stop current movement.
   */
  void ForceStopMovement();

  /*
   * @brief function for set simulation speed.
   */
  void SetSpeedTimes(const float& speed_times) {
    WriteLocker lock(access_);
    speed_times_ = speed_times;
  }

 protected:
  void UpdateMovement(InfoWrapper& curr_info, PlanInterface& current_plan);

  ReadWriteLock::SPtr access_;
  float speed_times_;
  atomic_bool thread_running_;
  atomic_bool stop_command_;
  atomic_bool stop_movement_command_;
  zima::ZimaThreadWrapper::ThreadParam thread_param_;
  InfoWrapper info_wrapper_;
  PlanInterface plan_;
  MapInterface cache_map_;
  Chassis::SPtr chassis_;
  MovementBase::UPtr movement_;
};

}  // namespace zima
#endif  // ZIMA_CHASSIS_CONTROLLER_H
