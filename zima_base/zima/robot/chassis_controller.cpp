/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/robot/chassis_controller.h"

#include "zima/common/time.h"

namespace zima {

ChassisController::ChassisController(const Chassis::SPtr& chassis,
                                     const float& speed_times)
    : access_(std::make_shared<ReadWriteLock>()),
      speed_times_(speed_times),
      thread_running_(false),
      stop_command_(false),
      stop_movement_command_(false),
      chassis_(chassis) {
  thread_param_ = zima::ZimaThreadWrapper::ThreadParam(
      "Chassis controller thread", zima::ZimaThreadManager::kCoreThreadIndex_,
      100, 0.05, 0.5);
}

ChassisController::~ChassisController() {}

void ChassisController::RunThread() {
  if (thread_running_.load()) {
    ZINFO << "Thread is already running.";
    return;
  }
  stop_command_.store(false);

  auto thread_manager = zima::ZimaThreadManager::Instance();
  thread_manager->RegisterThread(
      std::thread(&ChassisController::Thread, this, thread_param_),
      thread_param_);

  thread_running_.store(true);
  // Wait for thread up.
  Time::SleepMSec(100);
  ZGINFO << "Create chassis control thread success.";
}

void ChassisController::StopThread() {
  if (!thread_running_.load()) {
    ZGINFO << "Thread is already stopped.";
    return;
  }

  ForceStopMovement();
  stop_command_.store(true);
  ZGINFO << "Stopping chassis control thread.";
  auto thread_manager = zima::ZimaThreadManager::Instance();
  while (thread_manager->IsThreadRunning(thread_param_.thread_name_)) {
    Time::SleepMSec(1);
  }
  cache_map_.SetMap(nullptr);
  ZGINFO << "Chassis control thread joined.";
  thread_running_.store(false);
  ZGINFO << "Stop chassis control thread success.";
}

bool ChassisController::IsThreadRunning() { return thread_running_.load(); }

void ChassisController::Thread(const ZimaThreadWrapper::ThreadParam& param) {
  ZINFO << "Thread \"" << param.thread_name_ << "\" start.";
  auto thread_manager = ZimaThreadManager::Instance();
  if (param.bind_cpu_id_ >= 0) {
    thread_manager->BindCPUCore(param.thread_name_, param.bind_cpu_id_);
  }

  auto& tf = *TransformManager::Instance();
  {
    // Initialize for info.
    InfoWrapper info;
    info_wrapper_.MoveFrom(info);
    Transform robot_tf("", "");
    tf.GetTransform(tf.kWorldFrame_, tf.kRobotFrame_, robot_tf);
    MapPoint world_pose(robot_tf.X(), robot_tf.Y(), robot_tf.Degree());
    tf.GetTransform(tf.kOdomFrame_, tf.kRobotFrame_, robot_tf);
    MapPoint odom_pose(robot_tf.X(), robot_tf.Y(), robot_tf.Degree());
    info_wrapper_.SetWorldPose(world_pose);
    info_wrapper_.SetOdomPose(odom_pose);

    // ZINFO << info_wrapper_.DebugString();
  }

  bool map_ready = false;
  double map_ready_print_time = 0;
  while (!stop_command_.load()) {
    thread_manager->UpdateThreadCycle(param.thread_name_);
    WriteLocker lock(access_);
    Transform robot_tf("", "");
    tf.GetTransform(tf.kWorldFrame_, tf.kRobotFrame_, robot_tf);
    MapPoint world_pose(robot_tf.X(), robot_tf.Y(), robot_tf.Degree());
    tf.GetTransform(tf.kOdomFrame_, tf.kRobotFrame_, robot_tf);
    MapPoint odom_pose(robot_tf.X(), robot_tf.Y(), robot_tf.Degree());
    info_wrapper_.SetWorldPose(world_pose);
    info_wrapper_.SetOdomPose(odom_pose);

    if (movement_ == nullptr) {
      // For duplicate command.
      stop_movement_command_.store(false);

      movement_ = plan_.GetNextMovement();
      if (movement_ != nullptr) {
        // ZINFO << "Plan executed.";
        info_wrapper_.SetControllerState(
            InfoWrapper::ControllerState::kRunning);
        info_wrapper_.InitializeForMovement(movement_);
        plan_.SetIsExecuted(true);
        continue;
      }
      // Wait for plan.
      lock.Unlock();
      Time::SleepMSec(5 / speed_times_);
    } else {
      if (stop_movement_command_.load()) {
        // ZINFO << "Movement ended for movement stop command.";
        chassis_->StopWheels();
        info_wrapper_.SetLastMovement(movement_);
        info_wrapper_.SetControllerState(
            InfoWrapper::ControllerState::kStandby);
        continue;
      }

      if (!map_ready) {
        lock.Unlock();
        if (cache_map_.GetMap() == nullptr) {
          if (Time::Now() - map_ready_print_time > 1) {
            ZGINFO << "Map not ready.";
            map_ready_print_time = Time::Now();
          }
          Time::SleepMSec(5 / speed_times_);
          continue;
        } else {
          ZGINFO << "Map ok.";
          map_ready = true;
        }
      }

      movement_->Run(chassis_, world_pose, odom_pose, cache_map_.GetMap());
      info_wrapper_.SetMovementState(movement_->GetState());
      info_wrapper_.SetTargetsLeft(movement_->GetRestPathPointCount());
      info_wrapper_.SetIsSteppedOnPastPath(movement_->IsSteppedOnPastPath());

      // ZINFO << info_wrapper_.DebugString();

      auto movement_state = movement_->GetState();
      switch (movement_state) {
        case MotionBase::State::kFinish:
        case MotionBase::State::kReachTarget:
        case MotionBase::State::kStop:
        case MotionBase::State::kTimeout:
        case MotionBase::State::kException:
        case MotionBase::State::kError: {
          // End this movement.
          // ZINFO << "Movement ended for state " << movement_state;
          chassis_->StopWheels();
          info_wrapper_.SetLastMovement(movement_);
          info_wrapper_.SetControllerState(
              InfoWrapper::ControllerState::kStandby);
          continue;
        }
        default: {
          // Do nothing.
        }
      }

      lock.Unlock();
      Time::SleepMSec(20 / speed_times_);
    }
  }

  ZGINFO << "Movement ended for thread stop command.";
  chassis_->StopWheels();
  info_wrapper_.SetLastMovement(movement_);
  info_wrapper_.SetControllerState(InfoWrapper::ControllerState::kStop);

  ZGINFO << "Thread \"" << param.thread_name_ << "\" exiting.";
  thread_manager->MarkThreadExited(param.thread_name_, __FILE__, __FUNCTION__,
                                   __LINE__);
  ZINFO << "Thread \"" << param.thread_name_ << "\" exit.";
}

ChassisController::InfoWrapper::InfoWrapper()
    : access_(std::make_shared<ReadWriteLock>()),
      controller_state_(kStop),
      world_pose_(0, 0, 0),
      odom_pose_(0, 0),
      movement_state_(MotionBase::State::kStop),
      current_movement_name_(""),
      targets_left_(0),
      is_stepped_on_past_path_(false),
      last_movement_(nullptr){};

std::string ChassisController::InfoWrapper::DebugString() {
  ReadLocker lock(access_);
  std::string str;
  str += "\nController state: " + std::to_string(controller_state_);
  str += "\nWorld pose: " + world_pose_.DebugString();
  str += "\nOdom pose: " + odom_pose_.DebugString();
  str += "\nMovement state: " + std::to_string(movement_state_);
  str += "\nMovement name: " + current_movement_name_;
  str += "\nTarget left: " + std::to_string(targets_left_);
  str += "\nStepped on past path: " + std::to_string(is_stepped_on_past_path_);
  str += "\nLast movement: ";
  if (last_movement_ == nullptr) {
    str += "null";
  } else {
    str += last_movement_->Name();
  }
  return str;
}

void ChassisController::InfoWrapper::GlanceFrom(
    const ChassisController::InfoWrapper& info) {
  WriteLocker lock(access_);
  controller_state_ = info.GetControllerState();
  world_pose_ = info.GetWorldPose();
  odom_pose_ = info.GetOdomPose();
  movement_state_ = info.GetMovementState();
  current_movement_name_ = info.GetCurrentMovementName();
  targets_left_ = info.GetTargetsLeft();
  is_stepped_on_past_path_ = info.GetIsSteppedOnPastPath();
}

void ChassisController::InfoWrapper::MoveFrom(
    ChassisController::InfoWrapper& info) {
  WriteLocker lock(access_);
  controller_state_ = info.GetControllerState();
  world_pose_ = info.GetWorldPose();
  odom_pose_ = info.GetOdomPose();
  movement_state_ = info.GetMovementState();
  current_movement_name_ = info.GetCurrentMovementName();
  targets_left_ = info.GetTargetsLeft();
  is_stepped_on_past_path_ = info.GetIsSteppedOnPastPath();
  last_movement_ = std::move(info.GetLastMovement());
  info.Reset();
}

ChassisController::InfoWrapper::ControllerState
ChassisController::InfoWrapper::GetControllerState() const {
  ReadLocker lock(access_);
  return controller_state_;
}
void ChassisController::InfoWrapper::SetControllerState(
    const ChassisController::InfoWrapper::ControllerState& state) {
  WriteLocker lock(access_);
  controller_state_ = state;
}

MapPoint ChassisController::InfoWrapper::GetWorldPose() const {
  ReadLocker lock(access_);
  return world_pose_;
}

void ChassisController::InfoWrapper::SetWorldPose(const MapPoint& value) {
  WriteLocker lock(access_);
  world_pose_ = value;
}

MapPoint ChassisController::InfoWrapper::GetOdomPose() const {
  ReadLocker lock(access_);
  return odom_pose_;
}

void ChassisController::InfoWrapper::SetOdomPose(const MapPoint& value) {
  WriteLocker lock(access_);
  odom_pose_ = value;
}

MotionBase::State ChassisController::InfoWrapper::GetMovementState() const {
  ReadLocker lock(access_);
  return movement_state_;
}
void ChassisController::InfoWrapper::SetMovementState(
    const MotionBase::State& state) {
  WriteLocker lock(access_);
  movement_state_ = state;
}

std::string ChassisController::InfoWrapper::GetCurrentMovementName() const {
  ReadLocker lock(access_);
  return current_movement_name_;
}

void ChassisController::InfoWrapper::SetCurrentMovementName(
    const std::string& name) {
  WriteLocker lock(access_);
  current_movement_name_ = name;
}

uint32_t ChassisController::InfoWrapper::GetTargetsLeft() const {
  ReadLocker lock(access_);
  return targets_left_;
}

void ChassisController::InfoWrapper::SetTargetsLeft(
    const uint32_t& targets_left) {
  WriteLocker lock(access_);
  targets_left_ = targets_left;
}

bool ChassisController::InfoWrapper::GetIsSteppedOnPastPath() const {
  ReadLocker lock(access_);
  return is_stepped_on_past_path_;
}

void ChassisController::InfoWrapper::SetIsSteppedOnPastPath(const bool& value) {
  WriteLocker lock(access_);
  is_stepped_on_past_path_ = value;
}

MovementBase::UPtr ChassisController::InfoWrapper::GetLastMovement() {
  WriteLocker lock(access_);
  return std::move(last_movement_);
}

void ChassisController::InfoWrapper::SetLastMovement(
    MovementBase::UPtr& curr_movement) {
  WriteLocker lock(access_);
  if (curr_movement != nullptr) {
    curr_movement->Stop();
  }
  last_movement_ = std::move(curr_movement);
  movement_state_ = MotionBase::kStop;
  current_movement_name_ = "";
  targets_left_ = 0;
  is_stepped_on_past_path_ = false;
}

void ChassisController::InfoWrapper::InitializeForMovement(
    const MovementBase::UPtr& movement) {
  WriteLocker lock(access_);
  movement_state_ = movement->GetState();
  current_movement_name_ = movement->Name();
}

MovementBase::UPtr ChassisController::PlanInterface::GetNextMovement() {
  WriteLocker lock(access_);
  return std::move(next_movement_);
}

void ChassisController::PlanInterface::SetNextMovement(
    MovementBase::UPtr& next_movement) {
  WriteLocker lock(access_);
  next_movement_ = std::move(next_movement);
}

bool ChassisController::PlanInterface::GetIsExecuted() const {
  ReadLocker lock(access_);
  return is_executed_;
}

void ChassisController::PlanInterface::SetIsExecuted(const bool& value) {
  WriteLocker lock(access_);
  is_executed_ = value;
}

NavMap::SPtr ChassisController::MapInterface::GetMap() {
  ReadLocker lock(access_);
  return map_;
}

void ChassisController::MapInterface::SetMap(const NavMap::SPtr& map) {
  WriteLocker lock(access_);
  if (map == nullptr) {
    map_.reset();
  } else {
    // Copy map instance instead of copy pointer.
    map_ = std::make_shared<NavMap>(*map);
  }
}

// Interface functions.
void ChassisController::GlanceInfo(ChassisController::InfoWrapper& info) const {
  return info.GlanceFrom(info_wrapper_);
}

void ChassisController::ExtractInfo(ChassisController::InfoWrapper& info) {
  // ZINFO;
  info.MoveFrom(info_wrapper_);
}

bool ChassisController::GetCurrentPathExecutionInfo(
    Steps& past_steps, MapPointPath& targets_left) {
  ReadLocker lock(access_);
  if (movement_ == nullptr) {
    return false;
  }

  const uint32_t kReasonableTargetsCount = UINT32_MAX - 1;
  if (movement_->GetRestPathPointCount() < kReasonableTargetsCount) {
    past_steps = movement_->GetSteps();
    targets_left = movement_->GetRestPath();
    return true;
  }
  return false;
}

void ChassisController::SetPlan(MovementBase::UPtr& next_movement) {
  // ZINFO << "Set plan movement " << next_movement->Name();
  plan_.SetIsExecuted(false);
  plan_.SetNextMovement(next_movement);
  while (!plan_.GetIsExecuted()) {
    // ZINFO;
    Time::SleepMSec(1);
  }
  // ZINFO << "Plan executed.";
}

bool ChassisController::ExtendPath(const MapPointPath& path) {
  WriteLocker lock(access_);
  if (movement_ == nullptr) {
    return false;
  }

  return movement_->ExtendPath(path);
}

void ChassisController::UpdateCachedNavMap(const NavMap::SPtr& map) {
  cache_map_.SetMap(map);
}

void ChassisController::ForceStopMovement() {
  // ZINFO << "Force stop movement.";
  stop_movement_command_.store(true);

  while (stop_movement_command_.load()) {
    Time::SleepMSec(1);
  }

  // ZINFO << "Force stop movement command handled.";
}

}  // namespace zima
