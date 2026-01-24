/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/robot_data/operation_data.h"

#include "zima/grid_map/map_util.h"
#include "zima/logger/logger.h"

namespace zima {

// atomic_uint operation_data_count;

OperationData::OperationData()
    : OperationData(OperationType::kAllHouseCleaning) {}

OperationData::OperationData(const OperationType& type)
    : NavData(),
      start_time_(Time::SystemNow()),
      operation_stop_watch_("Operation stop watch", false),
      operation_stage_(OperationStage::kOperating),
      operation_type_(type),
      operation_result_(OperationResult::kStopped),
      start_point_(MapPoint()),
      pause_world_pose_(nullptr),
      unsync_step_index_(0) {
  StepsRecorder::Config step_recorder_config;
  step_recorder_config.filter_distance_ = NavMap::GetResolution() * 2;
  steps_recorder_ = std::make_shared<StepsRecorder>(step_recorder_config);
  index_ = static_cast<uint32_t>(start_time_);
  thread_param_ = zima::ZimaThreadWrapper::ThreadParam(
      "Operation data internal process thread",
      zima::ZimaThreadManager::kMiscThreadIndex_, 100, 0.5, 1);
  thread_running_.store(false);
  stop_command_.store(false);
  // ZGINFO << "Create operation data for " << std::to_string(index_);
  // operation_data_count.fetch_add(1);
  // ZGINFO << "Count " << operation_data_count.load() << " ptr " << this;
}

OperationData::OperationData(const bool& quiet)
    : NavData(quiet),
      start_time_(Time::SystemNow()),
      operation_stop_watch_("Operation stop watch", false),
      operation_stage_(OperationStage::kOperating),
      operation_type_(OperationType::kAllHouseCleaning),
      operation_result_(OperationResult::kStopped),
      start_point_(MapPoint()),
      pause_world_pose_(nullptr),
      unsync_step_index_(0) {
  StepsRecorder::Config step_recorder_config;
  step_recorder_config.filter_distance_ = NavMap::GetResolution() * 2;
  steps_recorder_ = std::make_shared<StepsRecorder>(step_recorder_config);
  index_ = static_cast<uint32_t>(start_time_);
  thread_param_ = zima::ZimaThreadWrapper::ThreadParam(
      "Operation data internal process thread",
      zima::ZimaThreadManager::kMiscThreadIndex_, 100, 0.5, 1);
  thread_running_.store(false);
  stop_command_.store(false);
  // ZGINFO << "Create operation data for " << std::to_string(index_);
  // operation_data_count.fetch_add(1);
  // ZGINFO << "Count " << operation_data_count.load() << " ptr " << this;
}

OperationData::OperationData(const NavData& ref)
    : NavData(ref),
      start_time_(Time::SystemNow()),
      operation_stop_watch_("Operation stop watch", false),
      operation_stage_(OperationStage::kOperating),
      operation_type_(OperationType::kAllHouseCleaning),
      operation_result_(OperationResult::kStopped),
      start_point_(MapPoint()),
      pause_world_pose_(nullptr),
      unsync_step_index_(0) {
  StepsRecorder::Config step_recorder_config;
  step_recorder_config.filter_distance_ = NavMap::GetResolution() * 2;
  steps_recorder_ = std::make_shared<StepsRecorder>(step_recorder_config);
  if (!ref.GetSelectedRoomsInfo().empty()) {
    operation_type_ = OperationType::kSelectRoomCleaning;
  } else if (ref.GetNavMapConstRef()->GetUserSelectAreaLayer()->IsMarked()) {
    operation_type_ = OperationType::kSelectAreaCleaning;
  }
  thread_param_ = zima::ZimaThreadWrapper::ThreadParam(
      "Operation data internal process thread",
      zima::ZimaThreadManager::kMiscThreadIndex_, 100, 0.5, 1);
  thread_running_.store(false);
  stop_command_.store(false);
  // ZGINFO << "Create operation data for " << std::to_string(index_);
  // operation_data_count.fetch_add(1);
  // ZGINFO << "Count " << operation_data_count.load() << " ptr " << this;
}

OperationData::OperationData(const OperationData& ref)
    : NavData(ref),
      start_time_(ref.start_time_),
      operation_stop_watch_(ref.operation_stop_watch_),
      steps_recorder_(std::make_shared<StepsRecorder>(*ref.steps_recorder_)),
      operation_stage_(ref.operation_stage_),
      operation_type_(ref.operation_type_),
      operation_result_(ref.operation_result_),
      start_point_(ref.start_point_),
      pause_world_pose_(ref.pause_world_pose_),
      thread_param_(ref.thread_param_),
      unsync_step_index_(ref.unsync_step_index_) {
  thread_running_.store(false);
  stop_command_.store(false);
  // ZGINFO << "Copy operation data for " << std::to_string(index_);
  // operation_data_count.fetch_add(1);
  // ZGINFO << "Count " << operation_data_count.load() << " ptr " << this;
}

OperationData::~OperationData() {
  // ZGINFO << "Release operation data for " << std::to_string(index_);
  // operation_data_count.fetch_sub(1);
  // ZGINFO << "Count " << operation_data_count.load() << " ptr " << this;
}

void OperationData::RunThread() {
  if (thread_running_.load()) {
    ZINFO << "Thread for " << DoubleToString(start_time_, 3)
          << " is already running.";
    return;
  }
  stop_command_.store(false);

  auto thread_manager = zima::ZimaThreadManager::Instance();
  thread_manager->RegisterThread(
      std::thread(&OperationData::Thread, this, thread_param_), thread_param_);

  thread_running_.store(true);
  // Wait for thread up.
  Time::SleepMSec(100);
  ZGINFO << "Create operation thread for " << DoubleToString(start_time_, 3)
         << " success.";
}

void OperationData::StopThread() {
  if (!thread_running_.load()) {
    ZGINFO << "Thread is already stopped.";
    return;
  }

  stop_command_.store(true);
  ZGINFO << "Stopping Operation thread for " << DoubleToString(start_time_, 3);
  auto thread_manager = zima::ZimaThreadManager::Instance();
  while (thread_manager->IsThreadRunning(thread_param_.thread_name_)) {
    Time::SleepMSec(1);
  }

  ZGINFO << "Operation thread for " << DoubleToString(start_time_, 3)
         << " joined.";
  thread_running_.store(false);
}

void OperationData::PushNewSlamValueMap(
    const SlamValueGridMap2D::SPtr& slam_value_map) {
  WriteLocker lock(access_);
  cached_slam_map_deque_.emplace_back(slam_value_map);
  if (cached_slam_map_deque_.size() > 1) {
    ZWARN << "There are still " << (cached_slam_map_deque_.size() - 1)
          << " slam map to be processed.";
  }
}

bool OperationData::ProcessSlamValueMap(
    const SlamValueGridMap2D::SPtr& slam_value_map) {
  if (slam_value_map != nullptr) {
    ReadLocker lock(slam_value_map->GetLock());
    StopWatch sw1("Update raw slam value map stop watch", true, true, true);
    UpdateRawSlamValueGridMap2D(slam_value_map);
    auto optimized_slam_grid_map =
        std::make_shared<SlamValueGridMap2D>(*slam_value_map);
    lock.Unlock();
    SlamMapUtil slam_map_util;
    slam_map_util.OptimizeSlamValueGridMap2DForUser(optimized_slam_grid_map);
    UpdateOptimizedSlamValueGridMap2D(optimized_slam_grid_map);
    ZGINFO << sw1.DebugString();
    CharGridMap2D::SPtr char_slam_grid_map = std::make_shared<CharGridMap2D>(
        "char slam map", 1, 1, optimized_slam_grid_map->GetResolution());
    MapConverter map_converter;
    map_converter.ConvertSlamValueGridMap2DToCharSlamMap(
        optimized_slam_grid_map, char_slam_grid_map);
    GetNavMapRef()->ChangeSlamLayer(char_slam_grid_map);
    if (GetNavMapConstRef()->GetRoomLayer()->IsMarked()) {
      auto room_map = GetNavMapRef()->GetRoomLayer();
      RoomMapUtil room_map_util;
      StopWatch sw2("Update room map stop watch", true, true, true);
      room_map_util.CheckAndUpdateRoomMap(optimized_slam_grid_map, room_map);
      ZGINFO << sw2.DebugString();
    }
    return true;
  }
  return false;
}

Steps OperationData::GetIncrementSteps() {
  WriteLocker lock(access_);
  Steps steps;
  auto all_step_size = steps_recorder_->GetPathSize();
  if (unsync_step_index_ < all_step_size) {
    steps = steps_recorder_->GetRestPath(unsync_step_index_);
    unsync_step_index_ = all_step_size;
  }
  return steps;
}

void OperationData::AddStep(const StepPoint& step_point) {
  WriteLocker lock(access_);
  if (steps_recorder_->AddPathPoint(step_point)) {
    MapCell step_cell;
    nav_map_->GetFootStepLayer()->WorldToMap(step_point.Pose(), step_cell);
    ZGINFO << step_point.DebugString() << ", " << step_cell.DebugString() << " "
           << steps_recorder_->GetPathSize();
  }
}

Steps OperationData::GetAllSteps() {
  ReadLocker lock(access_);
  return steps_recorder_->GetPath();
}

void OperationData::ClearSteps() {
  WriteLocker lock(access_);
  nav_map_->ClearSteps();
  steps_recorder_ = std::make_shared<StepsRecorder>();
}

bool OperationData::IsCleaningForAllHouse() const {
  ReadLocker lock(access_);
  return operation_type_ == OperationType::kAllHouseCleaning;
}

bool OperationData::UpdateSelectedRoomsInfo(const RoomsInfo& selected_rooms) {
  if (NavData::UpdateSelectedRoomsInfo(selected_rooms)) {
    WriteLocker lock(access_);
    operation_type_ = OperationType::kSelectRoomCleaning;
    return true;
  }
  return false;
}

void OperationData::ResumeSelectedRoomsInfo() {
  NavData::ResumeSelectedRoomsInfo();
  WriteLocker lock(access_);
  operation_type_ = OperationType::kAllHouseCleaning;
}

void OperationData::UpdateUserSelectAreaInNavMap(
    const CharGridMap2D::SPtr& new_user_select_area_map) {
  NavData::UpdateUserSelectAreaInNavMap(new_user_select_area_map);
  WriteLocker lock(access_);
  operation_type_ = OperationType::kSelectAreaCleaning;
}

void OperationData::ResumeUserSelectAreaInNavMap() {
  NavData::ResumeUserSelectAreaInNavMap();
  WriteLocker lock(access_);
  operation_type_ = OperationType::kAllHouseCleaning;
}

double OperationData::GetStartTime() const {
  ReadLocker lock(access_);
  return start_time_;
}

void OperationData::SetStartTime(const double& value) {
  WriteLocker lock(access_);
  start_time_ = value;
}

StopWatch& OperationData::GetStopWatchRef() {
  ReadLocker lock(access_);
  return operation_stop_watch_;
};

const StopWatch& OperationData::GetStopWatchConstRef() const {
  ReadLocker lock(access_);
  return operation_stop_watch_;
}

double OperationData::GetDuration() const {
  ReadLocker lock(access_);
  return std::max(operation_stop_watch_.Duration(), 1.0);
}

OperationData::OperationStage OperationData::GetOperationStage() const {
  ReadLocker lock(access_);
  return operation_stage_;
}

void OperationData::SetOperationStage(
    const OperationData::OperationStage& value) {
  WriteLocker lock(access_);
  operation_stage_ = value;
}

OperationData::OperationType OperationData::GetOperationType() const {
  ReadLocker lock(access_);
  return operation_type_;
}

OperationData::OperationResult OperationData::GetOperationResult() const {
  ReadLocker lock(access_);
  return operation_result_;
}

void OperationData::SetOperationResult(
    const OperationData::OperationResult& value) {
  WriteLocker lock(access_);
  operation_result_ = value;
}

MapPoint OperationData::GetStartPoint() const {
  ReadLocker lock(access_);
  return start_point_;
}

void OperationData::SetStartPoint(const MapPoint& value) {
  WriteLocker lock(access_);
  start_point_ = value;
}

MapPoint::SPtr OperationData::GetPauseWorldPose() const {
  ReadLocker lock(access_);
  return pause_world_pose_;
}

void OperationData::SetPauseWorldPose(const MapPoint::SPtr& value) {
  WriteLocker lock(access_);
  pause_world_pose_ = value;
}

std::string OperationData::DebugOperationInfo() const {
  ReadLocker lock(access_);
  auto area = GetNavMapRef()->GetStepAreaSize();
  std::string str =
      "\nStart time: " + Time::DebugString(GetStartTime()) + "(" +
      DoubleToString(GetStartTime(), 1) + ")" +
      "\nCover area: " + FloatToString(area, 2) +
      "m2\nOperated seconds:" + std::to_string(floor(GetDuration())) + "s (" +
      FloatToString(GetDuration() / 60, 2) +
      "min)\nEfficiency: " + std::to_string(area / GetDuration() * 60) +
      "m2/min.";
  return str;
}

void OperationData::Thread(const ZimaThreadWrapper::ThreadParam& param) {
  ZINFO << "Thread \"" << param.thread_name_ << "\" start.";
  auto thread_manager = ZimaThreadManager::Instance();
  if (param.bind_cpu_id_ >= 0) {
    thread_manager->BindCPUCore(param.thread_name_, param.bind_cpu_id_);
  }

  const float duration = 0.02;
  while (!stop_command_.load()) {
    thread_manager->UpdateThreadCycle(param.thread_name_);
    Time::SleepSec(duration);

    WriteLocker lock(access_);
    if (cached_slam_map_deque_.empty()) {
      continue;
    }
    auto slam_grid_map = cached_slam_map_deque_.front();
    cached_slam_map_deque_.pop_front();
    lock.Unlock();
    if (slam_grid_map == nullptr) {
      continue;
    }

    // Process for slam map.
    ProcessSlamValueMap(slam_grid_map);
  }

  ZGINFO << "Thread \"" << param.thread_name_ << "\" exiting.";
  thread_manager->MarkThreadExited(param.thread_name_, __FILE__, __FUNCTION__,
                                   __LINE__);
  ZINFO << "Thread \"" << param.thread_name_ << "\" exit.";
}

ZimaProto::OperationData::POperationData OperationDataSerializer::ToProto(
    const OperationData::SPtr& data) {
  ZimaProto::OperationData::POperationData proto;
  if (data == nullptr) {
    ZERROR << "Data empty.";
    return proto;
  }

  *proto.mutable_nav_data() = NavDataSerializer::ToProto(data);

  ReadLocker lock(data->access_);

  proto.set_start_time(data->start_time_);
  proto.mutable_start_point()->set_x(data->start_point_.X());
  proto.mutable_start_point()->set_y(data->start_point_.Y());
  for (auto&& step : data->steps_recorder_->GetPath()) {
    auto& point = *proto.add_step_points();
    switch (step.GetType()) {
      case StepPoint::Type::kPauseStep: {
        point.set_type(ZimaProto::OperationData::PStepPointType::kPauseStep);
        break;
      }
      case StepPoint::Type::kNormalStep:
      default: {
        point.set_type(ZimaProto::OperationData::PStepPointType::kNormalStep);
        break;
      }
    }
    point.mutable_point()->set_x(step.Pose().X());
    point.mutable_point()->set_y(step.Pose().Y());
  }
  proto.set_duration(data->GetDuration());

  switch (data->operation_type_) {
    case OperationData::OperationType::kSelectRoomCleaning: {
      proto.set_operation_type(
          ZimaProto::OperationData::POperationType::kSelectRoomCleaning);
      break;
    }
    case OperationData::OperationType::kSelectAreaCleaning: {
      proto.set_operation_type(
          ZimaProto::OperationData::POperationType::kSelectAreaCleaning);
      break;
    }
    case OperationData::OperationType::kAllHouseScanning: {
      proto.set_operation_type(
          ZimaProto::OperationData::POperationType::kAllHouseScanning);
      break;
    }
    case OperationData::OperationType::kAllHouseCleaning:
    default: {
      proto.set_operation_type(
          ZimaProto::OperationData::POperationType::kAllHouseCleaning);
      break;
    }
  }

  switch (data->operation_result_) {
    case OperationData::OperationResult::kFinishAutoCleaning: {
      proto.set_operation_result(
          ZimaProto::OperationData::POperationResult::kFinishAutoCleaning);
      break;
    }
    case OperationData::OperationResult::kStopped:
    default: {
      proto.set_operation_result(
          ZimaProto::OperationData::POperationResult::kStopped);
      break;
    }
  }

  return proto;
}

bool OperationDataSerializer::FromProto(
    OperationData::SPtr& data,
    const ZimaProto::OperationData::POperationData& proto) {
  if (data == nullptr) {
    ZERROR << "Data empty.";
    return false;
  }
  NavData::SPtr _data = data;
  if (!NavDataSerializer::FromProto(_data, proto.nav_data())) {
    return false;
  }

  WriteLocker lock(data->access_);

  data->start_time_ = proto.start_time();
  data->steps_recorder_ = std::make_shared<StepsRecorder>();
  for (auto&& step : proto.step_points()) {
    switch (step.type()) {
      case ZimaProto::OperationData::PStepPointType::kPauseStep: {
        data->steps_recorder_->AddPathPoint(
            StepPoint(MapPoint(step.point().x(), step.point().y(),
                               step.point().degree()),
                      nullptr, StepPoint::Type::kPauseStep),
            true, true);
        break;
      }
      case ZimaProto::OperationData::PStepPointType::kNormalStep:
      default: {
        data->steps_recorder_->AddPathPoint(
            StepPoint(MapPoint(step.point().x(), step.point().y(),
                               step.point().degree()),
                      nullptr, StepPoint::Type::kNormalStep),
            true, true);
        break;
      }
    }
  }

  data->operation_stop_watch_ =
      StopWatch(data->operation_stop_watch_.Name(), proto.duration());

  switch (proto.operation_type()) {
    case ZimaProto::OperationData::POperationType::kSelectRoomCleaning: {
      data->operation_type_ = OperationData::OperationType::kSelectRoomCleaning;
      break;
    }
    case ZimaProto::OperationData::POperationType::kSelectAreaCleaning: {
      data->operation_type_ = OperationData::OperationType::kSelectAreaCleaning;
      break;
    }
    case ZimaProto::OperationData::POperationType::kAllHouseScanning: {
      data->operation_type_ = OperationData::OperationType::kAllHouseScanning;
      break;
    }
    case ZimaProto::OperationData::POperationType::kAllHouseCleaning:
    default: {
      data->operation_type_ = OperationData::OperationType::kAllHouseCleaning;
      break;
    }
  }

  switch (proto.operation_result()) {
    case ZimaProto::OperationData::POperationResult::kFinishAutoCleaning: {
      data->operation_result_ =
          OperationData::OperationResult::kFinishAutoCleaning;
      break;
    }
    case ZimaProto::OperationData::POperationResult::kStopped:
    default: {
      data->operation_result_ = OperationData::OperationResult::kStopped;
      break;
    }
  }

  return true;
}

bool OperationDataLoader::LoadData(OperationData::SPtr& data,
                                   const bool& quiet) {
  ZimaProto::OperationData::POperationData proto;
  if (GetProtoFromBinaryFile(&proto) || GetProtoFromASCIIFile(&proto)) {
    if (OperationDataSerializer::FromProto(data, proto)) {
      if (!quiet) {
        ZINFO << "Load " << file_name_ << " success.";
      }
      return true;
    }
  }
  ZERROR << "Load " << file_name_ << " failed.";
  return false;
}

bool OperationDataWriter::WriteData(const OperationData::SPtr& data,
                                    const bool& is_binary) {
  auto proto = OperationDataSerializer::ToProto(data);
  if (is_binary ? SetProtoToBinaryFile(proto) : SetProtoToASCIIFile(proto)) {
    ZINFO << "Write " << file_name_ << " success.";
    return true;
  } else {
    ZERROR << "Write " << file_name_ << " failed.";
    return false;
  }
}

}  // namespace zima
