/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_OPERATION_DATA_H
#define ZIMA_OPERATION_DATA_H

#include "zima/common/thread.h"
#include "zima/proto/operation_data.pb.h"
#include "zima/robot_data/nav_data.h"

namespace zima {

class OperationData : public NavData {
 public:
  enum OperationStage {
    kOperating,
    kReturnToStartPointOrDock,
    kFinish,
  };

  enum OperationType {
    kAllHouseCleaning,
    kSelectRoomCleaning,
    kSelectAreaCleaning,
    kAllHouseScanning,
  };

  enum OperationResult {
    kStopped,
    kFinishAutoCleaning,
    kFinishAutoScanHouse,
  };

  OperationData();
  OperationData(const OperationType& type);
  OperationData(const bool& quiet);
  OperationData(const NavData& ref);
  OperationData(const OperationData& ref);
  ~OperationData();
  using SPtr = std::shared_ptr<OperationData>;
  using SCPtr = std::shared_ptr<const OperationData>;

  void RunThread();
  void StopThread();
  void PushNewSlamValueMap(const SlamValueGridMap2D::SPtr& slam_value_map);
  bool ProcessSlamValueMap(const SlamValueGridMap2D::SPtr& slam_value_map);

  Steps GetIncrementSteps();
  void AddStep(const StepPoint& step_point);
  Steps GetAllSteps();
  void ClearSteps();

  bool IsCleaningForAllHouse() const;

  bool UpdateSelectedRoomsInfo(const RoomsInfo& selected_rooms);
  void ResumeSelectedRoomsInfo();
  void UpdateUserSelectAreaInNavMap(
      const CharGridMap2D::SPtr& new_user_select_area_map);
  void ResumeUserSelectAreaInNavMap();

  DECLARE_DATA_GET_SET(double, StartTime)
  StopWatch& GetStopWatchRef();
  const StopWatch& GetStopWatchConstRef() const;
  double GetDuration() const;

  DECLARE_DATA_GET_SET(OperationStage, OperationStage)
  OperationType GetOperationType() const;
  DECLARE_DATA_GET_SET(OperationResult, OperationResult)

  DECLARE_DATA_GET_SET(MapPoint, StartPoint)
  DECLARE_DATA_GET_SET(MapPoint::SPtr, PauseWorldPose)

  std::string DebugOperationInfo() const;

 protected:
  friend class OperationDataSerializer;

  double start_time_;
  StopWatch operation_stop_watch_;
  StepsRecorder::SPtr steps_recorder_;

  OperationStage operation_stage_;
  OperationType operation_type_;
  OperationResult operation_result_;

  MapPoint start_point_;
  MapPoint::SPtr pause_world_pose_;

 private:
  void Thread(const ZimaThreadWrapper::ThreadParam& param);

  zima::ZimaThreadWrapper::ThreadParam thread_param_;
  atomic_bool thread_running_;
  atomic_bool stop_command_;
  std::deque<SlamValueGridMap2D::SPtr> cached_slam_map_deque_;

  uint32_t unsync_step_index_;
};

class OperationDataSerializer {
 public:
  OperationDataSerializer() = delete;

  static ZimaProto::OperationData::POperationData ToProto(
      const OperationData::SPtr& data);
  static bool FromProto(OperationData::SPtr& data,
                        const ZimaProto::OperationData::POperationData& proto);
};

class OperationDataLoader : public NavDataLoader {
 public:
  OperationDataLoader() = default;
  explicit OperationDataLoader(const std::string& file_dir,
                               const std::string& file_name)
      : NavDataLoader(file_dir, file_name){};
  ~OperationDataLoader() = default;

  bool LoadData(OperationData::SPtr& data, const bool& quiet = false);
};

class OperationDataWriter : public NavDataWriter {
 public:
  OperationDataWriter() = default;
  explicit OperationDataWriter(const std::string& file_dir,
                               const std::string& file_name)
      : NavDataWriter(file_dir, file_name){};
  ~OperationDataWriter() = default;

  bool WriteData(const OperationData::SPtr& data, const bool& is_binary);
};

}  // namespace zima

#endif  // ZIMA_OPERATION_DATA_H
