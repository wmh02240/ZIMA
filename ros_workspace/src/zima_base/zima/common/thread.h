/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_THREAD_H
#define ZIMA_THREAD_H

#include <map>
#include <memory>
#include <thread>

#include "zima/common/lock.h"
#include "zima/common/macro.h"

namespace zima {

class ZimaThreadWrapper {
 public:
  class ThreadParam {
   public:
    ThreadParam();
    ThreadParam(const std::string& name, const int16_t& bind_cpu_id,
                const int16_t& priority, const double& cycle_warning_period,
                const double& cycle_error_period);
    ~ThreadParam();

    std::string thread_name_;
    int16_t bind_cpu_id_;
    int16_t priority_;
    double cycle_warning_period_;
    double cycle_error_period_;
  };

  enum CycleState {
    kNormal,
    kWarning,
    kError,
  };

  ZimaThreadWrapper() = delete;
  ZimaThreadWrapper(std::thread&& thread, const ThreadParam& param);
  ~ZimaThreadWrapper();

  using SPtr = std::shared_ptr<ZimaThreadWrapper>;

  DECLARE_DATA_GET_SET(double, CycleStartTime);
  DECLARE_DATA_GET_SET(ThreadParam, Param);
  DECLARE_DATA_GET_SET(bool, IsMarkedExit);

  void PrintIfNeeded();

  void Join() { thread_->join(); };

 private:
  ReadWriteLock::SPtr access_;
  std::unique_ptr<std::thread> thread_;
  ThreadParam param_;
  CycleState cycle_state_;
  double cycle_start_time_;
  double last_cycle_time_;
  double last_print_time_;
  atomic_bool is_marked_exit_;
};

class ZimaThreadManager {
  DECLARE_SINGLETON(ZimaThreadManager)
 public:
  ~ZimaThreadManager();

  using ThreadWrapperPair = std::pair<std::string, ZimaThreadWrapper::SPtr>;

  static const uint8_t kCoreThreadIndex_;
  static const uint8_t kSlamThreadIndex_;
  static const uint8_t kMiscThreadIndex_;
  static const uint8_t kRealtimeThreadIndex_;

  // Functions for thread management. These functions should be call by logic
  // OUTSIDE thread.
  bool RegisterThread(std::thread&& thread,
                      const ZimaThreadWrapper::ThreadParam& param);
  bool MarkThreadExited(const std::string& thread_name, const std::string& file,
                        const std::string& function, const uint32_t& line);
  bool IsThreadRunning(const std::string& thread_name);
  bool WaitForThreadExit(const std::string& thread_name,
                         const double& timeout_s);

  uint16_t RunningThreadCount();
  void DebugRunningThreadName();

  bool UpdateThreadCycle(const std::string& thread_name);

  uint16_t GetCPUMaxCoreCount() const { return cpu_max_core_count_; };

  // Functions for thread logic. These functions should be call by logic INSIDE
  // thread.
  bool BindCPUCore(const std::string& thread_name, const uint16_t& cpu_core);

 private:
  ZimaThreadWrapper::SPtr GetThreadHandle(const std::string& thread_name);
  void MonitorThread(const ZimaThreadWrapper::ThreadParam& param);

  ReadWriteLock::SPtr access_;
  uint16_t cpu_max_core_count_;
  std::map<std::string, std::shared_ptr<ZimaThreadWrapper>> threads_map_;
  bool monitor_thread_on_;
};

class ZimaProcessManager {
  DECLARE_SINGLETON(ZimaProcessManager)
 public:
  ~ZimaProcessManager();

  bool SetSystemLevelProcessName(const std::string& system_level_process_name);
};

}  // namespace zima

#endif  // ZIMA_THREAD_H
