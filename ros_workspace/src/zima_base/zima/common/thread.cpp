/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/common/thread.h"

#include <sched.h>      // For process/core binding.
#include <sys/prctl.h>  // For renaming process on system level.
#include <unistd.h>     // For sysconf.

#include <deque>

#include "zima/common/time.h"
#include "zima/common/util.h"
#include "zima/logger/logger.h"
#include "zima/zima_base_version.h"

namespace zima {

ZimaThreadWrapper::ThreadParam::ThreadParam() {
  thread_name_ = "Unknown thread.";
  bind_cpu_id_ = -1;
  priority_ = 100;
  cycle_warning_period_ = 1;
  cycle_error_period_ = 1;
}

ZimaThreadWrapper::ThreadParam::ThreadParam(const std::string& name,
                                            const int16_t& bind_cpu_id,
                                            const int16_t& priority,
                                            const double& cycle_warning_period,
                                            const double& cycle_error_period)
    : thread_name_(name),
      bind_cpu_id_(bind_cpu_id),
      priority_(priority),
      cycle_warning_period_(cycle_warning_period),
      cycle_error_period_(cycle_error_period) {}

ZimaThreadWrapper::ThreadParam::~ThreadParam() {}

ZimaThreadWrapper::ZimaThreadWrapper(std::thread&& thread,
                                     const ThreadParam& param)
    : access_(std::make_shared<ReadWriteLock>()),
      param_(param),
      cycle_state_(kNormal),
      cycle_start_time_(Time::SystemNow()),
      last_cycle_time_(0),
      last_print_time_(0),
      is_marked_exit_(false) {
  ZGINFO << "Create thread " << param_.thread_name_ << ".";
  thread_ = std::make_unique<std::thread>(std::forward<std::thread>(thread));
}

ZimaThreadWrapper::~ZimaThreadWrapper() {
  if (thread_ == nullptr) {
    ZGINFO << "Thread \"" << param_.thread_name_ << "\" is released.";
  }
  if (thread_->joinable()) {
    ZWARN << "Why is it not join by manager thread? Wait for thread \""
          << param_.thread_name_ << "\" to join.";
    thread_->join();
    ZGINFO << "Thread \"" << param_.thread_name_ << "\" joined.";
  }
}

double ZimaThreadWrapper::GetCycleStartTime() const {
  ReadLocker lock(access_);
  return cycle_start_time_;
}

void ZimaThreadWrapper::SetCycleStartTime(const double& cycle_start_time) {
  WriteLocker lock(access_);
  cycle_start_time_ = cycle_start_time;
}

ZimaThreadWrapper::ThreadParam ZimaThreadWrapper::GetParam() const {
  ReadLocker lock(access_);
  return param_;
}

void ZimaThreadWrapper::SetParam(const ZimaThreadWrapper::ThreadParam& param) {
  ZERROR << "Never call this funcion.(name: " << param.thread_name_ << ")";
}

bool ZimaThreadWrapper::GetIsMarkedExit() const {
  return is_marked_exit_.load();
}

void ZimaThreadWrapper::SetIsMarkedExit(const bool& is_marked_exit) {
  is_marked_exit_.store(is_marked_exit);
}

void ZimaThreadWrapper::PrintIfNeeded() {
  if (param_.cycle_warning_period_ < 0 || param_.cycle_error_period_ < 0) {
    return;
  }

  ReadLocker lock(access_);
  auto now = Time::SystemNow();
  auto cycle_time = now - cycle_start_time_;
  switch (cycle_state_) {
    case kNormal: {
      if (cycle_time > param_.cycle_warning_period_) {
        ZWARN << "Thread " << param_.thread_name_ << " has run for over "
              << FloatToString(cycle_time, 4) << "s.";
        cycle_state_ = kWarning;
        last_print_time_ = now;
      }
      break;
    }
    case kWarning: {
      if (cycle_time <= param_.cycle_warning_period_) {
        cycle_state_ = kNormal;
        ZWARN << "Thread " << param_.thread_name_ << " last cycle run for "
              << FloatToString(last_cycle_time_, 4) << "s.";
        break;
      }
      if (cycle_time > param_.cycle_error_period_) {
        ZERROR << "Thread " << param_.thread_name_ << " has run for over "
               << FloatToString(cycle_time, 4) << "s.";
        cycle_state_ = kError;
        last_print_time_ = now;
        break;
      }
      if (now - last_print_time_ > 1) {
        ZWARN << "Thread " << param_.thread_name_ << " has run for over "
              << FloatToString(cycle_time, 4) << "s.";
        last_print_time_ = now;
      }
      break;
    }
    case kError: {
      if (cycle_time <= param_.cycle_warning_period_) {
        cycle_state_ = kNormal;
        ZERROR << "Thread " << param_.thread_name_ << " last cycle run for "
               << FloatToString(last_cycle_time_, 4) << "s.";
        break;
      }
      if (cycle_time <= param_.cycle_error_period_) {
        ZERROR << "Thread " << param_.thread_name_ << " last cycle run for "
               << FloatToString(last_cycle_time_, 4) << "s.";
        cycle_state_ = kWarning;
        last_print_time_ = now;
        break;
      }
      if (now - last_print_time_ > 1) {
        ZERROR << "Thread " << param_.thread_name_ << " has run for over "
               << FloatToString(cycle_time, 4) << "s.";
        last_print_time_ = now;
      }
      break;
    }
    default: {
      ZERROR << "Should never run here.";
      break;
    }
  }
  last_cycle_time_ = cycle_time;
}

const uint8_t ZimaThreadManager::kCoreThreadIndex_ = 0;
const uint8_t ZimaThreadManager::kSlamThreadIndex_ = 1;
const uint8_t ZimaThreadManager::kMiscThreadIndex_ = 2;
const uint8_t ZimaThreadManager::kRealtimeThreadIndex_ = 3;

ZimaThreadManager::ZimaThreadManager() {
  ZINFO << GetBaseVersionInfo();
  access_ = std::make_shared<ReadWriteLock>();
  cpu_max_core_count_ = sysconf(_SC_NPROCESSORS_CONF);
  ZINFO << "Current machine has " << cpu_max_core_count_ << " cores.";
  if (cpu_max_core_count_ < kRealtimeThreadIndex_ + 1) {
    ZERROR << "CPU core count mismatch setting, please reset and recompile.";
  }

  ZimaThreadWrapper::ThreadParam thread_param;
  thread_param.thread_name_ = "Thread manager monitor";
  thread_param.bind_cpu_id_ = kMiscThreadIndex_;
  thread_param.cycle_warning_period_ = 0.05;
  monitor_thread_on_ = true;
  RegisterThread(
      std::thread(&ZimaThreadManager::MonitorThread, this, thread_param),
      thread_param);
}

bool ZimaThreadManager::RegisterThread(
    std::thread&& thread, const ZimaThreadWrapper::ThreadParam& param) {
  ZGINFO << "Try to register for thread \"" << param.thread_name_ << "\"";
  WriteLocker lock(access_);
  if (threads_map_.count(param.thread_name_) != 0) {
    ZGWARN << "Thread \"" << param.thread_name_ << "\" was already registered.";
    return false;
  }
  threads_map_.emplace(param.thread_name_,
                       std::make_shared<ZimaThreadWrapper>(
                           std::forward<std::thread>(thread), param));
  ZGINFO << "Register for thread \"" << param.thread_name_ << "\", totally "
         << threads_map_.size() << " threads running.";
  return true;
}

bool ZimaThreadManager::MarkThreadExited(const std::string& thread_name,
                                         const std::string& file,
                                         const std::string& function,
                                         const uint32_t& line) {
  ReadLocker lock(access_);
  if (threads_map_.count(thread_name) == 0) {
    ZGWARN << "Thread \"" << thread_name << "\" was not registered.";
    return false;
  }
  threads_map_.at(thread_name)->SetIsMarkedExit(true);

  auto index = file.find_last_of('/');
  std::string file_name = file;
  if (index != file.npos) {
    file_name = file.substr(index + 1);
  }

  ZGINFO << file_name << ":" << line << " " << function << ": Mark thread \""
         << thread_name << "\" exited.";
  return true;
}

bool ZimaThreadManager::IsThreadRunning(const std::string& thread_name) {
  ReadLocker lock(access_);
  return threads_map_.count(thread_name) != 0;
}

bool ZimaThreadManager::WaitForThreadExit(const std::string& thread_name,
                                          const double& timeout_s) {
  auto start_time = Time::Now();
  while (true) {
    if (Time::Now() - start_time > timeout_s) {
      ZWARN << "Wait for thread \"" << thread_name << "\" exit timeout("
            << DoubleToString(timeout_s, 1) << "s).";
      return false;
    }
    if (!IsThreadRunning(thread_name)) {
      // ZWARN << "Thread \"" << thread_name << "\" is already exit.";
      break;
    }
    Time::SleepMSec(1);
  }
  return true;
}

uint16_t ZimaThreadManager::RunningThreadCount() {
  ReadLocker lock(access_);
  return threads_map_.size();
}

void ZimaThreadManager::DebugRunningThreadName() {
  ReadLocker lock(access_);
  for (auto&& thread_handle : threads_map_) {
    ZINFO << "Thread \"" << thread_handle.first << "\" is running.";
  }
}

bool ZimaThreadManager::UpdateThreadCycle(const std::string& thread_name) {
  ReadLocker lock(access_);
  if (threads_map_.count(thread_name) == 0) {
    ZWARN << "Thread \"" << thread_name << "\" was not registered.";
    return false;
  }
  threads_map_.at(thread_name)->SetCycleStartTime(Time::SystemNow());

  return true;
}

bool ZimaThreadManager::BindCPUCore(const std::string& thread_name,
                                    const uint16_t& cpu_core) {
  if (cpu_core >= cpu_max_core_count_) {
    ZERROR << "Current machine has only " << cpu_max_core_count_
           << " cores, unable to bind core " << cpu_core;
    return false;
  }
  cpu_set_t mask;
  CPU_ZERO(&mask);
  CPU_SET(cpu_core, &mask);
  ZGINFO << "Thread \"" << thread_name << "\" try to bind to cpu core "
         << cpu_core;

  if (-1 == pthread_setaffinity_np(pthread_self(), sizeof(mask), &mask)) {
    ZERROR << "Thread " << thread_name << " bind to cpu core " << cpu_core
           << " failed.";
    return false;
  }
  ZGINFO << "Thread \"" << thread_name << "\" bind to cpu core " << cpu_core;
  return true;
}

ZimaThreadWrapper::SPtr ZimaThreadManager::GetThreadHandle(
    const std::string& thread_name) {
  ReadLocker lock(access_);
  if (threads_map_.count(thread_name) == 0) {
    ZWARN << "Thread \"" << thread_name << "\" was not registered.";
    return nullptr;
  }
  return threads_map_.at(thread_name);
}

void ZimaThreadManager::MonitorThread(
    const ZimaThreadWrapper::ThreadParam& param) {
  ZINFO << "Thread start.";
  if (param.bind_cpu_id_ >= 0) {
    BindCPUCore(param.thread_name_, param.bind_cpu_id_);
  }

  std::deque<std::string> joined_thread_name{};
  while (monitor_thread_on_) {
    UpdateThreadCycle(param.thread_name_);
    {
      ReadLocker lock(access_);
      for (auto&& thread_handle : threads_map_) {
        thread_handle.second->PrintIfNeeded();
        // ZINFO << "Check for thread \"" << thread_handle.first << "\".";

        // Check for thread alive.
        auto is_marked_exit = thread_handle.second->GetIsMarkedExit();
        if (is_marked_exit) {
          ZGINFO << "Thread \"" << thread_handle.first << "\" is marked exit.";
          joined_thread_name.emplace_back(thread_handle.first);
        }
      }
      if (!joined_thread_name.empty()) {
        lock.Unlock();
        WriteLocker write_lock(access_);
        for (auto&& name : joined_thread_name) {
          ZGINFO << "Waiting for thread \"" << name << "\" to join.";
          threads_map_.at(name)->Join();
          ZGINFO << "Erase thread \"" << name << "\"";
          threads_map_.erase(name);
        }
        ZGINFO << "Erase finish, " << threads_map_.size() << " threads left.";
        joined_thread_name.clear();
      }
    }
    Time::SleepMSec(20);
  }
  ZINFO << "Thread end.";
}

bool ZimaProcessManager::SetSystemLevelProcessName(
    const std::string& system_level_process_name) {
  auto ret = prctl(PR_SET_NAME, system_level_process_name.c_str());
  if (ret != 0) {
    ZERROR << "Set name for process failed.";
    return false;
  }
  return true;
}

}  // namespace zima
