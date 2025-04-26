/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_TIME_H
#define ZIMA_TIME_H

#include <atomic>
#include <string>

#include "zima/common/lock.h"

// This file contains definition of
// 1. Time interface
// 2. Time manager
// 3. StopWatch
// 4. Timer

namespace zima {

class Time {
 public:
  Time() = delete;
  ~Time() = delete;

  static void Initial();
  static double Now();
  static double SystemNow();
  static double UpTime();
  static std::string DebugString(const double& now);
  static std::string DebugString(const uint32_t& now);

  static void SleepSec(const double& sec);
  static void SystemSleepSec(const double& sec);
  // For milliseconds
  static void SleepMSec(const double& msec);
  static void SystemSleepMSec(const double& msec);

  static void UpdateFromExternalClock(const double& time_s);
  static void UpdateTimeAcceleration(const float& time_acceleration);

 private:
  static std::atomic_bool ready_;
  static std::atomic_bool external_time_source_;
  static ReadWriteLock::SPtr lock_;
  static double external_clock_now_;
  static float time_acceleration_;
};

class UTCTime {
 public:
  UTCTime() = delete;
  ~UTCTime() = delete;

  static double Now() {
    ReadLocker lock(lock_);
    return Time::Now() + offset_sec_;
  }
  static void UpdateUTC(const double& utc_time) {
    WriteLocker lock(lock_);
    offset_sec_ = utc_time - Time::Now();
  }
  static void UpdateOffsetSec(const double& offset) {
    WriteLocker lock(lock_);
    offset_sec_ = offset;
  };
  static double GetOffsetSec() {
    ReadLocker lock(lock_);
    return offset_sec_;
  }

 private:
  static ReadWriteLock::SPtr lock_;
  static double offset_sec_;
};

class LocalTime {
 public:
  LocalTime() = delete;
  ~LocalTime() = delete;

  static double Now() {
    ReadLocker lock(lock_);
    return UTCTime::Now() + time_zone_sec_offset_;
  };
  static void UpdateLocalTime(const double& local_time) {
    WriteLocker lock(lock_);
    time_zone_sec_offset_ = local_time - UTCTime::Now();
  }
  static void UpdateTimeZoneSec(const double& time_zone_sec) {
    WriteLocker lock(lock_);
    time_zone_sec_offset_ = time_zone_sec;
  }
  static double getTimeZoneOffsetSec() {
    ReadLocker lock(lock_);
    return time_zone_sec_offset_;
  }

 private:
  static ReadWriteLock::SPtr lock_;
  static double time_zone_sec_offset_;
};

class StopWatch {
 public:
  StopWatch() = delete;
  StopWatch(const StopWatch& ref);
  StopWatch(const std::string& name, const bool& start_on_creation = true,
            const bool& silence = false, const bool& use_system_time = false);
  StopWatch(const std::string& name, const double& restore_duration,
            const bool& use_system_time = false);
  ~StopWatch() = default;

  void Start(const bool silence = false);
  void Stop();
  void Pause();
  void Resume();

  double Duration() const;

  std::string Name() const;
  std::string DebugString() const;

 private:
  enum State {
    kStopped,
    kRunning,
    kPause,
  };

  std::string name_;
  State state_;
  double start_up_time_;
  double saved_duration_;
  bool use_system_time_;
};

class Timer {
 public:
  Timer(const std::string& name, const double& duration_sec,
        const bool& start_on_creation = true, const bool& silence = false,
        const bool& use_system_time = false);
  ~Timer() = default;

  void Start();
  double GetPassSec() const;
  double GetRemainSec() const;
  bool TimeUp();
  void Reset();
  std::string DebugString() const;

 private:
  std::string name_;
  double target_duration_sec_;
  bool started_;
  bool use_system_time_;
  StopWatch stop_watch_;

  bool silence_;
};

}  // namespace zima

#endif  // ZIMA_TIME_H
