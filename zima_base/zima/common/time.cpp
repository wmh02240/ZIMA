/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/common/time.h"

#include <chrono>

#include "zima/common/maths.h"
#include "zima/common/util.h"
#include "zima/logger/logger.h"

namespace zima {

std::atomic_bool Time::ready_(false);
std::atomic_bool Time::external_time_source_(false);
ReadWriteLock::SPtr Time::lock_ = make_shared<ReadWriteLock>();
double Time::external_clock_now_ = 0;
float Time::time_acceleration_ = 1;

ReadWriteLock::SPtr UTCTime::lock_ = make_shared<ReadWriteLock>();
double UTCTime::offset_sec_ = 0;

ReadWriteLock::SPtr LocalTime::lock_ = make_shared<ReadWriteLock>();
double LocalTime::time_zone_sec_offset_ = 0;

void Time::Initial() {
  Time::ready_.store(true);
  ZINFO << "Now is " << std::to_string(Time::Now());
  UTCTime::UpdateUTC(Time::Now());
  LocalTime::UpdateTimeZoneSec(0);
}

double Time::Now() {
  if (!Time::ready_.load()) {
    Initial();
  }
  if (external_time_source_.load()) {
    ReadLocker lock(lock_);
    return external_clock_now_;
  }
  return std::chrono::system_clock::now().time_since_epoch().count() / 1e9;
}

double Time::SystemNow() {
  if (!Time::ready_.load()) {
    Initial();
  }
  return std::chrono::system_clock::now().time_since_epoch().count() / 1e9;
}

double Time::UpTime() {
  if (!Time::ready_.load()) {
    Initial();
  }
  if (external_time_source_.load()) {
    ReadLocker lock(lock_);
    return external_clock_now_;
  }
  return std::chrono::steady_clock::now().time_since_epoch().count() / 1e9;
}

std::string Time::DebugString(const double& now) {
  auto _now = static_cast<uint32_t>(now);
  return DebugString(_now);
}

std::string Time::DebugString(const uint32_t& now) {
  auto _now = static_cast<time_t>(now);
  std::string str(std::ctime(&_now));
  auto pos = str.find(" ");
  while (pos != str.npos) {
    str.replace(pos, 1, "_");
    pos = str.find(" ");
  }
  pos = str.find("\n");
  while (pos != str.npos) {
    str.replace(pos, 1, "_");
    pos = str.find("\n");
  }
  return str;
}

void Time::SleepSec(const double& sec) {
  if (external_time_source_.load()) {
    usleep(sec * 1e6 / time_acceleration_);
  } else {
    usleep(sec * 1e6);
  }
}

void Time::SystemSleepSec(const double& sec) { usleep(sec * 1e6); }

void Time::SleepMSec(const double& msec) {
  if (external_time_source_.load()) {
    usleep(msec * 1e3 / time_acceleration_);
  } else {
    usleep(msec * 1e3);
  }
}

void Time::SystemSleepMSec(const double& msec) { usleep(msec * 1e3); }

void Time::UpdateFromExternalClock(const double& time_s) {
  WriteLocker lock(lock_);
  external_time_source_.store(true);
  external_clock_now_ = time_s;
}

void Time::UpdateTimeAcceleration(const float& time_acceleration) {
  WriteLocker lock(lock_);
  external_time_source_.store(true);
  time_acceleration_ = time_acceleration;
}

StopWatch::StopWatch(const StopWatch& ref)
    : name_(ref.Name()),
      state_(ref.state_),
      start_up_time_(ref.start_up_time_),
      saved_duration_(ref.saved_duration_),
      use_system_time_(ref.use_system_time_) {}

StopWatch::StopWatch(const std::string& name, const bool& start_on_creation,
                     const bool& silence, const bool& use_system_time)
    : name_(name),
      state_(kStopped),
      start_up_time_(0),
      saved_duration_(0),
      use_system_time_(use_system_time) {
  if (start_on_creation) {
    Start(silence);
  }
}

StopWatch::StopWatch(const std::string& name, const double& restore_duration,
                     const bool& use_system_time)
    : name_(name),
      state_(kStopped),
      start_up_time_(0),
      saved_duration_(restore_duration),
      use_system_time_(use_system_time) {
  // ZINFO << "Restore duration: " << DoubleToString(saved_duration_, 1);
}

void StopWatch::Start(const bool silence) {
  switch (state_) {
    case kStopped: {
      saved_duration_ = 0;
      if (use_system_time_) {
        start_up_time_ = Time::SystemNow();
      } else {
        start_up_time_ = Time::UpTime();
      }
      state_ = kRunning;
      if (!silence) {
        ZINFO << "StopWatch " << name_ << " started.";
      }
      break;
    }
      // case kRunning:
      // case kPause:
    default: {
      if (!silence) {
        ZWARN << "StopWatch " << name_ << " already started.";
      }
      break;
    }
  }
}

void StopWatch::Stop() {
  switch (state_) {
    case kRunning:
    case kPause: {
      saved_duration_ = Duration();
      start_up_time_ = 0;
      state_ = kStopped;
      ZINFO << "StopWatch " << name_ << " stopped.";
      break;
    }
      // case kStopped:
    default: {
      ZWARN << "StopWatch " << name_ << " already stopped.";
      break;
    }
  }
}

void StopWatch::Pause() {
  switch (state_) {
    case kRunning: {
      saved_duration_ = Duration();
      state_ = kPause;
      ZINFO << "StopWatch " << name_ << " paused.";
      break;
    }
    case kStopped: {
      ZWARN << "StopWatch " << name_ << " is stopped.";
      break;
    }
    // case kPause:
    default: {
      ZWARN << "StopWatch " << name_ << " already paused.";
      break;
    }
  }
}

void StopWatch::Resume() {
  switch (state_) {
    case kPause: {
      if (use_system_time_) {
        start_up_time_ = Time::SystemNow();
      } else {
        start_up_time_ = Time::UpTime();
      }
      state_ = kRunning;
      ZINFO << "StopWatch " << name_ << " resumed.";
      break;
    }
    case kStopped: {
      ZWARN << "StopWatch " << name_ << " is stopped.";
      break;
    }
    // case kRunning:
    default: {
      ZWARN << "StopWatch " << name_ << " is already running.";
      break;
    }
  }
}

double StopWatch::Duration() const {
  switch (state_) {
    case kPause:
    case kStopped: {
      return saved_duration_;
    }
    // case kRunning:
    default: {
      if (use_system_time_) {
        return Time::SystemNow() - start_up_time_ + saved_duration_;
      } else {
        return Time::UpTime() - start_up_time_ + saved_duration_;
      }
    }
  }
}

std::string StopWatch::Name() const { return name_; }

std::string StopWatch::DebugString() const {
  switch (state_) {
    case kStopped: {
      if (DoubleEqual(saved_duration_, 0)) {
        return "StopWatch " + name_ + " is never started.";
      } else {
        return "StopWatch " + name_ + " was stopped at " +
               std::to_string(Duration()) + "s.";
      }
    }
    case kRunning: {
      return "StopWatch " + name_ + " has been running for " +
             std::to_string(Duration()) + "s.";
    }
    // case kPause:
    default: {
      return "StopWatch " + name_ + " was paused after running for " +
             std::to_string(Duration()) + "s.";
    }
  }
  return "";
}

Timer::Timer(const std::string& name, const double& duration_sec,
             const bool& start_on_creation, const bool& silence,
             const bool& use_system_time)
    : name_(name),
      target_duration_sec_(duration_sec),
      started_(false),
      use_system_time_(use_system_time),
      stop_watch_(name, false, silence),
      silence_(silence) {
  if (start_on_creation) {
    Start();
  }
}

void Timer::Start() {
  if (!started_) {
    stop_watch_ =
        StopWatch("Stop watch: " + name_, true, silence_, use_system_time_);
    started_ = true;
    if (!silence_) {
      ZINFO << "Timer " << name_ << " started.";
    }
  } else {
    ZWARN << "Timer " << name_ << " already started.";
  }
}

double Timer::GetPassSec() const {
  if (started_) {
    return std::min(stop_watch_.Duration(), target_duration_sec_);
  } else {
    ZWARN << "Timer " << name_ << " is not started.";
    return 0;
  }
}

double Timer::GetRemainSec() const {
  if (started_) {
    return target_duration_sec_ - GetPassSec();
  } else {
    ZWARN << "Timer " << name_ << " is not started.";
    return target_duration_sec_;
  }
}

bool Timer::TimeUp() {
  if (started_) {
    return DoubleEqual(GetRemainSec(), 0);
  } else {
    ZWARN << "Timer " << name_ << " is not started.";
    return false;
  }
}

void Timer::Reset() {
  started_ = false;
  Start();
}

std::string Timer::DebugString() const {
  if (!started_) {
    return "Timer " + name_ + " is not started.";
  } else {
    return "Timer " + name_ + "(" + std::to_string(target_duration_sec_) +
           "s) has run for " + std::to_string(GetPassSec()) + "s, remaining " +
           std::to_string(GetRemainSec()) + "s.";
  }
}

}  // namespace zima
