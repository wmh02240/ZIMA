/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/common/time.h"
#include "zima/logger/logger.h"

using namespace zima;

void TimeTest() {
  auto now = Time::Now();
  ZINFO << "Now is: " << std::to_string(now) << " format as "
        << Time::DebugString(now);

  auto up_time = Time::UpTime();
  ZINFO << "Uptime is: " << std::to_string(up_time);

  UTCTime::UpdateOffsetSec(3600);
  auto utc_now = UTCTime::Now();
  ZINFO << "UTC now is: " << std::to_string(utc_now) << " format as "
        << Time::DebugString(utc_now);
  UTCTime::UpdateUTC(UTCTime::Now() + 3600);
  auto utc_now2 = UTCTime::Now();
  ZINFO << "UTC now2 is: " << std::to_string(utc_now2) << " format as "
        << Time::DebugString(utc_now2);

  LocalTime::UpdateTimeZoneSec(3600 * 8);
  auto local_time = LocalTime::Now();
  ZINFO << "Local time is: " << std::to_string(local_time) << " format as "
        << Time::DebugString(local_time);
  LocalTime::UpdateLocalTime(LocalTime::Now() + 3600);
  auto local_time2 = LocalTime::Now();
  ZINFO << "Local time2 is: " << std::to_string(local_time2) << " format as "
        << Time::DebugString(local_time2);

  ZINFO << "Now is: " << std::to_string(Time::Now());
  Time::SleepSec(0.001);
  ZINFO << "Now is: " << std::to_string(Time::Now());
  Time::SleepSec(0.01);
  ZINFO << "Now is: " << std::to_string(Time::Now());
  Time::SleepSec(0.1);
  ZINFO << "Now is: " << std::to_string(Time::Now());
  Time::SleepSec(1);
  ZINFO << "Now is: " << std::to_string(Time::Now());

  Time::SleepMSec(0.001);
  ZINFO << "Now is: " << std::to_string(Time::Now());
  Time::SleepMSec(0.01);
  ZINFO << "Now is: " << std::to_string(Time::Now());
  Time::SleepMSec(0.1);
  ZINFO << "Now is: " << std::to_string(Time::Now());
  Time::SleepMSec(1);
  ZINFO << "Now is: " << std::to_string(Time::Now());
  Time::SleepMSec(10);
  ZINFO << "Now is: " << std::to_string(Time::Now());
  Time::SleepMSec(100);
  ZINFO << "Now is: " << std::to_string(Time::Now());
  Time::SleepMSec(1000);
  ZINFO << "Now is: " << std::to_string(Time::Now());

  Time::UpdateFromExternalClock(1);
  ZINFO << "Now is: " << std::to_string(Time::Now());
  Time::SleepSec(1);
  ZINFO << "Now is: " << std::to_string(Time::Now());
  Time::UpdateFromExternalClock(2);
  Time::UpdateTimeAcceleration(2);
  ZINFO << "Now is: " << std::to_string(Time::Now());
  Time::SleepSec(2);
  ZINFO << "Now is: " << std::to_string(Time::Now());
}

void StopWatchTest() {
  StopWatch stop_watch("test", false);
  Time::SleepSec(0.5);
  ZINFO << stop_watch.DebugString();
  ZINFO << "StopWatch duration: " << std::to_string(stop_watch.Duration());
  stop_watch.Start();
  Time::SleepSec(0.5);
  ZINFO << stop_watch.DebugString();
  ZINFO << "StopWatch duration: " << std::to_string(stop_watch.Duration());
  stop_watch.Start();
  Time::SleepSec(0.5);
  ZINFO << stop_watch.DebugString();
  ZINFO << "StopWatch duration: " << std::to_string(stop_watch.Duration());
  stop_watch.Pause();
  Time::SleepSec(0.5);
  ZINFO << stop_watch.DebugString();
  ZINFO << "StopWatch duration: " << std::to_string(stop_watch.Duration());
  stop_watch.Pause();
  Time::SleepSec(0.5);
  ZINFO << stop_watch.DebugString();
  ZINFO << "StopWatch duration: " << std::to_string(stop_watch.Duration());
  stop_watch.Resume();
  Time::SleepSec(0.5);
  ZINFO << stop_watch.DebugString();
  ZINFO << "StopWatch duration: " << std::to_string(stop_watch.Duration());
  stop_watch.Resume();
  Time::SleepSec(0.5);
  ZINFO << stop_watch.DebugString();
  ZINFO << "StopWatch duration: " << std::to_string(stop_watch.Duration());
  stop_watch.Stop();
  Time::SleepSec(0.5);
  ZINFO << stop_watch.DebugString();
  ZINFO << "StopWatch duration: " << std::to_string(stop_watch.Duration());
  stop_watch.Stop();
  Time::SleepSec(0.5);
  ZINFO << stop_watch.DebugString();
  ZINFO << "StopWatch duration: " << std::to_string(stop_watch.Duration());
}

void TimerTest() {
  Timer timer("Test", 3, false);
  ZINFO << timer.DebugString();
  timer.Start();
  ZINFO << timer.DebugString() << " Timeup " << std::to_string(timer.TimeUp());
  Time::SleepSec(0.5);
  ZINFO << timer.DebugString() << " Timeup " << std::to_string(timer.TimeUp());
  Time::SleepSec(0.5);
  ZINFO << timer.DebugString() << " Timeup " << std::to_string(timer.TimeUp());
  Time::SleepSec(1.5);
  ZINFO << timer.DebugString() << " Timeup " << std::to_string(timer.TimeUp());
  Time::SleepSec(1.5);
  ZINFO << timer.DebugString() << " Timeup " << std::to_string(timer.TimeUp());
}

int main(int argc, char **argv) {
  using namespace zima;
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  TimeTest();
  StopWatchTest();
  TimerTest();

  return 0;
}
