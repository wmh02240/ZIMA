/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include <memory>
#include <thread>

#include "zima/common/lock.h"
#include "zima/common/time.h"
#include "zima/logger/logger.h"

using namespace zima;

auto test_lock = std::make_shared<ReadWriteLock>();

bool thread_exit = false;
void ContinuousRead(int n) {
  ZINFO << "Thread " << n;
  bool turn = true;
  while (!thread_exit) {
    Time::SleepMSec(100);

    if (turn) {
      ReadLocker r_locker(test_lock, false);
      ZINFO << "Thread " << n << " try read lock.";
      r_locker.Lock();

      ZINFO << "Thread " << n << " read lock.";
      Time::SleepMSec(500);
      ZINFO << "Thread " << n << " read unlock.";
    } else {
      ReadLocker r_locker(test_lock);

      ZINFO << "Thread " << n << " read lock.";
      Time::SleepMSec(500);
      ZINFO << "Thread " << n << " read unlock.";
    }
  }
  ZINFO << "Thread " << n << " end.";
}

void ContinuousWrite(int n) {
  ZINFO << "Thread " << n;
  bool turn = true;
  while (!thread_exit) {
    Time::SleepMSec(100);

    if (turn) {
      WriteLocker w_locker(test_lock, false);
      ZINFO << "Thread " << n << " try write lock.";
      w_locker.Lock();

      ZINFO << "Thread " << n << " write lock.";
      Time::SleepMSec(500);
      ZINFO << "Thread " << n << " write unlock.";
    } else {
      WriteLocker r_locker(test_lock);

      ZINFO << "Thread " << n << " write lock.";
      Time::SleepMSec(500);
      ZINFO << "Thread " << n << " write unlock.";
    }
  }
  ZINFO << "Thread " << n << " end.";
}

void TestReadAtSameTime() {
  thread_exit = false;
  std::thread r1(ContinuousRead, 1);
  Time::SleepMSec(100);
  std::thread r2(ContinuousRead, 2);
  Time::SleepMSec(100);
  std::thread r3(ContinuousRead, 3);
  Time::SleepMSec(100);
  std::thread w1(ContinuousWrite, 4);

  Time::SleepSec(5);
  thread_exit = true;
  r1.join();
  r2.join();
  r3.join();
  w1.join();

  ZWARN << "Finish.";
}

void TestWriteAtSameTime() {
  thread_exit = false;
  std::thread r1(ContinuousRead, 1);
  Time::SleepMSec(100);
  std::thread w1(ContinuousWrite, 1);
  Time::SleepMSec(100);
  std::thread w2(ContinuousWrite, 2);

  Time::SleepSec(5);
  thread_exit = true;
  r1.join();
  w1.join();
  w2.join();

  ZWARN << "Finish.";
}

void TestDeadLock() {
  auto test_read_write_dead_lock = []() -> void {
    auto _test_lock = std::make_shared<ReadWriteLock>();
    ReadLocker r_locker(_test_lock, false);
    WriteLocker w_locker(_test_lock, false);
    ZINFO << "Read first.";
    r_locker.Lock();
    ZINFO << "Try write lock.";
    w_locker.Lock();
    ZERROR << "This line should never be reached.";
  };
  auto test_write_read_dead_lock = []() -> void {
    auto _test_lock = std::make_shared<ReadWriteLock>();
    ReadLocker r_locker(_test_lock, false);
    WriteLocker w_locker(_test_lock, false);
    ZINFO << "Write first.";
    w_locker.Lock();
    ZINFO << "Try read lock.";
    r_locker.Lock();
    ZERROR << "This line should never be reached.";
  };
  auto test_write_write_dead_lock = []() -> void {
    auto _test_lock = std::make_shared<ReadWriteLock>();
    WriteLocker w_locker(_test_lock, false);
    WriteLocker w_locker2(_test_lock, false);
    ZINFO << "Write first.";
    w_locker.Lock();
    ZINFO << "Lock again.";
    w_locker2.Lock();
    ZERROR << "This line should never be reached.";
  };
  std::thread t1(test_read_write_dead_lock);
  std::thread t2(test_write_read_dead_lock);
  std::thread t3(test_write_write_dead_lock);

  Time::SleepSec(5);
  t1.join();
  ZERROR << "This line should never be reached.";
  t2.join();
  ZERROR << "This line should never be reached.";
  t3.join();
  ZERROR << "This line should never be reached.";

  ZWARN << "Finish.";
}

int main(int argc, char **argv) {
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  // TestReadAtSameTime();
  // TestWriteAtSameTime();
  TestDeadLock();
  return 0;
}
