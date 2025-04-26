/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/common/thread.h"
#include "zima/common/time.h"
#include "zima/logger/logger.h"

using namespace zima;

void TestThread(const ZimaThreadWrapper::ThreadParam& param) {
  ZINFO << "Thread \"" << param.thread_name_ << "\" start.";
  auto thread_manager = ZimaThreadManager::Instance();
  if (param.bind_cpu_id_ >= 0) {
    thread_manager->BindCPUCore(param.thread_name_, param.bind_cpu_id_);
  }
  srand(static_cast<unsigned>(time(NULL)));

  while (true) {
    thread_manager->UpdateThreadCycle(param.thread_name_);
    // Simulate loading.
    auto times = std::rand() % 5;
    if (times == 0) {
      Time::SleepMSec(1500);
    } else {
      Time::SleepMSec(times * 10);
    }

    // Time::SleepMSec(20);
  }
  ZINFO << "Thread \"" << param.thread_name_ << "\" exit.";
}

void ThreadTest() {
  auto thread_manager = ZimaThreadManager::Instance();
  auto cpu_max_count = thread_manager->GetCPUMaxCoreCount();
  ZINFO;
  ZimaThreadWrapper::ThreadParam test1_param("Test1", cpu_max_count - 2, 100,
                                             0.01, 0.3);
  thread_manager->RegisterThread(std::thread(&TestThread, test1_param),
                                 test1_param);
  while (true) {
    Time::SleepMSec(20);
  }
}

void StressTestThread(const ZimaThreadWrapper::ThreadParam& param) {
  ZINFO << "Thread \"" << param.thread_name_ << "\" start.";
  // ZINFO << static_cast<const void*>(&param);
  auto thread_manager = ZimaThreadManager::Instance();
  if (param.bind_cpu_id_ >= 0) {
    thread_manager->BindCPUCore(param.thread_name_, param.bind_cpu_id_);
  }
  srand(static_cast<unsigned>(time(NULL)));
  auto run_sec = rand() % 5 + 5;
  ZINFO << "Thread \"" << param.thread_name_ << "\" should run for " << run_sec
        << "s.";

  auto now = Time::Now();
  while (Time::Now() - now < run_sec) {
    thread_manager->UpdateThreadCycle(param.thread_name_);
    // Simulate stress, so do not sleep at all.
    Time::SleepSec(1);
  }
  ZINFO << "Thread \"" << param.thread_name_ << "\" exiting.";
  thread_manager->MarkThreadExited(param.thread_name_, __FILE__, __FUNCTION__,
                                   __LINE__);
  ZINFO << "Thread \"" << param.thread_name_ << "\" exit.";
}

void ThreadStressTest() {
  auto thread_manager = ZimaThreadManager::Instance();
  auto cpu_max_count = thread_manager->GetCPUMaxCoreCount();
  ZINFO;
  std::string name1 = "Stress test1";
  {
    ZimaThreadWrapper::ThreadParam test1_param(name1, cpu_max_count - 2, 100,
                                               -1, -1);
    // ZINFO << static_cast<const void*>(&test1_param);
    thread_manager->RegisterThread(std::thread(&StressTestThread, test1_param),
                                   test1_param);
  }
  Time::SleepSec(1);
  std::string name2 = "Stress test2";
  {
    ZimaThreadWrapper::ThreadParam test2_param(name2, cpu_max_count - 4, 100,
                                               -1, -1);
    // ZINFO << static_cast<const void*>(&test2_param);
    thread_manager->RegisterThread(std::thread(&StressTestThread, test2_param),
                                   test2_param);
  }

  uint8_t count = 0;
  while (++count < 10) {
    Time::SleepSec(1);
    ZINFO << "thread 1: " << thread_manager->IsThreadRunning(name1);
    ZINFO << "thread 2: " << thread_manager->IsThreadRunning(name2);
  }
}

int main(int argc, char** argv) {
  using namespace zima;
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  // ThreadTest();
  ThreadStressTest();

  return 0;
}
