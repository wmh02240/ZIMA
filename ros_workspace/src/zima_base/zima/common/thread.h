/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 * 线程管理系统：包括线程的注册、生命周期管理、cpu绑定、状态监控、线程退出。
 */

#ifndef ZIMA_THREAD_H
#define ZIMA_THREAD_H

#include <map>
#include <memory>
#include <thread>

#include "zima/common/lock.h"
#include "zima/common/macro.h"

namespace zima {

// ZimaThreadWrapper: 封装单个线程对象，记录线程参数、状态、周期信息、提供线程的join、状态打印、周期监控功能
class ZimaThreadWrapper {
  public:
    class ThreadParam {
      public:
        ThreadParam();
        ThreadParam(const std::string &name, const int16_t &bind_cpu_id, const int16_t &priority, const double &cycle_warning_period,
                    const double &cycle_error_period);
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
    ZimaThreadWrapper(std::thread &&thread, const ThreadParam &param);
    ~ZimaThreadWrapper();

    using SPtr = std::shared_ptr<ZimaThreadWrapper>;

    DECLARE_DATA_GET_SET(double, CycleStartTime);
    DECLARE_DATA_GET_SET(ThreadParam, Param);
    DECLARE_DATA_GET_SET(bool, IsMarkedExit);

    void PrintIfNeeded();             // 根据周期时间打印线程运行状态
    void Join() { thread_->join(); }; // 等待线程结束

  private:
    ReadWriteLock::SPtr access_;
    std::unique_ptr<std::thread> thread_; // 实际c++线程对象
    ThreadParam param_;                   // 线程参数
    CycleState cycle_state_;              // 线程当前周期状态
    double cycle_start_time_;             // 本周期开始时间
    double last_cycle_time_;
    double last_print_time_;
    atomic_bool is_marked_exit_; // 被标记是否退出
};

// 全局线程管理器，负责所有线程的安全注册、退出标记、监控、cpu绑定、周期更新操作。
class ZimaThreadManager {
    DECLARE_SINGLETON(ZimaThreadManager)
  public:
    ~ZimaThreadManager();

    using ThreadWrapperPair = std::pair<std::string, ZimaThreadWrapper::SPtr>;

    static const uint8_t kCoreThreadIndex_;
    static const uint8_t kSlamThreadIndex_;
    static const uint8_t kMiscThreadIndex_;
    static const uint8_t kRealtimeThreadIndex_;

    // Functions for thread management. These functions should be call by logic OUTSIDE thread.
    bool RegisterThread(std::thread &&thread, const ZimaThreadWrapper::ThreadParam &param); // 注册新线程并加入管理表
    bool MarkThreadExited(const std::string &thread_name, const std::string &file, const std::string &function, const uint32_t &line);
    bool IsThreadRunning(const std::string &thread_name);
    bool WaitForThreadExit(const std::string &thread_name, const double &timeout_s);

    uint16_t RunningThreadCount();
    void DebugRunningThreadName();
    bool UpdateThreadCycle(const std::string &thread_name); // 更新线程周期起始时间，用于周期监控
    uint16_t GetCPUMaxCoreCount() const { return cpu_max_core_count_; };

    // Functions for thread logic. These functions should be call by logic INSIDE thread.
    bool BindCPUCore(const std::string &thread_name, const uint16_t &cpu_core);

  private:
    ZimaThreadWrapper::SPtr GetThreadHandle(const std::string &thread_name);
    void MonitorThread(const ZimaThreadWrapper::ThreadParam &param); // 专门的监控线程，定期检查所有线程状态，自动join并清理已退出的线程

    ReadWriteLock::SPtr access_;
    uint16_t cpu_max_core_count_;                                           // cpu核心数
    bool monitor_thread_on_;                                                // 线程监控是否开启
    std::map<std::string, std::shared_ptr<ZimaThreadWrapper>> threads_map_; // 所有线程管理表
};

class ZimaProcessManager {
    DECLARE_SINGLETON(ZimaProcessManager)
  public:
    ~ZimaProcessManager();

    bool SetSystemLevelProcessName(const std::string &system_level_process_name);
};

} // namespace zima

#endif // ZIMA_THREAD_H
