/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_DEBUG_H
#define ZIMA_DEBUG_H

#include <atomic>
#include <string>

namespace zima {
class DebugBase {
 public:
  DebugBase()
      : debug_func_enable_(false), debug_log_enable_(false), debug_id_(0){};
  ~DebugBase() = default;

  void EnableDebugFunc() { debug_func_enable_.store(true); }
  void DisableDebugFunc() { debug_func_enable_.store(false); }
  bool IsDebugFuncEnabled() const { return debug_func_enable_.load(); }
  void EnableDebugLog() { debug_log_enable_.store(true); }
  void DisableDebugLog() { debug_log_enable_.store(false); }
  bool IsDebugLogEnabled() const { return debug_log_enable_.load(); }

 protected:
  std::atomic_bool debug_func_enable_;
  std::atomic_bool debug_log_enable_;

  uint64_t debug_id_;
};

class PIDPrint {
 public:
  PIDPrint() = default;
  ~PIDPrint() = default;

  static std::string DebugString(const std::string& name,
                                 const float& target_value,
                                 const float& print_range_min,
                                 const float& print_range_max,
                                 const float& current_value);
};

}  // namespace zima
#endif  // ZIMA_DEBUG_H
