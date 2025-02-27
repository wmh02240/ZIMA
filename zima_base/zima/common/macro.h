/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_MACRO_H
#define ZIMA_MACRO_H

#include <mutex>

#define DISALLOW_COPY_AND_ASSIGN(classname) \
  classname(const classname &) = delete;    \
  classname &operator=(const classname &) = delete;
 
#define DECLARE_SINGLETON(classname)                                      \
 public:                                                                  \
  static classname *Instance(bool create_if_needed = true) {              \
    static classname *instance = nullptr;                                 \
    if (!instance && create_if_needed) {                                  \
      static std::once_flag flag;                                         \
      std::call_once(flag,                                                \
                     [&] { instance = new (std::nothrow) classname(); }); \
    }                                                                     \
    return instance;                                                      \
  }                                                                       \
                                                                          \
 private:                                                                 \
  classname();                                                            \
  DISALLOW_COPY_AND_ASSIGN(classname)

#define DECLARE_DATA_GET_SET(data_type, data_name) \
  data_type Get##data_name() const; \
  void Set##data_name(const data_type& value);

#endif  // ZIMA_MACRO_H
