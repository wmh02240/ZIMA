/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_KEYBOARD_LISTENER_H
#define ZIMA_KEYBOARD_LISTENER_H

#include <termios.h>

#include <atomic>
#include <functional>
#include <string>

#include "zima/common/thread.h"

namespace zima {

class KeyboardListener {
 public:
  using CallBackFunc = std::function<void(const char& key)>;

  KeyboardListener() = delete;
  KeyboardListener(const CallBackFunc& cb_func);
  ~KeyboardListener();

  bool Start();
  bool Stop();

 private:
  void OperateThread(const ZimaThreadWrapper::ThreadParam& param);

  std::atomic_bool working_;
  CallBackFunc cb_func_;
  ZimaThreadWrapper::ThreadParam thread_param_;
  struct termios init_settings_;
};

}  // namespace zima

#endif  // ZIMA_KEYBOARD_LISTENER_H
