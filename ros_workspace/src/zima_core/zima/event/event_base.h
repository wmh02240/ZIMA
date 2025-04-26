/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_EVENT_BASE_H
#define ZIMA_EVENT_BASE_H

#include "zima/common/debug.h"
#include "zima/common/time.h"

namespace zima {

class EventBase : public DebugBase {
 public:
  std::string GetLabel() const { return event_label_; }
  double GetTriggeredTime() const { return triggered_time_; }

 protected:
  EventBase() = delete;
  EventBase(const std::string& event_label) : event_label_(event_label) {
    triggered_time_ = Time::Now();
  }
  ~EventBase() = default;

  std::string event_label_;
  double triggered_time_;
};

}  // namespace zima

#endif  // ZIMA_EVENT_BASE_H
