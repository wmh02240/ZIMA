/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_BUTTON_H
#define ZIMA_BUTTON_H

#include <atomic>
#include <list>
#include <memory>
#include <string>

#include "zima/common/lock.h"
#include "zima/device/device.h"

namespace zima {

class Button : public DeviceBase {
 public:
  class Config : public DeviceBase::Config {
   public:
    Config();
    explicit Config(const JsonSPtr& json);
    ~Config() = default;

    bool ParseFromJson(const JsonSPtr& json);

    static const std::string kReleaseDelayKey_;
    float release_delay_;
  };

  class Event {
   public:
    enum EventType {
      kPressed,
      kReleased,
    };
    Event() = delete;
    explicit Event(const double& timestamp, const EventType& type);
    ~Event() = default;

    using SPtr = std::shared_ptr<Event>;

    const double timestamp_;
    const EventType type_;
  };

  Button() = delete;
  Button(const std::string name, const Config& config);
  ~Button() = default;

  using SPtr = std::shared_ptr<Button>;

  void UpdateRealtimeTriggeredState(const bool& triggered);

  Event::SPtr GetEvent();

  static const std::string kNullName_;

 private:
  ReadWriteLock::SPtr lock_;
  std::atomic_bool real_time_triggered_state_;
  std::list<Event::SPtr> event_list_;
  double data_timestamp_;
  bool mark_triggered_;
  double last_release_timestamp_;
  float release_delay_;
};

}  // namespace zima

#endif  // ZIMA_BUTTON_H
