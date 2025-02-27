/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/device/button.h"

#include "zima/common/config.h"
#include "zima/common/time.h"
#include "zima/logger/logger.h"

namespace zima {

Button::Config::Config() : Config(nullptr) {}

Button::Config::Config(const JsonSPtr& json) {
  // Load default setting.
  release_delay_ = 0.1;

  // Override if json config is provided.
  if (json != nullptr) {
    // ZINFO << "Load config from json.";
    config_valid_ = ParseFromJson(json);
  } else {
    config_valid_ = false;
  }
}

bool Button::Config::ParseFromJson(const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }

  if (!DeviceBase::Config::ParseFromJson(json)) {
    return false;
  }

  // Override.
  if (!JsonHelper::GetFloat(*json, kReleaseDelayKey_, release_delay_)) {
    ZERROR << "Config " << kReleaseDelayKey_ << " not found.";
    return false;
  }

  return true;
}

Button::Event::Event(const double& timestamp, const EventType& type)
    : timestamp_(timestamp), type_(type) {}

Button::Button(const std::string name, const Config& config)
    : DeviceBase(name, config),
      lock_(std::make_shared<ReadWriteLock>()),
      real_time_triggered_state_(false),
      data_timestamp_(0),
      mark_triggered_(false),
      last_release_timestamp_(0),
      release_delay_(config.release_delay_) {}

void Button::UpdateRealtimeTriggeredState(const bool& triggered) {
  auto last_triggered_state = real_time_triggered_state_.load();
  real_time_triggered_state_.store(triggered);
  WriteLocker lock(lock_);
  data_timestamp_ = Time::Now();
  if (triggered && !last_triggered_state) {
    ZINFO << name_ << " Triggered";
    event_list_.emplace_back(
        std::make_shared<Event>(data_timestamp_, Event::EventType::kPressed));
    mark_triggered_ = true;
  }
  if (!triggered && last_triggered_state) {
    last_release_timestamp_ = data_timestamp_;
  }
  if (mark_triggered_ && !triggered &&
      data_timestamp_ - last_release_timestamp_ > release_delay_) {
    ZINFO << name_ << " Released";
    event_list_.emplace_back(
        std::make_shared<Event>(data_timestamp_, Event::EventType::kReleased));
    mark_triggered_ = false;
  }
}

Button::Event::SPtr Button::GetEvent() {
  WriteLocker lock(lock_);
  if (event_list_.empty()) {
    return nullptr;
  }

  auto event = event_list_.front();
  event_list_.pop_front();
  return event;
}

}  // namespace zima
