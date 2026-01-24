/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/event/event_manager.h"

namespace zima {

EventManager::EventManager() : access_(std::make_shared<ReadWriteLock>()) {}

void EventManager::PushCommonNotice(const CommonNotice::SPtr& notice) {
  WriteLocker lock(access_);
  common_notice_list_.emplace_back(notice);
}

CommonNotice::SPtr EventManager::PopCommonNotice() {
  WriteLocker lock(access_);
  if (common_notice_list_.empty()) {
    return nullptr;
  }

  auto notice = common_notice_list_.front();
  common_notice_list_.pop_front();
  return notice;
}

void EventManager::ClearNotice() {
  WriteLocker lock(access_);
  common_notice_list_.clear();
}

void EventManager::PushUserEvent(const UserEvent::SPtr& event) {
  WriteLocker lock(access_);
  user_event_list_.emplace_back(event);
}

UserEvent::SPtr EventManager::PopUserEvent() {
  WriteLocker lock(access_);
  if (user_event_list_.empty()) {
    return nullptr;
  }

  auto event = user_event_list_.front();
  user_event_list_.pop_front();
  return event;
}

void EventManager::ClearUserEvent() {
  WriteLocker lock(access_);
  user_event_list_.clear();
}

}  // namespace zima
