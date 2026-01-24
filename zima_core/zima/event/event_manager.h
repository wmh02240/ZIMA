/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_EVENT_MANAGER_H
#define ZIMA_EVENT_MANAGER_H

#include <list>

#include "zima/common/lock.h"
#include "zima/common/macro.h"
#include "zima/event/common_notice.h"
#include "zima/event/user_event.h"

namespace zima {

class EventManager {
  DECLARE_SINGLETON(EventManager)
 public:
  ~EventManager() = default;

  void PushCommonNotice(const CommonNotice::SPtr& notice);
  CommonNotice::SPtr PopCommonNotice();
  void ClearNotice();
  void PushUserEvent(const UserEvent::SPtr& event);
  UserEvent::SPtr PopUserEvent();
  void ClearUserEvent();

 private:
  ReadWriteLock::SPtr access_;
  CommonNotice::SPtrList common_notice_list_;
  UserEvent::SPtrList user_event_list_;
};

}  // namespace zima

#endif  // ZIMA_EVENT_MANAGER_H
