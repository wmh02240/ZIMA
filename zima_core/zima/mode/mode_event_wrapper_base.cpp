/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/mode/mode_event_wrapper_base.h"

#include "zima/event/event_manager.h"

namespace zima {

ModeEventWrapperBase::ModeEventWrapperBase(const ModeLabel& mode_label)
    : is_exited_(false),
      current_mode_label_(mode_label),
      next_mode_label_(ModeLabel::kNull),
      use_simple_slam_(true) {
  ResetDefaultCallBack();
}

const std::string ModeEventWrapperBase::GetModeNameByLabel(
    const ModeEventWrapperBase::ModeLabel& label) {
  switch (label) {
    case ModeEventWrapperBase::ModeLabel::kAutoCleaningMode: {
      return "auto cleaning mode";
    }
    case ModeEventWrapperBase::ModeLabel::kNull:
    default: {
      return "empty mode";
    }
  }

  ZERROR << "Should never run here.";
  return "";
}

bool ModeEventWrapperBase::RegisterCommonNoticeCallBack(
    const std::string& label, const CommonNoticeCallback& cb) {
  if (common_notice_callback_map_.count(label) > 0) {
    ZGINFO << "Call back for common notice \"" << label << "\" is override.";
    common_notice_callback_map_.at(label) = cb;
  } else {
    common_notice_callback_map_.emplace(label, cb);
  }
  return true;
}

bool ModeEventWrapperBase::RegisterUserEventCallBack(
    const std::string& label, const UserEventCallback& cb) {
  if (user_event_callback_map_.count(label) > 0) {
    ZGINFO << "Call back for common notice \"" << label << "\" is override.";
    user_event_callback_map_.at(label) = cb;
  } else {
    user_event_callback_map_.emplace(label, cb);
  }
  return true;
}

bool ModeEventWrapperBase::HandleCommonNotice(
    const OperationData::SPtr& operation_data) {
  // Handle notice and event.
  auto& event_manager = *EventManager::Instance();
  auto new_common_notice = event_manager.PopCommonNotice();
  if (new_common_notice != nullptr) {
    if (common_notice_callback_map_.count(new_common_notice->GetLabel()) > 0) {
      return common_notice_callback_map_[new_common_notice->GetLabel()](
          new_common_notice, operation_data);
    } else {
      ZWARN << "Unrecognized conmmon notice \"" << new_common_notice->GetLabel()
            << "\".";
    }
  }
  return false;
}

bool ModeEventWrapperBase::HandleUserEvent(
    const OperationData::SPtr& operation_data) {
  // Handle notice and event.
  auto& event_manager = *EventManager::Instance();
  auto new_user_event = event_manager.PopUserEvent();
  if (new_user_event != nullptr) {
    if (user_event_callback_map_.count(new_user_event->GetLabel()) > 0) {
      return user_event_callback_map_[new_user_event->GetLabel()](
          new_user_event, operation_data);
    } else {
      ZWARN << "Unrecognized user event \"" << new_user_event->GetLabel()
            << "\".";
    }
  }

  return false;
}

void ModeEventWrapperBase::ResetDefaultCallBack() {
  common_notice_callback_map_.clear();
  user_event_callback_map_.clear();

  // Common notice.
  auto default_common_notice_call_back =
      [this](const CommonNotice::SPtr& common_notice,
             const NavData::SPtr& operation_data) -> bool {
    ZINFO << "Common notice \"" << common_notice->GetLabel() << "\" ignored by "
          << GetModeNameByLabel(current_mode_label_) << ".";
    return true;
  };
  RegisterCommonNoticeCallBack(StartCleaningNotice::kLabel_,
                               default_common_notice_call_back);
  RegisterCommonNoticeCallBack(StartRelocationNotice::kLabel_,
                               default_common_notice_call_back);
  RegisterCommonNoticeCallBack(RelocationSucceedNotice::kLabel_,
                               default_common_notice_call_back);
  RegisterCommonNoticeCallBack(RelocationFailedNotice::kLabel_,
                               default_common_notice_call_back);
  RegisterCommonNoticeCallBack(PauseNotice::kLabel_,
                               default_common_notice_call_back);
  RegisterCommonNoticeCallBack(ResumeCleaningNotice::kLabel_,
                               default_common_notice_call_back);
  RegisterCommonNoticeCallBack(FinishCleaningNotice::kLabel_,
                               default_common_notice_call_back);
  RegisterCommonNoticeCallBack(FatalErrorNotice::kLabel_,
                               default_common_notice_call_back);

  // User event.
  auto default_user_event_call_back =
      [this](const UserEvent::SPtr& user_event,
             const NavData::SPtr& operation_data) -> bool {
    ZINFO << "User event \"" << user_event->GetLabel() << "\" ignored by "
          << GetModeNameByLabel(current_mode_label_) << ".";
    return true;
  };
  RegisterUserEventCallBack(StartCleaningEvent::kLabel_,
                            default_user_event_call_back);
  RegisterUserEventCallBack(PauseEvent::kLabel_, default_user_event_call_back);
  RegisterUserEventCallBack(ResumeCleaningEvent::kLabel_,
                            default_user_event_call_back);
  RegisterUserEventCallBack(StopCleaningEvent::kLabel_,
                            default_user_event_call_back);
  RegisterUserEventCallBack(VirtualWallBlockAreaEvent::kLabel_,
                            default_user_event_call_back);
}

}  // namespace zima
