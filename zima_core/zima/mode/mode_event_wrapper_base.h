/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_MODE_EVENT_WRAPPER_BASE_H
#define ZIMA_MODE_EVENT_WRAPPER_BASE_H

#include <map>

#include "zima/algorithm/slam/slam_base.h"
#include "zima/event/common_notice.h"
#include "zima/event/user_event.h"
#include "zima/mode/mode_base.h"

namespace zima {

class ModeEventWrapperBase : public DebugBase {
 public:
  enum ModeLabel {
    kNull,
    kAutoCleaningMode,
    kAutoScanHouseMode,
    kPauseMode,
    kStandbyMode,
  };

  using SPtr = std::shared_ptr<ModeEventWrapperBase>;

  static const std::string GetModeNameByLabel(const ModeLabel& label);

  const ModeLabel GetModeLabel() const { return current_mode_label_; };
  const ModeLabel GetNextModeLabel() const { return next_mode_label_; };

  const bool IsExited() const { return is_exited_.load(); }

 protected:
  ModeEventWrapperBase() = delete;
  ModeEventWrapperBase(const ModeLabel& mode_label);
  virtual ~ModeEventWrapperBase() = default;

  virtual void Run(OperationData::SPtr& operation_data,
                   const SlamBase::SPtr& slam_wrapper) = 0;

  using CommonNoticeCallback =
      std::function<bool(const CommonNotice::SPtr& common_notice,
                         const OperationData::SPtr& operation_data)>;
  using UserEventCallback =
      std::function<bool(const UserEvent::SPtr& user_event,
                         const OperationData::SPtr& operation_data)>;

  bool RegisterCommonNoticeCallBack(const std::string& label,
                                    const CommonNoticeCallback& cb);
  bool RegisterUserEventCallBack(const std::string& label,
                                 const UserEventCallback& cb);
  bool HandleCommonNotice(const OperationData::SPtr& operation_data);
  bool HandleUserEvent(const OperationData::SPtr& operation_data);

  bool IsValid() const { return is_valid_; }

  void ResetDefaultCallBack();

  ModeBase::SPtr mode_;

  std::map<std::string, CommonNoticeCallback> common_notice_callback_map_;
  std::map<std::string, UserEventCallback> user_event_callback_map_;

  bool is_valid_;
  atomic_bool is_exited_;
  const ModeLabel current_mode_label_;
  ModeLabel next_mode_label_;

  bool use_simple_slam_;
};

}  // namespace zima

#endif  // ZIMA_MODE_EVENT_WRAPPER_BASE_H
