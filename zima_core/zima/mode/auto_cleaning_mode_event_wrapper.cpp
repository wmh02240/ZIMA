/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/mode/auto_cleaning_mode_event_wrapper.h"

#include "zima/robot_data/local_nav_data.h"
#include "zima/robot_data/operation_data_manager.h"

namespace zima {

AutoCleaningModeWrapper::AutoCleaningModeWrapper(
    const Chassis::SPtr& chassis,
    const ChassisController::SPtr& chassis_controller,
    const SlamBase::SPtr& slam_wrapper, OperationData::SPtr& operation_data,
    const AutoCleaning::AutoCleaningInfo::SPtr& cached_cleaning_info,
    const bool& use_simple_slam)
    : ModeEventWrapperBase(ModeLabel::kAutoCleaningMode) {
  if (operation_data == nullptr &&
      !OperationDataManager::CreateOperationData(
          operation_data, slam_wrapper,
          OperationData::OperationType::kAllHouseCleaning, use_simple_slam)) {
    ZERROR << "Nav data pointer empty.";
    is_valid_ = false;
    return;
  }
  AutoCleaningMode::EntranceType entrance_type =
      AutoCleaningMode::EntranceType::kTotallyNewCleaning;
  auto footstep_layer =
      operation_data->GetNavMapConstRef()->GetFootStepLayer();
  ReadLocker lock(footstep_layer->GetLock());
  if (!footstep_layer->IsMarked()) {
    // New cleaning.
    if (operation_data->GetRawSlamValueGridMap2DRef()->IsMarked()) {
      // With selected nav data.
      entrance_type =
          AutoCleaningMode::EntranceType::kNewCleaningBaseOnSelectedNavData;
    }
  } else {
    // Resume cleaning.
    entrance_type =
        AutoCleaningMode::EntranceType::kResumeCleaningWithRobotQuiescentState;
  }

  auto_cleaning_mode_ = std::make_shared<AutoCleaningMode>(
      chassis, chassis_controller, slam_wrapper, entrance_type,
      cached_cleaning_info);
  mode_ = auto_cleaning_mode_;
  auto_cleaning_mode_->Start();
  InitializeForCallBack();
  use_simple_slam_ = use_simple_slam;
}

AutoCleaningModeWrapper::~AutoCleaningModeWrapper() {}

void AutoCleaningModeWrapper::Run(OperationData::SPtr& operation_data,
                                  const SlamBase::SPtr& slam_wrapper) {
  // Handle notice and event.
  HandleCommonNotice(operation_data);
  HandleUserEvent(operation_data);

  if (next_mode_label_ != ModeLabel::kNull) {
    is_exited_.store(true);
    return;
  }

  auto_cleaning_mode_->Run(operation_data);
  if (auto_cleaning_mode_->IsPaused()) {
    next_mode_label_ = ModeLabel::kPauseMode;
  } else if (auto_cleaning_mode_->IsStopped() ||
             auto_cleaning_mode_->IsAutoFinished()) {
    next_mode_label_ = ModeLabel::kStandbyMode;
    if (!OperationDataManager::ReleaseOperationData(
            operation_data, slam_wrapper, use_simple_slam_)) {
      ZERROR;
    }
    slam_wrapper->StopSlam();
  }
}

AutoCleaning::AutoCleaningInfo::SPtr
AutoCleaningModeWrapper::GetCleaningInfo() {
  return auto_cleaning_mode_->GetCleaningInfo();
}

void AutoCleaningModeWrapper::InitializeForCallBack() {
  // Common notice.
  RegisterCommonNoticeCallBack(
      StartCleaningNotice::kLabel_,
      [](const CommonNotice::SPtr& common_notice,
         const OperationData::SPtr& operation_data) -> bool {
        ZINFO << "Notice: Cleaning start.(voice)";
        return true;
      });
  RegisterCommonNoticeCallBack(
      StartRelocationNotice::kLabel_,
      [](const CommonNotice::SPtr& common_notice,
         const OperationData::SPtr& operation_data) -> bool {
        ZINFO << "Notice: Relocation start.(voice)";
        return true;
      });
  RegisterCommonNoticeCallBack(
      RelocationSucceedNotice::kLabel_,
      [](const CommonNotice::SPtr& common_notice,
         const OperationData::SPtr& operation_data) -> bool {
        ZINFO << "Notice: Relocation succeed.(voice)";
        return true;
      });
  RegisterCommonNoticeCallBack(
      RelocationFailedNotice::kLabel_,
      [](const CommonNotice::SPtr& common_notice,
         const OperationData::SPtr& operation_data) -> bool {
        ZINFO << "Notice: Relocation failed.(voice)";
        return true;
      });
  RegisterCommonNoticeCallBack(
      PauseNotice::kLabel_,
      [](const CommonNotice::SPtr& common_notice,
         const OperationData::SPtr& operation_data) -> bool {
        ZINFO << "Notice: Pause.(voice)";
        return true;
      });
  RegisterCommonNoticeCallBack(
      ResumeCleaningNotice::kLabel_,
      [](const CommonNotice::SPtr& common_notice,
         const OperationData::SPtr& operation_data) -> bool {
        ZINFO << "Notice: Resume cleaning.(voice)";
        return true;
      });
  RegisterCommonNoticeCallBack(
      FinishCleaningNotice::kLabel_,
      [](const CommonNotice::SPtr& common_notice,
         const OperationData::SPtr& operation_data) -> bool {
        ZINFO << "Notice: Cleaning finished.(voice)";
        return true;
      });
  RegisterCommonNoticeCallBack(
      FatalErrorNotice::kLabel_,
      [](const CommonNotice::SPtr& common_notice,
         const OperationData::SPtr& operation_data) -> bool {
        ZWARN << "Notice: Fatal error.(voice)";
        return true;
      });

  // User event.
  RegisterUserEventCallBack(
      KeyboardEvent::kLabel_,
      [this](const UserEvent::SPtr& user_event,
             const OperationData::SPtr& operation_data) -> bool {
        auto keyboard_event =
            dynamic_cast<const KeyboardEvent*>(user_event.get());
        ZINFO << "Receive key \"" << keyboard_event->key_ << "\"";
        if (auto_cleaning_mode_->IsInitializing() ||
            auto_cleaning_mode_->IsRunning()) {
          switch (keyboard_event->key_) {
            case ' ': {
              auto_cleaning_mode_->Pause();
              break;
            }
            // case 's': {
            //   if (auto_cleaning_mode_->IsStallTestRunning()) {
            //     auto_cleaning_mode_->DisableStallTest();
            //   } else {
            //     auto_cleaning_mode_->EnableStallTest();
            //   }
            //   break;
            // }
            default: {
              break;
            }
          }
        }
        PrintInfo();
        return true;
      });

  PrintInfo();
}

void AutoCleaningModeWrapper::PrintInfo() {
  std::string info_str;
  info_str += ZCOLOR_GREEN;
  info_str += GetZimaPrintString();
  info_str += "\nAuto cleaning mode keyboard command list: ";
  info_str += "\n-------------------------------------";
  info_str += "\nSpace: Pause operation.";
  // info_str += "\ns: Stall test.";
  info_str += "\n-------------------------------------";
  info_str += ZCOLOR_NONE;

  ZINFO << info_str;
}

}  // namespace zima
