/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/mode/auto_scan_house_mode_event_wrapper.h"

#include "zima/robot_data/local_nav_data.h"
#include "zima/robot_data/operation_data_manager.h"

namespace zima {

AutoScanHouseModeWrapper::AutoScanHouseModeWrapper(
    const Chassis::SPtr& chassis,
    const ChassisController::SPtr& chassis_controller,
    const SlamBase::SPtr& slam_wrapper, OperationData::SPtr& operation_data,
    const AutoScanHouse::AutoScanningInfo::SPtr& cached_scanning_info,
    const bool& use_simple_slam)
    : ModeEventWrapperBase(ModeLabel::kAutoScanHouseMode) {
  if (operation_data == nullptr &&
      !OperationDataManager::CreateOperationData(
          operation_data, slam_wrapper,
          OperationData::OperationType::kAllHouseScanning, use_simple_slam)) {
    ZERROR << "Nav data pointer empty.";
    is_valid_ = false;
    return;
  }
  AutoScanHouseMode::EntranceType entrance_type =
      AutoScanHouseMode::EntranceType::kNewScanning;
  auto footstep_layer =
      operation_data->GetNavMapConstRef()->GetFootStepLayer();
  ReadLocker lock(footstep_layer->GetLock());
  if (footstep_layer->IsMarked()) {
    // Resume scanning.
    entrance_type = AutoScanHouseMode::EntranceType::kResumeScanning;
  }

  auto_scan_house_mode_ = std::make_shared<AutoScanHouseMode>(
      chassis, chassis_controller, slam_wrapper, entrance_type,
      cached_scanning_info);
  mode_ = auto_scan_house_mode_;
  auto_scan_house_mode_->Start();
  InitializeForCallBack();
  use_simple_slam_ = use_simple_slam;
}

AutoScanHouseModeWrapper::~AutoScanHouseModeWrapper() {}

void AutoScanHouseModeWrapper::Run(OperationData::SPtr& operation_data,
                                   const SlamBase::SPtr& slam_wrapper) {
  // Handle notice and event.
  HandleCommonNotice(operation_data);
  HandleUserEvent(operation_data);

  if (next_mode_label_ != ModeLabel::kNull) {
    is_exited_.store(true);
    return;
  }

  auto_scan_house_mode_->Run(operation_data);
  if (auto_scan_house_mode_->IsPaused()) {
    next_mode_label_ = ModeLabel::kPauseMode;
  } else if (auto_scan_house_mode_->IsStopped() ||
             auto_scan_house_mode_->IsAutoFinished()) {
    next_mode_label_ = ModeLabel::kStandbyMode;
    if (!OperationDataManager::ReleaseOperationData(
            operation_data, slam_wrapper, use_simple_slam_)) {
      ZERROR;
    }
    slam_wrapper->StopSlam();
  }
}

AutoScanHouse::AutoScanningInfo::SPtr
AutoScanHouseModeWrapper::GetScanningInfo() {
  return auto_scan_house_mode_->GetScanningInfo();
}

void AutoScanHouseModeWrapper::InitializeForCallBack() {
  // Common notice.
  RegisterCommonNoticeCallBack(
      StartScanningNotice::kLabel_,
      [](const CommonNotice::SPtr& common_notice,
         const OperationData::SPtr& operation_data) -> bool {
        ZINFO << "Notice: Scan house start.(voice)";
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
      ResumeScanningNotice::kLabel_,
      [](const CommonNotice::SPtr& common_notice,
         const OperationData::SPtr& operation_data) -> bool {
        ZINFO << "Notice: Resume scanning.(voice)";
        return true;
      });
  RegisterCommonNoticeCallBack(
      FinishScanningNotice::kLabel_,
      [](const CommonNotice::SPtr& common_notice,
         const OperationData::SPtr& operation_data) -> bool {
        ZINFO << "Notice: Scanning finished.(voice)";
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
        if (auto_scan_house_mode_->IsInitializing() ||
            auto_scan_house_mode_->IsRunning()) {
          switch (keyboard_event->key_) {
            case ' ': {
              auto_scan_house_mode_->Pause();
              break;
            }
            // case 's': {
            //   if (auto_scan_house_mode_->IsStallTestRunning()) {
            //     auto_scan_house_mode_->DisableStallTest();
            //   } else {
            //     auto_scan_house_mode_->EnableStallTest();
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

void AutoScanHouseModeWrapper::PrintInfo() {
  std::string info_str;
  info_str += ZCOLOR_GREEN;
  info_str += GetZimaPrintString();
  info_str += "\nAuto scan house mode keyboard command list: ";
  info_str += "\n-------------------------------------";
  info_str += "\nSpace: Pause operation.";
  // info_str += "\ns: Stall test.";
  info_str += "\n-------------------------------------";
  info_str += ZCOLOR_NONE;

  ZINFO << info_str;
}

}  // namespace zima
