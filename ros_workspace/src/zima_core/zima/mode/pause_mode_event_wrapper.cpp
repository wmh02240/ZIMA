/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/mode/pause_mode_event_wrapper.h"

#include "zima/event/event_manager.h"
#include "zima/robot_data/local_nav_data.h"
#include "zima/robot_data/operation_data_manager.h"

namespace zima {

PauseModeWrapper::PauseModeWrapper(
    const Chassis::SPtr& chassis,
    const ChassisController::SPtr& chassis_controller,
    const SlamBase::SPtr& slam_wrapper,
    const OperationData::SPtr& operation_data, const bool& use_simple_slam)
    : ModeEventWrapperBase(ModeLabel::kPauseMode) {
  if (operation_data == nullptr) {
    ZERROR << "Nav data pointer empty.";
    is_valid_ = false;
    return;
  }
  pause_mode_ =
      std::make_shared<PauseMode>(chassis, chassis_controller, slam_wrapper);
  mode_ = pause_mode_;
  pause_mode_->Start();
  InitializeForCallBack();
  use_simple_slam_ = use_simple_slam;
}

PauseModeWrapper::~PauseModeWrapper() {}

void PauseModeWrapper::Run(OperationData::SPtr& operation_data,
                           const SlamBase::SPtr& slam_wrapper) {
  // Handle notice and event.
  HandleCommonNotice(operation_data);
  HandleUserEvent(operation_data);

  if (next_mode_label_ != ModeLabel::kNull) {
    if (next_mode_label_ == ModeLabel::kStandbyMode) {
      if (!OperationDataManager::ReleaseOperationData(
              operation_data, slam_wrapper, use_simple_slam_)) {
        ZERROR;
      }
      slam_wrapper->StopSlam();
    }
    is_exited_.store(true);
    return;
  }

  pause_mode_->Run(operation_data);
}

void PauseModeWrapper::InitializeForCallBack() {
  // Common notice.
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
        ZINFO << "Receive key \""
              << (keyboard_event->key_ == ' '
                      ? "Space"
                      : std::string(1, keyboard_event->key_))
              << "\"";
        switch (keyboard_event->key_) {
          case ' ': {
            next_mode_label_ = GenerateNextMode(operation_data);
            break;
          }
          // case '1': {
          //   auto operations =
          //       VirtualWallBlockAreaEvent::DemoVirtualWallBlockAreaOperation();
          //   if (!operations.empty()) {
          //     auto& event_manager = *EventManager::Instance();
          //     event_manager.PushUserEvent(
          //         std::make_shared<VirtualWallBlockAreaEvent>(operations));
          //   }
          //   break;
          // }
          case 'q': {
            ZINFO << "Exit for keyboard command.";
            next_mode_label_ = kStandbyMode;
            break;
          }
          default: {
            break;
          }
        }

        PrintInfo();
        return true;
      });

  VirtualWallBlockAreaEvent::CreateDemoOperationTemplateFile();

  RegisterUserEventCallBack(
      VirtualWallBlockAreaEvent::kLabel_,
      [this](const UserEvent::SPtr& user_event,
             const OperationData::SPtr& operation_data) -> bool {
        auto virtual_wall_block_area_event =
            dynamic_cast<const VirtualWallBlockAreaEvent*>(user_event.get());
        ZINFO << "Receive " << VirtualWallBlockAreaEvent::kLabel_;
        bool operated = false;
        for (auto&& operation : virtual_wall_block_area_event->operations_) {
          if (operation.data_type_ ==
              VirtualWallBlockAreaEvent::Operation::DataType::kVirtualWall) {
            auto virtual_wall = operation.virtual_wall_;
            switch (operation.operation_type_) {
              case VirtualWallBlockAreaEvent::Operation::OperationType::kAdd: {
                operation_data->AddVirtualWall(virtual_wall);
                operated = true;
                break;
              }
              case VirtualWallBlockAreaEvent::Operation::OperationType::
                  kRemove: {
                operation_data->RemoveVirtualWall(operation.index_);
                operated = true;
                break;
              }
              case VirtualWallBlockAreaEvent::Operation::OperationType::kUpdate:
              default: {
                operation_data->UpdateVirtualWall(operation.index_,
                                                  virtual_wall);
                operated = true;
                break;
              }
            }
          } else {
            auto block_area = operation.block_area_;
            switch (operation.operation_type_) {
              case VirtualWallBlockAreaEvent::Operation::OperationType::kAdd: {
                operation_data->AddBlockArea(block_area);
                operated = true;
                break;
              }
              case VirtualWallBlockAreaEvent::Operation::OperationType::
                  kRemove: {
                operation_data->RemoveBlockArea(operation.index_);
                operated = true;
                break;
              }
              case VirtualWallBlockAreaEvent::Operation::OperationType::kUpdate:
              default: {
                operation_data->UpdateBlockArea(operation.index_, block_area);
                operated = true;
                break;
              }
            }
          }
        }
        if (operated) {
          operation_data->UpdateUserBlockMapInNavMap();
        }
        return true;
      });

  PrintInfo();
}

void PauseModeWrapper::PrintInfo() {
  std::string info_str;
  info_str += ZCOLOR_GREEN;
  info_str += GetZimaPrintString();
  info_str += "\nPause mode keyboard command list: ";
  info_str += "\n-------------------------------------";
  info_str += "\nSpace: Resume operation.";
  info_str += "\nq: Exit operation.";
  // info_str += "\n1: Virtual wall / block area operation from file.";
  info_str += "\n-------------------------------------";
  info_str += ZCOLOR_NONE;

  ZINFO << info_str;
}

ModeEventWrapperBase::ModeLabel PauseModeWrapper::GenerateNextMode(
    const OperationData::SPtr& operation_data) {
  if (operation_data->GetOperationType() ==
      OperationData::OperationType::kAllHouseScanning) {
    return ModeLabel::kAutoScanHouseMode;
  } else {
    return ModeLabel::kAutoCleaningMode;
  }
  ZERROR << "Should never run here.";
  return ModeLabel::kAutoCleaningMode;
}

}  // namespace zima
