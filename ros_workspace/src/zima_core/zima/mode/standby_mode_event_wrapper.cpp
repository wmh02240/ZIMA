/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/mode/standby_mode_event_wrapper.h"

#include "zima/event/event_manager.h"
#include "zima/robot_data/local_nav_data.h"

namespace zima {

StandbyModeWrapper::StandbyModeWrapper(
    const Chassis::SPtr& chassis,
    const ChassisController::SPtr& chassis_controller,
    const SlamBase::SPtr& slam_wrapper)
    : ModeEventWrapperBase(ModeLabel::kStandbyMode) {
  standby_mode_ =
      std::make_shared<StandbyMode>(chassis, chassis_controller, slam_wrapper);
  mode_ = standby_mode_;
  standby_mode_->Start();
  InitializeForCallBack();
}

StandbyModeWrapper::~StandbyModeWrapper() {}

void StandbyModeWrapper::Run(OperationData::SPtr& operation_data,
                             const SlamBase::SPtr& slam_wrapper) {
  // Handle notice and event.
  HandleCommonNotice(operation_data);
  HandleUserEvent(operation_data);

  if (next_mode_label_ != ModeLabel::kNull) {
    is_exited_.store(true);
    return;
  }

  standby_mode_->Run(operation_data);
}

void StandbyModeWrapper::InitializeForCallBack() {
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
        auto local_nav_data_manager = LocalNavDataManager::Instance();
        auto local_nav_data_index_list =
            local_nav_data_manager->GetNavDataIndexList();
        switch (keyboard_event->key_) {
          case '0':
          case '1':
          case '2': {
            auto index = stoi(std::string(1, keyboard_event->key_));
            if (local_nav_data_index_list.size() > static_cast<uint>(index)) {
              local_nav_data_manager->SelectNavData(
                  local_nav_data_index_list.at(index));
            } else {
              ZINFO << "Null option.";
            }
            break;
          }
          case '3': {
            local_nav_data_manager->UnSelectNavData();
            break;
          }
          case '4':
          case '5':
          case '6': {
            auto index = stoi(std::string(1, keyboard_event->key_)) - 4;
            if (local_nav_data_index_list.size() > static_cast<uint>(index)) {
              local_nav_data_manager->RemoveNavData(
                  local_nav_data_index_list.at(index));
            } else {
              ZINFO << "Null option.";
            }
            break;
          }
          case '7':
          case '8':
          case '9': {
            auto index = stoi(std::string(1, keyboard_event->key_)) - 7;
            if (local_nav_data_index_list.size() > static_cast<uint>(index)) {
              auto nav_data = local_nav_data_manager->GetNavData(
                  local_nav_data_index_list.at(index));
              ZINFO << "Index: "
                    << std::to_string(local_nav_data_index_list.at(index));
              nav_data->GetConstNavDataDataRef()
                  ->GetOptimizedSlamValueGridMap2DRef()
                  ->Print(__FILE__, __FUNCTION__, __LINE__);
              auto virtual_walls =
                  nav_data->GetConstNavDataDataRef()->GetAllVirtualWall();
              for (auto&& virtual_wall : virtual_walls) {
                ZINFO << "Virtual wall:"
                      << virtual_wall.second->DebugString(
                             nav_data->GetConstNavDataDataRef()
                                 ->GetNavMapConstRef()
                                 ->GetPrintLayer());
              }
              auto block_areas =
                  nav_data->GetConstNavDataDataRef()->GetAllBlockArea();
              for (auto&& block_area : block_areas) {
                ZINFO << "Block area:"
                      << block_area.second->DebugString(
                             nav_data->GetConstNavDataDataRef()
                                 ->GetNavMapConstRef()
                                 ->GetPrintLayer());
              }
            } else {
              ZINFO << "Null option.";
            }
            break;
          }
          case 'a': {
            next_mode_label_ = ModeLabel::kAutoCleaningMode;
            break;
          }
          case 's': {
            next_mode_label_ = ModeLabel::kAutoScanHouseMode;
            break;
          }
          // case 'e': {
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
            next_mode_label_ = ModeLabel::kNull;
            is_exited_.store(true);
            break;
          }
          default: {
            break;
          }
        }

        PrintInfo();
        return true;
      });

  RegisterUserEventCallBack(
      LocalNavDataEvent::kLabel_,
      [this](const UserEvent::SPtr& user_event,
             const OperationData::SPtr& operation_data) -> bool {
        auto local_nav_data_event =
            dynamic_cast<const LocalNavDataEvent*>(user_event.get());
        auto local_nav_data_manager = LocalNavDataManager::Instance();
        switch (local_nav_data_event->operation_type_) {
          case LocalNavDataEvent::OperationType::kSelect: {
            return local_nav_data_manager->SelectNavData(local_nav_data_event->index_);
          }
          case LocalNavDataEvent::OperationType::kUnSelect: {
            return local_nav_data_manager->UnSelectNavData();
          }
          case LocalNavDataEvent::OperationType::kDelete: {
            return local_nav_data_manager->RemoveNavData(local_nav_data_event->index_);
          }
          default: {
            break;
          }
        }

        return true;
      });

  VirtualWallBlockAreaEvent::CreateDemoOperationTemplateFile();

  RegisterUserEventCallBack(
      VirtualWallBlockAreaEvent::kLabel_,
      [this](const UserEvent::SPtr& user_event,
             const OperationData::SPtr& operation_data) -> bool {
        auto _operation_data = operation_data;
        auto local_nav_data_manager = LocalNavDataManager::Instance();
        if (_operation_data == nullptr) {
          ZINFO << "Operation data not created, try to create from selected "
                   "nav data";

          auto selected_operation_data_index =
              local_nav_data_manager->GetSelectedNavDataIndex();
          if (selected_operation_data_index != 0) {
            ZINFO << "Selected nav_data " << selected_operation_data_index
                  << ".";
            auto selected_nav_data = local_nav_data_manager->GetNavData(
                selected_operation_data_index);
            if (selected_nav_data == nullptr) {
              ZERROR << "Nav data invalid.";
              return false;
            }
            ZINFO << "Load nav_data " << selected_operation_data_index << ".";
            _operation_data = std::make_shared<OperationData>(
                *selected_nav_data->GetNavDataCopyData());
          } else {
            ZWARN << "No nav data selected.";
            return false;
          }
        }
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
                _operation_data->AddVirtualWall(virtual_wall);
                operated = true;
                break;
              }
              case VirtualWallBlockAreaEvent::Operation::OperationType::
                  kRemove: {
                _operation_data->RemoveVirtualWall(operation.index_);
                operated = true;
                break;
              }
              case VirtualWallBlockAreaEvent::Operation::OperationType::kUpdate:
              default: {
                _operation_data->UpdateVirtualWall(operation.index_,
                                                   virtual_wall);
                operated = true;
                break;
              }
            }
          } else {
            auto block_area = operation.block_area_;
            switch (operation.operation_type_) {
              case VirtualWallBlockAreaEvent::Operation::OperationType::kAdd: {
                _operation_data->AddBlockArea(block_area);
                operated = true;
                break;
              }
              case VirtualWallBlockAreaEvent::Operation::OperationType::
                  kRemove: {
                _operation_data->RemoveBlockArea(operation.index_);
                operated = true;
                break;
              }
              case VirtualWallBlockAreaEvent::Operation::OperationType::kUpdate:
              default: {
                _operation_data->UpdateBlockArea(operation.index_, block_area);
                operated = true;
                break;
              }
            }
          }
        }
        if (operated) {
          _operation_data->UpdateUserBlockMapInNavMap();
          local_nav_data_manager->UpdateNavData(_operation_data);
        }
        return true;
      });

  PrintInfo();
}

void StandbyModeWrapper::PrintInfo() {
  auto local_nav_data_manager = LocalNavDataManager::Instance();
  // auto max_local_nav_data_size =
  // local_nav_data_manager->local_nav_data_count_limit_;
  // Debug use, set to 3.
  uint max_local_nav_data_size = 3;
  auto local_nav_data_index_list =
      local_nav_data_manager->GetNavDataIndexList();
  auto selected_nav_data_index =
      local_nav_data_manager->GetSelectedNavDataIndex();
  std::string info_str;
  if (local_nav_data_index_list.size() == max_local_nav_data_size) {
    info_str += ZCOLOR_YELLOW;
    info_str +=
        "\nLocal nav data list full, please remove at least one before you "
        "add a new one.";
    info_str += ZCOLOR_NONE;
  }
  info_str += ZCOLOR_GREEN;
  info_str += GetZimaPrintString();
  info_str += "\nStandby mode keyboard command list: ";
  info_str += "\n-------------------------------------";
  {
    uint8_t num = 0;
    for (auto&& index : local_nav_data_index_list) {
      info_str += "\n" + std::to_string(num) + ": Select nav data " +
                  std::to_string(index) + "(" + Time::DebugString(index) + ")";
      if (selected_nav_data_index == index) {
        info_str += "(selected)";
      }
      num++;
    }
    while (num < max_local_nav_data_size) {
      // info_str += "\n" + std::to_string(num) + ": Null option.";
      num++;
    }
    if (!local_nav_data_index_list.empty()) {
      info_str += "\n" + std::to_string(num) + ": Un-select any nav data.";
    }
    num++;

    for (auto&& index : local_nav_data_index_list) {
      info_str += "\n" + std::to_string(num) + ": Remove nav data " +
                  std::to_string(index) + "(" + Time::DebugString(index) + ")";
      if (selected_nav_data_index == index) {
        info_str += "(selected)";
      }
      num++;
    }
    while (num < 2 * max_local_nav_data_size + 1) {
      // info_str += "\n" + std::to_string(num) + ": Null option.";
      num++;
    }

    for (auto&& index : local_nav_data_index_list) {
      info_str += "\n" + std::to_string(num) + ": Print info in nav data " +
                  std::to_string(index) + "(" + Time::DebugString(index) + ")";
      if (selected_nav_data_index == index) {
        info_str += "(selected)";
      }
      num++;
    }
    while (num < 3 * max_local_nav_data_size + 1) {
      // info_str += "\n" + std::to_string(num) + ": Null option.";
      num++;
    }
  }
  info_str += "\na: Start auto cleaning.";
  info_str += "\ns: Start auto scanning house.";
  // info_str += "\ne: Virtual wall / block area operation from file (Please see ";
  // info_str += VirtualWallBlockAreaEvent::kFileDir_ + ").";
  info_str += "\nq: Exit program.";
  info_str += "\n-------------------------------------";
  info_str += ZCOLOR_NONE;
  ZINFO << info_str;
}

}  // namespace zima
