/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/event/user_event.h"
#include "zima/hal/system/file.h"

namespace zima {

const std::string GeneralOperationEvent::kLabel_ = "General operation";
const std::string StartCleaningEvent::kLabel_ = "Start cleaning";
const std::string PauseEvent::kLabel_ = "Pause";
const std::string ResumeCleaningEvent::kLabel_ = "Resume cleaning";
const std::string StopCleaningEvent::kLabel_ = "Stop cleaning";

const std::string VirtualWallBlockAreaEvent::kLabel_ =
    "Virtual wall / block area operation";
const std::string VirtualWallBlockAreaEvent::kFileDir_ =
    "/tmp/virtual_wall_block_area_event/";
const std::string VirtualWallBlockAreaEvent::kAddVirtualWallSuffix_ =
    ".add_virtual_wall";
const std::string VirtualWallBlockAreaEvent::kRemoveVirtualWallSuffix_ =
    ".remove_virtual_wall";
const std::string VirtualWallBlockAreaEvent::kUpdateVirtualWallSuffix_ =
    ".update_virtual_wall";
const std::string VirtualWallBlockAreaEvent::kAddBlockAreaSuffix_ =
    ".add_block_area";
const std::string VirtualWallBlockAreaEvent::kRemoveBlockAreaSuffix_ =
    ".remove_block_area";
const std::string VirtualWallBlockAreaEvent::kUpdateBlockAreaSuffix_ =
    ".update_block_area";

const std::string LocalNavDataEvent::kLabel_ = "Local nav data operation";
const std::string KeyboardEvent::kLabel_ = "Keyboard trigger";

VirtualWallBlockAreaEvent::Operation::List
VirtualWallBlockAreaEvent::DemoVirtualWallBlockAreaOperation() {
  Operation::List operation_list;
  std::vector<std::string> file_names;
  if (!FileSystemHelper::GetFilesNamesInDirectory(kFileDir_, file_names)) {
    ZERROR << "Failed.";
    return operation_list;
  }

  std::vector<std::string> add_virtual_wall_file_names;
  std::vector<std::string> remove_virtual_wall_file_names;
  std::vector<std::string> update_virtual_wall_file_names;
  std::vector<std::string> add_block_area_file_names;
  std::vector<std::string> remove_block_area_file_names;
  std::vector<std::string> update_block_area_file_names;
  for (auto&& file_name : file_names) {
    if (StringEndsWith(file_name, kAddVirtualWallSuffix_)) {
      add_virtual_wall_file_names.emplace_back(file_name);
    } else if (StringEndsWith(file_name, kRemoveVirtualWallSuffix_)) {
      remove_virtual_wall_file_names.emplace_back(file_name);
    } else if (StringEndsWith(file_name, kUpdateVirtualWallSuffix_)) {
      update_virtual_wall_file_names.emplace_back(file_name);
    } else if (StringEndsWith(file_name, kAddBlockAreaSuffix_)) {
      add_block_area_file_names.emplace_back(file_name);
    } else if (StringEndsWith(file_name, kRemoveBlockAreaSuffix_)) {
      remove_block_area_file_names.emplace_back(file_name);
    } else if (StringEndsWith(file_name, kUpdateBlockAreaSuffix_)) {
      update_block_area_file_names.emplace_back(file_name);
    }
  }

  auto get_index = [&](const std::string& name, const std::string& suffix,
                       uint8_t& index) -> bool {
    auto index_str = name;
    for (uint i = 0; i < suffix.size(); i++) {
      index_str.pop_back();
    }
    auto _index = static_cast<uint32_t>(atoi(index_str.c_str()));
    if (_index <= UINT8_MAX) {
      index = static_cast<uint8_t>(_index);
      return true;
    }
    ZWARN << "Index " << static_cast<int>(_index)
          << " too large, should be under " << static_cast<int>(UINT8_MAX);
    return false;
  };

  for (auto&& name : add_virtual_wall_file_names) {
    NavDataLoader loader(kFileDir_, name);
    NavData::VirtualWall::SPtr virtual_wall;
    if (loader.LoadData(virtual_wall)) {
      operation_list.emplace_back(Operation::DataType::kVirtualWall,
                                  Operation::OperationType::kAdd, virtual_wall,
                                  nullptr, 0);
    }
    FileSystemHelper::RemoveDirectoryOrFile(kFileDir_ + name);
  }
  for (auto&& name : remove_virtual_wall_file_names) {
    uint8_t index;
    if (get_index(name, kRemoveVirtualWallSuffix_, index)) {
      operation_list.emplace_back(Operation::DataType::kVirtualWall,
                                  Operation::OperationType::kRemove, nullptr,
                                  nullptr, index);
    }
    FileSystemHelper::RemoveDirectoryOrFile(kFileDir_ + name);
  }
  for (auto&& name : update_virtual_wall_file_names) {
    uint8_t index;
    if (get_index(name, kUpdateVirtualWallSuffix_, index)) {
      NavDataLoader loader(kFileDir_, name);
      NavData::VirtualWall::SPtr virtual_wall;
      if (loader.LoadData(virtual_wall)) {
        operation_list.emplace_back(Operation::DataType::kVirtualWall,
                                    Operation::OperationType::kUpdate,
                                    virtual_wall, nullptr, index);
      }
    }
    FileSystemHelper::RemoveDirectoryOrFile(kFileDir_ + name);
  }

  return operation_list;
}

bool VirtualWallBlockAreaEvent::CreateDemoOperationTemplateFile() {
  if (!FileSystemHelper::CreateDirectory(kFileDir_)) {
    ZWARN << "Create dir " << kFileDir_ << " failed.";
    return false;
  }

  auto virtual_wall =
      std::make_shared<NavData::VirtualWall>(MapPoint(2, 2), MapPoint(2, -2));

  NavDataWriter virtual_wall_writer(kFileDir_, "virtual_wall_template");
  if (!virtual_wall_writer.WriteData(virtual_wall, false)) {
    ZWARN << "Write virtual wall template failed.";
  }

  auto block_area = std::make_shared<NavData::BlockArea>(
      MapPoint(2, 2), MapPoint(3, 2), MapPoint(3, -2), MapPoint(2, -2));

  NavDataWriter block_area_writer(kFileDir_, "block_area_template");
  if (!block_area_writer.WriteData(block_area, false)) {
    ZWARN << "Write block area template failed.";
  }

  return true;
}

}  // namespace zima
