/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/robot_data/operation_data_manager.h"

#include "zima/algorithm/slam/simple_slam.h"
#include "zima/grid_map/map_util.h"
#include "zima/logger/logger.h"
#include "zima/robot_data/cleaning_record.h"
#include "zima/robot_data/local_nav_data.h"

namespace zima {

bool OperationDataManager::CreateOperationData(
    OperationData::SPtr& operation_data, const SlamBase::SPtr& slam_wrapper,
    const OperationData::OperationType& operation_type,
    const bool& use_simple_slam) {
  if (slam_wrapper == nullptr) {
    ZERROR << "Slam wrapper is null.";
    return false;
  }

  if (operation_data != nullptr) {
    ZWARN << "Operation data is not released, auto release it.";
    if (!ReleaseOperationData(operation_data, slam_wrapper, use_simple_slam)) {
      ZERROR << "Release failed.";
      return false;
    }
    ZINFO << "Operation data released.";
  }

  if (operation_type == OperationData::OperationType::kAllHouseScanning) {
    ZINFO << "House scan selected.";

    operation_data = std::make_shared<OperationData>(
        OperationData::OperationType::kAllHouseScanning);
  } else {
    auto local_nav_data_manager = LocalNavDataManager::Instance();

    auto selected_operation_data_index =
        local_nav_data_manager->GetSelectedNavDataIndex();
    if (selected_operation_data_index != 0) {
      ZINFO << "Selected nav_data " << selected_operation_data_index << ".";
      auto selected_nav_data = local_nav_data_manager->GetNavData(
          selected_operation_data_index);
      if (selected_nav_data == nullptr) {
        ZERROR << "Nav data invalid.";
        return false;
      }
      ZINFO << "Load nav_data " << selected_operation_data_index << ".";
      operation_data = std::make_shared<OperationData>(
          *selected_nav_data->GetNavDataCopyData());
      if (FLAGS_debug_enable) {
        operation_data->GetRawSlamValueGridMap2DRef()->Print(
            __FILE__, __FUNCTION__, __LINE__);
      }
      operation_data->GetOptimizedSlamValueGridMap2DRef()->Print(
          __FILE__, __FUNCTION__, __LINE__);
    } else {
      ZINFO << "No nav_data selected.";
      operation_data = std::make_shared<OperationData>();
    }
  }

  operation_data->RunThread();
  return true;
}

bool OperationDataManager::ReleaseOperationData(
    OperationData::SPtr& operation_data, const SlamBase::SPtr& slam_wrapper,
    const bool& use_simple_slam) {
  if (operation_data == nullptr) {
    ZINFO << "Operation data is already released.";
    return true;
  }
  operation_data->StopThread();

  if (slam_wrapper == nullptr) {
    ZERROR << "Slam wrapper is null.";
    return false;
  }

  auto slam_grid_map = slam_wrapper->GetSlamMap();
  operation_data->ProcessSlamValueMap(slam_grid_map);

  if (use_simple_slam) {
    auto slam = dynamic_pointer_cast<SimpleSlam>(slam_wrapper);
    auto probability_map = slam->GetProbabilityMap();
    if (probability_map != nullptr) {
      operation_data->UpdateProbabilityIndexGridMap2D(probability_map);
      if (FLAGS_debug_enable) {
        probability_map->Print(__FILE__, __FUNCTION__, __LINE__);
      }
      operation_data->GetOptimizedSlamValueGridMap2DRef()->Print(
          __FILE__, __FUNCTION__, __LINE__);
    }
  }

  auto local_nav_data_manager = LocalNavDataManager::Instance();
  auto clean_record_manager = CleaningRecordManager::Instance();
  auto index = operation_data->GetIndex();
  auto start_time = operation_data->GetStartTime();

  ZGINFO << "Index: " << std::to_string(index) << ", start time "
         << DoubleToString(start_time, 1);
  if (index == static_cast<uint32_t>(start_time)) {
    ZINFO << "New operation data.";
    auto init_room_map = std::make_shared<CharGridMap2D>(
        NavMap::kRoomMapName_, 1, 1,
        operation_data->GetOptimizedSlamValueGridMap2DRef()->GetResolution());
    MapConverter map_converter;
    map_converter.ConvertSlamValueGridMap2DToRoomMap(
        operation_data->GetOptimizedSlamValueGridMap2DRef(), init_room_map);
    operation_data->GetNavMapRef()->ChangeRoomLayer(init_room_map);
    RoomMapUtil room_map_util;
    RoomsInfo init_rooms_info;
    if (room_map_util.UpdateRoomsInfo(init_room_map, init_rooms_info)) {
      operation_data->SetRoomsInfo(init_rooms_info);
      if (operation_data->GetOperationType() !=
          OperationData::OperationType::kAllHouseScanning) {
        clean_record_manager->AddNewRecord(operation_data);
      }
      if (operation_data->GetProbabilityIndexGridMap2DRef() != nullptr &&
          operation_data->GetProbabilityIndexGridMap2DRef()->IsMarked()) {
        local_nav_data_manager->AddNewNavData(operation_data);
      } else {
        ZINFO << "Do not create nav data due to map missing.";
      }
    } else {
      ZERROR << "Update rooms info failed.";
      return false;
    }
  } else {
    ZINFO << "Existing operation data.";
    RoomMapUtil room_map_util;
    RoomsInfo new_rooms_info;
    if (room_map_util.UpdateRoomsInfo(
            operation_data->GetNavMapConstRef()->GetRoomLayer(),
            new_rooms_info)) {
      operation_data->SetRoomsInfo(new_rooms_info);
      clean_record_manager->AddNewRecord(operation_data);
      local_nav_data_manager->UpdateNavData(operation_data);
    } else {
      ZERROR << "Update rooms info failed.";
      return false;
    }
  }

  operation_data.reset(); 
  return true;
}

}  // namespace zima
