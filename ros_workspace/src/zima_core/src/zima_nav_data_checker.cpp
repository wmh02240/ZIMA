/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include <list>

#include "zima/hal/system/process.h"
#include "zima/logger/logger.h"
#include "zima/robot_data/operation_data.h"

using namespace zima;

void CheckNavData(const NavData::SPtr& data) {
  ZINFO << "Relocation slam file: " << data->GetSlamMapFileName();
  data->GetRawSlamValueGridMap2DRef()->Print(__FILE__, __FUNCTION__, __LINE__);

  data->GetOptimizedSlamValueGridMap2DRef()->Print(__FILE__, __FUNCTION__,
                                                   __LINE__);

  ZINFO << data->DebugAvailableRoomsInfo();
  ZINFO << data->DebugUserSelectRoomsInfo();
  ZINFO << "Room map:";
  data->GetNavMapRef()->GetRoomLayer()->Print(__FILE__, __FUNCTION__, __LINE__);

  ZINFO << data->DebugUserBlocksInfo();
  ZINFO << "User block map:";
  data->GetNavMapRef()->GetUserBlockLayer()->Print(__FILE__, __FUNCTION__,
                                                   __LINE__);
  ZINFO << "User select area map:";
  data->GetNavMapRef()->GetUserSelectAreaLayer()->Print(__FILE__, __FUNCTION__,
                                                        __LINE__);
}

void CheckOperationData(const OperationData::SPtr& data) {
  NavData::SPtr _data = data;
  CheckNavData(_data);
  auto path_size = data->GetAllSteps().size();
  ZINFO << "Data contain " << path_size << " path points.";
  if (path_size > 0) {
    ZINFO << "Maybe it is a clean record.";
    ZINFO << data->GetNavMapRef()->DebugString();
    data->GetNavMapRef()->GetPrintLayer()->Print(__FILE__, __FUNCTION__,
                                                 __LINE__);
  }

  ZINFO << data->DebugOperationInfo();
}

void CheckNavDataMemoryUsage() {
  ZINFO << "Current memory usage: "
        << std::to_string(ZimaGetProcessMemoryUsageInKB());

  int i = 0;
  std::list<NavData> nav_data_list;
  while (++i < 5) {
    nav_data_list.emplace_back(NavData(true));
  }
  ZINFO << "Current memory usage: "
        << std::to_string(ZimaGetProcessMemoryUsageInKB());
}

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  // CheckNavDataMemoryUsage();
  // return 0;

  if (argc < 2) {
    ZERROR << "options needed. zima_nav_data_checker filename";
    return -1;
  }

  auto file_full_name = std::string(argv[1]);

  auto index = file_full_name.find_last_of('/');
  std::string file_dir{};
  std::string file_name{};
  if (index != file_full_name.npos) {
    file_dir = file_full_name.substr(0, index + 1);
    file_name = file_full_name.substr(index + 1);
  } else {
    file_name = file_full_name;
  }

  {
    NavDataLoader loader(file_dir, file_name);
    if (!loader.IsOpened()) {
      ZWARN << "Open for record file " << file_dir << file_name << " failed.";
      return -1;
    }
    auto data = std::make_shared<NavData>();
    if (loader.LoadData(data)) {
      CheckNavData(data);
      // return 0;
    }
  }

  {
    OperationDataLoader loader(file_dir, file_name);
    if (!loader.IsOpened()) {
      ZWARN << "Open for record file " << file_dir << file_name << " failed.";
      return -1;
    }
    auto data = std::make_shared<OperationData>();
    if (loader.LoadData(data)) {
      CheckOperationData(data);
      return 0;
    }
  }

  return 0;
}
