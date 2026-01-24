/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/common/point_cell.h"
#include "zima/common/json.h"
#include "zima/grid_map/nav_map_2d.h"
#include "zima/grid_map/map_2d.h"
#include "zima/hal/system/cmd_line.h"
#include "zima/hal/system/file.h"
#include "zima/logger/logger.h"

using namespace zima;

void LoadCharGridMap(const std::string& file_full_name) {
  auto index = file_full_name.find_last_of('/');
  std::string file_dir{};
  std::string file_name{};
  if (index != file_full_name.npos) {
    file_dir = file_full_name.substr(0, index + 1);
    file_name = file_full_name.substr(index + 1);
  } else {
    file_name = file_full_name;
  }

  CharGridMap2DLoader loader(file_dir, file_name);
  CharGridMap2D::SPtr test_map(
      new CharGridMap2D("test", 50, 50, 0.1));
  test_map->Print(__FILE__, __FUNCTION__, __LINE__);
  if (loader.LoadMap(test_map)) {
    test_map->Print(__FILE__, __FUNCTION__, __LINE__);
  } else {
    ZERROR;
  }
}

void LoadSlamValueGridMap(const std::string& file_full_name) {
  auto index = file_full_name.find_last_of('/');
  std::string file_dir{};
  std::string file_name{};
  if (index != file_full_name.npos) {
    file_dir = file_full_name.substr(0, index + 1);
    file_name = file_full_name.substr(index + 1);
  } else {
    file_name = file_full_name;
  }
  SlamValueGridMap2DLoader loader(file_dir, file_name);
  SlamValueGridMap2D::SPtr test_map(new SlamValueGridMap2D(
      "test", 50, 50, 0.1));
  test_map->Print(__FILE__, __FUNCTION__, __LINE__);
  if (loader.LoadMap(test_map)) {
    test_map->Print(__FILE__, __FUNCTION__, __LINE__);
  } else {
    ZERROR;
  }
}

void LoadNavMap(const std::string& file_full_name) {
  auto index = file_full_name.find_last_of('/');
  std::string file_dir{};
  std::string file_name{};
  if (index != file_full_name.npos) {
    file_dir = file_full_name.substr(0, index + 1);
    file_name = file_full_name.substr(index + 1);
  } else {
    file_name = file_full_name;
  }
  NavMapLoader loader(file_dir, file_name);
  NavMap::SPtr test_map(new NavMap());
  if (loader.LoadMap(test_map)) {
    ZINFO << test_map->DebugString();
    {
      auto layer = test_map->GetFootStepLayer();
      ReadLocker lock(layer->GetLock());
      layer->Print(__FILE__, __FUNCTION__, __LINE__);
    }
    {
      auto layer = test_map->GetSensorLayer();
      ReadLocker lock(layer->GetLock());
      layer->Print(__FILE__, __FUNCTION__, __LINE__);
    }
    {
      auto layer = test_map->GetPrintLayer();
      ReadLocker lock(layer->GetLock());
      layer->Print(__FILE__, __FUNCTION__, __LINE__);
    }
  } else {
    ZERROR;
  }
}

void TestDir(const std::string& dir_name) {
  std::string cmd = "ls -lh " + dir_name;
  ZimaRunCommand(cmd);

  std::vector<std::string> file_names;
  if (FileSystemHelper::GetFilesNamesInDirectory(dir_name, file_names)) {
    ZINFO << "Dir " << dir_name << " contains files: ";
    std::string debug_str;
    for (auto&& name : file_names) {
      debug_str += "\n" + name;
    }
    if (file_names.empty()) {
      ZWARN << "\nNone.";
    } else {
      ZINFO << debug_str;
    }
  }

  if (FileSystemHelper::GetDirectoryNamesInDirectory(dir_name, file_names)) {
    ZINFO << "Dir " << dir_name << " contains directories: ";
    std::string debug_str;
    for (auto&& name : file_names) {
      debug_str += "\n" + name;
    }
    if (file_names.empty()) {
      ZWARN << "\nNone.";
    } else {
      ZINFO << debug_str;
    }
  }
}

void TestJson(const std::string& file_full_name) {
  auto index = file_full_name.find_last_of('/');
  std::string file_dir{};
  std::string file_name{};
  if (index != file_full_name.npos) {
    file_dir = file_full_name.substr(0, index + 1);
    file_name = file_full_name.substr(index + 1);
  } else {
    file_name = file_full_name;
  }

  std::string json_str = "{\"name\":\"test\",\"age\":18,\"score\":99.99}";
  {
    LocalJsonFileWritter writter(file_dir, file_name + ".json_test");
    auto json = Json::parse(json_str);
    if (!writter.SetJsonToFile(json)) {
      ZERROR;
    }
  }
  {
    LocalJsonFileLoader loader(file_dir, file_name + ".json_test");
    Json json;
    if (!loader.GetJsonFromFile(json)) {
      ZERROR;
    }
    auto _json_str = json.dump(2);
    if (json_str != _json_str) {
      ZINFO << "json_str: " << json_str;
      ZINFO << "_json_str: " << _json_str;
    }
  }
  {
    std::string wrong_str = "{sdfsdf";
    Json json;
    JsonHelper::TryParse(wrong_str, json);
    ZINFO << json.dump(2);
  }
}

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  if (argc < 3) {
    ZERROR << "options needed. zima_map_checker type filename";
    ZERROR << "Type 1 for char grid map."
           << "\nType 2 for slam value grid map."
           << "\nType 3 for nav map."
           << "\nType 4 for test dir."
           << "\nType 5 for test json file.";
    return -1;
  }

  const char kCharGridMapType = '1';
  const char kSlamValueGridMapType = '2';
  const char kNavMapType = '3';
  const char kTestDir = '4';
  const char kTestJson = '5';

  switch (*argv[1]) {
    case kCharGridMapType: {
      LoadCharGridMap(std::string(argv[2]));
      break;
    }
    case kSlamValueGridMapType: {
      LoadSlamValueGridMap(std::string(argv[2]));
      break;
    }
    case kNavMapType: {
      LoadNavMap(std::string(argv[2]));
      break;
    }
    case kTestDir: {
      TestDir(std::string(argv[2]));
      break;
    }
    case kTestJson: {
      TestJson(std::string(argv[2]));
      break;
    }
    default: {
      ZERROR << "Invalid option: " << *argv[1];
      break;
    }
  }

  return 0;
}
