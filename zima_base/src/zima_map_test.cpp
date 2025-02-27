/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include <list>

#include "zima/common/point_cell.h"
#include "zima/common/steps_recorder.h"
#include "zima/common/time.h"
#include "zima/common/util.h"
#include "zima/grid_map/map_2d.h"
#include "zima/grid_map/nav_map_2d.h"
#include "zima/hal/system/cmd_line.h"
#include "zima/hal/system/process.h"
#include "zima/logger/logger.h"

using namespace zima;

void CellTest() {
  MapCell cell(1, -1);
  ZINFO << cell;
  ZINFO << cell.DebugString();
  auto cell2 = cell;
  cell2 += MapCell(1, -2);
  ZINFO << cell2.DebugString();
  cell2 -= MapCell(3, -2);
  ZINFO << cell2.DebugString();
  auto cell3 = cell2 + cell;
  ZINFO << cell3.DebugString();

  MapCells cells;
  cells.emplace_back(MapCell(1, -3));
  cells.emplace_back(MapCell(-1, -1));
  ZINFO << MapCell::DebugString(cells);
  MapCellsSet cell_set;
  cell_set.emplace(MapCell(1, -3));
  cell_set.emplace(MapCell(-1, -1));
  ZINFO << MapCell::DebugString(cell_set);
}

void PointTest() {
  MapPoint point(0.5, -0.5);
  ZINFO << point;
  ZINFO << point.DebugString();
  auto point2 = point;
  ZINFO << point2.DebugString();
  point2 += MapPoint(1.50023, -1.51234);
  ZINFO << point2.DebugString();

  MapPoints points;
  points.emplace_back(MapPoint(1, -3));
  points.emplace_back(MapPoint(-1, -1));
  ZINFO << MapPoint::DebugString(points);
  MapPointsSet point_set;
  point_set.emplace(MapPoint(1, -3));
  point_set.emplace(MapPoint(-1, -1));
  ZINFO << MapPoint::DebugString(point_set);
}

void MapTest() {
  // {
  //   FloatValueGridMap2D map("test map", 5, 5, 0);
  //   map.PrintWithPrecision(__FILE__, __FUNCTION__, __LINE__, 3);
  //   map.Print(__FILE__, __FUNCTION__, __LINE__);
  //   WriteLocker lock(map.GetLock());
  //   map.SetValue(-100, 0, 1.2);
  //   map.SetValue(-101, 0, 1.3);
  //   map.SetValue(-102, 0, 1.4);
  //   map.SetValue(-100, -2, -1.2);
  //   map.SetValue(-101, -2, -1.3);
  //   map.SetValue(-102, -2, -1.4);
  //   lock.Unlock();
  //   map.PrintWithPrecision(__FILE__, __FUNCTION__, __LINE__, 3);
  //   map.PrintWithPrecision(__FILE__, __FUNCTION__, __LINE__, 4);
  //   map.PrintWithPrecision(__FILE__, __FUNCTION__, __LINE__, 5);
  // }
  CharGridMap2D map("testmap", 199, 199, 0.05);
  ZINFO;
  {
    WriteLocker lock(map.GetLock());
    map.SetValue(-3, -3, 'a');
    map.SetValue(1, 1, 'a');
    lock.Unlock();
    map.Print(__FILE__, __FUNCTION__, __LINE__);

    lock.Lock();
    map.SetValue(-30, -30, 'a');
    lock.Unlock();
    map.Print(__FILE__, __FUNCTION__, __LINE__);
  }
  return;

  auto map2 = map;
  ZINFO << "map: " << &(*map.GetLock());
  ZINFO << "map2: " << &(*map2.GetLock());
  CharGridMap2D map3("aa", 11, 11, 12);
  ZINFO;
  map3 = map;
  ZINFO;
  {
    WriteLocker lock(map.GetLock());
    CHECK(!map.GetLock()->InRead());
    CHECK(map.GetLock()->InWrite());
    map.SetValue(-300, -300, 'a');
    map.SetValue(299, 299, 'a');
    lock.Unlock();
    map.Print(__FILE__, __FUNCTION__, __LINE__);
  }

  ZINFO;
  {
    ReadLocker lock(map2.GetLock());
    map2.Print(__FILE__, __FUNCTION__, __LINE__);
  }

  ZINFO;
  {
    WriteLocker lock(map.GetLock());
    map.SetValue(0, 0, 'a');
    lock.Unlock();
    map.Print(__FILE__, __FUNCTION__, __LINE__);
  }

  ZINFO;
  {
    WriteLocker lock(map.GetLock());
    map.SetValue(10, 5, 'a');
    lock.Unlock();
    map.Print(__FILE__, __FUNCTION__, __LINE__);
  }

  ZINFO;
  {
    WriteLocker lock(map.GetLock());
    CHECK(!map.GetLock()->InRead());
    CHECK(map.GetLock()->InWrite());
    map.SetValue(-10, -5, 'a');
    lock.Unlock();
    map.Print(__FILE__, __FUNCTION__, __LINE__);
  }

  ZINFO;
  {
    WriteLocker lock(map.GetLock());
    map.SetValue(110, 5, 'a');
    map.SetValue(-150, -3, 'a');
    lock.Unlock();
    map.Print(__FILE__, __FUNCTION__, __LINE__);
  }

  ZINFO;
  {
    ReadLocker lock(map.GetLock());
    MapPoint point(3, -3);
    int x, y;
    map.WorldToMap(point.X(), point.Y(), x, y);
    ZINFO << x << ", " << y;
    MapPoint point2(5.846, -99.53);
    MapCell cell2;
    CHECK(!map.WorldToMap(point2, cell2));
    ZINFO << cell2.DebugString();
    MapPoint point3;
    map.MapToWorld(cell2, point3);
    ZINFO << point3.DebugString();
    CHECK(!map.WorldToMap(point3, cell2));
    ZINFO << cell2.DebugString();
  }
}

void MultiMapTest() {
  NavMap nav_map;
  ZINFO << nav_map.DebugString();

  Steps steps;
  steps.emplace_back(StepPoint(MapPoint(0, 0, 0)));
  steps.emplace_back(StepPoint(MapPoint(5, 0, 0)));
  auto range_x = nav_map.GetRangeX();
  auto range_y = nav_map.GetRangeY();
  DynamicMapCellBound bound(-range_x, range_x, -range_y, range_y);
  ZINFO;
  ReadLocker read_lock(nav_map.GetFootStepLayer()->GetLock());
  auto resolution = nav_map.GetResolution();
  Steps steps2 = StepPoint::GenerateStepsBetweenTwoSteps(
      nav_map.GetFootStepLayer(), StepPoint(MapPoint(0, 0, 0)),
      StepPoint(MapPoint(5, 0, 0),
                {MarkerPoint(MapPoint(3 * resolution, -1 * resolution),
                             nav_map.kBumper_)}));
  ZINFO;
  read_lock.Unlock();

  nav_map.MarkForCleaningSteps(*TransformManager::Instance(), steps2, bound);
  ZINFO;
  nav_map.GetPrintLayer()->Print(__FILE__, __FUNCTION__, __LINE__);

  read_lock.Lock();
  Steps steps3 = StepPoint::GenerateStepsBetweenTwoSteps(
      nav_map.GetFootStepLayer(), StepPoint(MapPoint(5, 0, 0)),
      StepPoint(MapPoint(0, 3, 150),
                {MarkerPoint(MapPoint(3 * resolution, -1 * resolution),
                             nav_map.kBumper_),
                 MarkerPoint(MapPoint(2 * resolution, 3 * resolution),
                             nav_map.kBumper_)}));
  ZINFO;
  read_lock.Unlock();

  nav_map.MarkForCleaningSteps(*TransformManager::Instance(), steps3, bound);
  ZINFO;

  nav_map.GetPrintLayer()->Print(__FILE__, __FUNCTION__, __LINE__);

  read_lock.Lock();
  Steps steps4 = StepPoint::GenerateStepsBetweenTwoSteps(
      nav_map.GetFootStepLayer(), StepPoint(MapPoint(0, 3, 150)),
      StepPoint(MapPoint(-1.2, 3, 150)));
  ZINFO;
  read_lock.Unlock();

  nav_map.MarkForCleaningSteps(*TransformManager::Instance(), steps4, bound);
  ZINFO;

  nav_map.GetPrintLayer()->Print(__FILE__, __FUNCTION__, __LINE__);
}

void MultiMapPtrTest() {
  auto map = NavMap::SPtr(new NavMap());
  auto nav_map = NavMap::SPtr(new NavMap(*map));

  auto range_x = nav_map->GetRangeX();
  auto range_y = nav_map->GetRangeY();
  DynamicMapCellBound bound(-range_x, range_x, -range_y, range_y);

  auto resolution = map->GetResolution();
  auto footstep_layer = map->GetFootStepLayer();
  auto mark_footstep = [&](const MapPoint &a, const MapPoint &b) -> void {
    ReadLocker read_lock(footstep_layer->GetLock());
    Steps steps = StepPoint::GenerateStepsBetweenTwoSteps(
        footstep_layer, StepPoint(a), StepPoint(b));
    read_lock.Unlock();
    map->MarkForCleaningSteps(*TransformManager::Instance(), steps, bound);
  };
  mark_footstep(MapPoint(2, 2), MapPoint(2, -2));
  mark_footstep(MapPoint(2, -2), MapPoint(-2, -2));
  mark_footstep(MapPoint(-2, -2), MapPoint(-2, 2));
  mark_footstep(MapPoint(-2, 2), MapPoint(2, 2));
  mark_footstep(MapPoint(0, 0), MapPoint(0, 1));
  mark_footstep(MapPoint(1 * resolution, 0), MapPoint(1 * resolution, 1));
  mark_footstep(MapPoint(-1 * resolution, 0), MapPoint(-1 * resolution, 1));
  mark_footstep(MapPoint(0, 0), MapPoint(-2, 0));
  mark_footstep(MapPoint(0, 1 * resolution), MapPoint(-2, 1 * resolution));
  mark_footstep(MapPoint(0, -1 * resolution), MapPoint(-2, -1 * resolution));
  mark_footstep(MapPoint(0, -1), MapPoint(2, -1));
  mark_footstep(MapPoint(0, -1 + 1 * resolution),
                 MapPoint(2, -1 + 1 * resolution));
  mark_footstep(MapPoint(0, -1 - 1 * resolution),
                 MapPoint(2, -1 - 1 * resolution));

  map->GetFootStepLayer()->Print(__FILE__, __FUNCTION__, __LINE__);
  nav_map->GetFootStepLayer()->Print(__FILE__, __FUNCTION__, __LINE__);
}

void LoadMapTest() {
  {
    auto file_dir = "../zima/proto/";
    auto file_name = "test_map.pb.txt";
    CharGridMap2DLoader loader(file_dir, file_name);
    CharGridMap2D::SPtr test_map(
        new CharGridMap2D("test", 50, 50, 0.1));
    test_map->Print(__FILE__, __FUNCTION__, __LINE__);
    if (loader.LoadMap(test_map)) {
      test_map->Print(__FILE__, __FUNCTION__, __LINE__);
    } else {
      ZERROR;
    }
    std::string file_content;
    LocalTextFileReader reader(file_dir, file_name);
    if (reader.ReadWholeFile(file_content)) {
      ZINFO << "\n" << file_content;
    } else {
      ZERROR;
    }
  }

  {
    auto file_dir = "../zima/proto/";
    auto file_name = "test_room.pb.txt";
    CharGridMap2DLoader loader(file_dir, file_name);
    CharGridMap2D::SPtr test_map(
        new CharGridMap2D("test", 50, 50, 0.1));
    test_map->Print(__FILE__, __FUNCTION__, __LINE__);
    if (loader.LoadMap(test_map)) {
      test_map->Print(__FILE__, __FUNCTION__, __LINE__);
    } else {
      ZERROR;
    }
    std::string file_content;
    LocalTextFileReader reader(file_dir, file_name);
    if (reader.ReadWholeFile(file_content)) {
      ZINFO << "\n" << file_content;
    } else {
      ZERROR;
    }
  }

  {
    auto file_dir = "../zima/proto/";
    auto file_name = "test_slam_value_map.pb.txt";
    SlamValueGridMap2DLoader loader(file_dir, file_name);
    SlamValueGridMap2D::SPtr test_map(new SlamValueGridMap2D(
        "test", 50, 50, 0.1));
    test_map->Print(__FILE__, __FUNCTION__, __LINE__);
    if (loader.LoadMap(test_map)) {
      test_map->Print(__FILE__, __FUNCTION__, __LINE__);
    } else {
      ZERROR;
    }
    // std::string file_content;
    // LocalTextFileReader reader(file_name);
    // if (reader.ReadWholeFile(file_content)) {
    //   ZINFO << "\n" << file_content;
    // } else {
    //   ZERROR;
    // }
  }

  {
    auto file_dir = "../zima/proto/";
    auto file_name = "test_nav_map.pb.txt";
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
    // std::string file_content;
    // LocalTextFileReader reader(file_name);
    // if (reader.ReadWholeFile(file_content)) {
    //   ZINFO << "\n" << file_content;
    // } else {
    //   ZERROR;
    // }
  }
}

void MultiResolutionSlamValueMapTest() {
  SlamValueGridMap2D::SPtr map(new SlamValueGridMap2D(
      "Slam map", 5, 10, 0.05));
  map->Print(__FILE__, __FUNCTION__, __LINE__);

  WriteLocker lock(map->GetLock());
  map->SetValue(0, 0, -1);
  map->SetValue(1, 0, 3);
  map->SetValue(2, 0, 30);
  map->SetValue(3, 0, 79);
  map->SetValue(4, 0, 55);
  map->SetValue(0, 1, 77);
  map->SetValue(1, 1, 3);
  map->SetValue(2, 1, 2);
  map->SetValue(3, 1, 30);
  map->SetValue(4, 1, 5);
  // map->SetValue(0, 3, -1);
  // map->SetValue(1, 3, -1);
  // map->SetValue(2, 3, -1);
  // map->SetValue(3, 3, -1);
  // map->SetValue(4, 3, -1);
  // map->SetValue(0, 2, -1);
  // map->SetValue(1, 2, -1);
  // map->SetValue(2, 2, -1);
  // map->SetValue(3, 2, -1);
  // map->SetValue(4, 2, -1);
  map->Print(__FILE__, __FUNCTION__, __LINE__);
  lock.Unlock();

  MultiResolutionSlamValueGridMap2D multi_resolution_map(
      "Multi resolution slam map", map, 5);
}

void WriteMapTest() {
  {
    auto file_dir = "../zima/proto/";
    auto file_name = "test_slam_value_map.pb.txt";
    SlamValueGridMap2DWriter writer(file_dir, file_name);
    SlamValueGridMap2D::SPtr map(new SlamValueGridMap2D(
        "test", 50, 50, 0.1));
    WriteLocker lock(map->GetLock());
    map->SetValue(0, 0, -1);
    map->SetValue(1, 0, 3);
    map->SetValue(2, 0, 30);
    map->SetValue(3, 0, 79);
    map->SetValue(4, 0, 55);
    map->SetValue(0, 1, 77);
    map->SetValue(1, 1, 3);
    map->SetValue(2, 1, 2);
    map->SetValue(3, 1, 30);
    map->SetValue(4, 1, 5);
    lock.Unlock();

    map->Print(__FILE__, __FUNCTION__, __LINE__);
    if (writer.WriteMap(map, false)) {
      ZINFO << "Success.";
    } else {
      ZERROR;
    }
  }

  {
    auto file_dir = "../zima/proto/";
    auto file_name = "test_nav_map.pb.txt";
    NavMapWriter writer(file_dir, file_name);
    NavMap::SPtr map(new NavMap());
    {
      WriteLocker lock(map->GetLock());
      auto footstep_layer = map->GetFootStepLayer();
      {
        WriteLocker lock2(footstep_layer->GetLock());
        footstep_layer->SetValue(0, 0, NavMap::kFootStep_);
        footstep_layer->SetValue(1, 0, NavMap::kFootStep_);
        footstep_layer->SetValue(2, 0, NavMap::kFootStep_);
        footstep_layer->SetValue(3, 0, NavMap::kFootStep_);
        footstep_layer->SetValue(4, 0, NavMap::kFootStep_);
        footstep_layer->SetValue(0, 1, NavMap::kFootStep_);
        footstep_layer->SetValue(1, 1, NavMap::kFootStep_);
        footstep_layer->SetValue(2, 1, NavMap::kFootStep_);
        footstep_layer->SetValue(3, 1, NavMap::kFootStep_);
        footstep_layer->SetValue(4, 1, NavMap::kFootStep_);
      }
    }

    ZINFO << map->DebugString();
    if (writer.WriteMap(map, false)) {
      ZINFO << "Success.";
    } else {
      ZERROR;
    }
  }
}

void MapMemoryTest() {
  ZINFO << "Current memory usage: "
        << std::to_string(ZimaGetProcessMemoryUsageInKB());
  Time::SleepSec(3);
  ZINFO << "Current memory usage: "
        << std::to_string(ZimaGetProcessMemoryUsageInKB());

  int count = 0;
  while (++count < 5) {
    int i = 0;
    std::list<CharGridMap2D> grid_map_list;
    while (++i < 500) {
      grid_map_list.emplace_back(CharGridMap2D("test map", 500, 500, 0.05));
      WriteLocker lock(grid_map_list.back().GetLock());
      grid_map_list.back().SetValue(0, 0, 'a');
      grid_map_list.back().SetValue(400, 0, 'a');
    }
    Time::SleepSec(5);
    ZINFO << "Current memory usage: "
          << std::to_string(ZimaGetProcessMemoryUsageInKB());
    grid_map_list.clear();
  }
  // {
  //   int i = 0;
  //   std::list<uint64_t> uint64_list;
  //   while (++i < 50000) {
  //     uint64_list.emplace_back(0);
  //   }
  //   Time::SleepSec(5);
  //   ZINFO << "Current memory usage: "
  //         << std::to_string(ZimaGetProcessMemoryUsageInKB());
  //   uint64_list.clear();
  // }

  Time::SleepSec(5);
  ZINFO << "Current memory usage: "
        << std::to_string(ZimaGetProcessMemoryUsageInKB());
}

void StepRecorderTest() {
  ZINFO << "Current memory usage: "
        << std::to_string(ZimaGetProcessMemoryUsageInKB());
  StepsRecorder sr;
  for (auto i = 0; i < 10000; i++) {
    sr.AddPathPoint(StepPoint(MapPoint()), true, true);
  }

  Time::SleepSec(3);
  ZINFO << "Current memory usage: "
        << std::to_string(ZimaGetProcessMemoryUsageInKB());

  for (auto i = 0; i < 10000; i++) {
    sr.AddPathPoint(StepPoint(MapPoint()), true, true);
  }

  Time::SleepSec(3);
  ZINFO << "Current memory usage: "
        << std::to_string(ZimaGetProcessMemoryUsageInKB());

  Steps cache_steps;
  for (auto i = 0; i < 10000; i++) {
    cache_steps.emplace_back(StepPoint(MapPoint()));
  }

  Time::SleepSec(3);
  ZINFO << "Current memory usage: "
        << std::to_string(ZimaGetProcessMemoryUsageInKB());

  for (auto i = 0; i < 10000; i++) {
    cache_steps.emplace_back(StepPoint(MapPoint()));
  }

  Time::SleepSec(3);
  ZINFO << "Current memory usage: "
        << std::to_string(ZimaGetProcessMemoryUsageInKB());

  std::list<MapPoint> test_map_point;

  for (auto i = 0; i < 10000; i++) {
    test_map_point.emplace_back(MapPoint());
  }

  Time::SleepSec(3);
  ZINFO << "Current memory usage: "
        << std::to_string(ZimaGetProcessMemoryUsageInKB());

  for (auto i = 0; i < 10000; i++) {
    test_map_point.emplace_back(MapPoint());
  }

  Time::SleepSec(3);
  ZINFO << "Current memory usage: "
        << std::to_string(ZimaGetProcessMemoryUsageInKB());

  std::list<std::deque<MapPoint>> test_map_point_list_list;
  std::deque<MapPoint> test_map_point_list;

  for (auto i = 0; i < 10000; i++) {
    test_map_point_list_list.emplace_back(test_map_point_list);
  }

  Time::SleepSec(3);
  ZINFO << "Current memory usage: "
        << std::to_string(ZimaGetProcessMemoryUsageInKB());

  for (auto i = 0; i < 10000; i++) {
    test_map_point_list_list.emplace_back(test_map_point_list);
  }

  Time::SleepSec(3);
  ZINFO << "Current memory usage: "
        << std::to_string(ZimaGetProcessMemoryUsageInKB());
}

int main(int argc, char **argv) {
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  // CellTest();
  // PointTest();
  // MapTest();
  // MultiMapTest();
  // MultiMapPtrTest();
  // LoadMapTest();
  // MultiResolutionSlamValueMapTest();
  // WriteMapTest();
  int count = 0;
  while (++count < 5) {
    MapMemoryTest();
  }
  // StepRecorderTest();

  return 0;
}
