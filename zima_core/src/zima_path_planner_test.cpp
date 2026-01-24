/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/logger/logger.h"
#include "zima/path_planner/zigzag_path_planner.h"

using namespace zima;

void ZigZagTest() {
  auto nav_map = NavMap::SPtr(new NavMap());
  auto section_nav_map = NavMap::SPtr(new NavMap());
  ZINFO << nav_map->DebugString();

  ReadLocker read_lock(nav_map->GetFootStepLayer()->GetLock());
  Steps steps = StepPoint::GenerateStepsBetweenTwoSteps(
      nav_map->GetFootStepLayer(), StepPoint(MapPoint(2, 2, 0)),
      StepPoint(MapPoint(2, -2, 0)));
  Steps steps2 = StepPoint::GenerateStepsBetweenTwoSteps(
      nav_map->GetFootStepLayer(), StepPoint(MapPoint(2, -2, 0)),
      StepPoint(MapPoint(-2, -2, 0)));
  Steps steps3 = StepPoint::GenerateStepsBetweenTwoSteps(
      nav_map->GetFootStepLayer(), StepPoint(MapPoint(-2, -2, 0)),
      StepPoint(MapPoint(-2, 2, 0)));
  Steps steps4 = StepPoint::GenerateStepsBetweenTwoSteps(
      nav_map->GetFootStepLayer(), StepPoint(MapPoint(-2, 2, 0)),
      StepPoint(MapPoint(2, 2, 0)));
  read_lock.Unlock();

  MapCell bound_min;
  MapCell bound_max;
  read_lock.Lock();
  nav_map->GetFootStepLayer()->WorldToMap(MapPoint(2, 2), bound_max);
  nav_map->GetFootStepLayer()->WorldToMap(MapPoint(-2, -2), bound_min);
  read_lock.Unlock();
  DynamicMapCellBound bound(bound_min.X(), bound_max.X(), bound_min.Y(),
                            bound_max.Y());

  nav_map->MarkForCleaningSteps(*TransformManager::Instance(), steps, bound);
  nav_map->MarkForCleaningSteps(*TransformManager::Instance(), steps2, bound);
  nav_map->MarkForCleaningSteps(*TransformManager::Instance(), steps3, bound);
  nav_map->MarkForCleaningSteps(*TransformManager::Instance(), steps4, bound);

  nav_map->GetPrintLayer()->Print(__FILE__, __FUNCTION__, __LINE__);

  ZigZagPlannerManager planner_manager;
  planner_manager.Initialize();

  MapPoint current_pose(0, 0);
  ZigZagPlanner::ZigZagPath path;
  MapCellPath cell_path;
  while (planner_manager.GeneratePath(
      nav_map, section_nav_map, current_pose, bound,
      nav_map->GetRoomLayer()->GetDefaultValue(), path, cell_path, true)) {
    ZINFO << MapCell::DebugString(cell_path);

    // Simulate robot has gone by this path.
    Steps _steps;
    for (auto &&point : path.path) {
      _steps.emplace_back(StepPoint(point));
    }
    nav_map->MarkForCleaningSteps(*TransformManager::Instance(), _steps,
                                    bound);
    section_nav_map->MarkForCleaningSteps(*TransformManager::Instance(),
                                            _steps, bound);

    nav_map->GetPrintLayer()->Print(__FILE__, __FUNCTION__, __LINE__);
    current_pose = path.path.back();
  }
}

int main(int argc, char **argv) {
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  ZigZagTest();

  return 0;
}
