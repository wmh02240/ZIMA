/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */
#include "zima/path_planner/zigzag_path_planner.h"
#include "zima_ros/zima_node.h"

namespace zima_ros {

using namespace zima;

void ZimaNode::Run(const ZimaThreadWrapper::ThreadParam& param) {
  ZINFO << "Thread \"" << param.thread_name_ << "\" start.";
  auto thread_manager = ZimaThreadManager::Instance();
  if (param.bind_cpu_id_ >= 0) {
    thread_manager->BindCPUCore(param.thread_name_, param.bind_cpu_id_);
  }

  auto nav_map = NavMap::SPtr(new NavMap());
  ZINFO << nav_map->DebugString();
  auto section_nav_map = NavMap::SPtr(new NavMap());

  ReadLocker read_lock(nav_map->GetFootStepLayer()->GetLock());
  // auto resolution = nav_map->GetResolution();
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

  PublishMap(nav_map->GetPrintLayer(), kPrintMap);
  Time::SleepSec(1);

  ZigZagPlannerManager planner_manager;
  planner_manager.Initialize();

  MapPoint current_pose(0, 0);
  ZigZagPlanner::ZigZagPath path;
  MapCellPath cell_path;
  while (!shutdown_request_ &&
         planner_manager.GeneratePath(nav_map, section_nav_map, current_pose,
                                      bound, 0, path, cell_path, true)) {
    thread_manager->UpdateThreadCycle(param.thread_name_);
    ZINFO << MapCell::DebugString(cell_path);
    PublishMap(nav_map->GetPrintLayer(), kPrintMap);
    PublishPlanPath(cell_path, nav_map->GetResolution());
    Time::SleepSec(1);
    // Simulate robot has gone by this path.
    Steps _steps;
    for (auto &&_point : path.path) {
      _steps.emplace_back(StepPoint(_point));
    }
    nav_map->MarkForCleaningSteps(*TransformManager::Instance(), _steps, bound);

    nav_map->GetPrintLayer()->Print(__FILE__, __FUNCTION__, __LINE__);
    current_pose = path.path.back();
  }

  ros::shutdown();

  ZINFO << "Thread \"" << param.thread_name_ << "\" exiting.";
  thread_manager->MarkThreadExited(param.thread_name_, __FILE__, __FUNCTION__,
                                   __LINE__);
  ZINFO << "Thread \"" << param.thread_name_ << "\" exit.";
}

}  // namespace zima_ros

int main(int argc, char **argv) {
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "zima_path_planner_test");
  zima_ros::ZimaNode test(true);
  test.InitForChassis();
  test.InitForAutoCleaning();
  test.InitForRos();

  auto thread_manager = zima::ZimaThreadManager::Instance();
  zima::ZimaThreadWrapper::ThreadParam core_thread_param(
      "Core thread", zima::ZimaThreadManager::kCoreThreadIndex_, 100, 0.2, 1);
  thread_manager->RegisterThread(
      std::thread(&zima_ros::ZimaNode::Run, &test, core_thread_param),
      core_thread_param);

  // Start running.
  while (ros::ok()) {
    zima::Time::SleepMSec(50);
  }
  test.Shutdown();

  // Exiting.
  const double wait_for_exit_timeout_s = 1;
  thread_manager->WaitForThreadExit(core_thread_param.thread_name_,
                                    wait_for_exit_timeout_s);

  return 0;
}
