/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */
#include "zima/auto_cleaning/section_cleaning.h"
#include "zima/simulator/simulator.h"
#include "zima_ros/zima_node.h"

namespace zima_ros {

using namespace zima;

void ZimaNode::Run(const ZimaThreadWrapper::ThreadParam &param) {
  ZINFO << "Thread \"" << param.thread_name_ << "\" start.";
  auto thread_manager = ZimaThreadManager::Instance();
  if (param.bind_cpu_id_ >= 0) {
    thread_manager->BindCPUCore(param.thread_name_, param.bind_cpu_id_);
  }
  Simulator simulator;
  auto &tf = *TransformManager::Instance();

  auto map = NavMap::SPtr(new NavMap());
  auto nav_map = NavMap::SPtr(new NavMap(*map));
  // ZINFO << map->footstep_layer_->Name();
  // ZINFO << nav_map->footstep_layer_->Name();
  // return;

  auto range_x = nav_map->GetRangeX();
  auto range_y = nav_map->GetRangeY();
  DynamicMapCellBound tmp_bound(-range_x, range_x, -range_y, range_y);
  auto resolution = map->GetResolution();
  auto footstep_layer = map->GetFootStepLayer();
  // auto mark_footstep = [&](const MapPoint &a, const MapPoint &b) -> void {
  //   ReadLocker read_lock(footstep_layer->GetLock());
  //   Steps steps = StepPoint::GenerateStepsBetweenTwoSteps(
  //       footstep_layer, StepPoint(a), StepPoint(b));
  //   read_lock.Unlock();
  //   map->MarkForCleaningSteps(*TransformManager::Instance(), steps,
  //   tmp_bound);
  // };
  // mark_footstep(MapPoint(2, 2), MapPoint(2, -2));
  // mark_footstep(MapPoint(2, -2), MapPoint(-2, -2));
  // mark_footstep(MapPoint(-2, -2), MapPoint(-2, 2));
  // mark_footstep(MapPoint(-2, 2), MapPoint(2, 2));
  // mark_footstep(MapPoint(0, 0), MapPoint(0, 1));
  // mark_footstep(MapPoint(1 * resolution, 0), MapPoint(1 * resolution, 1));
  // mark_footstep(MapPoint(-1 * resolution, 0), MapPoint(-1 * resolution,
  // 1)); mark_footstep(MapPoint(0, 0), MapPoint(-2, 0));
  // mark_footstep(MapPoint(0, 1 * resolution), MapPoint(-2, 1 *
  // resolution)); mark_footstep(MapPoint(0, -1 * resolution), MapPoint(-2,
  // -1 * resolution)); mark_footstep(MapPoint(0, -1), MapPoint(2, -1));
  // mark_footstep(MapPoint(0, -1 + 1 * resolution), MapPoint(2, -1 + 1 *
  // resolution)); mark_footstep(MapPoint(0, -1 - 1 * resolution),
  // MapPoint(2, -1 - 1 * resolution));

  // Mark for obs
  auto mark_for_obs = [&](const MapPoint &a, const MapPoint &b) -> void {
    MapCell a_cell, b_cell;
    map->GetFootStepLayer()->WorldToMap(a, a_cell);
    map->GetFootStepLayer()->WorldToMap(b, b_cell);
    MapCells bumper_cells;
    bumper_cells =
        map->GetFootStepLayer()->GenerateCellsBetweenTwoCells(a_cell, b_cell);
    for (auto &&cell : bumper_cells) {
      map->DebugMarkForSensor(cell, map->kBumper_);
    }
  };

  // mark_for_obs(MapPoint(2, 2), MapPoint(2, -2));
  // mark_for_obs(MapPoint(2, -2), MapPoint(-2, -2));
  // mark_for_obs(MapPoint(-2, -2), MapPoint(-2, 2));
  // mark_for_obs(MapPoint(-2, 2), MapPoint(2, 2));
  mark_for_obs(MapPoint(1, 1 - 1 * resolution), MapPoint(-1, 0));
  mark_for_obs(MapPoint(-2, 0), MapPoint(-1, 0));
  mark_for_obs(MapPoint(1 * resolution, -1), MapPoint(2, -1));
  mark_for_obs(MapPoint(1 * resolution, -1), MapPoint(1 * resolution, -1.5));
  mark_for_obs(MapPoint(-1, 1), MapPoint(-1, 1));

  map->GetPrintLayer()->Print(__FILE__, __FUNCTION__, __LINE__);
  nav_map->GetPrintLayer()->Print(__FILE__, __FUNCTION__, __LINE__);

  PublishMap(map->GetPrintLayer(), kVirtualObsMap);
  // while (!shutdown_request_) {
  //   PublishMap(map->GetPrintLayer(), kVirtualObsMap);
  //   Time::SleepMSec(200);
  // }
  Time::SleepMSec(200);

  MapPoint current_pose(-1, -1);
  Transform new_chassis_tf(tf.kOdomFrame_, tf.kRobotFrame_, current_pose.X(),
                           current_pose.Y(), current_pose.Degree());
  tf.UpdateTransform(new_chassis_tf);

  MapCell bound_min;
  MapCell bound_max;
  ReadLocker read_lock(footstep_layer->GetLock());
  map->GetFootStepLayer()->WorldToMap(MapPoint(2, 2), bound_max);
  map->GetFootStepLayer()->WorldToMap(MapPoint(-2, -2), bound_min);
  read_lock.Unlock();
  DynamicMapCellBound bound(bound_min.X(), bound_max.X(), bound_min.Y(),
                            bound_max.Y());

  ZINFO;
  NavData::SPtr nav_data = std::make_shared<NavData>();
  SectionCleaning section_cleaning(bound, nav_map);
  ChassisController::SPtr chassis_controller(new ChassisController(chassis_));
  chassis_controller->RunThread();

  auto publish_time = Time::Now();
  while (!shutdown_request_ &&
         section_cleaning.GetState() != SectionCleaning::State::kFinished) {
    thread_manager->UpdateThreadCycle(param.thread_name_);
    if (Time::Now() - publish_time > 3) {
      PublishMap(nav_map->GetPrintLayer(), kPrintMap);
      Time::SleepMSec(100);
      PublishMap(map->GetPrintLayer(), kVirtualObsMap);
      Time::SleepMSec(100);
      publish_time = Time::Now();
    }

    const float duration = 0.02;
    Transform robot_tf("", "");
    tf.GetTransform(tf.kOdomFrame_, tf.kRobotFrame_, robot_tf);
    MapPoint world_pose(robot_tf.X(), robot_tf.Y(), robot_tf.Degree());
    const auto odom_pose = world_pose;
    PublishRobot(map->GetPrintLayer(), world_pose);
    PublishWallSensor(world_pose);
    section_cleaning.ChassisSupervise(chassis_, chassis_controller, nav_map,
                                      nav_data->GetCurrentRoomInfoRef());

    simulator.Run(chassis_, tf, duration, map);

    float speed_times;
    // if (count > 42) {
    //   speed_times = 1;
    // } else {
    //   speed_times = 150;
    // }
    speed_times = 20;
    chassis_controller->SetSpeedTimes(speed_times);

    Time::SleepSec(duration / speed_times);
  }

  chassis_controller->StopThread();

  PublishMap(nav_map->GetPrintLayer(), kPrintMap);
  Time::SleepSec(1);

  ros::shutdown();
  ZINFO << "Finish cleaning.";
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

  ros::init(argc, argv, "zima_section_cleaning_simulation_test");
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
