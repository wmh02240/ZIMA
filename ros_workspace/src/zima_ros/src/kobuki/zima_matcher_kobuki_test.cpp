/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include <std_msgs/ColorRGBA.h>

#include <thread>

#include "kobuki/zima_kobuki_node.h"
#include "zima/algorithm/point_cloud_matcher.h"
#include "zima/algorithm/slam/simple_slam.h"
#include "zima/common/util.h"
#include "zima/robot_data/local_nav_data.h"
#include "zima_ros/util.h"

DEFINE_bool(use_simple_slam, true,
            "Indicator for using cartographer as slam.");

namespace zima_ros {

using namespace zima;

void ZimaKobukiNode::MainThread(const ZimaThreadWrapper::ThreadParam &param) {
  ZINFO << "Thread \"" << param.thread_name_ << "\" start.";
  auto thread_manager = ZimaThreadManager::Instance();
  if (param.bind_cpu_id_ >= 0) {
    thread_manager->BindCPUCore(param.thread_name_, param.bind_cpu_id_);
  }

  auto local_nav_data_manager = LocalNavDataManager::Instance();

  auto selected_operation_data_index =
      local_nav_data_manager->GetSelectedNavDataIndex();

  if (selected_operation_data_index == 0) {
    ZERROR << "Selected data invalid.";
    ros::shutdown();
    return;
  }
  ZINFO << "Selected nav_data " << selected_operation_data_index << ".";
  auto selected_nav_data = local_nav_data_manager->GetNavData(selected_operation_data_index);
  if (selected_nav_data == nullptr) {
    ZERROR << "Nav data invalid.";
    ros::shutdown();
    return;
  }
  ZINFO << "Load nav_data " << selected_operation_data_index << ".";
  operation_data_ =
      std::make_shared<OperationData>(*selected_nav_data->GetNavDataCopyData());
  operation_data_->GetRawSlamValueGridMap2DRef()->Print(__FILE__, __FUNCTION__,
                                                  __LINE__);

  SlamValueGridMap2D::SPtr slam_grid_map =
      operation_data_->GetRawSlamValueGridMap2DRef();

  auto char_map = std::make_shared<CharGridMap2D>(
      "test", slam_grid_map->GetRangeX(), slam_grid_map->GetRangeY(),
      slam_grid_map->GetResolution());

  std::shared_ptr<Timer> compensate_point_cloud_timer(
      new Timer("compensate point cloud timer", 1, true, true));
  while (!shutdown_request_) {
    thread_manager->UpdateThreadCycle(param.thread_name_);
    static double now = Time::Now();
    if (false) {
      // Test for lidar compensate.
      Time::SleepMSec(20);

      // kobuki_chassis_->GetSerialRef()->SetVelocityCmdByWheel(
      //     0.15, -0.15, kobuki_chassis_->GetTrackLength());

      if (compensate_point_cloud_timer->TimeUp()) {
        auto point_cloud_in_lidar_frame =
            chassis_->GetLidar(chassis_->kLidar_)->GetPointCloudInLidarFrame();
        {
          ReadLocker lock(point_cloud_in_lidar_frame->GetLock());
          ZINFO << "Point cloud size: " << point_cloud_in_lidar_frame->Size()
                << ", scan time: "
                << FloatToString(point_cloud_in_lidar_frame->GetScanTime(), 3);

          ZINFO << "Point 20 : "
                << point_cloud_in_lidar_frame->GetPointsConstRef()
                       .at(20)
                       .DebugString();
        }
        auto point_cloud_in_chassis_frame =
            point_cloud_in_lidar_frame->TransformBA(
                "point cloud in chassis frame",
                chassis_->GetLidar(chassis_->kLidar_)->GetTf());

        ReadLocker lock(receive_msg_list_lock_);
        auto odom_data_list = odom_data_list_;
        ZINFO << "Odom size: " << odom_data_list.size();
        lock.Unlock();
        ReadLocker pc_lock(point_cloud_in_lidar_frame->GetLock());
        auto compensate_point_cloud_in_lidar_frame =
            SimpleSlam::PointCloudInLidarFrameMotionCompensate(
                odom_data_list, chassis_->GetLidar(chassis_->kLidar_)->GetTf(),
                point_cloud_in_lidar_frame);
        ZINFO << "Point 20 : "
              << compensate_point_cloud_in_lidar_frame->GetPointsConstRef()
                     .at(20)
                     .DebugString();
        auto compensate_point_cloud_in_chassis_frame =
            compensate_point_cloud_in_lidar_frame->TransformBA(
                "compensate point cloud in chassis frame",
                chassis_->GetLidar(chassis_->kLidar_)->GetTf());

        PublishPointCloudInChassisFrame(
            compensate_point_cloud_in_chassis_frame);
        PublishPointCloudInWorldFrame(point_cloud_in_chassis_frame);
        ZINFO << "Check.";

        compensate_point_cloud_timer.reset(
            new Timer("compensate point cloud timer", 1, true, true));
      }
      PublishOdom();
      continue;
    }
    // ZINFO << DoubleToString(now, 3);
    if (slam_grid_map != nullptr && Time::Now() - now > 3) {
      // ZWARN << DoubleToString(now, 3);
      now = Time::Now();

      PointCloudMatcher::Config config;
      static PointCloudMatcher matcher(config);

      auto point_cloud_in_chassis_frame =
          chassis_->GetLidar(chassis_->kLidar_)->GetPointCloudInChassisFrame();
      {
        ReadLocker lock(point_cloud_in_chassis_frame->GetLock());
        if (point_cloud_in_chassis_frame->Empty()) {
          ZINFO << "Waiting for scan.";
          Time::SleepSec(2);
          continue;
        }
      }

      MapPoint init_pose(0, 0, 0);
      MapCell init_cell;
      slam_grid_map->WorldToMap(init_pose, init_cell);
      auto point_cloud_in_map_frame = point_cloud_in_chassis_frame->TransformBA(
          "point cloud in world", MapPoint(0, 0, init_pose.Degree()));

      point_cloud_in_chassis_frame->Print(NavMap::GetResolution());
      // matcher.DebugPointCloudInMap(
      //     map, init_cell, matcher.ToMapCells(point_cloud_in_map_frame,
      //     map));
      // ZWARN << "Match score: "
      //       << FloatToString(
      //              matcher.SimpleMatchScore(
      //                  map, init_cell,
      //                  matcher.ToMapCells(point_cloud_in_map_frame, map)),
      //              4);

      DynamicMapPointBound bound_in_map_frame(
          slam_grid_map->GetDataPointBound());
      PointCloudMatcher::BABSearchParameter parameter(
          slam_grid_map, init_pose, bound_in_map_frame, 180, 3, 3, 0.6);
      // PointCloudMatcher::BABSearchParameter parameter(
      //     map, init_pose, DynamicMapPointBound(-1, 1, -1, 1), 10, 1, 7,
      //     0.6);
      MapPoint match_pose(0, 0, 0);
      PointCloudMatcher::MatchResult BAB_result;
      PointCloudMatcher::MatchResult boost_BAB_result;
      ZINFO;
      bool BAB_found = false;
      BAB_found = matcher.BABSearch(slam_grid_map, point_cloud_in_chassis_frame,
                                    parameter, BAB_result);
      if (BAB_found) {
        match_pose = BAB_result.pose_;
      }
      ZERROR << "============================================";
      // bool boost_BAB_found = false;
      // boost_BAB_found =
      //     matcher.BoostBABSearch(slam_grid_map, point_cloud_in_chassis_frame,
      //                            parameter, boost_BAB_result);
      // if (boost_BAB_found) {
      //   match_pose = boost_BAB_result.pose_;
      // }
      // ZERROR << "============================================";
      ZERROR << "Boost cost " << FloatToString(boost_BAB_result.match_time_, 2)
             << "s("
             << FloatToString(
                    boost_BAB_result.match_time_ / BAB_result.match_time_ * 100,
                    2)
             << "%), non-boost cost "
             << FloatToString(BAB_result.match_time_, 2) << "s(100%).";
      // if (BAB_found && !boost_BAB_found) {
      //   ZERROR << "\n\n\n\n!!!!!!";
      //   Time::SleepSec(5);
      // }

      PublishSlamValueMap(ros::Time(), slam_grid_map);
      PublishRobot(char_map, match_pose);
      auto now = ros::Time::now();
      auto match_pose_ros_tf = MapPointToTFTransform(match_pose);
      auto& tf_manager = *TransformManager::Instance();
      PublishTf(zima_ros::GeometryTransformToStampedTransform(
          now, tf_manager.kWorldFrame_, tf_manager.kRobotFrame_,
          zima_ros::TFToGeometryMsgTransform(match_pose_ros_tf)));
      PublishPointCloudInChassisFrame(point_cloud_in_chassis_frame);
      Time::SleepSec(2);
    } else {
      // ZINFO << FloatToString(now, 3);
      Time::SleepSec(2);
    }
  }
  Time::SleepSec(1);
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

  ros::init(argc, argv, "zima_matcher_kobuki_test");

  ros::NodeHandle priv_nh("~");
  std::string port;
  priv_nh.param("port", port, std::string("/dev/ttyS1"));

  zima_ros::ZimaKobukiNode test(port, FLAGS_use_simple_slam);
    auto thread_manager = zima::ZimaThreadManager::Instance();
  zima::ZimaThreadWrapper::ThreadParam ros_thread_param(
      "Ros spin thread", zima::ZimaThreadManager::kMiscThreadIndex_, 100, 0.2,
      1);
  thread_manager->RegisterThread(
      std::thread(&zima_ros::ZimaNode::ROSSpinThread, &test, ros_thread_param),
      ros_thread_param);

  zima::ZimaThreadWrapper::ThreadParam core_thread_param(
      "Core thread", zima::ZimaThreadManager::kCoreThreadIndex_, 100,
      0.2, 1);
  thread_manager->RegisterThread(
      std::thread(&zima_ros::ZimaKobukiNode::MainThread, &test,
                  core_thread_param),
      core_thread_param);

  zima::ZimaThreadWrapper::ThreadParam data_process_thread_param(
      "Data process thread", zima::ZimaThreadManager::kMiscThreadIndex_, 100,
      0.2, 1);
  thread_manager->RegisterThread(
      std::thread(&zima_ros::ZimaNode::DataProcessThread, &test,
                  data_process_thread_param),
      data_process_thread_param);

  test.ChassisStartThread();

  // Start running.
  while (ros::ok()) {
    zima::Time::SleepMSec(50);
  }
  test.Shutdown();

  // Exiting.

  const double wait_for_exit_timeout_s = 1;
  thread_manager->WaitForThreadExit(ros_thread_param.thread_name_,
                                    wait_for_exit_timeout_s);
  thread_manager->WaitForThreadExit(core_thread_param.thread_name_,
                                    wait_for_exit_timeout_s);
  thread_manager->WaitForThreadExit(data_process_thread_param.thread_name_,
                                    wait_for_exit_timeout_s);

  test.ChassisShutdownThread();

  ZINFO << "Exit.";
  if (thread_manager->RunningThreadCount() > 1) {
    thread_manager->DebugRunningThreadName();
  }

  return 0;
}
