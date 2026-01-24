/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include <thread>

#include "gazebo/zima_gazebo_node.h"
#include "zima/algorithm/line_segment_extractor.h"
#include "zima/algorithm/point_cloud_matcher.h"
#include "zima/common/util.h"
#include "zima/hal/system/keyboard_listener.h"
#include "zima/robot_data/local_nav_data.h"
#include "zima_ros/util.h"

DEFINE_bool(use_simple_slam, true,
            "Indicator for using cartographer as slam.");

bool keyboard_event = false;
enum Function {
  kLineSegmentTest,
  kMatcherTest,
};
Function test_function = kMatcherTest;

namespace zima_ros {

using namespace zima;

void ZimaGazeboNode::Run(const ZimaThreadWrapper::ThreadParam &param) {
  ZINFO << "Thread \"" << param.thread_name_ << "\" start.";
  auto thread_manager = ZimaThreadManager::Instance();
  if (param.bind_cpu_id_ >= 0) {
    thread_manager->BindCPUCore(param.thread_name_, param.bind_cpu_id_);
  }

  auto local_nav_data_manager = LocalNavDataManager::Instance();

  auto selected_operation_data_index =
      local_nav_data_manager->GetSelectedNavDataIndex();

  if (test_function == kMatcherTest) {
    if (selected_operation_data_index == 0) {
      ZERROR << "Selected data invalid.";
      ros::shutdown();
      return;
    }

    ZINFO << "Selected nav_data " << selected_operation_data_index << ".";
    auto selected_nav_data =
        local_nav_data_manager->GetNavData(selected_operation_data_index);
    if (selected_nav_data == nullptr) {
      ZERROR << "Nav data invalid.";
      ros::shutdown();
      return;
    }
    ZINFO << "Load nav_data " << selected_operation_data_index << ".";
    operation_data_ = std::make_shared<OperationData>(
        *selected_nav_data->GetNavDataCopyData());
    operation_data_->GetRawSlamValueGridMap2DRef()->Print(
        __FILE__, __FUNCTION__, __LINE__);
  }

  SlamValueGridMap2D::SPtr slam_grid_map =
      operation_data_->GetRawSlamValueGridMap2DRef();

  auto char_map = std::make_shared<CharGridMap2D>(
      "test", slam_grid_map->GetRangeX(), slam_grid_map->GetRangeY(),
      slam_grid_map->GetResolution());

  // Wait for sim time.
  Time::SleepSec(2);
  while (!shutdown_request_) {
    thread_manager->UpdateThreadCycle(param.thread_name_);
    if (test_function == kLineSegmentTest && keyboard_event) {
      keyboard_event = false;
      auto point_cloud_in_chassis_frame =
          chassis_->GetLidar(chassis_->kLidar_)->GetPointCloudInChassisFrame();
      if (point_cloud_in_chassis_frame != nullptr) {
        ZINFO;
        PublishPointCloudInChassisFrame(point_cloud_in_chassis_frame);
        ZINFO;
        auto separate_point_clouds =
            PointCloudLineSegmentExtractor::SeparatePointCloudByPointDistance(
                *point_cloud_in_chassis_frame);
        ZINFO << "Saperate to " << separate_point_clouds.size()
              << " point clouds";
        LineSegments saperate_point_cloud_group;
        LineSegments line_segments;
        for (auto&& point_cloud : separate_point_clouds) {
          ReadLocker lock(point_cloud->GetLock());
          if (point_cloud->Size() <
              PointCloudLineSegmentExtractor::kPointCloudPointsCountLimit_) {
            ZWARN << "Point cloud size: " << point_cloud->Size();
            continue;
          }
          auto& point_cloud_points = point_cloud->GetPointsConstRef();
          saperate_point_cloud_group.emplace_back(LineSegment(
              point_cloud_points.front().X(), point_cloud_points.front().Y(),
              point_cloud_points.back().X(), point_cloud_points.back().Y()));
          // ZINFO << "Add point cloud group: "
          //       << saperate_point_cloud_group.back().DebugString(2);

          auto _line_segments =
              PointCloudLineSegmentExtractor::ExtractLineSegments(*point_cloud);
          line_segments.insert(line_segments.end(), _line_segments.begin(),
                               _line_segments.end());
        }
        ZINFO;
        PublishLineSegments(saperate_point_cloud_group,
                            "Saperate point clouds");

        PublishLineSegments(line_segments,
                            "Line segments");

        double degree;
        if (LineSegmentAlgorithm::MostValuedLineDegree(line_segments, degree)) {
          ZINFO << "Degree: " << FloatToString(degree, 1);
        }
      } else {
        ZWARN << "Point cloud not ready.";
      }
      Time::SleepMSec(200);
      continue;
    }

    static double now = Time::Now();
    // ZINFO << DoubleToString(now, 3);
    if (test_function == kMatcherTest && slam_grid_map != nullptr &&
        Time::Now() - now > 3) {
      // ZWARN << DoubleToString(now, 3);
      now = Time::Now();
      PointCloudMatcher::Config config;
      static PointCloudMatcher matcher(config);

      auto point_cloud_in_chassis_frame =
          chassis_->GetLidar(chassis_->kLidar_)->GetPointCloudInChassisFrame();
      // point_cloud_in_chassis_frame->Print(slam_grid_map->GetResolution());
      MapPoint init_pose(0, 0, 0);
      MapCell init_cell;
      slam_grid_map->WorldToMap(init_pose, init_cell);
      // auto point_cloud_in_map_frame = point_cloud_in_chassis_frame->TransformBA(
      //     "point cloud in world", MapPoint(0, 0, init_pose.Degree()));

      // matcher.DebugPointCloudInMap(
      //     slam_grid_map, init_cell,
      //     matcher.ToMapCells(point_cloud_in_map_frame, slam_grid_map));
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

      MapPoint start_pose;
      if (point_cloud_in_chassis_frame != nullptr) {
        ZINFO;
        auto separate_point_clouds =
            PointCloudLineSegmentExtractor::SeparatePointCloudByPointDistance(
                *point_cloud_in_chassis_frame);
        // ZINFO << "Saperate to " << separate_point_clouds.size()
        //       << " point clouds";
        LineSegments saperate_point_cloud_group;
        LineSegments line_segments;
        for (auto&& point_cloud : separate_point_clouds) {
          ReadLocker lock(point_cloud->GetLock());
          if (point_cloud->Size() <
              PointCloudLineSegmentExtractor::kPointCloudPointsCountLimit_) {
            // ZWARN << "Point cloud size: " << point_cloud->Size();
            continue;
          }
          auto& point_cloud_points = point_cloud->GetPointsConstRef();
          saperate_point_cloud_group.emplace_back(LineSegment(
              point_cloud_points.front().X(), point_cloud_points.front().Y(),
              point_cloud_points.back().X(), point_cloud_points.back().Y()));
          // ZINFO << "Add point cloud group: "
          //       << saperate_point_cloud_group.back().DebugString(2);

          auto _line_segments =
              PointCloudLineSegmentExtractor::ExtractLineSegments(*point_cloud);
          line_segments.insert(line_segments.end(), _line_segments.begin(),
                               _line_segments.end());
        }
        ZINFO << "Get " << line_segments.size() << " lines.";
        double degree;
        if (LineSegmentAlgorithm::MostValuedLineDegree(line_segments, degree)) {
          ZINFO << "Most valued degree: " << FloatToString(degree, 1);

          start_pose.SetDegree(-1 * degree);

          PointCloudMatcher::BABSearchParameter optimized_parameter(
              slam_grid_map, start_pose, bound_in_map_frame, 3, 1, 3, 0.6,
              true);
          PointCloudMatcher::MatchResult optimized_BAB_result;
          bool optimized_BAB_found =
              matcher.BABSearch(slam_grid_map, point_cloud_in_chassis_frame,
                                optimized_parameter, optimized_BAB_result);
          if (optimized_BAB_found) {
            match_pose = optimized_BAB_result.pose_;
          }
          ZERROR << "============================================";
          ZERROR << "Optimize cost "
                 << FloatToString(optimized_BAB_result.match_time_, 2) << "s("
                 << FloatToString(optimized_BAB_result.match_time_ /
                                      BAB_result.match_time_ * 100,
                                  2)
                 << "%), non-optimized cost "
                 << FloatToString(BAB_result.match_time_, 2) << "s(100%).";
        } else {
          ZWARN << "Failed to get most valued degree.";
        }
      }

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
      // ZINFO << DoubleToString(now, 3);
      Time::SleepSec(2);
    }
  }
  Time::SleepMSec(100);
  ros::shutdown();
  ZINFO << "Thread \"" << param.thread_name_ << "\" exiting.";
  thread_manager->MarkThreadExited(param.thread_name_, __FILE__, __FUNCTION__,
                                   __LINE__);
  ZINFO << "Thread \"" << param.thread_name_ << "\" exit.";
}

}  // namespace zima_ros

int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "zima_matcher_gazebo_test");
  if (argc < 2) {
    test_function = kMatcherTest;
    ZINFO << "Test for matcher.";
  } else {
    test_function = kLineSegmentTest;
    ZINFO << "Test for line segment.";
  }

  zima::JsonSPtr global_config(nullptr);
  if(!zima::GlobalJsonConfig::Instance()->GetGlobalConfig(global_config)) {
    ZERROR << "Missing config.";
    return -1;
  }

  zima::KeyboardListener::CallBackFunc cb = [&](const char& key) -> void {
    keyboard_event = true;
  };
  zima::KeyboardListener keyboard_listener(cb);
  keyboard_listener.Start();

  zima_ros::ZimaGazeboNode test(FLAGS_use_simple_slam);
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
      std::thread(&zima_ros::ZimaGazeboNode::Run, &test,
                  core_thread_param),
      core_thread_param);

  // Start running.
  while (ros::ok()) {
    zima::Time::SleepMSec(50);
  }
  test.Shutdown();

  // Exiting.
  keyboard_listener.Stop();

  const double wait_for_exit_timeout_s = 1;
  thread_manager->WaitForThreadExit(ros_thread_param.thread_name_,
                                    wait_for_exit_timeout_s);
  thread_manager->WaitForThreadExit(core_thread_param.thread_name_,
                                    wait_for_exit_timeout_s);
  ZINFO << "Exit.";
  if (thread_manager->RunningThreadCount() > 1) {
    thread_manager->DebugRunningThreadName();
  }

  return 0;
}
