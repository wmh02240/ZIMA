/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

// #include <cartographer_ros_msgs/StartTrajectory.h>

#include <thread>

#include "xtark/zima_xtark_node.h"
#include "zima/auto_cleaning/auto_cleaning.h"
#include "zima/common/util.h"
#include "zima/grid_map/map_util.h"

DEFINE_bool(use_simple_slam, true,
            "Indicator for using cartographer as slam.");

namespace zima_ros {

using namespace zima;

const std::string kWriteMapDir = "/tmp/zima_map/";

void ZimaXTarkNode::MainThread() {
  auto &tf = *TransformManager::Instance();

  // ZINFO << nav_map->footstep_layer_->Name();
  // return;

  operation_data_->GetNavMapRef()->GetPrintLayer()->Print(
      __FILE__, __FUNCTION__, __LINE__);

  // while (!shutdown_request_) {
  //   PublishMap(map->GetPrintLayer(), false);
  //   Time::SleepMSec(200);
  // }
  Time::SleepMSec(200);

  // MapPoint current_pose(
  //     0, -operation_data_->GetNavMapRef()->GetResolution() * 0.9);
  MapPoint current_pose(0, 0);
  Transform new_chassis_tf(tf.kOdomFrame_, tf.kRobotFrame_, current_pose.X(),
                           current_pose.Y(), current_pose.Degree());
  tf.UpdateTransform(new_chassis_tf);

  // Start cartographer.
  // cartographer_ros_msgs::StartTrajectory request;
  // request.request.use_initial_pose = false;
  // while (!shutdown_request_) {
  //   if (!cartographer_start_trajectory_client_.call(request)) {
  //     ZERROR << "Call service failed.";
  //     Time::SleepSec(1);
  //     continue;
  //   }
  //   break;
  // }

  Steps all_steps;
  ZINFO;
  AutoCleaning auto_cleaning(operation_data_);

  ZINFO << "Wait for slam map";
  while (!shutdown_request_ && !slam_map_received_.load()) {
    Time::SleepSec(0.05);
    continue;
  }
  if (!shutdown_request_) {
    ZINFO << "Slam map received";
  }

  auto publish_time = Time::Now();
  auto start_time = Time::Now();

  operation_data_->GetStopWatchRef().Start();
  StepsRecorder steps_recorder;
  while (!shutdown_request_ &&
         auto_cleaning.GetState() != AutoCleaning::State::kFinished) {
    const float duration = 0.02;
    if (Time::Now() - start_time < 2) {
      Time::SleepSec(duration);
      continue;
    }

    if (Time::Now() - publish_time > 3) {
      PublishMap(operation_data_->GetNavMapRef()->GetPrintLayer(),
                 MapType::kPrintMap);
      Time::SleepMSec(50);
      publish_time = Time::Now();
      all_steps = steps_recorder.GetPath();
      PublishPath(operation_data_->GetNavMapRef()->GetPrintLayer(), all_steps);
      Time::SleepMSec(50);
      ZINFO << operation_data_->DebugOperationInfo();
      auto step_area = operation_data_->GetNavMapRef()->GetStepAreaSize();
      PublishCleaningInfo(operation_data_->GetOptimizedSlamValueGridMap2DRef(),
                          step_area, operation_data_->GetDuration() / 60);
      PublishUnderLayerMap(
          operation_data_->GetOptimizedSlamValueGridMap2DRef());
    }

    Transform robot_tf("", "");
    tf.GetTransform(tf.kWorldFrame_, tf.kRobotFrame_, robot_tf);
    MapPoint world_pose(robot_tf.X(), robot_tf.Y(), robot_tf.Degree());
    tf.GetTransform(tf.kOdomFrame_, tf.kRobotFrame_, robot_tf);
    MapPoint odom_pose(robot_tf.X(), robot_tf.Y(), robot_tf.Degree());

    // ZINFO << "Pose " << world_pose.DebugString();

    steps_recorder.AddPathPoint(StepPoint(world_pose));
    PublishRobot(operation_data_->GetNavMapRef()->GetPrintLayer(), world_pose);
    PublishWallSensor(world_pose);
    auto_cleaning.Run(chassis_, world_pose, odom_pose);

    auto left_speed = chassis_->GetWheel(chassis_->kLeftWheel_)->TargetSpeed();
    auto right_speed =
        chassis_->GetWheel(chassis_->kRightWheel_)->TargetSpeed();

    // ZINFO << "Set Left: " << FloatToString(left_speed, 2)
    //       << ", right: " << FloatToString(right_speed, 2);
    serial_->SetVelocityCmdByWheel(left_speed, right_speed,
                                   chassis_->GetTrackLength());

    Time::SleepSec(duration);
  }
  operation_data_->GetStopWatchRef().Stop();
  ZINFO << "Finish cleaning, clean for " << operation_data_->GetDuration()
        << "s.";
  auto step_area = operation_data_->GetNavMapRef()->GetStepAreaSize();
  ZINFO << "Totally clean for " << step_area << "m2, efficiency: "
        << step_area / operation_data_->GetDuration() * 60;

  PublishMap(operation_data_->GetNavMapRef()->GetPrintLayer(),
             MapType::kPrintMap);
  all_steps = steps_recorder.GetPath();
  PublishPath(operation_data_->GetNavMapRef()->GetPrintLayer(), all_steps);

  if (false) {
    SlamValueGridMap2DWriter writer(
        kWriteMapDir,
        Time::DebugString(Time::SystemNow()) + "slam_map.pbstream");
    writer.WriteMap(operation_data_->GetRawSlamValueGridMap2DRef(), true);
  }
  if (false) {
    CharGridMap2DWriter writer(
        kWriteMapDir,
        Time::DebugString(Time::SystemNow()) + "room_map.pbstream");
    auto init_room_map = std::make_shared<CharGridMap2D>(
        NavMap::kRoomMapName_, 1, 1,
        operation_data_->GetOptimizedSlamValueGridMap2DRef()->GetResolution());
    MapConverter map_converter;
    map_converter.ConvertSlamValueGridMap2DToRoomMap(
        operation_data_->GetOptimizedSlamValueGridMap2DRef(), init_room_map);
    writer.WriteMap(init_room_map, true);
  }
  Time::SleepSec(1);
  ros::shutdown();
  ZINFO << "Finish cleaning.";
}

void ZimaXTarkNode::ReadThread() {
  ZINFO << "Start.";
  if (!serial_->Open()) {
    ZERROR;
    return;
  }
  ZINFO << "Ready.";
  while (!shutdown_request_) {
    serial_->Read();
  }
  Time::SleepSec(1);
  ros::shutdown();
  ZINFO << "Finish.";
}

void ZimaXTarkNode::WriteThread() {
  ZINFO << "Start.";
  if (!serial_->Open()) {
    ZERROR;
    return;
  }
  ZINFO << "Ready.";
  Serial::BytesFrames write_frames;
  while (!shutdown_request_) {
    // static bool debug_bool = true;
    // static uint8_t debug_count = 0;
    // if (debug_bool) {
    //   debug_bool = ++debug_count < 50;
    //   if (!debug_bool) {
    //     debug_count = 0;
    //   }
    //   serial_->SetVelocityCmd(0.1, 0, 0);
    //   // ZINFO << "0.1";
    // } else {
    //   debug_bool = ++debug_count > 50;
    //   if (debug_bool) {
    //     debug_count = 0;
    //   }
    //   serial_->SetVelocityCmd(-0.1, 0, 0);
    //   // ZINFO << "-0.1";
    // }
    serial_->PackWriteFrame();
    if (serial_->GetWriteFrames(write_frames)) {
      if (write_frames.size() > 1) {
        ZWARN << "Write buffer jamed.";
      }
      for (auto &frame : write_frames) {
        serial_->Write(frame);
      }
    }
    Time::SleepMSec(20);
  }
  Time::SleepSec(1);
  ros::shutdown();
  ZINFO << "Finish.";
}

void ZimaXTarkNode::ProcessDataThread() {
  ZINFO << "Start.";
  if (!serial_->Open()) {
    ZERROR;
    return;
  }

  auto &tf = *TransformManager::Instance();
  ZINFO << "Ready.";
  Serial::BytesFrames valid_frames_data;
  while (!shutdown_request_) {
    if (serial_->GetReadValidFramesData(valid_frames_data)) {
      if (valid_frames_data.size() > 1) {
        ZWARN << "Read buffer jamed.";
      }
      for (auto &frame_data : valid_frames_data) {
        serial_->ParseReadData(frame_data);
        auto odom = serial_->GetOdomData();

        Transform new_odom_tf(tf.kOdomFrame_, tf.kRobotFrame_, odom.pose_x_,
                              odom.pose_y_, RadiansToDegrees(odom.angular_z_));
        tf.UpdateTransform(new_odom_tf);

        float left_speed, right_speed;
        serial_->GetVelocityDataForWheel(left_speed, right_speed,
                                         chassis_->GetTrackLength());
        // ZINFO << "Speed Left: " << left_speed << ", right: " << right_speed;
        chassis_->GetWheel(chassis_->kLeftWheel_)->SetCurrentSpeed(left_speed);
        chassis_->GetWheel(chassis_->kRightWheel_)
            ->SetCurrentSpeed(right_speed);
      }
    }
    Time::SleepMSec(10);
  }
  Time::SleepSec(1);
  ros::shutdown();
  ZINFO << "Finish.";
}

}  // namespace zima_ros

int main(int argc, char **argv) {
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  ros::init(argc, argv, "zima_xtark_test");
  zima_ros::ZimaXTarkNode test(FLAGS_use_simple_slam);
  std::thread ros_thread(&zima_ros::ZimaNode::ROSSpinThread, &test);
  std::thread write_thread(&zima_ros::ZimaXTarkNode::WriteThread, &test);
  std::thread data_thread(&zima_ros::ZimaXTarkNode::ProcessDataThread, &test);
  std::thread read_thread(&zima_ros::ZimaXTarkNode::ReadThread, &test);
  std::thread tf_thread(&zima_ros::ZimaNode::TfThread, &test);
  std::thread publish_odom_thread(&zima_ros::ZimaNode::PublishSensorDataThread,
                                  &test);
  std::thread data_process_thread(&zima_ros::ZimaNode::DataProcessThread,
                                  &test);

  test.MainThread();

  ros_thread.join();

  write_thread.join();
  data_thread.join();
  read_thread.join();

  tf_thread.join();
  publish_odom_thread.join();
  data_process_thread.join();

  ZINFO << "Exit test.";

  return 0;
}
