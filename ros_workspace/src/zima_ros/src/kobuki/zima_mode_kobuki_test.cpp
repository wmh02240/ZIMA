/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "kobuki/zima_kobuki_node.h"
#include "zima/event/event_manager.h"
#include "zima/grid_map/map_util.h"
#include "zima/hal/system/keyboard_listener.h"
#include "zima/mode/mode_switcher.h"
#include "zima_ros/util.h"

DEFINE_bool(use_simple_slam, false, "Indicator for using simple slam.");

namespace zima_ros {

using namespace zima;

void ZimaKobukiNode::MainThread(const ZimaThreadWrapper::ThreadParam &param) {
  ZINFO << "Thread \"" << param.thread_name_ << "\" start.";
  auto thread_manager = ZimaThreadManager::Instance();
  if (param.bind_cpu_id_ >= 0) {
    thread_manager->BindCPUCore(param.thread_name_, param.bind_cpu_id_);
  }

  ChassisController::SPtr chassis_controller(new ChassisController(chassis_));

  ModeSwitcher::SPtr mode_switcher_(new ModeSwitcher());
  ModeEventWrapperBase::SPtr mode_wrapper = nullptr;

  Timer process_map_timer("process map timer", 2, true, true);
  uint32_t map_seq = 0;
  const float duration = 0.02;

  while (!shutdown_request_) {
    thread_manager->UpdateThreadCycle(param.thread_name_);

    {
      WriteLocker lock(operation_data_access_);
      bool finish_cleaning =
          !mode_switcher_->Run(mode_wrapper, chassis_controller, chassis_,
                               operation_data_, slam_wrapper_);
      if (finish_cleaning) {
        break;
      }
    }

    auto left_speed = chassis_->GetWheel(chassis_->kLeftWheel_)->TargetSpeed();
    auto right_speed =
        chassis_->GetWheel(chassis_->kRightWheel_)->TargetSpeed();

    // ZINFO << "Set Left: " << FloatToString(left_speed, 2)
    //       << ", right: " << FloatToString(right_speed, 2);
    kobuki_chassis_->GetSerialRef()->SetVelocityCmdByWheel(
        left_speed, right_speed, chassis_->GetTrackLength());

    {
      ReadLocker lock(operation_data_access_);
      auto operation_data = operation_data_;
      lock.Unlock();
      if (operation_data != nullptr) {
        if (!use_simple_slam_ && slam_map_received_) {
          ReadLocker lock(cache_slam_occupancy_grid_map_access_);
          if (cache_slam_occupancy_grid_map_->header.seq != map_seq) {
            map_seq = cache_slam_occupancy_grid_map_->header.seq;

            auto slam_grid_map = zima_ros::OccupancyGridToSlamValueGridMap(
                "slam map", cache_slam_occupancy_grid_map_);
            operation_data->PushNewSlamValueMap(slam_grid_map);
          }
        }
        if (use_simple_slam_ && slam_wrapper_->IsRunning() &&
            process_map_timer.TimeUp()) {
          auto slam_grid_map = slam_wrapper_->GetSlamMap();
          operation_data->PushNewSlamValueMap(slam_grid_map);

          process_map_timer.Reset();
        }
      }
    }

    Time::SleepSec(duration);
  }

  chassis_controller->StopThread();

  Time::SleepMSec(300);
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
  google::ParseCommandLineFlags(&argc, &argv, true);

  zima::JsonSPtr global_config(nullptr);
  if(!zima::GlobalJsonConfig::Instance()->GetGlobalConfig(global_config)) {
    ZERROR << "Missing config.";
    return -1;
  }

  ros::init(argc, argv, "zima_mode_kobuki_test");

  ros::NodeHandle priv_nh("~");
  std::string port;

  priv_nh.param("port", port, std::string("/dev/ttyS1"));

  zima_ros::ZimaKobukiNode test(port, FLAGS_use_simple_slam);

  zima::KeyboardListener::CallBackFunc cb = [&](const char &key) -> void {
    auto &event_manager = *zima::EventManager::Instance();
    event_manager.PushUserEvent(std::make_shared<zima::KeyboardEvent>(key));
  };
  zima::KeyboardListener keyboard_listener(cb);
  keyboard_listener.Start();

  auto thread_manager = zima::ZimaThreadManager::Instance();

  zima::ZimaThreadWrapper::ThreadParam ros_thread_param(
      "Ros spin thread", zima::ZimaThreadManager::kMiscThreadIndex_, 100, 0.2,
      1);
  thread_manager->RegisterThread(
      std::thread(&zima_ros::ZimaNode::ROSSpinThread, &test, ros_thread_param),
      ros_thread_param);

  test.ChassisStartThread();

  zima::ZimaThreadWrapper::ThreadParam tf_thread_param(
      "Tf thread", zima::ZimaThreadManager::kMiscThreadIndex_, 100, 0.2, 1);
  thread_manager->RegisterThread(
      std::thread(&zima_ros::ZimaNode::TfThread, &test, tf_thread_param),
      tf_thread_param);

  zima::ZimaThreadWrapper::ThreadParam publish_odom_thread_param(
      "Publish sensor data thread", zima::ZimaThreadManager::kMiscThreadIndex_,
      100, 0.2, 1);
  thread_manager->RegisterThread(
      std::thread(&zima_ros::ZimaNode::PublishSensorDataThread, &test,
                  publish_odom_thread_param),
      publish_odom_thread_param);

  zima::ZimaThreadWrapper::ThreadParam publish_for_rviz_param(
      "Publish for rviz thread", zima::ZimaThreadManager::kMiscThreadIndex_,
      100, 0.2, 1);
  thread_manager->RegisterThread(
      std::thread(&zima_ros::ZimaNode::PublishForRvizThread, &test,
                  publish_for_rviz_param),
      publish_for_rviz_param);

  zima::ZimaThreadWrapper::ThreadParam data_process_thread_param(
      "Data process thread", zima::ZimaThreadManager::kMiscThreadIndex_, 100,
      0.2, 1);
  thread_manager->RegisterThread(
      std::thread(&zima_ros::ZimaNode::DataProcessThread, &test,
                  data_process_thread_param),
      data_process_thread_param);

  zima::ZimaThreadWrapper::ThreadParam core_thread_param(
      "Core thread", zima::ZimaThreadManager::kCoreThreadIndex_, 100, 0.2, 1);
  thread_manager->RegisterThread(
      std::thread(&zima_ros::ZimaKobukiNode::MainThread, &test,
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
  test.ChassisShutdownThread();
  thread_manager->WaitForThreadExit(tf_thread_param.thread_name_,
                                    wait_for_exit_timeout_s);
  thread_manager->WaitForThreadExit(publish_odom_thread_param.thread_name_,
                                    wait_for_exit_timeout_s);
  thread_manager->WaitForThreadExit(data_process_thread_param.thread_name_,
                                    wait_for_exit_timeout_s);
  thread_manager->WaitForThreadExit(publish_for_rviz_param.thread_name_,
                                    wait_for_exit_timeout_s);
  thread_manager->WaitForThreadExit(core_thread_param.thread_name_,
                                    wait_for_exit_timeout_s);
  ZINFO << "Exit.";
  if (thread_manager->RunningThreadCount() > 1) {
    thread_manager->DebugRunningThreadName();
  }

  return 0;
}
