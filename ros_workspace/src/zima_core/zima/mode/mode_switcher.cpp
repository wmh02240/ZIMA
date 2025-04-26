/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/mode/mode_switcher.h"

#include "zima/mode/auto_cleaning_mode_event_wrapper.h"
#include "zima/mode/auto_scan_house_mode_event_wrapper.h"
#include "zima/mode/pause_mode_event_wrapper.h"
#include "zima/mode/standby_mode_event_wrapper.h"

namespace zima {

ModeSwitcher::ModeSwitcher()
    : auto_cleaning_info_(nullptr), auto_scanning_info_(nullptr) {}

bool ModeSwitcher::Run(ModeEventWrapperBase::SPtr &mode_wrapper,
                       const ChassisController::SPtr &chassis_controller,
                       const Chassis::SPtr &chassis,
                       OperationData::SPtr &operation_data,
                       const SlamBase::SPtr slam_wrapper) {
  // static std::shared_ptr<StopWatch> test_pause_resume_stop_watch(
  //     new StopWatch("Test"));
  // srand(static_cast<unsigned>(time(NULL)));
  // static double test_time = rand() % 9 + 9;

  if (mode_wrapper == nullptr) {
    mode_wrapper.reset(
        new StandbyModeWrapper(chassis, chassis_controller, slam_wrapper));
  } else {
    switch (mode_wrapper->GetModeLabel()) {
      case ModeEventWrapperBase::ModeLabel::kStandbyMode: {
        auto standby_mode =
            dynamic_cast<StandbyModeWrapper *>(mode_wrapper.get());
        standby_mode->Run(operation_data, slam_wrapper);
        if (standby_mode->IsExited()) {
          auto next_mode_label = standby_mode->GetNextModeLabel();
          switch (next_mode_label) {
            case ModeEventWrapperBase::ModeLabel::kAutoCleaningMode: {
              mode_wrapper.reset(new AutoCleaningModeWrapper(
                  chassis, chassis_controller, slam_wrapper, operation_data,
                  auto_cleaning_info_));
              break;
            }
            case ModeEventWrapperBase::ModeLabel::kAutoScanHouseMode: {
              mode_wrapper.reset(new AutoScanHouseModeWrapper(
                  chassis, chassis_controller, slam_wrapper, operation_data,
                  auto_scanning_info_));
              break;
            }
            default: {
              return false;
            }
          }
        }
        break;
      }
      case ModeEventWrapperBase::ModeLabel::kPauseMode: {
        // if (test_pause_resume_stop_watch->Duration() > test_time) {
        //   ZERROR << "Test resume.";
        //   event_manager.PushUserEvent(
        //       std::make_shared<zima::KeyboardEvent>(' '));
        //   srand(static_cast<unsigned>(time(NULL)));
        //   test_time = rand() % 12 + 6;
        //   test_pause_resume_stop_watch.reset(new StopWatch("Test"));
        // }
        auto pause_mode = dynamic_cast<PauseModeWrapper *>(mode_wrapper.get());
        pause_mode->Run(operation_data, slam_wrapper);
        if (pause_mode->IsExited()) {
          auto next_mode_label = pause_mode->GetNextModeLabel();
          switch (next_mode_label) {
            case ModeEventWrapperBase::ModeLabel::kAutoCleaningMode: {
              mode_wrapper.reset(new AutoCleaningModeWrapper(
                  chassis, chassis_controller, slam_wrapper, operation_data,
                  auto_cleaning_info_));
              break;
            }
            case ModeEventWrapperBase::ModeLabel::kAutoScanHouseMode: {
              mode_wrapper.reset(new AutoScanHouseModeWrapper(
                  chassis, chassis_controller, slam_wrapper, operation_data,
                  auto_scanning_info_));
              break;
            }
            case ModeEventWrapperBase::ModeLabel::kStandbyMode: {
              mode_wrapper.reset(new StandbyModeWrapper(
                  chassis, chassis_controller, slam_wrapper));
              auto_cleaning_info_.reset();
              auto_scanning_info_.reset();
              break;
            }
            default: {
              return false;
            }
          }
        }
        break;
      }
      case ModeEventWrapperBase::ModeLabel::kAutoCleaningMode: {
        // if (test_pause_resume_stop_watch->Duration() > test_time) {
        //   ZERROR << "Test pause.";
        //   event_manager.PushUserEvent(
        //       std::make_shared<zima::KeyboardEvent>(' '));
        //   srand(static_cast<unsigned>(time(NULL)));
        //   test_time = rand() % 12 + 3;
        //   test_pause_resume_stop_watch.reset(new StopWatch("Test"));
        // }

        auto auto_cleaning_mode =
            dynamic_cast<AutoCleaningModeWrapper *>(mode_wrapper.get());
        auto_cleaning_mode->Run(operation_data, slam_wrapper);
        if (auto_cleaning_mode->IsExited()) {
          auto next_mode_label = auto_cleaning_mode->GetNextModeLabel();
          switch (next_mode_label) {
            case ModeEventWrapperBase::ModeLabel::kPauseMode: {
              auto_cleaning_info_ = auto_cleaning_mode->GetCleaningInfo();
              auto_scanning_info_.reset();
              mode_wrapper.reset(new PauseModeWrapper(
                  chassis, chassis_controller, slam_wrapper, operation_data));

              // auto &tf = *TransformManager::Instance();
              // Transform old_odom_tf(tf.kOdomFrame_, tf.kRobotFrame_,
              //                                 0, 0, 0);
              // tf.GetTransform(tf.kOdomFrame_, tf.kRobotFrame_,
              //                 old_odom_tf);
              // MapPoint new_odom(
              //     old_odom_tf.X(), old_odom_tf.Y(),
              //     NormalizeDegree(old_odom_tf.Degree() + rand() % 12 *
              //     10));
              // ZINFO << "New odom: " << new_odom.DebugString();
              // Transform new_odom_tf(tf.kOdomFrame_, tf.kRobotFrame_,
              //                       new_odom.X(), new_odom.Y(),
              //                       new_odom.Degree());
              // tf.UpdateTransform(new_odom_tf);

              break;
            }
            case ModeEventWrapperBase::ModeLabel::kStandbyMode: {
              mode_wrapper.reset(new StandbyModeWrapper(
                  chassis, chassis_controller, slam_wrapper));
              auto_cleaning_info_.reset();
              auto_scanning_info_.reset();
              break;
            }
            default: {
              return false;
            }
          }
        }
        break;
      }
      case ModeEventWrapperBase::ModeLabel::kAutoScanHouseMode: {
        // if (test_pause_resume_stop_watch->Duration() > test_time) {
        //   ZERROR << "Test pause.";
        //   event_manager.PushUserEvent(
        //       std::make_shared<zima::KeyboardEvent>(' '));
        //   srand(static_cast<unsigned>(time(NULL)));
        //   test_time = rand() % 12 + 3;
        //   test_pause_resume_stop_watch.reset(new StopWatch("Test"));
        // }

        auto auto_scan_house_mode =
            dynamic_cast<AutoScanHouseModeWrapper *>(mode_wrapper.get());
        auto_scan_house_mode->Run(operation_data, slam_wrapper);
        if (auto_scan_house_mode->IsExited()) {
          auto next_mode_label = auto_scan_house_mode->GetNextModeLabel();
          switch (next_mode_label) {
            case ModeEventWrapperBase::ModeLabel::kPauseMode: {
              auto_scanning_info_ = auto_scan_house_mode->GetScanningInfo();
              mode_wrapper.reset(new PauseModeWrapper(
                  chassis, chassis_controller, slam_wrapper, operation_data));

              // auto &tf = *TransformManager::Instance();
              // Transform old_odom_tf(tf.kOdomFrame_, tf.kRobotFrame_,
              //                                 0, 0, 0);
              // tf.GetTransform(tf.kOdomFrame_, tf.kRobotFrame_,
              //                 old_odom_tf);
              // MapPoint new_odom(
              //     old_odom_tf.X(), old_odom_tf.Y(),
              //     NormalizeDegree(old_odom_tf.Degree() + rand() % 12 *
              //     10));
              // ZINFO << "New odom: " << new_odom.DebugString();
              // Transform new_odom_tf(tf.kOdomFrame_, tf.kRobotFrame_,
              //                       new_odom.X(), new_odom.Y(),
              //                       new_odom.Degree());
              // tf.UpdateTransform(new_odom_tf);

              break;
            }
            case ModeEventWrapperBase::ModeLabel::kStandbyMode: {
              mode_wrapper.reset(new StandbyModeWrapper(
                  chassis, chassis_controller, slam_wrapper));
              auto_cleaning_info_.reset();
              auto_scanning_info_.reset();
              break;
            }
            default: {
              return false;
            }
          }
        }
        break;
      }
      default: {
        ZERROR << "Should never run here.";
        break;
      }
    }
  }

  return true;
}

}  // namespace zima
