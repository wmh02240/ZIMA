/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/event/event_manager.h"
#include "zima/hal/system/keyboard_listener.h"
#include "zima/logger/logger.h"
#include "zima/mode/auto_cleaning_mode_event_wrapper.h"
#include "zima/mode/auto_scan_house_mode_event_wrapper.h"

using namespace zima;

class Slam1 : public SlamBase {
 public:
  Slam1() : SlamBase(SlamBase::Config()) {}
  ~Slam1() = default;

  bool StartSlam(const MapPoint& init_pose, const std::string& save_file_name,
                 const std::string& load_file_name) override {
    ZINFO << "Call new method.";
    return true;
  }
};

void AutoCleaningModeEventWrapperTest() {
  auto chassis = std::make_shared<Chassis>(Chassis::Config());
  chassis->Initialize();
  auto chassis_controller = std::make_shared<ChassisController>(chassis);
  auto slam1 = std::make_shared<Slam1>();
  auto nav_data = std::make_shared<OperationData>();
  auto auto_totally_new_cleaning = std::make_shared<AutoCleaningModeWrapper>(
      chassis, chassis_controller, slam1, nav_data);
  // auto& event_manager = *EventManager::Instance();

  while (true) {
    auto_totally_new_cleaning->Run(nav_data);
    if (auto_totally_new_cleaning->IsExited()) {
      break;
    }

    Time::SleepMSec(20);
  }
  chassis_controller->StopThread();
}

void AutoCleaningModeTest() {
  auto chassis = std::make_shared<Chassis>(Chassis::Config());
  chassis->Initialize();
  auto chassis_controller = std::make_shared<ChassisController>(chassis);
  auto slam1 = std::make_shared<Slam1>();
  auto auto_totally_new_cleaning =
      std::make_shared<AutoCleaningMode>(chassis, chassis_controller, slam1);

  auto nav_data = std::make_shared<OperationData>();
  auto_totally_new_cleaning->Start();
  while (true) {
    auto_totally_new_cleaning->Run(nav_data);

    if (auto_totally_new_cleaning->IsReady() ||
        auto_totally_new_cleaning->IsAutoFinished()) {
      break;
    }
    Time::SleepMSec(20);
  }
  chassis_controller->StopThread();
}

void AutoScanHouseModeEventWrapperTest() {
  auto chassis = std::make_shared<Chassis>(Chassis::Config());
  chassis->Initialize();
  auto chassis_controller = std::make_shared<ChassisController>(chassis);
  auto slam1 = std::make_shared<Slam1>();
  auto nav_data = std::make_shared<OperationData>();
  auto auto_scan_house = std::make_shared<AutoScanHouseModeWrapper>(
      chassis, chassis_controller, slam1, nav_data);
  // auto& event_manager = *EventManager::Instance();

  while (true) {
    auto_scan_house->Run(nav_data);
    if (auto_scan_house->IsExited()) {
      break;
    }

    Time::SleepMSec(20);
  }
  chassis_controller->StopThread();
}


int main(int argc, char** argv) {
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  zima::KeyboardListener::CallBackFunc cb = [&](const char& key) -> void {
    auto& event_manager = *zima::EventManager::Instance();
    event_manager.PushUserEvent(std::make_shared<zima::KeyboardEvent>(key));
  };
  zima::KeyboardListener keyboard_listener(cb);
  keyboard_listener.Start();

  // AutoCleaningModeTest();
  // AutoCleaningModeEventWrapperTest();
  AutoScanHouseModeEventWrapperTest();
  keyboard_listener.Stop();

  return 0;
}
