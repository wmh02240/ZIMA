/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/common/thread.h"
#include "zima/common/time.h"
#include "zima/hal/chassis/kobuki/chassis.h"
#include "zima/hal/system/keyboard_listener.h"
#include "zima/logger/logger.h"

using namespace zima;

int main(int argc, char **argv) {
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;
  google::InitGoogleLogging(argv[0]);
  google::InstallFailureSignalHandler();

  if (argc < 2)  {
    ZERROR << "options needed. zima_kobuki_checker port";
    ZERROR << "Use correct device port string.";
    return -1;
  }

  std::string port(argv[1]);

  auto kobuki_chassis = std::make_shared<KobukiChassis>(port);
  kobuki_chassis->Initialize();
  Chassis::MergedOdomDataCb data_cb =
      [](const MergedOdomData::SPtr& data) -> void {};
  kobuki_chassis->StartThread(data_cb);
  // kobuki_chassis->EnableDebugLog();

  float left_speed = 0;
  float right_speed = 0;
  auto receive_key_time = Time::Now();
  zima::KeyboardListener::CallBackFunc cb = [&](const char& key) -> void {
    ZINFO << "Pressed "
          << ((key == ' ') ? "Space" : std::string(1, key));
    const float kSpeedStep = 0.02;
    switch (key) {
      case 'w': {
        if (left_speed <
            kobuki_chassis->GetWheel(kobuki_chassis->kLeftWheel_)->MaxSpeed() *
                3 / 4) {
          left_speed += kSpeedStep;
        }
        if (right_speed <
            kobuki_chassis->GetWheel(kobuki_chassis->kRightWheel_)->MaxSpeed() *
                3 / 4) {
          right_speed += kSpeedStep;
        }

        receive_key_time = Time::Now();
        break;
      }
      case 'a': {
        if (left_speed >
            -kobuki_chassis->GetWheel(kobuki_chassis->kLeftWheel_)->MaxSpeed() /
                2) {
          left_speed -= kSpeedStep;
        }
        if (right_speed <
            kobuki_chassis->GetWheel(kobuki_chassis->kRightWheel_)->MaxSpeed() /
                2) {
          right_speed += kSpeedStep;
        }
        receive_key_time = Time::Now();
        break;
      }
      case 's': {
        if (left_speed >
            -kobuki_chassis->GetWheel(kobuki_chassis->kLeftWheel_)->MaxSpeed() /
                2) {
          left_speed -= kSpeedStep;
        }
        if (right_speed >
            -kobuki_chassis->GetWheel(kobuki_chassis->kRightWheel_)
                    ->MaxSpeed() /
                2) {
          right_speed -= kSpeedStep;
        }
        receive_key_time = Time::Now();
        break;
      }
      case 'd': {
        if (left_speed <
            kobuki_chassis->GetWheel(kobuki_chassis->kLeftWheel_)->MaxSpeed() /
                2) {
          left_speed += kSpeedStep;
        }
        if (right_speed >
            -kobuki_chassis->GetWheel(kobuki_chassis->kRightWheel_)
                    ->MaxSpeed() /
                2) {
          right_speed -= kSpeedStep;
        }
        receive_key_time = Time::Now();
        break;
      }
      case 'q': {
        if (left_speed <
            kobuki_chassis->GetWheel(kobuki_chassis->kLeftWheel_)->MaxSpeed() /
                3) {
          left_speed += kSpeedStep;
        }
        if (right_speed <
            kobuki_chassis->GetWheel(kobuki_chassis->kRightWheel_)->MaxSpeed() *
                2 / 3) {
          right_speed += kSpeedStep;
        }
        receive_key_time = Time::Now();
        break;
      }
      case 'e': {
        if (left_speed <
            kobuki_chassis->GetWheel(kobuki_chassis->kLeftWheel_)->MaxSpeed() *
                2 / 3) {
          left_speed += kSpeedStep;
        }
        if (right_speed <
            kobuki_chassis->GetWheel(kobuki_chassis->kRightWheel_)->MaxSpeed() /
                3) {
          right_speed += kSpeedStep;
        }
        receive_key_time = Time::Now();
        break;
      }
      case ' ': {
        left_speed += (0 - left_speed) / 2;
        right_speed += (0 - right_speed) / 2;
        receive_key_time = Time::Now();
        break;
      }
      default: {
        ZINFO << "Invalid cmd.";
      }
    }
  };
  zima::KeyboardListener keyboard_listener(cb);
  keyboard_listener.Start();

  auto thread_manager = zima::ZimaThreadManager::Instance();
  thread_manager->BindCPUCore("main", 0);
  while (true) {
    if (Time::Now() - receive_key_time > 0.55) {
      left_speed += (0 - left_speed) / 2;
      right_speed += (0 - right_speed) / 2;
    }
    kobuki_chassis->GetSerialRef()->SetVelocityCmdByWheel(
        left_speed, right_speed, kobuki_chassis->GetTrackLength());
    Time::SleepMSec(20);
  }

  return 0;
}
