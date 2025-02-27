/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_CHASSIS_H
#define ZIMA_CHASSIS_H

#include <atomic>
#include <string>

#include "zima/common/config.h"
#include "zima/common/macro.h"
#include "zima/device/device_manager.h"

namespace zima {

class Chassis : public DeviceManager {
 public:
  class Config {
    public:
    Config();
    explicit Config(const JsonSPtr& json);
    ~Config() = default;

    bool ParseFromJson(const JsonSPtr& json);

    bool config_valid_;

    static const std::string kChassisConfigKey_;

    static const std::string kDeviceConfigKey_;
    JsonSPtr device_config_;

    static const std::string kTrackLengthKey_;
    float track_length_;
    static const std::string kRadiusKey_;
    float radius_;

    static const std::string kEncircleObstacleOnLeftKey_;
    bool encircle_obstacle_on_left_;
  };

  Chassis() = delete;
  explicit Chassis(const Config& config);
  ~Chassis() = default;

  using SPtr = std::shared_ptr<Chassis>;

  using MergedOdomDataCb =
      std::function<void(const MergedOdomData::SPtr& data)>;

  bool IsReady() const { return ready_.load(); }

  virtual void Initialize();
  bool GetBumperEvent(std::vector<std::string>& bumper_names);
  bool ClearBumperEvent();

  float GetTrackLength() const { return track_length_; }
  float GetRadius() const { return radius_; }
  bool EncircleObstacleOnLeft() const { return encircle_obstacle_on_left_; };

  void StopWheels();

  void EnableStallTest();
  void DisableStallTest();
  bool IsStallTestRunning() const { return stall_test_running_.load(); };

  static const std::string kLeftWheel_;
  static const std::string kRightWheel_;

  static const std::string kLeftBumper_;
  static const std::string kCenterBumper_;
  static const std::string kRightBumper_;

  static const std::string kButton1_;

  static const std::string kLeftWallSensor_;
  static const std::string kRightWallSensor_;

  static const std::string kGyro_;

  static const std::string kLidar_;

  static const std::string kBattery_;

 protected:
  Config config_;
  atomic_bool ready_;

  float track_length_;
  float radius_;

  bool encircle_obstacle_on_left_;

  atomic_bool stall_test_running_;
};

}  // namespace zima
#endif  // ZIMA_CHASSIS_H
