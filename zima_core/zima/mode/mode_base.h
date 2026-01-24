/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_MODE_BASE_H
#define ZIMA_MODE_BASE_H

#include "zima/algorithm/point_cloud_matcher.h"
#include "zima/common/debug.h"
#include "zima/common/lock.h"
#include "zima/robot/chassis.h"
#include "zima/robot/chassis_controller.h"
#include "zima/robot_data/operation_data.h"

namespace zima {

class ModeBase : public DebugBase {
 public:
  using SPtr = std::shared_ptr<ModeBase>;

  void EnableStallTest();
  void DisableStallTest();
  bool IsStallTestRunning() const;

 protected:
  ModeBase() = delete;
  ModeBase(const std::string& name, const Chassis::SPtr& chassis,
           const ChassisController::SPtr& chassis_controller,
           const bool& use_simple_slam = true);
  virtual ~ModeBase() = default;

  virtual void Run(const OperationData::SPtr& operation_data) = 0;

  std::string name_;
  ReadWriteLock::SPtr access_;
  Chassis::SPtr chassis_;
  ChassisController::SPtr chassis_controller_;
  bool valid_;

  bool use_simple_slam_;
};

class OperationMode : public ModeBase {
 public:
  class Config {
   public:
    Config();
    Config(const JsonSPtr& json);
    ~Config() = default;

    using SPtr = std::shared_ptr<Config>;

    bool ParseFromJson(const JsonSPtr& json);

    bool config_valid_;

    static const std::string kRelocationCountMaxKey_;
    uint relocation_count_max_;
    static const std::string kSmallRangeLinearRangeKey_;
    float small_range_linear_range_;
    static const std::string kOptimizedSmallRangeSearchConfigKey_;
    PointCloudMatcher::BABSearchConfig::SPtr
        optimized_small_range_search_config_;
    static const std::string kSmallRangeSearchConfigKey_;
    PointCloudMatcher::BABSearchConfig::SPtr small_range_search_config_;
    static const std::string kOptimizedGlobalSearchConfigKey_;
    PointCloudMatcher::BABSearchConfig::SPtr optimized_global_search_config_;
    static const std::string kGlobalSearchConfigKey_;
    PointCloudMatcher::BABSearchConfig::SPtr global_search_config_;
  };

 protected:
  OperationMode() = delete;
  OperationMode(const std::string& name, const Chassis::SPtr& chassis,
                const ChassisController::SPtr& chassis_controller,
                const bool& use_simple_slam = true);
  virtual ~OperationMode() = default;

  bool start_request_;
  bool stop_request_;
  bool pause_request_;

  double initialize_start_time_;
  double operate_on_slam_time_;
  bool lidar_ready_;
  bool odom_ready_;
  AsyncPointCloudMatcher::SPtr async_point_cloud_matcher_;
  uint8_t relocation_count_;
};

}  // namespace zima

#endif  // ZIMA_MOTION_H
