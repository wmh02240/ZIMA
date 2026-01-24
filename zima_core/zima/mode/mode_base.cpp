/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/mode/mode_base.h"

namespace zima {

ModeBase::ModeBase(const std::string& name, const Chassis::SPtr& chassis,
                   const ChassisController::SPtr& chassis_controller,
                   const bool& use_simple_slam)
    : name_(name),
      access_(make_shared<ReadWriteLock>()),
      chassis_(chassis),
      chassis_controller_(chassis_controller),
      valid_(true),
      use_simple_slam_(use_simple_slam) {
  if (chassis == nullptr) {
    ZERROR << "Chassis pointer empty.";
    valid_ = false;
  }
  if (chassis_controller == nullptr) {
    ZERROR << "Chassis controller pointer empty.";
    valid_ = false;
  }
}

void ModeBase::EnableStallTest() {
  ReadLocker lock(access_);
  if (chassis_ != nullptr) {
    chassis_->EnableStallTest();
  } else {
    ZWARN << "Chassis not initialized.";
  }
}

void ModeBase::DisableStallTest() {
  ReadLocker lock(access_);
  if (chassis_ != nullptr) {
    chassis_->DisableStallTest();
  } else {
    ZWARN << "Chassis not initialized.";
  }
}

bool ModeBase::IsStallTestRunning() const {
  ReadLocker lock(access_);
  if (chassis_ != nullptr) {
    return chassis_->IsStallTestRunning();
  } else {
    ZWARN << "Chassis not initialized.";
    return false;
  }
}

OperationMode::Config::Config() : Config(nullptr) {}

OperationMode::Config::Config(const JsonSPtr& json)
    : config_valid_(true),
      optimized_small_range_search_config_(nullptr),
      small_range_search_config_(nullptr),
      optimized_global_search_config_(nullptr),
      global_search_config_(nullptr) {
  // Load default setting.
  small_range_linear_range_ = 1.0;
  relocation_count_max_ = 4;

  if (json != nullptr) {
    // ZINFO << "Load config from json.";
    config_valid_ = ParseFromJson(json);
  }

  // Load default setting.
  if (optimized_small_range_search_config_ == nullptr) {
    optimized_small_range_search_config_.reset(
        new PointCloudMatcher::BABSearchConfig());
    optimized_small_range_search_config_->degree_range_ = 3;
    optimized_small_range_search_config_->degree_step_ = 1;
    optimized_small_range_search_config_->search_depth_ = 4;
    optimized_small_range_search_config_->min_score_ = 0.6;
    optimized_small_range_search_config_->config_valid_ = true;
    ZGINFO;
  }
  if (small_range_search_config_ == nullptr) {
    small_range_search_config_.reset(new PointCloudMatcher::BABSearchConfig());
    small_range_search_config_->degree_range_ = 180;
    small_range_search_config_->degree_step_ = 1;
    small_range_search_config_->search_depth_ = 4;
    small_range_search_config_->min_score_ = 0.6;
    small_range_search_config_->config_valid_ = true;
    ZGINFO;
  }
  if (optimized_global_search_config_ == nullptr) {
    optimized_global_search_config_.reset(
        new PointCloudMatcher::BABSearchConfig());
    optimized_global_search_config_->degree_range_ = 3;
    optimized_global_search_config_->degree_step_ = 1;
    optimized_global_search_config_->search_depth_ = 4;
    optimized_global_search_config_->min_score_ = 0.51;
    optimized_global_search_config_->config_valid_ = true;
    ZGINFO;
  }
  if (global_search_config_ == nullptr) {
    global_search_config_.reset(new PointCloudMatcher::BABSearchConfig());
    global_search_config_->degree_range_ = 180;
    global_search_config_->degree_step_ = 1;
    global_search_config_->search_depth_ = 4;
    global_search_config_->min_score_ = 0.51;
    global_search_config_->config_valid_ = true;
    ZGINFO;
  }
}

bool OperationMode::Config::ParseFromJson(const JsonSPtr& json) {
  if (json == nullptr) {
    ZERROR << "Invalid pointer.";
    return false;
  }

  // Override.
  if (!JsonHelper::GetFloat(*json, kSmallRangeLinearRangeKey_,
                            small_range_linear_range_)) {
    ZGERROR << "Config " << kSmallRangeLinearRangeKey_ << " not found.";
  }
  if (small_range_linear_range_ < 0) {
    ZERROR << "Config " << kSmallRangeLinearRangeKey_
           << " invalid: " << FloatToString(small_range_linear_range_, 3)
           << ".";
    return false;
  }

  if (!JsonHelper::GetUInt(*json, kRelocationCountMaxKey_,
                           relocation_count_max_)) {
    ZGERROR << "Config " << kRelocationCountMaxKey_ << " not found.";
  }
  if (relocation_count_max_ < 3) {
    ZERROR << "Config " << kRelocationCountMaxKey_
           << " invalid: " << relocation_count_max_ << ".";
    return false;
  }

  JsonSPtr _os_json_config(new Json());
  if (JsonHelper::GetObject(*json, kOptimizedSmallRangeSearchConfigKey_,
                            *_os_json_config)) {
    ZGINFO << "Override config from json.";
    optimized_small_range_search_config_.reset(
        new PointCloudMatcher::BABSearchConfig(_os_json_config));
    if (!optimized_small_range_search_config_->config_valid_) {
      ZERROR << "Config " << kOptimizedSmallRangeSearchConfigKey_
             << " invalid.";
      return false;
    }
  }

  JsonSPtr _s_json_config(new Json());
  if (JsonHelper::GetObject(*json, kSmallRangeSearchConfigKey_,
                            *_s_json_config)) {
    ZGINFO << "Override config from json.";
    small_range_search_config_.reset(
        new PointCloudMatcher::BABSearchConfig(_s_json_config));
    if (!small_range_search_config_->config_valid_) {
      ZERROR << "Config " << kSmallRangeSearchConfigKey_ << " invalid.";
      return false;
    }
  }

  JsonSPtr _og_json_config(new Json());
  if (JsonHelper::GetObject(*json, kOptimizedGlobalSearchConfigKey_,
                            *_og_json_config)) {
    ZGINFO << "Override config from json.";
    optimized_global_search_config_.reset(
        new PointCloudMatcher::BABSearchConfig(_og_json_config));
    if (!optimized_global_search_config_->config_valid_) {
      ZERROR << "Config " << kOptimizedGlobalSearchConfigKey_ << " invalid.";
      return false;
    }
  }

  JsonSPtr _g_json_config(new Json());
  if (JsonHelper::GetObject(*json, kGlobalSearchConfigKey_, *_g_json_config)) {
    ZGINFO << "Override config from json.";
    global_search_config_.reset(
        new PointCloudMatcher::BABSearchConfig(_g_json_config));
    if (!global_search_config_->config_valid_) {
      ZERROR << "Config " << kGlobalSearchConfigKey_ << " invalid.";
      return false;
    }
  }

  return true;
}

OperationMode::OperationMode(const std::string& name,
                             const Chassis::SPtr& chassis,
                             const ChassisController::SPtr& chassis_controller,
                             const bool& use_simple_slam)
    : ModeBase(name, chassis, chassis_controller, use_simple_slam),
      start_request_(false),
      stop_request_(false),
      pause_request_(false),
      initialize_start_time_(0),
      operate_on_slam_time_(0),
      lidar_ready_(false),
      odom_ready_(false),
      async_point_cloud_matcher_(nullptr),
      relocation_count_(4){};

}  // namespace zima
