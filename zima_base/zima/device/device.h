/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_DEVICE_H
#define ZIMA_DEVICE_H

#include <string>
#include "zima/common/point_cell.h"

namespace zima {

class DeviceBase {
 public:
  class Config {
   public:
    Config();
    ~Config() = default;

    bool ParseFromJson(const JsonSPtr& json);

    bool config_valid_;

    static const std::string kTfToBaseKey_;
    MapPoint tf_to_base_;
  };

  std::string Name() const { return name_; }
  MapPoint GetTf() const { return tf_to_base_; }

  bool IsDeviceValid() const { return device_valid_; }

 protected:
  DeviceBase() = delete;
  DeviceBase(const std::string name, const Config& config)
      : name_(name),
        tf_to_base_(config.tf_to_base_),
        device_valid_(config.config_valid_) {}
  ~DeviceBase() = default;

  std::string name_;
  MapPoint tf_to_base_;

  bool device_valid_;
};

}  // namespace zima

#endif  // ZIMA_DEVICE_H
