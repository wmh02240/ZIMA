/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_GYRO_H
#define ZIMA_GYRO_H

#include <atomic>
#include <memory>

#include "zima/common/lock.h"
#include "zima/device/device.h"

namespace zima {

class Gyro : public DeviceBase {
 public:
  class Config : public DeviceBase::Config {
   public:
    Config();
    explicit Config(const JsonSPtr& json);
    ~Config() = default;

    bool ParseFromJson(const JsonSPtr& json);
  };

  Gyro() = delete;
  Gyro(const std::string name, const Config& config);
  ~Gyro() = default;

  using SPtr = std::shared_ptr<Gyro>;

  void SetDegree(const float& degree, const double& data_timestamp);

  float GetDegree() const;

  double GetDataTimeStamp() const;

  bool CheckFresh(const double& limit) const;

  static const std::string kNullName_;

 private:
  ReadWriteLock::SPtr lock_;
  float degree_;
  double data_timestamp_;
};

}  // namespace zima

#endif  // ZIMA_GYRO_H
