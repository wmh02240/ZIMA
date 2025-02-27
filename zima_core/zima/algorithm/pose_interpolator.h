/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_POSE_INTERPOLATOR_H
#define ZIMA_POSE_INTERPOLATOR_H

#include "zima/common/debug.h"
#include "zima/device/device_manager.h"

namespace zima {

/*
 * According to cached data, interpolate or predict pose for given timestamp.
 */
class PoseInterpolatorBase {
 public:
  static MapPoint::SPtr CalculateVelocity(const MergedOdomData& a,
                                          const MergedOdomData& b);

  static MergedOdomData::SPtr PredictPose(
      const std::deque<MergedOdomData::SPtr> buffer, const double& timestamp);
  static bool IsInterpolatable(const std::deque<MergedOdomData::SPtr> buffer,
                               const double& timestamp);
  static MergedOdomData::SPtr InterpolatePose(
      const std::deque<MergedOdomData::SPtr> buffer, const double& timestamp);

 protected:
  PoseInterpolatorBase() = default;
  ~PoseInterpolatorBase() = default;
};

class PoseInterpolator : public PoseInterpolatorBase, public DebugBase {
 public:
  class Config {
   public:
    Config();
    explicit Config(const JsonSPtr& json);
    ~Config() = default;

    bool ParseFromJson(const JsonSPtr& json);

    bool config_valid_;

    static const std::string kConfigKey_;
    static const std::string kBufferTimeLimitKey_;
    float buffer_time_limit_;
    static const std::string kBufferCountLimitKey_;
    uint buffer_count_limit_;
  };

  PoseInterpolator() = delete;
  PoseInterpolator(const Config& config);
  ~PoseInterpolator() = default;

  using SPtr = std::shared_ptr<PoseInterpolator>;

  void AddData(const MergedOdomData::SPtr& data);
  MergedOdomData::SPtr PredictPose(const double& timestamp) const;
  bool IsInterpolatable(const double& timestamp) const;
  MergedOdomData::SPtr InterpolatePose(const double& timestamp) const;

 private:
  ReadWriteLock::SPtr access_;
  float buffer_time_limit_;
  uint16_t buffer_count_limit_;
  std::deque<MergedOdomData::SPtr> buffer_;
};

}  // namespace zima

#endif  // ZIMA_POSE_INTERPOLATOR_H
