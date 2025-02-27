/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_STEPS_RECORDER_H
#define ZIMA_STEPS_RECORDER_H

#include "zima/common/marker_points.h"

namespace zima {

class StepsRecorder {
 public:
  class Config {
   public:
    Config();
    ~Config() = default;

    bool Initialize();

    float filter_distance_;
    float filter_degree_;
    float filter_max_distance_;
    float filter_least_ignore_points_;

    float stuck_distance_limit_;
    float stuck_time_limit_;

    float small_area_trapped_range_;
  };

  StepsRecorder();
  StepsRecorder(const Config& config);
  StepsRecorder(const StepsRecorder& ref);
  ~StepsRecorder() = default;

  using SPtr = std::shared_ptr<StepsRecorder>;

  enum SituationCode {
    OK,
    NORMAL_CLOSE_LOOP,
    TRAPPED_IN_SMALL_AREA,
    STUCK,
  };

  bool AddPathPoint(const StepPoint& point, const bool& force_add = false,
                    const bool& force_add_without_log = false);
  SituationCode CheckPath();
  Steps GetPath() const;
  // Get rest path from start index to path end.
  Steps GetRestPath(const uint32_t& start_index);
  uint32_t GetPathSize() const;

 private:
  ReadWriteLock::SPtr access_;
  Config config_;
  Steps cache_steps_;
};

}  // namespace zima

#endif  // ZIMA_STEPS_RECORDER_H
