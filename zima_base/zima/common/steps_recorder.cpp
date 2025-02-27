/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/common/steps_recorder.h"

#include "zima/grid_map/nav_map_2d.h"

namespace zima {
StepsRecorder::Config::Config() { Initialize(); }

bool StepsRecorder::Config::Initialize() {
  // TODO(Austin): Load it from config file.
  filter_distance_ = NavMap::GetResolution() / 2;
  filter_degree_ = 30;
  filter_max_distance_ = 50;
  filter_least_ignore_points_ = 360 / filter_degree_;

  stuck_distance_limit_ = 0.5;
  stuck_time_limit_ = 10;

  small_area_trapped_range_ = 0.5;

  if (stuck_distance_limit_ > filter_max_distance_) {
    // It should never run this.
    ZERROR << "stuck_distance_limit_:" << stuck_distance_limit_
           << " vs filter_max_distance_:" << filter_max_distance_;
    stuck_distance_limit_ = filter_max_distance_;
    return false;
  }

  return true;
}

StepsRecorder::StepsRecorder() : access_(std::make_shared<ReadWriteLock>()) {
  cache_steps_.clear();
};

StepsRecorder::StepsRecorder(const StepsRecorder::Config& config)
    : access_(std::make_shared<ReadWriteLock>()) {
  config_ = config;
  cache_steps_.clear();
};

StepsRecorder::StepsRecorder(const StepsRecorder& ref)
    : access_(std::make_shared<ReadWriteLock>()) {
  config_ = ref.config_;
  cache_steps_ = ref.GetPath();
}

bool StepsRecorder::AddPathPoint(const StepPoint& point, const bool& force_add,
                                 const bool& force_add_without_log) {
  WriteLocker lock(access_);
  if (force_add) {
    if (!force_add_without_log) {
      ZINFO << "Force cache " << point.DebugString();
    }
    cache_steps_.emplace_back(point);
    return true;
  }

  while (!cache_steps_.empty()) {
    if (cache_steps_.front().Pose().Distance(point.Pose()) >
        config_.filter_max_distance_) {
      cache_steps_.pop_front();
      continue;
    } else {
      break;
    }
  }

  if (cache_steps_.empty()) {
    cache_steps_.emplace_back(point);
    // ZINFO << "(" << cache_steps_.size() << ")Cache " << point.DebugString();
    return true;
  } else if (cache_steps_.back().Pose().Distance(point.Pose()) >
             config_.filter_distance_) {
    cache_steps_.emplace_back(point);
    // ZINFO << "(" << cache_steps_.size() << ")Cache " << point.DebugString();
    return true;
  } else {
    if (fabs(NormalizeDegree(cache_steps_.back().Pose().AngleDiff(
            point.Pose()))) > config_.filter_degree_) {
      cache_steps_.emplace_back(point);
      // ZINFO << "(" << cache_steps_.size() << ")Cache " << point.DebugString();
      return true;
    }
  }
  return false;
}

StepsRecorder::SituationCode StepsRecorder::CheckPath() {
  ReadLocker lock(access_);
  if (cache_steps_.size() < 2) {
    return OK;
  }

  unsigned int index = 0;
  auto point_size = cache_steps_.size();
  auto curr_pose = cache_steps_.back().Pose();
  while (index < point_size) {
    if (point_size < config_.filter_least_ignore_points_ ||
        point_size - index < config_.filter_least_ignore_points_) {
      break;
    }
    auto it = cache_steps_.begin();
    for (uint32_t i = 0; i < index; i++) {
      it++;
    }
    auto distance = it->Pose().Distance(curr_pose);
    auto similar_position = distance < config_.filter_distance_;
    if (similar_position) {
      auto angle_diff = fabs(
          NormalizeDegree(it->Pose().AngleDiff(curr_pose)));
      auto similar_degree = angle_diff < config_.filter_degree_;
      if (similar_degree) {
        // Found similar pose
        ZGINFO << "Found similar pose at " << index << " of " << point_size
               << " steps. Curr: " << curr_pose.DebugString()
               << ", cache pose: " << it->Pose();
        return NORMAL_CLOSE_LOOP;
      }
    }

    auto scale = distance / config_.filter_distance_;
    if (scale > config_.filter_least_ignore_points_) {
      index += static_cast<unsigned int>(
          scale - config_.filter_least_ignore_points_ / 2);
    } else {
      index++;
    }
  }

  return OK;
}

Steps StepsRecorder::GetPath() const {
  ReadLocker lock(access_);
  return cache_steps_;
}

Steps StepsRecorder::GetRestPath(const uint32_t& start_index) {
  ReadLocker lock(access_);
  Steps steps;
  auto start_it = cache_steps_.begin();
  for (uint32_t i = 0; i < start_index; i++) {
    start_it++;
  }
  if (start_index < cache_steps_.size()) {
    steps.insert(steps.end(), start_it, cache_steps_.end());
  }
  return steps;
}

uint32_t StepsRecorder::GetPathSize() const {
  ReadLocker lock(access_);
  return cache_steps_.size();
}

}  // namespace zima
