/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_RATIO_SAMPLER_H
#define ZIMA_RATIO_SAMPLER_H

#include <cstdint>
#include <string>

namespace zima {

// Learn from cartographer.
class RatioSampler {
 public:
  RatioSampler() = delete;
  RatioSampler(const float& ratio);
  ~RatioSampler() = default;

  bool Pulse();

  std::string DebugString() const;

 private:
  float ratio_;

  uint64_t num_pulses_;
  uint64_t num_samples_;
};

}  // namespace zima

#endif  // ZIMA_RATIO_SAMPLER_H
