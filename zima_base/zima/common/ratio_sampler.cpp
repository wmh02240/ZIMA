/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/common/ratio_sampler.h"
#include "zima/common/util.h"

namespace zima {

RatioSampler::RatioSampler(const float& ratio)
    : ratio_(ratio), num_pulses_(0), num_samples_(0) {}

bool RatioSampler::Pulse() {
  ++num_pulses_;
  if (static_cast<double>(num_samples_) / num_pulses_ < ratio_) {
    ++num_samples_;
    return true;
  }
  return false;
}

std::string RatioSampler::DebugString() const {
  return std::to_string(num_samples_) + " (" +
         FloatToString(100. * num_samples_ / num_pulses_, 1) + "%/" +
         FloatToString(100 * ratio_, 1) + "%)";
}

}  // namespace zima
