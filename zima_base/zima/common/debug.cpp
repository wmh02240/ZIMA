/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/common/debug.h"

#include "zima/common/util.h"
#include "zima/logger/logger.h"

namespace zima {

std::string PIDPrint::DebugString(const std::string& name,
                                  const float& target_value,
                                  const float& print_range_min,
                                  const float& print_range_max,
                                  const float& current_value) {
  if (print_range_max <= print_range_min) {
    return "Range invalid: min " + FloatToString(print_range_min, 3) + " max " +
           FloatToString(print_range_max, 3);
  }
  const uint8_t kDevideCount = 80;
  const float step = (print_range_max - print_range_min) / kDevideCount;
  int8_t current_value_index = -1;
  int8_t target_value_index = -1;
  if (current_value >= print_range_min && current_value <= print_range_max) {
    current_value_index =
        static_cast<int8_t>((current_value - print_range_min) / step);
  }
  if (target_value >= print_range_min && target_value <= print_range_max) {
    target_value_index =
        static_cast<int8_t>((target_value - print_range_min) / step);
  }
  std::string str;
  for (int8_t i = 0; i < kDevideCount; i++) {
    if (i == current_value_index && i == target_value_index) {
      str += "o";
    } else if (i == current_value_index) {
      str += "|";
    } else if (i == target_value_index) {
      str += ".";
    } else {
      str += " ";
    }
  }

  return name + ": " + FloatToString(current_value, 2) + "/" +
         FloatToString(target_value, 2) + "(" +
         FloatToString(print_range_min, 2) + "~" +
         FloatToString(print_range_max, 2) + ")" + str;
}

}  // namespace zima
