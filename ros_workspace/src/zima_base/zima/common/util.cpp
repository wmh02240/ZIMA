/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/common/util.h"

#include "zima/logger/logger.h"

namespace zima {

std::string FloatToString(const float& val, const int& precision) {
  std::ostringstream out;
  if (precision > 0) {
    out << std::setiosflags(std::ios::fixed) << std::setprecision(precision) << val;
  } else {
    out << val;
  }

  return out.str();
}

std::string DoubleToString(const double& val, const int& precision) {
  std::ostringstream out;
  if (precision > 0) {
    out << std::setiosflags(std::ios::fixed) << std::setprecision(precision) << val;
  } else {
    out << val;
  }

  return out.str();

}

std::string UintToHexString(const uint32_t& i) {
  std::string str;
  uint32_t temp = i / 16;
  uint32_t left = i % 16;
  if (temp > 0) {
    str += UintToHexString(temp);
  }
  if (left < 10) {
    str += (left + '0');
  } else {
    str += ('A' + left - 10);
  }
  return str;
}

bool StringStartsWith(const std::string& str, const std::string& prefix) {
  return (str.rfind(prefix, 0) == 0);
}

bool StringEndsWith(const std::string& str, const std::string& suffix) {
  if (str.length() < suffix.length()) {
    ZWARN << "String \"" << str << "\" is shorter than suffix \"" << suffix << "\"";
    return false;
  }

  return (str.rfind(suffix) == (str.length() - suffix.length()));
}

std::string GetZimaPrintString() {
  std::string str;
  std::string swapline = "\n";

  str += swapline + R"(  ________                              )";
  str += swapline + R"( /\_____  \  __                         )";
  str += swapline + R"( \/_____/ / /\_\    ___ ____     __     )";
  str += swapline + R"(       / /  \/\ \  / __` __ \  /'__`\   )";
  str += swapline + R"(      / /____\ \ \/\ \/\ \/\ \/\ \_\.\_ )";
  str += swapline + R"(     /\_______\ \_\ \_\ \_\ \_\ \__/ \_\)";
  str += swapline + R"(     \/_______/\/_/\/_/\/_/\/_/\/__/\/_/)";
  str += swapline + R"(                                        )";
  str += swapline;

  return str;
}

}  // namespace zima
