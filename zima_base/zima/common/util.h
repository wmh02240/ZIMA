/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_UTIL_H
#define ZIMA_UTIL_H

#include <string>
#include <sstream>
#include <iomanip>

#include "zima/common/debug.h"

namespace zima {

std::string FloatToString(const float& val, const int& precision);
std::string DoubleToString(const double& val, const int& precision);

std::string UintToHexString(const uint32_t& i);

bool StringStartsWith(const std::string& str, const std::string& prefix);
bool StringEndsWith(const std::string& str, const std::string& suffix);

std::string GetZimaPrintString();

}  // namespace zima

#endif  // ZIMA_UTIL_H
