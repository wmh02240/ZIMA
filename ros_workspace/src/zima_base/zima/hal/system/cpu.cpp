/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/hal/system/cpu.h"

#include <iostream>

#include "zima/logger/logger.h"

namespace zima {

bool IsRunningOnARM() {
#ifdef __aarch64__
  return true;
#else
  return false;
#endif
}

bool IsRunningOnX86() {
#ifdef __aarch64__
  return false;
#else
  return true;
#endif
}

}  // namespace zima
