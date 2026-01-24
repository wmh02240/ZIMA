/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_CPU_H
#define ZIMA_CPU_H

#include <string>

namespace zima {

bool IsRunningOnARM();
bool IsRunningOnX86();

}  // namespace zima

#endif  // ZIMA_CPU_H
