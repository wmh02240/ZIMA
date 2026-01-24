/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/hal/system/process.h"
#include "zima/hal/system/cmd_line.h"

#include "zima/logger/logger.h"

namespace zima {

uint32_t ZimaGetProcessMemoryUsageInKB() {
  FILE* file = fopen("/proc/self/status", "r");
  int memory_usage = -1;
  char line[128];

  while (fgets(line, 128, file) != nullptr) {
    if (strncmp(line, "VmRSS:", 6) == 0) {
      // ZINFO << line;
      int len = strlen(line);

      const char* p = line;
      for (; std::isdigit(*p) == false; ++p) {
      }

      line[len - 3] = 0;
      memory_usage = atoi(p);

      break;
    }
  }

  fclose(file);
  // ZINFO << "Result: " << memory_usage;

  return memory_usage;
}

}  // namespace zima
