/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima/hal/system/cmd_line.h"

#include "zima/logger/logger.h"

namespace zima {

int ZimaRunCommand(const std::string& command) {
  ZGINFO << "Run command \"" << command << "\"";
  auto ret = system(command.c_str());
  ZGINFO << "Run command \"" << command << "\" return " << ret;
  return ret;
}

std::string ZimaGetCommandResult(const std::string &command) {
  char buf[10240] = {0};
  FILE *pf = NULL;

  ZGINFO << "Run command \"" << command << "\"";

  if ((pf = popen(command.c_str(), "r")) == NULL) {
    return "";
  }

  std::string result_str = "\n";
  while (fgets(buf, sizeof(buf), pf)) {
    result_str += buf;
  }

  pclose(pf);

  unsigned int result_size = result_str.size();
  if (result_size > 0 && result_str[result_size - 1] == '\n')  // linux
  {
    result_str = result_str.substr(0, result_size - 1);
  }

  return result_str;
}

}  // namespace zima
