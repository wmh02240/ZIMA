/*
 * This file is part of Project Zima.
 * Copyright © 2022-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_CMD_LINE_H
#define ZIMA_CMD_LINE_H

#include <string>

namespace zima {

int ZimaRunCommand(const std::string& command);

std::string ZimaGetCommandResult(const std::string &command);

}  // namespace zima

#endif  // ZIMA_CMD_LINE_H
