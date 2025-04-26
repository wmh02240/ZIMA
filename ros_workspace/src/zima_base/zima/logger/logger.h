/*
 * This file is part of Project Zima.
 * Copyright © 2021-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#ifndef ZIMA_LOGGER_H
#define ZIMA_LOGGER_H

#include <cstdio>
#include <glog/logging.h>

#include "zima/common/debug.h"
#include "zima/common/gflags.h"

#define ZISODBG LOG_IF(INFO, IsDebugLogEnabled()) << __FUNCTION__ << ": "
#define ZGINFO LOG_IF(INFO, FLAGS_debug_enable) << __FUNCTION__ << ": "
#define ZGWARN LOG_IF(WARNING, FLAGS_debug_enable) << __FUNCTION__ << ": "
#define ZGERROR LOG_IF(ERROR, FLAGS_debug_enable) << __FUNCTION__ << ": "

#define ZINFO LOG(INFO) << __FUNCTION__ << ": "
#define ZWARN LOG(WARNING) << __FUNCTION__ << ": "
#define ZERROR LOG(ERROR) << __FUNCTION__ << ": "
#define ZFATAL LOG(FATAL) << __FUNCTION__ << ": "

#define ZCOLOR_NONE "\033[m"
#define ZCOLOR_RED "\033[0;32;31m"
#define ZCOLOR_LIGHT_RED "\033[1;31m"
#define ZCOLOR_GREEN "\033[0;32;32m"
#define ZCOLOR_LIGHT_GREEN "\033[1;32m"  
#define ZCOLOR_BLUE "\033[0;32;34m"
#define ZCOLOR_LIGHT_BLUE "\033[1;34m"
#define ZCOLOR_DARY_GRAY "\033[1;30m"
#define ZCOLOR_CYAN "\033[0;36m"
#define ZCOLOR_LIGHT_CYAN "\033[1;36m"
#define ZCOLOR_PURPLE "\033[0;35m"
#define ZCOLOR_LIGHT_PURPLE "\033[1;35m"
#define ZCOLOR_YELLOW "\033[0;33m"
#define ZCOLOR_WHITE "\033[1;37m"

#endif  // ZIMA_LOGGER_H
