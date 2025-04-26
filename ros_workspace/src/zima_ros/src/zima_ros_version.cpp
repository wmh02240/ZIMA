/*
 * This file is part of Project Zima.
 * Copyright © 2023-2025 Austin Liu.
 * Released under the [MIT] License.
 */

#include "zima_ros/zima_ros_version.h"

#include "zima/common/util.h"

namespace zima {

#define ZIMA_ROS_MAJOR_VERSION "0"
#define ZIMA_ROS_MINOR_VERSION "1"
#define ZIMA_ROS_PATCH_VERSION "1"

std::string GetROSVersionInfo() {
  static std::string version_info;
  if (version_info.empty()) {
    version_info += GetZimaPrintString();
    version_info += "Zima ros version " + std::string(ZIMA_ROS_MAJOR_VERSION) +
                    "." + std::string(ZIMA_ROS_MINOR_VERSION) + "." +
                    std::string(ZIMA_ROS_PATCH_VERSION) + ", build at " +
                    __DATE__ + " " + __TIME__;
  }
  return version_info;
}

}  // namespace zima
